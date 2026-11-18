using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace Showcase.Editor
{
    /// <summary>
    /// Generates every showcase asset from code: materials, studio meshes, URP pipeline assets,
    /// the post-processing volume, the look and the Showcase scene. The code is the source of truth;
    /// rerun it after changing anything here or when the training scene's prefabs change.
    ///
    /// Unity -batchmode -projectPath . -executeMethod Showcase.Editor.ShowcaseBuilder.BuildFromCommandLine -quit
    /// </summary>
    public static class ShowcaseBuilder
    {
        const string Root = "Assets/Showcase";
        const string TrainingScene = "Assets/Scenes/Playground.unity";

        [MenuItem("Showcase/Rebuild assets and scene")]
        public static void Build()
        {
            var training = ReadTrainingScene();
            foreach (var dir in new[] { "Materials", "Meshes", "Pipeline", "Reels" })
                Directory.CreateDirectory($"{Root}/{dir}");
            AssetDatabase.Refresh();

            var look = BuildLook(training);
            var showcasePipeline = BuildPipeline("ShowcaseURP", offline: true);
            BuildPipeline("LiveURP", offline: false);
            var profile = BuildVolumeProfile();
            BuildScene(look, profile, training.floorY);
            AssetDatabase.SaveAssets();
            Debug.Log($"[showcase] built; offline pipeline {AssetDatabase.GetAssetPath(showcasePipeline)}");
        }

        public static void BuildFromCommandLine()
        {
            try { Build(); EditorApplication.Exit(0); }
            catch (Exception e) { Debug.LogException(e); EditorApplication.Exit(1); }
        }

        // ---------------------------------------------------------------- training scene

        struct TrainingRefs { public GameObject arm, ball, target; public float floorY; }

        /// <summary>The agent template's prefabs and ground height, so the replay matches what training spawns.</summary>
        static TrainingRefs ReadTrainingScene()
        {
            var scene = EditorSceneManager.OpenScene(TrainingScene, OpenSceneMode.Single);
            var agent = scene.GetRootGameObjects()
                .SelectMany(g => g.GetComponentsInChildren<MLAgent>(true))
                .First(a => a.simulated_prefab != null);
            var so = new SerializedObject(agent);
            var refs = new TrainingRefs
            {
                arm = agent.simulated_prefab,
                ball = (GameObject)so.FindProperty("projectilePrefab").objectReferenceValue,
                target = (GameObject)so.FindProperty("targetPrefab").objectReferenceValue,
                floorY = 0f,
            };
            var ground = so.FindProperty("groundCollider").objectReferenceValue as BoxCollider;
            if (ground != null)
            {
                var t = ground.transform;
                refs.floorY = t.TransformPoint(ground.center + 0.5f * ground.size.y * Vector3.up).y;
            }
            Debug.Log($"[showcase] training prefabs: arm {refs.arm.name}, ball {refs.ball.name}, target {refs.target.name}; floor y {refs.floorY}");
            return refs;
        }

        // ---------------------------------------------------------------- materials

        static ShowcaseLook BuildLook(TrainingRefs training)
        {
            var look = LoadOrCreate<ShowcaseLook>($"{Root}/ShowcaseLook.asset");
            look.armPrefab = training.arm;
            look.ballPrefab = training.ball;
            look.targetPrefab = training.target;

            // Glazed white ceramic shell: a soft diffuse body under a glossy clear coat.
            look.armShell = Lit("ArmShell", Hex("E9E6E0"), smoothness: 0.45f, clearCoat: 0.8f, clearCoatSmoothness: 0.92f);
            // Satin graphite hand: the business end reads darker, and the orange ball pops against it.
            look.handShell = Lit("HandShell", Hex("2A2C30"), smoothness: 0.58f, metallic: 0.35f, clearCoat: 0.3f, clearCoatSmoothness: 0.8f);
            // Rubber ball in a saturated signal orange.
            look.ball = Lit("Ball", Hex("FF5A14"), smoothness: 0.38f);
            // Target: deep teal glass-like body with a soft inner glow that blooms a little.
            look.target = Lit("Target", Hex("0E6F78"), smoothness: 0.85f, emission: Hex("19C4CF") * 1.6f);
            look.targetHit = Lit("TargetHit", Hex("D8FFFF"), smoothness: 0.85f, emission: Hex("B8FBFF") * 9f);
            EditorUtility.SetDirty(look);
            return look;
        }

        static Material Lit(string name, Color color, float smoothness, float metallic = 0f,
            float clearCoat = 0f, float clearCoatSmoothness = 0f, Color? emission = null)
        {
            var shader = Shader.Find(clearCoat > 0f ? "Universal Render Pipeline/Complex Lit" : "Universal Render Pipeline/Lit");
            var path = $"{Root}/Materials/{name}.mat";
            var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (mat == null) { mat = new Material(shader); AssetDatabase.CreateAsset(mat, path); }
            mat.shader = shader;
            mat.SetColor("_BaseColor", color);
            mat.SetFloat("_Smoothness", smoothness);
            mat.SetFloat("_Metallic", metallic);
            mat.SetFloat("_EnvironmentReflections", 1f);
            mat.SetFloat("_SpecularHighlights", 1f);
            if (clearCoat > 0f)
            {
                mat.SetFloat("_ClearCoat", 1f);
                mat.EnableKeyword("_CLEARCOAT");
                mat.SetFloat("_ClearCoatMask", clearCoat);
                mat.SetFloat("_ClearCoatSmoothness", clearCoatSmoothness);
            }
            if (emission is Color e)
            {
                mat.SetColor("_EmissionColor", e);
                mat.EnableKeyword("_EMISSION");
                mat.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;
            }
            else
            {
                mat.DisableKeyword("_EMISSION");
                mat.SetColor("_EmissionColor", Color.black);
            }
            mat.enableInstancing = true;
            EditorUtility.SetDirty(mat);
            return mat;
        }

        static Color Hex(string hex) => ColorUtility.TryParseHtmlString("#" + hex, out var c) ? c : Color.magenta;

        // ---------------------------------------------------------------- pipeline

        /// <summary>
        /// Offline: 4K shadow maps, SSAO, HDR at 64-bit precision, HDR grading.
        /// Live (WebGL): the same look at a cost a wall-screen browser can hold at 60 fps.
        /// </summary>
        static UniversalRenderPipelineAsset BuildPipeline(string name, bool offline)
        {
            string rendererPath = $"{Root}/Pipeline/{name}_Renderer.asset";
            string assetPath = $"{Root}/Pipeline/{name}.asset";
            AssetDatabase.DeleteAsset(rendererPath);
            AssetDatabase.DeleteAsset(assetPath);

            var renderer = ScriptableObject.CreateInstance<UniversalRendererData>();
            AssetDatabase.CreateAsset(renderer, rendererPath);
            ResourceReloader.ReloadAllNullIn(renderer, UniversalRenderPipelineAsset.packagePath);
            AddSsao(renderer, offline);

            var asset = UniversalRenderPipelineAsset.Create(renderer);
            AssetDatabase.CreateAsset(asset, assetPath);
            var so = new SerializedObject(asset);
            Set(so, "m_SupportsHDR", true);
            Set(so, "m_HDRColorBufferPrecision", offline ? 1 : 0); // 64-bit offline
            Set(so, "m_MSAA", offline ? 1 : 4);                     // offline antialiasing comes from accumulation
            Set(so, "m_RenderScale", 1f);
            Set(so, "m_MainLightRenderingMode", 1);
            Set(so, "m_MainLightShadowsSupported", true);
            Set(so, "m_MainLightShadowmapResolution", offline ? 4096 : 2048);
            Set(so, "m_AdditionalLightsRenderingMode", 1);
            Set(so, "m_AdditionalLightShadowsSupported", offline);
            Set(so, "m_AdditionalLightsShadowmapResolution", 2048);
            Set(so, "m_ShadowDistance", 160f);
            Set(so, "m_ShadowCascadeCount", offline ? 4 : 2);
            Set(so, "m_SoftShadowsSupported", true);
            Set(so, "m_ColorGradingMode", 1); // HDR grading
            Set(so, "m_ColorGradingLutSize", offline ? 65 : 32);
            Set(so, "m_SupportsCameraDepthTexture", true);
            Set(so, "m_SupportsCameraOpaqueTexture", false);
            Set(so, "m_VolumeFrameworkUpdateMode", 0); // every frame: the replay animates DoF per sub-frame
            so.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(asset);
            return asset;
        }

        /// <summary>URP's SSAO feature is internal, so it is created by type name and wired into the renderer by hand.</summary>
        static void AddSsao(UniversalRendererData renderer, bool offline)
        {
            var type = typeof(UniversalRendererData).Assembly.GetType("UnityEngine.Rendering.Universal.ScreenSpaceAmbientOcclusion");
            var feature = (ScriptableRendererFeature)ScriptableObject.CreateInstance(type);
            feature.name = "SSAO";
            AssetDatabase.AddObjectToAsset(feature, renderer);
            renderer.rendererFeatures.Add(feature);
            AssetDatabase.TryGetGUIDAndLocalFileIdentifier(feature, out _, out long localId);
            var rso = new SerializedObject(renderer);
            var map = rso.FindProperty("m_RendererFeatureMap");
            map.arraySize = 1;
            map.GetArrayElementAtIndex(0).longValue = localId;
            rso.ApplyModifiedPropertiesWithoutUndo();

            var fso = new SerializedObject(feature);
            Set(fso, "m_Settings.Downsample", !offline);
            Set(fso, "m_Settings.Source", 1); // depth-normals
            Set(fso, "m_Settings.NormalSamples", 2); // high quality reconstruction
            Set(fso, "m_Settings.Intensity", 0.7f);
            Set(fso, "m_Settings.DirectLightingStrength", 0.2f);
            Set(fso, "m_Settings.Radius", 0.6f);
            Set(fso, "m_Settings.SampleCount", offline ? 12 : 4);
            fso.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(renderer);
        }

        static void Set(SerializedObject so, string path, object value)
        {
            var p = so.FindProperty(path);
            if (p == null) { Debug.LogWarning($"[showcase] {so.targetObject.GetType().Name} has no {path}"); return; }
            switch (value)
            {
                case bool b: p.boolValue = b; break;
                case int i when p.propertyType == SerializedPropertyType.Enum: p.enumValueIndex = i; break;
                case int i: p.intValue = i; break;
                case float f: p.floatValue = f; break;
            }
        }

        // ---------------------------------------------------------------- post-processing

        static VolumeProfile BuildVolumeProfile()
        {
            string path = $"{Root}/Pipeline/ShowcaseVolume.asset";
            AssetDatabase.DeleteAsset(path);
            var profile = ScriptableObject.CreateInstance<VolumeProfile>();
            AssetDatabase.CreateAsset(profile, path);

            var tone = Add<Tonemapping>(profile);
            tone.mode.Override(TonemappingMode.ACES);

            var bloom = Add<Bloom>(profile);
            bloom.threshold.Override(1.1f);
            bloom.intensity.Override(0.35f);
            bloom.scatter.Override(0.72f);
            bloom.highQualityFiltering.Override(true);

            var grade = Add<ColorAdjustments>(profile);
            grade.postExposure.Override(0.15f);
            grade.contrast.Override(8f);
            grade.saturation.Override(4f);

            var wb = Add<WhiteBalance>(profile);
            wb.temperature.Override(4f);

            // Gentle shadow/highlight split: cool shadows, warm highlights.
            var smh = Add<ShadowsMidtonesHighlights>(profile);
            smh.shadows.Override(new Vector4(0.96f, 0.99f, 1.04f, 0f));
            smh.highlights.Override(new Vector4(1.03f, 1.0f, 0.97f, 0f));

            var vignette = Add<Vignette>(profile);
            vignette.intensity.Override(0.22f);
            vignette.smoothness.Override(0.5f);

            var dof = Add<DepthOfField>(profile);
            dof.mode.Override(DepthOfFieldMode.Bokeh);
            dof.focusDistance.Override(10f);
            dof.focalLength.Override(50f);
            dof.aperture.Override(4f);
            dof.bladeCount.Override(7);
            dof.bladeCurvature.Override(0.9f);

            // Motion blur is accumulated from real sub-frames offline; the live view gets URP's camera blur.
            var blur = Add<MotionBlur>(profile);
            blur.active = false;
            blur.quality.Override(MotionBlurQuality.High);
            blur.intensity.Override(0.4f);

            EditorUtility.SetDirty(profile);
            return profile;
        }

        static T Add<T>(VolumeProfile profile) where T : VolumeComponent
        {
            var c = profile.Add<T>(true);
            c.name = typeof(T).Name;
            c.hideFlags = HideFlags.HideInInspector | HideFlags.HideInHierarchy;
            AssetDatabase.AddObjectToAsset(c, profile);
            return c;
        }

        // ---------------------------------------------------------------- scene

        static void BuildScene(ShowcaseLook look, VolumeProfile profile, float floorY)
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var stageGo = new GameObject("Stage");
            var stage = stageGo.AddComponent<Stage>();
            stage.floorY = floorY;

            var studio = Lit("Studio", Hex("A7A39D"), smoothness: 0.18f);
            var pedestalMat = Lit("Pedestal", Hex("3A3C40"), smoothness: 0.42f, metallic: 0.2f);

            stage.cove = MeshObject("Cove", stageGo.transform, SaveMesh(CoveMesh(), "Cove"), studio, castShadows: false).transform;
            stage.pedestal = MeshObject("Pedestal", stageGo.transform, SaveMesh(PedestalMesh(), "Pedestal"), pedestalMat, castShadows: true).transform;
            stage.cove.position = new Vector3(0, floorY, 0);
            stage.cove.localScale = Vector3.one * 200f;

            // Key: high, warm, from the front-left of the throw; soft shadows.
            var key = DirectionalLight("Key", stageGo.transform, Hex("FFF1DE"), 2.4f, new Vector3(48f, -35f, 0f), shadows: true);
            key.shadowStrength = 0.82f;
            key.shadowBias = 0.05f;
            key.shadowNormalBias = 0.3f;
            key.shadowSoftness = 0.6f;
            // Rim: cool, from behind and above, picks the silhouette out of the backdrop.
            DirectionalLight("Rim", stageGo.transform, Hex("CFE0FF"), 1.6f, new Vector3(28f, 150f, 0f), shadows: false);
            // Fill: low and dim from the camera side, keeps the shadow side from going dead.
            DirectionalLight("Fill", stageGo.transform, Hex("E8EEF5"), 0.35f, new Vector3(8f, -120f, 0f), shadows: false);

            var probeGo = new GameObject("Reflections");
            probeGo.transform.SetParent(stageGo.transform, false);
            stage.probe = probeGo.AddComponent<ReflectionProbe>();
            stage.probe.mode = ReflectionProbeMode.Realtime;
            stage.probe.refreshMode = ReflectionProbeRefreshMode.ViaScripting;
            stage.probe.timeSlicingMode = ReflectionProbeTimeSlicingMode.NoTimeSlicing;
            stage.probe.resolution = 512;
            stage.probe.hdr = true;
            stage.probe.importance = 1;
            stage.probe.size = Vector3.one * 400f;

            var volumeGo = new GameObject("Post");
            var volume = volumeGo.AddComponent<Volume>();
            volume.isGlobal = true;
            volume.sharedProfile = profile;

            var camGo = new GameObject("ShowcaseCamera");
            var cam = camGo.AddComponent<Camera>();
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = Hex("A7A39D");
            cam.nearClipPlane = 0.05f;
            cam.farClipPlane = 5000f;
            cam.fieldOfView = 30f;
            cam.allowHDR = true;
            var data = camGo.AddComponent<UniversalAdditionalCameraData>();
            data.renderPostProcessing = true;
            data.renderShadows = true;
            data.requiresDepthOption = CameraOverrideOption.On;
            camGo.transform.SetPositionAndRotation(new Vector3(0, 0, -40), Quaternion.identity);

            // Soft studio ambient: a cool sky over a warm bounce from the floor.
            RenderSettings.skybox = null;
            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = Hex("8E97A3");
            RenderSettings.ambientEquatorColor = Hex("77736D");
            RenderSettings.ambientGroundColor = Hex("4A4642");
            RenderSettings.defaultReflectionMode = DefaultReflectionMode.Custom;
            RenderSettings.fog = false;
            RenderSettings.sun = key;

            EditorSceneManager.SaveScene(scene, ReplayRenderer.ScenePath);
        }

        static Light DirectionalLight(string name, Transform parent, Color color, float intensity, Vector3 euler, bool shadows)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.rotation = Quaternion.Euler(euler);
            var light = go.AddComponent<Light>();
            light.type = LightType.Directional;
            light.color = color;
            light.intensity = intensity;
            light.shadows = shadows ? LightShadows.Soft : LightShadows.None;
            go.AddComponent<UniversalAdditionalLightData>();
            return light;
        }

        static GameObject MeshObject(string name, Transform parent, Mesh mesh, Material mat, bool castShadows)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            var r = go.AddComponent<MeshRenderer>();
            r.sharedMaterial = mat;
            r.shadowCastingMode = castShadows ? ShadowCastingMode.On : ShadowCastingMode.Off;
            r.receiveShadows = true;
            return go;
        }

        static Mesh SaveMesh(Mesh mesh, string name)
        {
            string path = $"{Root}/Meshes/{name}.asset";
            AssetDatabase.DeleteAsset(path);
            mesh.name = name;
            AssetDatabase.CreateAsset(mesh, path);
            return mesh;
        }

        // ---------------------------------------------------------------- procedural studio meshes

        /// <summary>
        /// Round cyclorama of unit radius: a flat floor that curves smoothly up into a wall all around,
        /// so no camera angle ever finds a corner or a horizon line.
        /// </summary>
        static Mesh CoveMesh()
        {
            var profile = new List<(Vector2 p, Vector2 n)>();
            const float floor = 0.62f, curve = 0.38f, wall = 1.4f;
            for (int i = 0; i <= 24; i++) // floor rings, dense enough for smooth large-scale shading
                profile.Add((new Vector2(floor * i / 24f, 0f), Vector2.up));
            for (int i = 1; i <= 32; i++)
            {
                float a = 0.5f * Mathf.PI * i / 32f;
                var center = new Vector2(floor, curve);
                var dir = new Vector2(Mathf.Sin(a), -Mathf.Cos(a));
                profile.Add((center + dir * curve, -dir));
            }
            for (int i = 1; i <= 8; i++)
                profile.Add((new Vector2(floor + curve, curve + (wall - curve) * i / 8f), Vector2.left));
            return Lathe(profile, 192);
        }

        /// <summary>Unit cylinder (radius 0.5, height 1, pivot at the base) with a softened top edge.</summary>
        static Mesh PedestalMesh()
        {
            var profile = new List<(Vector2 p, Vector2 n)>
            {
                (new Vector2(0.5f, 0f), Vector2.right),
                (new Vector2(0.5f, 0.97f), Vector2.right),
            };
            const float bevel = 0.03f;
            for (int i = 1; i <= 8; i++)
            {
                float a = 0.5f * Mathf.PI * i / 8f;
                var dir = new Vector2(Mathf.Cos(a), Mathf.Sin(a));
                profile.Add((new Vector2(0.5f - bevel, 1f - bevel) + dir * bevel, dir));
            }
            profile.Add((new Vector2(0f, 1f), Vector2.up));
            return Lathe(profile, 96);
        }

        /// <summary>Surface of revolution about +Y from a (radius, height) profile with per-point normals.</summary>
        static Mesh Lathe(List<(Vector2 p, Vector2 n)> profile, int segments)
        {
            var verts = new List<Vector3>();
            var normals = new List<Vector3>();
            var uvs = new List<Vector2>();
            var tris = new List<int>();
            int rings = profile.Count;
            for (int s = 0; s <= segments; s++)
            {
                float a = 2f * Mathf.PI * s / segments;
                float c = Mathf.Cos(a), sn = Mathf.Sin(a);
                for (int i = 0; i < rings; i++)
                {
                    var (p, n) = profile[i];
                    verts.Add(new Vector3(p.x * c, p.y, p.x * sn));
                    normals.Add(new Vector3(n.x * c, n.y, n.x * sn).normalized);
                    uvs.Add(new Vector2(s / (float)segments, i / (float)(rings - 1)));
                }
            }
            for (int s = 0; s < segments; s++)
                for (int i = 0; i < rings - 1; i++)
                {
                    int a = s * rings + i, b = a + 1, c = a + rings, d = c + 1;
                    // Wind each quad so its geometric normal agrees with the authored one.
                    var geo = Vector3.Cross(verts[b] - verts[a], verts[c] - verts[a]);
                    if (geo.sqrMagnitude < 1e-12f) geo = Vector3.Cross(verts[d] - verts[b], verts[a] - verts[b]);
                    bool flip = Vector3.Dot(geo, normals[a] + normals[b]) < 0f;
                    if (flip) tris.AddRange(new[] { a, c, b, b, c, d });
                    else tris.AddRange(new[] { a, b, c, b, d, c });
                }
            var mesh = new Mesh { indexFormat = IndexFormat.UInt32 };
            mesh.SetVertices(verts);
            mesh.SetNormals(normals);
            mesh.SetUVs(0, uvs);
            mesh.SetTriangles(tris, 0);
            mesh.RecalculateBounds();
            mesh.RecalculateTangents();
            return mesh;
        }

        static T LoadOrCreate<T>(string path) where T : ScriptableObject
        {
            var asset = AssetDatabase.LoadAssetAtPath<T>(path);
            if (asset != null) return asset;
            asset = ScriptableObject.CreateInstance<T>();
            AssetDatabase.CreateAsset(asset, path);
            return asset;
        }
    }
}
