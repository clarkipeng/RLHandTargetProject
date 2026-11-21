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

        [MenuItem("Showcase/Rebuild assets and scenes")]
        public static void Build()
        {
            var training = ReadTrainingScene();
            foreach (var dir in new[] { "Materials/Minimal", "Materials/Art", "Meshes", "Pipeline", "Reels", "Looks" })
                Directory.CreateDirectory($"{Root}/{dir}");
            // Assets of the single look before there were two.
            foreach (var old in new[] { "Showcase.unity", "ShowcaseLook.asset", "Pipeline/ShowcaseVolume.asset" })
                AssetDatabase.DeleteAsset($"{Root}/{old}");
            AssetDatabase.Refresh();

            var showcasePipeline = BuildPipeline("ShowcaseURP", offline: true);
            BuildPipeline("LiveURP", offline: false);
            BuildMinimal(training);
            BuildArt(training);
            AssetDatabase.SaveAssets();
            Debug.Log($"[showcase] built looks {string.Join(", ", ReplayRenderer.Looks)}; offline pipeline {AssetDatabase.GetAssetPath(showcasePipeline)}");
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

        // ---------------------------------------------------------------- looks

        static ShowcaseLook NewLook(string name, TrainingRefs training)
        {
            var look = LoadOrCreate<ShowcaseLook>(ReplayRenderer.LookPath(name));
            look.armPrefab = training.arm;
            look.ballPrefab = training.ball;
            look.targetPrefab = training.target;
            look.smoothMeshes = true;
            look.joint = null;
            look.targetDressing = null;
            return look;
        }

        /// <summary>
        /// Minimal: the training shapes, refined. A warm grey cyclorama, one big soft key, a cool rim,
        /// glazed ceramic arm, graphite hand, an orange ball and a glass target.
        /// </summary>
        static void BuildMinimal(TrainingRefs training)
        {
            const string M = "Minimal/";
            var look = NewLook("Minimal", training);
            // Glazed white ceramic shell: a soft diffuse body under a glossy clear coat.
            look.armShell = Lit(M + "ArmShell", Hex("E9E6E0"), smoothness: 0.45f, clearCoat: 0.8f, clearCoatSmoothness: 0.92f);
            // Satin graphite hand: the business end reads darker, and the orange ball pops against it.
            look.handShell = Lit(M + "HandShell", Hex("3C4048"), smoothness: 0.62f, metallic: 0.45f, clearCoat: 0.3f, clearCoatSmoothness: 0.8f);
            look.ball = Lit(M + "Ball", Hex("FF5A14"), smoothness: 0.38f);
            // A tinted glass bubble with a faint inner glow; the arm stays readable behind it.
            look.target = Lit(M + "Target", Hex("2FC6D2", 0.22f), smoothness: 0.95f, emission: Hex("19C4CF") * 0.3f, transparent: true);
            look.prop = Lit(M + "BallStand", Hex("2B2F36"), smoothness: 0.5f, metallic: 0.3f);
            look.targetHit = Lit(M + "TargetHit", Hex("9FF4FA", 0.4f), smoothness: 0.95f, emission: Hex("4FD8E4") * 1.3f, transparent: true);
            EditorUtility.SetDirty(look);

            var profile = BuildVolumeProfile("Minimal", exposure: -0.1f, contrast: 8f, saturation: 4f, bloomThreshold: 1.1f, bloomIntensity: 0.3f,
                vignette: 0.2f, shadowsTint: new Vector4(0.96f, 0.99f, 1.04f, 0f), highlightsTint: new Vector4(1.03f, 1f, 0.97f, 0f));
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var stage = NewStage(training.floorY);
            var studio = Lit(M + "Studio", Hex("75716C"), smoothness: 0.12f);
            stage.cove = MeshObject("Cove", stage.transform, SaveMesh(CoveMesh(), "Cove"), studio, castShadows: false).transform;
            stage.pedestal = MeshObject("Pedestal", stage.transform, SaveMesh(PedestalMesh(), "Pedestal"),
                Lit(M + "Pedestal", Hex("3A3C40"), smoothness: 0.42f, metallic: 0.2f), castShadows: true).transform;
            stage.pedestalRadius = 0.11f;

            // Key: high, warm, from the front-left of the throw; a big soft source (8 degrees across).
            var key = DirectionalLight("Key", stage.transform, Hex("FFF1DE"), 3.2f, new Vector3(42f, -35f, 0f), shadows: true, softDegrees: 4f);
            key.shadowStrength = 0.8f;
            // Rim: cool, from behind and above, picks the silhouette out of the backdrop.
            DirectionalLight("Rim", stage.transform, Hex("CFE0FF"), 1.4f, new Vector3(22f, 150f, 0f), shadows: true, softDegrees: 3f).shadowStrength = 0.5f;
            // Fill: low and dim from the camera side, keeps the shadow side from going dead.
            DirectionalLight("Fill", stage.transform, Hex("E8EEF5"), 0.18f, new Vector3(8f, -120f, 0f), shadows: false);
            Probe(stage, 0.6f);
            Post(profile);
            MakeCamera(Hex("3A3835"));
            Ambient(Hex("5A606A"), Hex("4A4743"), Hex("2C2A28"));
            RenderSettings.fog = false;
            RenderSettings.sun = key;
            EditorSceneManager.SaveScene(scene, ReplayRenderer.ScenePath("Minimal"));
        }

        /// <summary>
        /// Art: a dark hall of light fins in haze around a spotlit plinth. Ceramic and graphite arm with
        /// machined joints, a red and white bullseye that always faces the camera, a cyan ball.
        /// </summary>
        static void BuildArt(TrainingRefs training)
        {
            const string A = "Art/";
            var look = NewLook("Art", training);
            look.armShell = Lit(A + "ArmShell", Hex("ECE8E1"), smoothness: 0.5f, clearCoat: 0.9f, clearCoatSmoothness: 0.95f);
            look.handShell = Lit(A + "HandShell", Hex("8A9099"), smoothness: 0.62f, metallic: 0.85f); // brushed titanium: catches the strips
            look.joint = Lit(A + "Joint", Hex("26282C"), smoothness: 0.72f, metallic: 0.8f);
            look.ball = Lit(A + "Ball", Hex("1FC3DC"), smoothness: 0.55f, emission: Hex("1FC3DC") * 0.18f);
            look.target = Lit(A + "Target", Hex("E0402F", 0.2f), smoothness: 0.9f, transparent: true);
            look.prop = Lit(A + "BallStand", Hex("2A2C30"), smoothness: 0.58f, metallic: 0.65f);
            look.targetHit = Lit(A + "TargetHit", Hex("FFE2A8", 0.55f), smoothness: 0.9f, emission: Hex("FFC468") * 1.7f, transparent: true);
            look.targetDressing = BullseyePrefab(A, look.targetHit);
            EditorUtility.SetDirty(look);

            var fogColor = Hex("171B22");
            var profile = BuildVolumeProfile("Art", exposure: 0.4f, contrast: 18f, saturation: 6f, bloomThreshold: 0.95f, bloomIntensity: 0.65f,
                vignette: 0.32f, shadowsTint: new Vector4(0.93f, 0.98f, 1.08f, 0f), highlightsTint: new Vector4(1.05f, 1f, 0.94f, 0f));
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var stage = NewStage(training.floorY);
            stage.fogDensityTimesArm = 0.095f;

            // Everything below is authored with the arm length as the unit, origin on the floor under the shoulder.
            var env = new GameObject("Environment").transform;
            env.SetParent(stage.transform, false);
            stage.environment = env;

            var floor = MeshObject("Floor", env, SaveMesh(DiscMesh(), "Disc"),
                Lit(A + "Floor", Hex("16171A"), smoothness: 0.8f), castShadows: false);
            floor.transform.localScale = Vector3.one * 60f;

            // A ring of tall fins, each with a light strip on its inner face: they frame every angle,
            // fall away into the haze, and give the background its bokeh.
            var cube = Resources.GetBuiltinResource<Mesh>("Cube.fbx");
            var finMat = Lit(A + "Fin", Hex("2C2E33"), smoothness: 0.45f, metallic: 0.3f);
            var stripMat = Lit(A + "Strip", Hex("0A0A0A"), smoothness: 0.3f, emission: Hex("C9D8F5") * 1.1f);
            var warmStripMat = Lit(A + "WarmStrip", Hex("0A0A0A"), smoothness: 0.3f, emission: Hex("FFC98A") * 1.5f);
            const int fins = 44;
            const float ring = 7.5f, height = 9f;
            for (int i = 0; i < fins; i++)
            {
                float a = 360f * i / fins;
                var rot = Quaternion.Euler(0, a, 0);
                var fin = MeshObject($"Fin{i}", env, cube, finMat, castShadows: false).transform;
                fin.localRotation = rot;
                fin.localPosition = rot * new Vector3(0, 0, ring + 0.25f) + Vector3.up * height * 0.5f;
                fin.localScale = new Vector3(0.22f, height, 0.5f);
                if (i % 2 == 1) continue; // a strip on every other fin: rhythm, not bars
                var strip = MeshObject($"Strip{i}", env, cube, i % 8 == 0 ? warmStripMat : stripMat, castShadows: false).transform;
                strip.localRotation = rot;
                float h = i % 8 == 0 ? 6f : 4.2f;
                strip.localPosition = rot * new Vector3(0, 0, ring - 0.02f) + Vector3.up * (0.25f + h * 0.5f);
                strip.localScale = new Vector3(0.035f, h, 0.02f);
            }
            // The stage mark: a thin warm ring on the floor around the plinth.
            var markMat = Lit(A + "Mark", Hex("0A0A0A"), smoothness: 0.3f, emission: Hex("FFB766") * 1.6f);
            var mark = MeshObject("Mark", env, SaveMesh(TorusMesh(1f, 0.006f), "MarkRing"), markMat, castShadows: false).transform;
            mark.localScale = new Vector3(0.75f, 1f, 0.75f);

            stage.pedestal = MeshObject("Plinth", stage.transform, SaveMesh(PlinthMesh(), "Plinth"),
                Lit(A + "Plinth", Hex("2A2C30"), smoothness: 0.58f, metallic: 0.65f), castShadows: true).transform;
            stage.pedestalRadius = 0.12f;

            // Key: a warm spot from high front-left, a pool of light on the plinth and the arm; soft (0.35 arm lengths across).
            SpotLight("Key", env, Hex("FFE6C4"), 46f, new Vector3(-1.6f, 4.2f, -2.2f), new Vector3(0.7f, 0.7f, 0f), 68f, size: 0.18f, range: 12f, shadows: true);
            // Kicker: a cool spot low from behind, carving the arm's far edge out of the dark.
            SpotLight("Kicker", env, Hex("9EC6FF"), 26f, new Vector3(2.2f, 1.4f, 3.2f), new Vector3(0.2f, 0.8f, 0f), 45f, size: 0.12f, range: 12f, shadows: true);
            // Top: a faint cool sky so the shadow side keeps its shape.
            var top = DirectionalLight("Top", stage.transform, Hex("C8D6F0"), 0.35f, new Vector3(80f, 20f, 0f), shadows: true, softDegrees: 8f);
            top.shadowStrength = 0.6f;
            Probe(stage, 1f);
            Post(profile);
            MakeCamera(fogColor);
            Ambient(Hex("262B34"), Hex("1C1F24"), Hex("111113"));
            RenderSettings.fogColor = fogColor;
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.ExponentialSquared;
            RenderSettings.sun = top;
            EditorSceneManager.SaveScene(scene, ReplayRenderer.ScenePath("Art"));
        }

        /// <summary>
        /// The art target: a unit-radius bullseye facing +Y (turned to the camera by the replay), a machined rim,
        /// and a 'Shell' ring that the replay expands and fades on a hit.
        /// </summary>
        static GameObject BullseyePrefab(string prefix, Material hit)
        {
            var red = Lit(prefix + "TargetRed", Hex("D93A2B"), smoothness: 0.55f, emission: Hex("D93A2B") * 0.45f, clearCoat: 0.6f, clearCoatSmoothness: 0.9f);
            var white = Lit(prefix + "TargetWhite", Hex("EEE9DF"), smoothness: 0.55f, emission: Hex("EEE9DF") * 0.12f, clearCoat: 0.6f, clearCoatSmoothness: 0.9f);
            var rimMat = Lit(prefix + "TargetRim", Hex("2A2C30"), smoothness: 0.7f, metallic: 0.8f);

            var root = new GameObject("Bullseye");
            var face = MeshObject("Face", root.transform, SaveMesh(BullseyeMesh(5), "Bullseye"), red, castShadows: true);
            face.GetComponent<MeshRenderer>().sharedMaterials = new[] { red, white };
            MeshObject("Rim", root.transform, SaveMesh(TorusMesh(1f, 0.045f), "TargetRim"), rimMat, castShadows: true);
            var shell = MeshObject("Shell", root.transform, SaveMesh(TorusMesh(1f, 0.06f), "TargetShell"), hit, castShadows: false);
            shell.GetComponent<MeshRenderer>().enabled = false;
            string path = "Assets/Showcase/Looks/ArtTarget.prefab";
            var prefab = PrefabUtility.SaveAsPrefabAsset(root, path);
            UnityEngine.Object.DestroyImmediate(root);
            return prefab;
        }

        // ---------------------------------------------------------------- materials

        static Material Lit(string name, Color color, float smoothness, float metallic = 0f,
            float clearCoat = 0f, float clearCoatSmoothness = 0f, Color? emission = null, bool transparent = false)
        {
            var shader = Shader.Find(clearCoat > 0f ? "Universal Render Pipeline/Complex Lit" : "Universal Render Pipeline/Lit");
            var path = $"{Root}/Materials/{name}.mat";
            Directory.CreateDirectory(Path.GetDirectoryName(path));
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
            SetTransparent(mat, transparent);
            mat.enableInstancing = true;
            EditorUtility.SetDirty(mat);
            return mat;
        }

        static Color Hex(string hex, float alpha = 1f)
        {
            var c = ColorUtility.TryParseHtmlString("#" + hex, out var parsed) ? parsed : Color.magenta;
            c.a = alpha;
            return c;
        }

        /// <summary>URP Lit surface type, set the way the material inspector sets it.</summary>
        static void SetTransparent(Material mat, bool on)
        {
            mat.SetFloat("_Surface", on ? 1f : 0f);
            mat.SetFloat("_Blend", 0f); // alpha
            mat.SetFloat("_BlendModePreserveSpecular", 1f);
            mat.SetFloat("_SrcBlend", on ? (float)BlendMode.One : (float)BlendMode.One); // premultiplied-style with preserved specular
            mat.SetFloat("_DstBlend", on ? (float)BlendMode.OneMinusSrcAlpha : (float)BlendMode.Zero);
            mat.SetFloat("_SrcBlendAlpha", 1f);
            mat.SetFloat("_DstBlendAlpha", on ? (float)BlendMode.OneMinusSrcAlpha : 0f);
            mat.SetFloat("_ZWrite", on ? 0f : 1f);
            mat.SetOverrideTag("RenderType", on ? "Transparent" : "Opaque");
            mat.renderQueue = on ? (int)RenderQueue.Transparent : -1;
            if (on) { mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT"); mat.EnableKeyword("_ALPHAPREMULTIPLY_ON"); }
            else { mat.DisableKeyword("_SURFACE_TYPE_TRANSPARENT"); mat.DisableKeyword("_ALPHAPREMULTIPLY_ON"); }
            mat.SetShaderPassEnabled("DepthOnly", !on);
            mat.SetShaderPassEnabled("ShadowCaster", !on);
        }

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
            // Without post-process data URP silently skips every volume effect: tone mapping, bloom, DoF.
            renderer.postProcessData = AssetDatabase.LoadAssetAtPath<PostProcessData>(
                $"{UniversalRenderPipelineAsset.packagePath}/Runtime/Data/PostProcessData.asset")
                ?? throw new Exception("URP's PostProcessData.asset was not found");
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
            Set(so, "m_RequireDepthTexture", true);
            Set(so, "m_RequireOpaqueTexture", false);
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

        static VolumeProfile BuildVolumeProfile(string name, float exposure, float contrast, float saturation,
            float bloomThreshold, float bloomIntensity, float vignette, Vector4 shadowsTint, Vector4 highlightsTint)
        {
            string path = $"{Root}/Pipeline/{name}Volume.asset";
            AssetDatabase.DeleteAsset(path);
            var profile = ScriptableObject.CreateInstance<VolumeProfile>();
            AssetDatabase.CreateAsset(profile, path);

            Add<Tonemapping>(profile).mode.Override(TonemappingMode.ACES);

            var bloom = Add<Bloom>(profile);
            bloom.threshold.Override(bloomThreshold);
            bloom.intensity.Override(bloomIntensity);
            bloom.scatter.Override(0.72f);
            bloom.highQualityFiltering.Override(true);

            var grade = Add<ColorAdjustments>(profile);
            grade.postExposure.Override(exposure);
            grade.contrast.Override(contrast);
            grade.saturation.Override(saturation);

            Add<WhiteBalance>(profile).temperature.Override(4f);

            // Shadow/highlight split: cool shadows, warm highlights.
            var smh = Add<ShadowsMidtonesHighlights>(profile);
            smh.shadows.Override(shadowsTint);
            smh.highlights.Override(highlightsTint);

            var vig = Add<Vignette>(profile);
            vig.intensity.Override(vignette);
            vig.smoothness.Override(0.5f);

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

        // ---------------------------------------------------------------- scene parts

        static Stage NewStage(float floorY)
        {
            var stage = new GameObject("Stage").AddComponent<Stage>();
            stage.floorY = floorY;
            return stage;
        }

        static void Probe(Stage stage, float intensity)
        {
            var probeGo = new GameObject("Reflections");
            probeGo.transform.SetParent(stage.transform, false);
            stage.probe = probeGo.AddComponent<ReflectionProbe>();
            stage.probe.mode = ReflectionProbeMode.Realtime;
            stage.probe.refreshMode = ReflectionProbeRefreshMode.ViaScripting;
            stage.probe.timeSlicingMode = ReflectionProbeTimeSlicingMode.NoTimeSlicing;
            stage.probe.resolution = 512;
            stage.probe.hdr = true;
            stage.probe.importance = 1;
            stage.probe.intensity = intensity;
            stage.probe.size = Vector3.one * 400f;
        }

        static void Post(VolumeProfile profile)
        {
            var volume = new GameObject("Post").AddComponent<Volume>();
            volume.isGlobal = true;
            volume.sharedProfile = profile;
        }

        static void MakeCamera(Color background)
        {
            var camGo = new GameObject("ShowcaseCamera");
            var cam = camGo.AddComponent<Camera>();
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = background;
            cam.nearClipPlane = 0.05f;
            cam.farClipPlane = 5000f;
            cam.fieldOfView = 30f;
            cam.allowHDR = true;
            var data = camGo.AddComponent<UniversalAdditionalCameraData>();
            data.renderPostProcessing = true;
            data.renderShadows = true;
            data.requiresDepthOption = CameraOverrideOption.On;
            camGo.transform.SetPositionAndRotation(new Vector3(0, 0, -40), Quaternion.identity);
        }

        static void Ambient(Color sky, Color equator, Color ground)
        {
            RenderSettings.skybox = null;
            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = sky;
            RenderSettings.ambientEquatorColor = equator;
            RenderSettings.ambientGroundColor = ground;
            RenderSettings.defaultReflectionMode = DefaultReflectionMode.Custom;
        }

        static Light DirectionalLight(string name, Transform parent, Color color, float intensity, Vector3 euler, bool shadows, float softDegrees = 0f)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.rotation = Quaternion.Euler(euler);
            var light = go.AddComponent<Light>();
            light.type = LightType.Directional;
            light.color = color;
            light.intensity = intensity;
            light.shadows = shadows ? LightShadows.Soft : LightShadows.None;
            light.shadowBias = 0.05f;
            light.shadowNormalBias = 0.3f;
            go.AddComponent<UniversalAdditionalLightData>();
            if (softDegrees > 0f) go.AddComponent<StageLight>().angularRadius = softDegrees;
            return light;
        }

        /// <summary>A spot in arm-length units: intensity is the illuminance at one arm length, so it holds at any scale.</summary>
        static Light SpotLight(string name, Transform env, Color color, float intensity, Vector3 position, Vector3 aim,
            float angle, float size, float range, bool shadows)
        {
            var go = new GameObject(name);
            go.transform.SetParent(env, false);
            go.transform.localPosition = position;
            go.transform.localRotation = Quaternion.LookRotation(aim - position);
            var light = go.AddComponent<Light>();
            light.type = LightType.Spot;
            light.color = color;
            light.spotAngle = angle;
            light.innerSpotAngle = angle * 0.55f;
            light.shadows = shadows ? LightShadows.Soft : LightShadows.None;
            light.shadowBias = 0.05f;
            light.shadowNormalBias = 0.4f;
            go.AddComponent<UniversalAdditionalLightData>();
            var sl = go.AddComponent<StageLight>();
            sl.size = size;
            sl.range = range;
            sl.intensity = intensity;
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

        /// <summary>Flat unit disc facing +Y.</summary>
        static Mesh DiscMesh() => Lathe(new List<(Vector2, Vector2)>
        {
            (new Vector2(0f, 0f), Vector2.up), (new Vector2(0.25f, 0f), Vector2.up),
            (new Vector2(0.5f, 0f), Vector2.up), (new Vector2(1f, 0f), Vector2.up),
        }, 128);

        /// <summary>Torus around +Y with ring radius R and tube radius r.</summary>
        static Mesh TorusMesh(float R, float r)
        {
            var profile = new List<(Vector2, Vector2)>();
            for (int i = 0; i <= 32; i++)
            {
                float a = 2f * Mathf.PI * i / 32f;
                var n = new Vector2(Mathf.Cos(a), Mathf.Sin(a));
                profile.Add((new Vector2(R, 0f) + n * r, n));
            }
            return Lathe(profile, 160);
        }

        /// <summary>
        /// The art plinth, in the pedestal's unit space (radius 0.5, height 1): a wide machined foot,
        /// the column, and a collar under the shoulder.
        /// </summary>
        static Mesh PlinthMesh()
        {
            var p = new List<(Vector2 p, Vector2 n)>();
            void Edge(float r, float y, Vector2 n) => p.Add((new Vector2(r, y), n));
            void Round(Vector2 center, float radius, float a0, float a1, int steps)
            {
                for (int i = 0; i <= steps; i++)
                {
                    float a = Mathf.Lerp(a0, a1, i / (float)steps) * Mathf.Deg2Rad;
                    var n = new Vector2(Mathf.Cos(a), Mathf.Sin(a));
                    p.Add((center + n * radius, n));
                }
            }
            Edge(1.6f, 0f, Vector2.right);
            Round(new Vector2(1.6f - 0.006f, 0.012f - 0.006f), 0.006f, 0f, 90f, 6);     // foot top edge
            Edge(0.5f + 0.012f, 0.012f, Vector2.up);
            Round(new Vector2(0.5f + 0.012f, 0.024f), 0.012f, 270f, 180f, 6);           // fillet into the column
            Edge(0.5f, 0.97f, Vector2.right);
            Edge(0.62f, 0.97f, Vector2.down);
            Round(new Vector2(0.62f - 0.004f, 0.996f), 0.004f, 0f, 90f, 6);             // collar edge
            Edge(0f, 1f, Vector2.up);
            return Lathe(p, 128);
        }

        /// <summary>Unit-radius bullseye facing +Y: equal-width rings alternating between submesh 0 (red, center) and 1 (white).</summary>
        static Mesh BullseyeMesh(int rings)
        {
            const int segments = 160;
            var verts = new List<Vector3>();
            var tris = new[] { new List<int>(), new List<int>() };
            for (int ring = 0; ring < rings; ring++)
            {
                float r0 = ring / (float)rings, r1 = (ring + 1) / (float)rings;
                int start = verts.Count;
                for (int s = 0; s <= segments; s++)
                {
                    float a = 2f * Mathf.PI * s / segments;
                    var d = new Vector3(Mathf.Cos(a), 0f, Mathf.Sin(a));
                    verts.Add(d * r0);
                    verts.Add(d * r1);
                }
                var t = tris[ring % 2];
                for (int s = 0; s < segments; s++)
                {
                    int i0 = start + 2 * s, i1 = i0 + 1, i2 = i0 + 2, i3 = i0 + 3;
                    t.AddRange(new[] { i0, i1, i2, i2, i1, i3 });
                }
            }
            var mesh = new Mesh { indexFormat = IndexFormat.UInt32, subMeshCount = 2 };
            mesh.SetVertices(verts);
            mesh.SetNormals(verts.Select(_ => Vector3.up).ToList());
            mesh.SetUVs(0, verts.Select(v => new Vector2(v.x * 0.5f + 0.5f, v.z * 0.5f + 0.5f)).ToList());
            mesh.SetTriangles(tris[0], 0);
            mesh.SetTriangles(tris[1], 1);
            // Face +Y: flip any triangle whose geometric normal points down.
            for (int sm = 0; sm < 2; sm++)
            {
                var idx = mesh.GetTriangles(sm);
                for (int i = 0; i < idx.Length; i += 3)
                    if (Vector3.Cross(verts[idx[i + 1]] - verts[idx[i]], verts[idx[i + 2]] - verts[idx[i]]).y < 0f)
                        (idx[i + 1], idx[i + 2]) = (idx[i + 2], idx[i + 1]);
                mesh.SetTriangles(idx, sm);
            }
            mesh.RecalculateBounds();
            mesh.RecalculateTangents();
            return mesh;
        }

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
