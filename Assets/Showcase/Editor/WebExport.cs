using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace Showcase.Editor
{
    /// <summary>
    /// Exports a take for the browser live view (tools/showcase/web): every part's shape, as the smooth
    /// stand-in the offline renders use, and its pose at 60 Hz, in a right-handed frame (z flipped) for three.js.
    ///
    /// Unity -batchmode -projectPath . -executeMethod Showcase.Editor.WebExport.ExportFromCommandLine -take run.take -out run.json
    /// </summary>
    public static class WebExport
    {
        const float Hz = 60f;

        public static void ExportFromCommandLine()
        {
            try
            {
                var args = Environment.GetCommandLineArgs();
                string Arg(string n) { int i = Array.IndexOf(args, n); return i >= 0 && i + 1 < args.Length ? args[i + 1] : throw new ArgumentException($"missing {n}"); }
                Export(Take.Load(Arg("-take")), Arg("-out"));
                EditorApplication.Exit(0);
            }
            catch (Exception e) { Debug.LogException(e); EditorApplication.Exit(1); }
        }

        public static void Export(Take take, string path)
        {
            ReplayPuppets.DropNestedBalls(take);
            var look = AssetDatabase.LoadAssetAtPath<ShowcaseLook>(ReplayRenderer.LookPath("Minimal"));
            int frames = Mathf.FloorToInt(take.Duration * Hz);
            var sb = new StringBuilder();
            sb.Append("{\"format\":1");
            sb.Append($",\"policy\":{Str(take.policy)},\"hz\":{F(Hz)},\"frames\":{frames},\"duration\":{F(take.Duration)}");
            sb.Append($",\"hits\":[{string.Join(",", take.hitSteps.Select(s => F(s * take.stepSeconds)))}]");
            sb.Append($",\"episodes\":[{string.Join(",", take.episodeStarts.Select(s => F(s * take.stepSeconds)))}]");
            sb.Append($",\"throws\":{take.actors.Count(a => a.kind == ActorKind.Ball)}");
            sb.Append($",\"floorY\":{F(UnityEngine.Object.FindObjectOfType<Stage>()?.floorY ?? FloorY())}");
            sb.Append(",\"actors\":[");
            bool firstActor = true;
            foreach (var a in take.actors)
            {
                var prefab = a.kind == ActorKind.Arm ? look.armPrefab : a.kind == ActorKind.Ball ? look.ballPrefab : look.targetPrefab;
                int first = Mathf.CeilToInt(a.birthStep * take.stepSeconds * Hz);
                int last = Mathf.FloorToInt(a.LastStep * take.stepSeconds * Hz);
                if (last < first) continue;
                if (!firstActor) sb.Append(',');
                firstActor = false;
                sb.Append($"{{\"id\":{a.id},\"kind\":\"{a.kind.ToString().ToLowerInvariant()}\",\"hit\":{(a.hit ? "true" : "false")},\"first\":{first},\"parts\":[");
                for (int p = 0; p < a.partPaths.Length; p++)
                {
                    if (p > 0) sb.Append(',');
                    var source = string.IsNullOrEmpty(a.partPaths[p]) ? prefab.transform : prefab.transform.Find(a.partPaths[p]);
                    var mesh = source != null && source.TryGetComponent<MeshFilter>(out var mf) ? mf.sharedMesh : null;
                    sb.Append(Part(a, p, mesh, look));
                }
                // Poses: per frame, per part: position xyz, rotation xyzw (right-handed), as base64 float32.
                var data = new float[(last - first + 1) * a.partPaths.Length * 7];
                int k = 0;
                for (int f = first; f <= last; f++)
                {
                    float step = f / Hz / take.stepSeconds;
                    for (int p = 0; p < a.partPaths.Length; p++)
                    {
                        var pose = a.Sample(p, step);
                        data[k++] = pose.position.x; data[k++] = pose.position.y; data[k++] = -pose.position.z;
                        data[k++] = -pose.rotation.x; data[k++] = -pose.rotation.y; data[k++] = pose.rotation.z; data[k++] = pose.rotation.w;
                    }
                }
                var bytes = new byte[data.Length * 4];
                Buffer.BlockCopy(data, 0, bytes, 0, bytes.Length);
                sb.Append($"],\"count\":{last - first + 1},\"poses\":\"{Convert.ToBase64String(bytes)}\"}}");
            }
            sb.Append("]}");
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path)));
            File.WriteAllText(path, sb.ToString());
            Debug.Log($"[showcase] web export {path}: {frames} frames, {take.actors.Count} actors, {new FileInfo(path).Length / 1024} KB");
        }

        /// <summary>One part: its smooth shape in world units, offset from the part's pivot, and its look slot.</summary>
        static string Part(TakeActor a, int p, Mesh mesh, ShowcaseLook look)
        {
            var scale = a.frames.Count > 0 ? a.frames[0][p].scale : Vector3.one;
            var s = new Vector3(Mathf.Abs(scale.x), Mathf.Abs(scale.y), Mathf.Abs(scale.z));
            var (shape, axis) = mesh != null ? SmoothMeshes.Classify(mesh) : (SmoothMeshes.Shape.Other, 0);
            var b = mesh != null ? mesh.bounds : new Bounds(Vector3.zero, Vector3.one);
            var center = Vector3.Scale(b.center, scale);
            var ext = Vector3.Scale(b.extents, s);
            string slot = a.kind == ActorKind.Arm ? (look.MaterialFor(a.kind, a.partPaths[p]) == look.handShell ? "hand" : "arm") : a.kind.ToString().ToLowerInvariant();
            string kind = shape == SmoothMeshes.Shape.Other ? "box" : shape.ToString().ToLowerInvariant();
            return $"{{\"name\":{Str(a.partPaths[p])},\"shape\":\"{kind}\",\"axis\":{axis},\"slot\":\"{slot}\"," +
                   $"\"center\":[{F(center.x)},{F(center.y)},{F(-center.z)}],\"extents\":[{F(ext.x)},{F(ext.y)},{F(ext.z)}]}}";
        }

        static float FloorY()
        {
            var scene = UnityEditor.SceneManagement.EditorSceneManager.OpenScene(ReplayRenderer.ScenePath("Minimal"));
            return UnityEngine.Object.FindObjectOfType<Stage>().floorY;
        }

        static string F(float v) => v.ToString("0.#####", System.Globalization.CultureInfo.InvariantCulture);
        static string Str(string s) => "\"" + s.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
    }
}
