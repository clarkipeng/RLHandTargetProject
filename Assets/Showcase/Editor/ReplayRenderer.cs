using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using Debug = UnityEngine.Debug;

namespace Showcase.Editor
{
    /// <summary>
    /// Renders a recorded take offline, in edit mode, with no physics and no policy in the loop.
    /// Each output frame accumulates sub-frames spread across the shutter interval, so moving objects
    /// and cameras get true motion blur and every edge gets supersampled antialiasing; DoF, bloom and
    /// tone mapping come from the URP volume of the showcase scene.
    ///
    /// Unity -batchmode -projectPath . -executeMethod Showcase.Editor.ReplayRenderer.RenderFromCommandLine
    ///   -take run.take -look minimal|art -reel Assets/Showcase/Reels/website.json -out website.mp4
    ///   [-stills dir] [-scale 0.25] [-subframes 4] [-maxFrames 120]
    /// </summary>
    public static class ReplayRenderer
    {
        /// <summary>The looks ShowcaseBuilder generates; each is a scene plus a ShowcaseLook asset.</summary>
        public static readonly string[] Looks = { "Minimal", "Art" };
        public static string ScenePath(string look) => $"Assets/Showcase/Looks/{look}.unity";
        public static string LookPath(string look) => $"Assets/Showcase/Looks/{look}Look.asset";

        /// <summary>-look minimal|art, as the canonical look name.</summary>
        static string LookArg() =>
            Looks.FirstOrDefault(l => string.Equals(l, Arg("-look", "Minimal"), StringComparison.OrdinalIgnoreCase))
            ?? throw new ArgumentException($"unknown -look {Arg("-look", "")}; expected one of {string.Join(", ", Looks)}");

        public const string PipelinePath = "Assets/Showcase/Pipeline/ShowcaseURP.asset";

        /// <summary>Opens the studio, switches to the offline pipeline and fits the stage to the take.</summary>
        static (ShowcaseLook, Camera, DepthOfField, TakeSubject, Stage) Setup(Take take, string lookName)
        {
            EditorSceneManager.OpenScene(ScenePath(lookName), OpenSceneMode.Single);
            // In-memory only for this batch run; the project's quality settings are never saved by it.
            QualitySettings.renderPipeline = AssetDatabase.LoadAssetAtPath<RenderPipelineAsset>(PipelinePath);
            var look = AssetDatabase.LoadAssetAtPath<ShowcaseLook>(LookPath(lookName));
            var cam = GameObject.Find("ShowcaseCamera").GetComponent<Camera>();
            var volume = UnityEngine.Object.FindObjectsOfType<Volume>().First(v => v.isGlobal);
            volume.profile = UnityEngine.Object.Instantiate(volume.sharedProfile); // never dirty the asset
            volume.profile.TryGet(out DepthOfField dof);
            ReplayPuppets.DropNestedBalls(take);
            // Hits end their episode before the recorder sees them; find them where the ball touches its target.
            var ballActor = take.actors.FirstOrDefault(a => a.kind == ActorKind.Ball);
            var targetActor = take.actors.FirstOrDefault(a => a.kind == ActorKind.Target);
            float contact = ballActor != null && targetActor != null
                ? 1.06f * (ReplayPuppets.WorldRadius(look.ballPrefab, ballActor) + ReplayPuppets.WorldRadius(look.targetPrefab, targetActor)) : 0f;
            var subject = new TakeSubject(take, contact);

            // -stageTake fits the stage to another take, so renders of different takes share one set.
            string stagePath = Arg("-stageTake", "");
            var stageTake = stagePath != "" ? Take.Load(stagePath) : take;
            var stageSubject = stageTake == take ? subject : new TakeSubject(stageTake, contact);
            var bounds = new Bounds(stageSubject.ArmBase(0), Vector3.zero);
            foreach (var a in stageTake.actors)
                for (int f = 0; f < a.frames.Count; f += 8)
                    bounds.Encapsulate(a.frames[f][0].position);
            var stage = UnityEngine.Object.FindObjectOfType<Stage>();
            stage.Fit(stageSubject.ArmBase(0), bounds, stageSubject.ArmLength);
            Debug.Log($"[showcase] look {lookName}; take: {take.Duration:F1}s, {take.hitSteps.Count} hits, {take.episodeStarts.Count} episodes, arm length {subject.ArmLength:F2}, action bounds {bounds}");
            return (look, cam, dof, subject, stage);
        }

        public static void RenderFromCommandLine()
        {
            try
            {
                var reel = JsonUtility.FromJson<Reel>(File.ReadAllText(Arg("-reel", "Assets/Showcase/Reels/website.json")));
                if (float.TryParse(Arg("-scale", "1"), out float scale) && scale != 1f)
                {
                    reel.width = Mathf.RoundToInt(reel.width * scale / 2) * 2;
                    reel.height = Mathf.RoundToInt(reel.height * scale / 2) * 2;
                }
                if (int.TryParse(Arg("-subframes", ""), out int sub)) reel.subframes = sub;
                int.TryParse(Arg("-maxFrames", "0"), out int maxFrames);
                Render(Take.Load(Arg("-take", null)), LookArg(), reel, Arg("-out", null), Arg("-stills", null), maxFrames);
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                EditorApplication.Exit(1);
            }
        }

        public static void Render(Take take, string lookName, Reel reel, string videoPath, string stillsDir, int maxFrames = 0)
        {
            var (look, cam, dof, subject, stage) = Setup(take, lookName);
            using var puppets = new ReplayPuppets(take, look, null);
            using var frame = new FrameAccumulator(reel.width, reel.height, cam);

            Process ffmpeg = videoPath != null ? StartEncoder(videoPath, reel) : null;
            var stream = ffmpeg?.StandardInput.BaseStream;
            if (stillsDir != null) Directory.CreateDirectory(stillsDir);

            int written = 0;
            var wall = Stopwatch.StartNew();
            foreach (var shot in reel.shots)
            {
                var clock = new ShotClock(shot, subject);
                float start = clock.start, end = clock.sceneEnd;
                int frames = Mathf.RoundToInt(clock.duration * reel.fps);
                Debug.Log($"[showcase] shot '{shot.name}' ({shot.Type}) {start:F2}s..{end:F2}s of the take, {frames} frames" +
                          (clock.impactAt >= 0f ? $", hit frozen at {clock.impactAt:F2}s" : ""));
                var still = ShotEvaluator.Evaluate(shot, clock.subject, start, 0f, start, end, reel.width / (float)reel.height);
                Debug.Log($"[showcase] shot '{shot.name}' camera eye {still.position:F3} look {still.position + still.rotation * Vector3.forward * still.focusDistance:F3}");
                for (int f = 0; f < frames; f++)
                {
                    float tau = f / (float)reel.fps;
                    float u = f / (float)Mathf.Max(1, frames - 1);
                    frame.Begin();
                    for (int s = 0; s < reel.subframes; s++)
                    {
                        // Sub-frame times spread evenly over the open shutter, centered on the frame time.
                        float taus = tau + ((s + 0.5f) / reel.subframes - 0.5f) * reel.shutter / reel.fps;
                        float ts = clock.Scene(taus);
                        var pose = ShotEvaluator.Evaluate(shot, clock.subject, ts, u, start, end, reel.width / (float)reel.height);
                        Apply(cam, dof, pose, subject.ArmLength);
                        puppets.Pose(ts, pose.position, clock.Impact(taus), shot.trail, clock.focus);
                        stage.Jitter(s, reel.subframes);
                        frame.AddSubframe(s);
                    }
                    var pixels = frame.Resolve();
                    stream?.Write(pixels, 0, pixels.Length);
                    if (stillsDir != null && f == StillFrame(clock, frames, reel.fps))
                        File.WriteAllBytes(Path.Combine(stillsDir, $"{written:D2}_{shot.name}.png"), frame.EncodePng());
                    if (maxFrames > 0 && f + 1 >= maxFrames) break;
                }
                written++;
                Debug.Log($"[showcase] shot '{shot.name}' done, {wall.Elapsed.TotalSeconds:F0}s elapsed");
            }

            if (ffmpeg != null)
            {
                stream.Close();
                ffmpeg.WaitForExit();
                if (ffmpeg.ExitCode != 0) throw new Exception($"ffmpeg exited with {ffmpeg.ExitCode}");
                Debug.Log($"[showcase] wrote {videoPath}");
            }
        }

        /// <summary>The still of a shot: a quarter second into its impact when it holds on a hit, else mid-shot.</summary>
        static int StillFrame(ShotClock clock, int frames, int fps) =>
            clock.impactAt >= 0f ? Mathf.Min(frames - 1, Mathf.RoundToInt((clock.impactAt + 0.25f) * fps)) : frames / 2;

        /// <summary>
        /// Renders one hero still: a single instant (no motion blur) with many sub-frames of
        /// antialiasing, at any resolution.
        /// </summary>
        public static void RenderStill(Take take, string lookName, Shot shot, float u, int width, int height, int subframes, string pngPath)
        {
            var (look, cam, dof, subject, stage) = Setup(take, lookName);
            using var puppets = new ReplayPuppets(take, look, null);
            using var frame = new FrameAccumulator(width, height, cam);
            var clock = new ShotClock(shot, subject);
            float tau = u * clock.duration, t = clock.Scene(tau);
            var pose = ShotEvaluator.Evaluate(shot, clock.subject, t, u, clock.start, clock.sceneEnd, width / (float)height);
            Apply(cam, dof, pose, subject.ArmLength);
            puppets.Pose(t, pose.position, clock.Impact(tau), shot.trail, clock.focus);
            if (float.TryParse(Arg("-debugAperture", ""), out float ap)) dof.aperture.Override(ap);
            frame.Begin();
            for (int s = 0; s < subframes; s++) { stage.Jitter(s, subframes); frame.AddSubframe(s); }
            var live = VolumeManager.instance.stack.GetComponent<DepthOfField>();
            Debug.Log($"[showcase] still: lens focus {dof?.focusDistance.value:F2} m, {dof?.focalLength.value:F0} mm, f/{dof?.aperture.value:F1}; " +
                      $"stack after render: mode {live.mode.value}, focus {live.focusDistance.value:F2}, f/{live.aperture.value:F1}");
            frame.Resolve();
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(pngPath)));
            File.WriteAllBytes(pngPath, frame.EncodePng());
            Debug.Log($"[showcase] wrote {pngPath}");
        }

        public static void RenderStillFromCommandLine()
        {
            try
            {
                var reel = JsonUtility.FromJson<Reel>(File.ReadAllText(Arg("-reel", "Assets/Showcase/Reels/website.json")));
                string name = Arg("-shot", reel.shots[0].name);
                var shot = reel.shots.FirstOrDefault(s => s.name == name)
                           ?? throw new ArgumentException($"no shot '{name}' in the reel; it has {string.Join(", ", reel.shots.Select(s => s.name))}");
                RenderStill(Take.Load(Arg("-take", null)), LookArg(), shot, float.Parse(Arg("-u", "0.5")),
                    int.Parse(Arg("-width", "5120")), int.Parse(Arg("-height", "2880")),
                    int.Parse(Arg("-subframes", "64")), Arg("-out", null));
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                EditorApplication.Exit(1);
            }
        }

        static void Apply(Camera cam, DepthOfField dof, CameraPose pose, float armLength)
        {
            cam.transform.SetPositionAndRotation(pose.position, pose.rotation);
            cam.fieldOfView = pose.fov;
            if (dof == null) return;
            dof.mode.Override(DepthOfFieldMode.Bokeh);
            dof.focusDistance.Override(ShotEvaluator.LensDistance(pose.focusDistance, armLength));
            dof.focalLength.Override(ShotEvaluator.FocalLengthMm(pose.fov));
            dof.aperture.Override(pose.aperture);
        }

        static Process StartEncoder(string path, Reel reel)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path)));
            string ffmpeg = new[] { "/opt/homebrew/bin/ffmpeg", "/usr/local/bin/ffmpeg", "/usr/bin/ffmpeg" }.FirstOrDefault(File.Exists) ?? "ffmpeg";
            // Readback rows are bottom-up, hence vflip. H.264 High at a low CRF plays everywhere a portfolio page is viewed.
            string args = $"-y -loglevel error -threads 4 -f rawvideo -pix_fmt rgba -s {reel.width}x{reel.height} -r {reel.fps} -i - " +
                          $"-vf vflip -c:v libx264 -preset slow -crf 14 -pix_fmt yuv420p -profile:v high -movflags +faststart \"{path}\"";
            // On the Windows PC, Unity encodes through the Linux ffmpeg in WSL (tools/win/unity.sh passes -wslFfmpeg).
            string wslFfmpeg = Arg("-wslFfmpeg", "");
            if (Application.platform == RuntimePlatform.WindowsEditor && wslFfmpeg != "")
            {
                string full = Path.GetFullPath(path);
                string linuxPath = full.StartsWith(@"\\")   // \\wsl.localhost\<distro>\home\... -> /home/...
                    ? "/" + string.Join("/", full.Split(new[] { '\\' }, StringSplitOptions.RemoveEmptyEntries).Skip(2))
                    : "/mnt/" + char.ToLowerInvariant(full[0]) + full.Substring(2).Replace('\\', '/');
                args = "-e " + wslFfmpeg + " " + args.Replace($"\"{path}\"", $"\"{linuxPath}\"");
                ffmpeg = "wsl.exe";
            }
            var p = Process.Start(new ProcessStartInfo(ffmpeg, args) { UseShellExecute = false, RedirectStandardInput = true });
            return p ?? throw new Exception("could not start ffmpeg");
        }

        static string Arg(string name, string fallback)
        {
            var args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, name);
            if (i >= 0 && i + 1 < args.Length) return args[i + 1];
            if (fallback == null) throw new ArgumentException($"missing {name}");
            return fallback;
        }
    }

    /// <summary>Renders a camera several times into a float buffer and resolves the average to 8-bit.</summary>
    class FrameAccumulator : IDisposable
    {
        readonly Camera cam;
        readonly RenderTexture sub, accum, output;
        readonly Material mat;
        readonly Texture2D readback;
        readonly int width, height;

        public FrameAccumulator(int width, int height, Camera cam)
        {
            this.width = width;
            this.height = height;
            this.cam = cam;
            sub = new RenderTexture(width, height, 24, RenderTextureFormat.ARGBHalf, RenderTextureReadWrite.Linear) { antiAliasing = 1 };
            accum = new RenderTexture(width, height, 0, RenderTextureFormat.ARGBFloat, RenderTextureReadWrite.Linear);
            output = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);
            mat = new Material(Shader.Find("Hidden/Showcase/Accumulate"));
            readback = new Texture2D(width, height, TextureFormat.RGBA32, false, true);
            cam.targetTexture = sub;
            var data = cam.GetUniversalAdditionalCameraData();
            data.renderPostProcessing = true;
            data.antialiasing = AntialiasingMode.None; // accumulation with jitter is the antialiasing
            data.dithering = false;                    // the resolve dithers once, at the end
            data.stopNaN = true;
        }

        public void Begin()
        {
            var prev = RenderTexture.active;
            RenderTexture.active = accum;
            GL.Clear(false, true, Color.clear);
            RenderTexture.active = prev;
        }

        public void AddSubframe(int index)
        {
            // Sub-pixel jitter on a Halton(2,3) sequence: antialiasing for free.
            cam.ResetProjectionMatrix();
            var p = cam.projectionMatrix;
            p.m02 += (Halton(index + 1, 2) - 0.5f) * 2f / width;
            p.m12 += (Halton(index + 1, 3) - 0.5f) * 2f / height;
            cam.projectionMatrix = p;
            cam.Render();
            mat.SetFloat("_Weight", 1f);
            Graphics.Blit(sub, accum, mat, 0);
        }

        public byte[] Resolve()
        {
            mat.SetFloat("_Seed", UnityEngine.Random.value);
            Graphics.Blit(accum, output, mat, 1);
            var prev = RenderTexture.active;
            RenderTexture.active = output;
            readback.ReadPixels(new Rect(0, 0, width, height), 0, 0, false);
            readback.Apply(false);
            RenderTexture.active = prev;
            return readback.GetRawTextureData();
        }

        public byte[] EncodePng() => readback.EncodeToPNG();

        static float Halton(int i, int b)
        {
            float f = 1f, r = 0f;
            for (; i > 0; i /= b) { f /= b; r += f * (i % b); }
            return r;
        }

        public void Dispose()
        {
            cam.targetTexture = null;
            cam.ResetProjectionMatrix();
            UnityEngine.Object.DestroyImmediate(sub);
            UnityEngine.Object.DestroyImmediate(accum);
            UnityEngine.Object.DestroyImmediate(output);
            UnityEngine.Object.DestroyImmediate(mat);
            UnityEngine.Object.DestroyImmediate(readback);
        }
    }
}
