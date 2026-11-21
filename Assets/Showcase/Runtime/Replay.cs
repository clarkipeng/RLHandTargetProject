using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Showcase
{
    /// <summary>Frames a recorded take: subject positions at any time, interpolated between physics steps.</summary>
    public class TakeSubject : ISubject
    {
        readonly Take take;
        readonly TakeActor arm;
        readonly int basePart, handPart;
        readonly TakeActor[] balls, targets;

        public float ArmLength { get; }
        public float StepSeconds => take.stepSeconds;

        /// <summary>One episode of the take: the ball leaves the hand at release and the episode ends at end.</summary>
        public struct Throw { public int start, release, end; public bool hit; }
        public readonly List<Throw> throws = new List<Throw>();

        /// <param name="contactDistance">Ball-to-target center distance that counts as a hit (0: trust the recorded hits).</param>
        public TakeSubject(Take take, float contactDistance = 0f)
        {
            this.take = take;
            arm = take.actors.FirstOrDefault(a => a.kind == ActorKind.Arm)
                  ?? throw new ArgumentException("take has no arm");
            basePart = 0; // the arm root: the shoulder ball the whole chain hangs from
            handPart = FindPart(arm, "palm", arm.partPaths.Length - 1);
            balls = take.actors.Where(a => a.kind == ActorKind.Ball).ToArray();
            targets = take.actors.Where(a => a.kind == ActorKind.Target).ToArray();
            // Arm reach: the farthest any part gets from the shoulder in the first second.
            float reach = 0f;
            for (int f = 0; f < Mathf.Min(arm.frames.Count, 256); f += 16)
                foreach (var pose in arm.frames[f])
                    reach = Mathf.Max(reach, Vector3.Distance(pose.position, arm.frames[f][basePart].position));
            ArmLength = Mathf.Max(reach, 0.01f);
            FindThrows(contactDistance);
        }

        /// <summary>
        /// Splits the take into throws. A hit ends its episode in the same physics step, before the recorder sees the
        /// ball's hit flag, so hits are found from the geometry: the episode ends with the ball touching its target.
        /// The release is where the ball first pulls away from the palm after it settled in the grasp.
        /// </summary>
        void FindThrows(float contactDistance)
        {
            if (balls.Length == 0 || targets.Length == 0) return;
            var starts = take.episodeStarts.Count > 0 ? take.episodeStarts : new List<int> { 0 };
            var recorded = new HashSet<int>(take.hitSteps);
            for (int i = 0; i < starts.Count; i++)
            {
                int start = starts[i], end = (i + 1 < starts.Count ? starts[i + 1] : take.stepCount) - 1;
                if (end - start < 60) continue;
                Vector3 BallAt(int k) { Latest(balls, k * take.stepSeconds, out var b); return b; }
                float Palm(int k) => Vector3.Distance(BallAt(k), arm.Sample(handPart, k).position);
                // The grasp: the closest the ball gets to the palm; its hold distance is the median just after.
                int grasp = start; float closest = float.MaxValue;
                for (int k = start; k <= end; k++) { float d = Palm(k); if (d < closest) { closest = d; grasp = k; } }
                var hold = new List<float>();
                for (int k = grasp; k <= Mathf.Min(end, grasp + 120); k++) hold.Add(Palm(k));
                hold.Sort();
                float away = 1.35f * hold[hold.Count / 2];
                int release = grasp;
                while (release < end && Palm(release) <= away) release++;
                Target(end * take.stepSeconds, out var target);
                bool hit = contactDistance > 0f
                    ? Vector3.Distance(BallAt(end), target) <= contactDistance
                    : take.hitSteps.Any(h => h >= start && h <= end);
                hit |= recorded.Any(h => h >= start && h <= end);
                throws.Add(new Throw { start = start, release = release, end = end, hit = hit && release < end });
            }
            // Hit anchors are the inferred hits: the last step before the episode resets.
            var hits = throws.Where(th => th.hit).Select(th => th.end).ToList();
            if (hits.Count >= take.hitSteps.Count) { take.hitSteps.Clear(); take.hitSteps.AddRange(hits); }
            Debug.Log($"[showcase] {throws.Count} throws, {hits.Count} hits (the recorder caught {recorded.Count})");
        }

        /// <summary>The throw a shot starting at time t follows: the first hit at or after t, else the throw under way.</summary>
        public Throw? ThrowFor(float t)
        {
            float step = t / take.stepSeconds;
            foreach (var th in throws) if (th.hit && th.end >= step) return th;
            foreach (var th in throws) if (th.end >= step) return th;
            return null;
        }

        static int FindPart(TakeActor a, string leaf, int fallback)
        {
            for (int i = 0; i < a.partPaths.Length; i++)
                if (a.partPaths[i].ToLowerInvariant().EndsWith(leaf)) return i;
            return fallback;
        }

        float Step(float t) => Mathf.Clamp(t / take.stepSeconds, 0, take.stepCount - 1);

        public Vector3 ArmBase(float t) => arm.Sample(basePart, Step(t)).position;
        public Vector3 Hand(float t) => arm.Sample(handPart, Step(t)).position;
        public bool Ball(float t, out Vector3 position) => Latest(balls, t, out position);
        public bool Target(float t, out Vector3 position) => Latest(targets, t, out position);

        bool Latest(TakeActor[] actors, float t, out Vector3 position)
        {
            float step = Step(t);
            for (int i = actors.Length - 1; i >= 0; i--)
                if (actors[i].AliveAt(step)) { position = actors[i].Sample(0, step).position; return true; }
            position = default;
            return false;
        }

        /// <summary>Seconds into the take of a shot's anchor.</summary>
        public float AnchorTime(Shot shot)
        {
            List<int> steps = shot.anchor == "hit" ? take.hitSteps : shot.anchor == "episode" ? take.episodeStarts
                : shot.anchor == "release" ? throws.Where(th => th.hit).Select(th => th.release).ToList() : null;
            if (steps == null) return shot.offset;
            if (steps.Count == 0)
            {
                Debug.LogWarning($"[showcase] shot '{shot.name}' anchors on {shot.anchor} {shot.index} but the take has none; using episode starts");
                steps = take.episodeStarts.Count > 0 ? take.episodeStarts : new List<int> { 0 };
            }
            float anchor = steps[shot.index % steps.Count] * take.stepSeconds;
            return Mathf.Clamp(anchor + shot.offset, 0, Mathf.Max(0, take.Duration - shot.duration));
        }
    }

    /// <summary>
    /// A shot's clock: maps seconds of output to seconds of the take. Plays slow around the followed throw's
    /// release and hit, and with a hold freezes the take on the hit while the impact effect keeps its own clock.
    /// </summary>
    public class ShotClock
    {
        const float Dt = 1f / 2400f;
        readonly float[] scene; // take time at each output tick
        public readonly float start, sceneEnd, duration;
        public readonly float impactAt = -1f; // output seconds of the frozen hit (-1: no hold)
        public readonly TakeSubject.Throw? focus;
        public readonly ISubject subject;   // the take, clamped to the followed throw

        public ShotClock(Shot shot, TakeSubject take)
        {
            start = take.AnchorTime(shot);
            focus = take.ThrowFor(start);
            float release = -1f, hit = -1f, from = 0f, to = float.MaxValue;
            if (focus is TakeSubject.Throw th)
            {
                release = th.release * take.StepSeconds;
                if (th.hit) hit = th.end * take.StepSeconds;
                from = th.start * take.StepSeconds;
                if (th.hit && shot.hold > 0f) to = hit;
            }
            subject = to < float.MaxValue ? new ClampedSubject(take, from, to) : (ISubject)take;

            const float ramp = 0.12f; // seconds of the take to ease into and out of slow motion
            float Bump(float x, float a, float b) =>
                x < a ? Mathf.SmoothStep(0, 1, 1f - (a - x) / ramp) : x > b ? Mathf.SmoothStep(0, 1, 1f - (x - b) / ramp) : 1f;
            float Rate(float x)
            {
                float w = 0f;
                if (release >= 0f && shot.slowRelease > 0f) w = Mathf.Max(w, Bump(x, release - shot.slowRelease, release + shot.slowRelease));
                if (hit >= 0f && shot.slowHit > 0f) w = Mathf.Max(w, Bump(x, hit - shot.slowHit, hit + 1f));
                return Mathf.Lerp(1f, Mathf.Clamp(shot.slow, 0.02f, 1f), w);
            }

            bool holding = hit >= 0f && shot.hold > 0f;
            var ticks = new List<float>();
            float tau = 0f, x = start, limit = holding ? 60f : shot.duration;
            while (tau <= limit + 2f * Dt)
            {
                ticks.Add(x);
                tau += Dt;
                if (holding && impactAt < 0f && x >= hit) { impactAt = tau; limit = tau + shot.hold; }
                if (impactAt >= 0f) continue;
                x += Rate(x) * Dt;
                if (holding && x > hit) x = hit;
            }
            scene = ticks.ToArray();
            duration = limit;
            sceneEnd = scene[scene.Length - 1];
        }

        /// <summary>Take time at output time tau.</summary>
        public float Scene(float tau)
        {
            float i = Mathf.Clamp(tau / Dt, 0, scene.Length - 1);
            int a = Mathf.FloorToInt(i), b = Mathf.Min(a + 1, scene.Length - 1);
            return Mathf.Lerp(scene[a], scene[b], i - a);
        }

        /// <summary>Output seconds since the frozen hit, or -1 before it (or with no hold).</summary>
        public float Impact(float tau) => impactAt >= 0f && tau >= impactAt ? tau - impactAt : -1f;
    }

    /// <summary>
    /// Physics-free stand-ins for every actor of a take, built from the training prefabs' meshes
    /// and dressed with the showcase look. Posed by interpolating the recorded steps.
    /// </summary>
    public class ReplayPuppets : IDisposable
    {
        class Puppet
        {
            public GameObject go;
            public Transform[] parts;
            public Transform dressing;   // turned to face the camera
            public Renderer[] glow;      // what lights up on a hit
            public Renderer[] shells;    // the expanding shockwave on a hit
        }

        readonly Take take;
        readonly ShowcaseLook look;
        readonly Transform root;
        readonly Dictionary<int, Puppet> puppets = new Dictionary<int, Puppet>();
        readonly MaterialPropertyBlock block = new MaterialPropertyBlock();
        static readonly int BaseColor = Shader.PropertyToID("_BaseColor");
        static readonly int EmissionColor = Shader.PropertyToID("_EmissionColor");

        /// <summary>Seconds a hit target lingers, glowing, while its shockwave expands and fades.</summary>
        public float hitLinger = 0.7f;

        public ReplayPuppets(Take take, ShowcaseLook look, Transform parent)
        {
            this.take = take;
            this.look = look;
            root = new GameObject("ReplayPuppets").transform;
            root.SetParent(parent, false);
            DropNestedBalls(take);
            foreach (var a in take.actors) puppets[a.id] = Build(a);
            BuildEffects();
        }

        /// <summary>
        /// Takes recorded before the recorder skipped nested ProjectileScripts hold each ball twice:
        /// once from the prefab root and once from its mesh child. Keep the root's record.
        /// </summary>
        public static void DropNestedBalls(Take take)
        {
            var roots = take.actors.Where(a => a.kind == ActorKind.Ball && a.partPaths.Length == 1 && a.partPaths[0].Length > 0)
                .Select(a => a.birthStep).ToHashSet();
            int removed = take.actors.RemoveAll(a => a.kind == ActorKind.Ball && a.partPaths.Length == 1 && a.partPaths[0].Length == 0 && roots.Contains(a.birthStep));
            if (removed > 0) Debug.Log($"[showcase] dropped {removed} nested duplicate balls from an older take");
        }

        Puppet Build(TakeActor a)
        {
            var prefab = a.kind == ActorKind.Arm ? look.armPrefab : a.kind == ActorKind.Ball ? look.ballPrefab : a.kind == ActorKind.Target ? look.targetPrefab : look.ballPrefab;
            var p = new Puppet { go = new GameObject($"{a.kind}_{a.id}"), parts = new Transform[a.partPaths.Length] };
            p.go.transform.SetParent(root, false);
            var renderers = new Renderer[a.partPaths.Length];
            for (int i = 0; i < p.parts.Length; i++)
            {
                var source = string.IsNullOrEmpty(a.partPaths[i]) ? prefab.transform : prefab.transform.Find(a.partPaths[i]);
                var original = a.kind == ActorKind.Prop ? Resources.GetBuiltinResource<Mesh>("Cube.fbx")
                    : source != null && source.TryGetComponent<MeshFilter>(out var mf) ? mf.sharedMesh : null;
                if (original == null) Debug.LogWarning($"[showcase] no mesh for {a.kind} part '{a.partPaths[i]}' in {prefab.name}");
                var scale = a.frames.Count > 0 ? a.frames[0][i].scale : Vector3.one;
                var mesh = look.smoothMeshes ? SmoothMeshes.For(original, scale) : original;
                var part = new GameObject(a.partPaths[i].Length > 0 ? a.partPaths[i].Replace('/', '.') : prefab.name);
                part.transform.SetParent(p.go.transform, false);
                part.AddComponent<MeshFilter>().sharedMesh = mesh;
                var r = part.AddComponent<MeshRenderer>();
                var mat = look.MaterialFor(a.kind, a.partPaths[i]);
                r.sharedMaterials = Enumerable.Repeat(mat, mesh != null ? mesh.subMeshCount : 1).ToArray();
                p.parts[i] = part.transform;
                renderers[i] = r;
            }
            if (a.kind == ActorKind.Arm && look.joint != null) AddJoints(a, prefab, p.parts);

            if (a.kind == ActorKind.Target && look.targetDressing != null)
            {
                // The dressing stands in for the target's own mesh, at the radius of the collision sphere.
                var part = p.parts[0];
                var mesh = renderers[0].GetComponent<MeshFilter>().sharedMesh;
                var s = Abs(a.frames.Count > 0 ? a.frames[0][0].scale : Vector3.one);
                float radius = mesh != null ? mesh.bounds.extents.x * s.x : 0.5f * s.x;
                renderers[0].enabled = false;
                p.dressing = UnityEngine.Object.Instantiate(look.targetDressing, part, false).transform;
                p.dressing.localScale = new Vector3(radius / s.x, radius / s.y, radius / s.z);
                var all = p.dressing.GetComponentsInChildren<Renderer>(true);
                p.shells = all.Where(r => r.name == "Shell").ToArray();
                p.glow = all.Except(p.shells).ToArray();
                foreach (var sh in p.shells) sh.enabled = false;
            }
            else if (a.kind == ActorKind.Target)
            {
                p.glow = renderers;
                // A second copy of the target that expands and fades on the hit: the shockwave.
                p.shells = renderers.Select(r =>
                {
                    var shell = new GameObject("Shell");
                    shell.transform.SetParent(r.transform, false);
                    shell.AddComponent<MeshFilter>().sharedMesh = r.GetComponent<MeshFilter>().sharedMesh;
                    var sr = shell.AddComponent<MeshRenderer>();
                    sr.sharedMaterial = look.targetHit;
                    sr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                    sr.enabled = false;
                    return (Renderer)sr;
                }).ToArray();
            }
            p.go.SetActive(false);
            return p;
        }

        /// <summary>
        /// Joint spheres over the caps of capsule parts, so the arm reads as a machine, not as pills.
        /// Where two capsules meet (the elbow) there is one joint, carried by the part farther from the shoulder.
        /// </summary>
        void AddJoints(TakeActor a, GameObject prefab, Transform[] parts)
        {
            if (a.frames.Count == 0) return;
            var frame = a.frames[0];
            var shoulder = frame[0].position;
            var ends = new List<(int part, Vector3 local, Vector3 world, float radius, bool proximal)>();
            for (int i = 0; i < parts.Length; i++)
            {
                var source = string.IsNullOrEmpty(a.partPaths[i]) ? prefab.transform : prefab.transform.Find(a.partPaths[i]);
                var mesh = source != null && source.TryGetComponent<MeshFilter>(out var mf) ? mf.sharedMesh : null;
                if (!SmoothMeshes.CapsuleEnds(mesh, frame[i].scale, out var e0, out var e1, out float r)) continue;
                Vector3 World(Vector3 local) => frame[i].position + frame[i].rotation * Vector3.Scale(local, frame[i].scale);
                bool firstNear = Vector3.Distance(World(e0), shoulder) < Vector3.Distance(World(e1), shoulder);
                ends.Add((i, e0, World(e0), r, firstNear));
                ends.Add((i, e1, World(e1), r, !firstNear));
            }
            foreach (var e in ends)
            {
                // A distal end that meets another capsule's proximal end leaves the joint to that capsule.
                if (!e.proximal && ends.Any(o => o.part != e.part && o.proximal && Vector3.Distance(o.world, e.world) < 2.5f * e.radius)) continue;
                var s = Abs(frame[e.part].scale);
                float d = 2f * e.radius * 1.14f;
                var j = new GameObject("Joint");
                j.transform.SetParent(parts[e.part], false);
                j.transform.localPosition = e.local;
                j.transform.localScale = new Vector3(d / s.x, d / s.y, d / s.z);
                j.AddComponent<MeshFilter>().sharedMesh = SmoothMeshes.Sphere;
                j.AddComponent<MeshRenderer>().sharedMaterial = look.joint;
            }
            AddKnuckles(a, prefab, parts);
        }

        /// <summary>
        /// A knuckle at the base of every finger and thumb segment, so the hand reads as jointed and the grasp as
        /// fingers curling, not as boxes changing angle.
        /// </summary>
        void AddKnuckles(TakeActor a, GameObject prefab, Transform[] parts)
        {
            var frame = a.frames[0];
            var paths = a.partPaths.ToList();
            for (int i = 0; i < parts.Length; i++)
            {
                string path = a.partPaths[i];
                string leaf = path.Substring(path.LastIndexOf('/') + 1).ToLowerInvariant();
                if (!leaf.StartsWith("finger") && !leaf.StartsWith("thumb")) continue;
                var source = prefab.transform.Find(path);
                var mesh = source != null && source.TryGetComponent<MeshFilter>(out var mf) ? mf.sharedMesh : null;
                int parent = path.Contains('/') ? paths.IndexOf(path.Substring(0, path.LastIndexOf('/'))) : -1;
                if (mesh == null || parent < 0) continue;
                var b = mesh.bounds;
                var size = Vector3.Scale(b.size, Abs(frame[i].scale));
                int axis = size.x >= size.y && size.x >= size.z ? 0 : size.y >= size.z ? 1 : 2;
                var along = Vector3.zero; along[axis] = b.extents[axis];
                Vector3 World(int k, Vector3 local) => frame[k].position + frame[k].rotation * Vector3.Scale(local, frame[k].scale);
                var parentMesh = prefab.transform.Find(a.partPaths[parent])?.GetComponent<MeshFilter>()?.sharedMesh;
                var parentCenter = World(parent, parentMesh != null ? parentMesh.bounds.center : Vector3.zero);
                var e0 = b.center - along; var e1 = b.center + along;
                var knuckle = Vector3.Distance(World(i, e0), parentCenter) < Vector3.Distance(World(i, e1), parentCenter) ? e0 : e1;
                var sorted = new[] { size.x, size.y, size.z }.OrderBy(v => v).ToArray();
                float d = 1.25f * sorted[1];
                var s = Abs(frame[i].scale);
                var j = new GameObject("Knuckle");
                j.transform.SetParent(parts[i], false);
                j.transform.localPosition = knuckle;
                j.transform.localScale = new Vector3(d / s.x, d / s.y, d / s.z);
                j.AddComponent<MeshFilter>().sharedMesh = SmoothMeshes.Sphere;
                j.AddComponent<MeshRenderer>().sharedMaterial = look.joint;
            }
        }

        /// <summary>Seconds the impact of a frozen hit plays: glow, pop, shockwave and sparks.</summary>
        public float impactSeconds = 1.1f;

        LineRenderer trailLine;
        Transform[] sparks;
        Vector3[] sparkDirs;
        float[] sparkSpeeds;
        float ballRadius = 0.5f, targetRadius = 1f;

        void BuildEffects()
        {
            var ballActor = take.actors.FirstOrDefault(a => a.kind == ActorKind.Ball);
            var targetActor = take.actors.FirstOrDefault(a => a.kind == ActorKind.Target);
            if (ballActor != null) ballRadius = WorldRadius(look.ballPrefab, ballActor);
            if (targetActor != null) targetRadius = WorldRadius(look.targetPrefab, targetActor);

            if (look.trail != null)
            {
                var go = new GameObject("Trail");
                go.transform.SetParent(root, false);
                trailLine = go.AddComponent<LineRenderer>();
                trailLine.sharedMaterial = look.trail;
                trailLine.useWorldSpace = true;
                trailLine.alignment = LineAlignment.View;
                trailLine.numCapVertices = 6;
                trailLine.numCornerVertices = 2;
                trailLine.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                trailLine.receiveShadows = false;
                trailLine.widthCurve = new AnimationCurve(new Keyframe(0f, 0f), new Keyframe(0.7f, 0.55f), new Keyframe(1f, 1f));
                trailLine.widthMultiplier = 1.5f * ballRadius;
                var g = new Gradient();
                g.SetKeys(new[] { new GradientColorKey(Color.white, 0f), new GradientColorKey(Color.white, 1f) },
                          new[] { new GradientAlphaKey(0f, 0f), new GradientAlphaKey(0.35f, 0.5f), new GradientAlphaKey(1f, 1f) });
                trailLine.colorGradient = g;
                trailLine.enabled = false;
            }

            // Sparks: small glowing beads thrown out of the impact point, slowing and shrinking as they fade.
            var rng = new System.Random(7);
            const int count = 28;
            sparks = new Transform[count];
            sparkDirs = new Vector3[count];
            sparkSpeeds = new float[count];
            var sparkMat = look.spark != null ? look.spark : look.targetHit;
            for (int i = 0; i < count; i++)
            {
                var sp = new GameObject("Spark");
                sp.transform.SetParent(root, false);
                sp.AddComponent<MeshFilter>().sharedMesh = SmoothMeshes.Sphere;
                var r = sp.AddComponent<MeshRenderer>();
                r.sharedMaterial = sparkMat;
                r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                sp.SetActive(false);
                sparks[i] = sp.transform;
                float z = 2f * (float)rng.NextDouble() - 1f, phi = 2f * Mathf.PI * (float)rng.NextDouble();
                float rr = Mathf.Sqrt(1f - z * z);
                sparkDirs[i] = new Vector3(rr * Mathf.Cos(phi), z, rr * Mathf.Sin(phi));
                sparkSpeeds[i] = 0.6f + 0.8f * (float)rng.NextDouble();
            }
        }

        public static float WorldRadius(GameObject prefab, TakeActor a)
        {
            var mf = prefab != null ? prefab.GetComponentInChildren<MeshFilter>() : null;
            var s = a.frames.Count > 0 ? a.frames[0][0].scale : Vector3.one;
            return (mf != null && mf.sharedMesh != null ? mf.sharedMesh.bounds.extents.x : 0.5f) * Mathf.Abs(s.x);
        }

        /// <summary>Center distance at which the ball touches the target.</summary>
        public float ContactDistance => ballRadius + targetRadius;

        /// <summary>
        /// Poses every actor at take time t; dressings turn to face the viewer.
        /// impact: seconds since a frozen hit (-1: none). trail: seconds of flight to draw behind the ball of the followed throw.
        /// </summary>
        public void Pose(float t, Vector3? viewer = null, float impact = -1f, float trail = 0f, TakeSubject.Throw? focus = null)
        {
            float step = t / take.stepSeconds;
            Vector3 ballPos = default; bool haveBall = false;
            foreach (var a in take.actors)
            {
                var p = puppets[a.id];
                bool lingering = a.kind == ActorKind.Target && a.hit && step > a.LastStep && step <= a.LastStep + hitLinger / take.stepSeconds;
                bool visible = a.AliveAt(step) || lingering;
                if (p.go.activeSelf != visible) p.go.SetActive(visible);
                if (!visible) continue;
                for (int i = 0; i < p.parts.Length; i++)
                {
                    var pose = a.Sample(i, step);
                    p.parts[i].SetPositionAndRotation(pose.position, pose.rotation);
                    p.parts[i].localScale = pose.scale;
                }
                if (a.kind == ActorKind.Ball && a.AliveAt(step)) { ballPos = p.parts[0].position; haveBall = true; }
                if (p.dressing != null && viewer is Vector3 v)
                {
                    var toViewer = v - p.dressing.position;
                    if (toViewer.sqrMagnitude > 1e-8f) p.dressing.rotation = Quaternion.FromToRotation(Vector3.up, toViewer.normalized);
                }
                if (a.kind != ActorKind.Target) continue;
                if (impact >= 0f) Flash(p, Mathf.Clamp01(impact / impactSeconds), true);
                else if (a.hit) Flash(p, lingering ? (step - a.LastStep) * take.stepSeconds / hitLinger : 0f, lingering);
            }
            PoseSparks(impact, haveBall, ballPos, viewer);
            PoseTrail(step, trail, focus);
        }

        void PoseSparks(float impact, bool haveBall, Vector3 ballPos, Vector3? viewer)
        {
            float k = impact / impactSeconds;
            bool on = impact >= 0f && k < 1f && haveBall;
            Vector3 target = ballPos;
            if (on) foreach (var a in take.actors)
                if (a.kind == ActorKind.Target && puppets[a.id].go.activeSelf) target = puppets[a.id].parts[0].position;
            // Out of the contact point, biased back towards where the ball came from and towards the camera.
            Vector3 contact = on ? target + (ballPos - target).normalized * targetRadius : default;
            Vector3 bias = on ? (ballPos - target).normalized : default;
            if (on && viewer is Vector3 v) bias = (bias + 0.5f * (v - contact).normalized).normalized;
            for (int i = 0; i < sparks.Length; i++)
            {
                var sp = sparks[i];
                if (sp.gameObject.activeSelf != on) sp.gameObject.SetActive(on);
                if (!on) continue;
                var dir = (sparkDirs[i] + 0.9f * bias).normalized;
                float travel = sparkSpeeds[i] * 1.6f * targetRadius * (1f - Mathf.Exp(-5f * k));
                sp.position = contact + dir * travel + Vector3.down * 0.35f * targetRadius * k * k;
                sp.localScale = Vector3.one * (0.42f * ballRadius * sparkSpeeds[i] * Mathf.Pow(1f - k, 1.5f));
            }
        }

        void PoseTrail(float step, float seconds, TakeSubject.Throw? focus)
        {
            if (trailLine == null) return;
            var ball = take.actors.FirstOrDefault(a => a.kind == ActorKind.Ball && a.AliveAt(step));
            bool on = seconds > 0f && ball != null && focus is TakeSubject.Throw th && step > th.release && step <= th.end + 0.5f;
            trailLine.enabled = on;
            if (!on) return;
            var f = focus.Value;
            float head = Mathf.Min(step, f.end), tail = Mathf.Max(f.release, head - seconds / take.stepSeconds);
            const int n = 64;
            trailLine.positionCount = n;
            for (int i = 0; i < n; i++)
                trailLine.SetPosition(i, ball.Sample(0, Mathf.Lerp(tail, head, i / (n - 1f))).position);
        }

        /// <summary>
        /// The hit: the target glows from within and pops, and a shell expands out of it and fades,
        /// so the moment reads in a still and in motion without blowing out to white. k runs 0 to 1 across it.
        /// </summary>
        void Flash(Puppet p, float k, bool active)
        {
            float glow = active ? Mathf.Exp(-4f * k) : 0f;
            var hitGlow = look.targetHit.GetColor(EmissionColor);
            foreach (var r in p.glow)
            {
                var mats = r.sharedMaterials;
                for (int m = 0; m < mats.Length; m++)
                {
                    if (!active) { r.SetPropertyBlock(null, m); continue; }
                    block.Clear();
                    var e = mats[m].IsKeywordEnabled("_EMISSION") ? mats[m].GetColor(EmissionColor) : Color.black;
                    block.SetColor(EmissionColor, Color.Lerp(e, hitGlow, glow));
                    r.SetPropertyBlock(block, m);
                }
            }
            if (active)
                foreach (var part in p.parts) part.localScale *= 1f + 0.12f * Mathf.Sin(Mathf.PI * Mathf.Min(1f, 2.5f * k));

            float grow = 1f - Mathf.Pow(1f - k, 3f); // fast out, settling
            float fade = Mathf.Pow(1f - k, 2f);
            foreach (var shell in p.shells)
            {
                shell.enabled = active && k < 1f;
                if (!shell.enabled) continue;
                shell.transform.localScale = Vector3.one * (1f + 1.6f * grow);
                var c = look.targetHit.GetColor(BaseColor); c.a *= fade * 0.6f;
                block.Clear();
                block.SetColor(BaseColor, c);
                block.SetColor(EmissionColor, hitGlow * (0.6f * fade));
                shell.SetPropertyBlock(block);
            }
        }

        static Vector3 Abs(Vector3 v) => new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));

        public void Dispose()
        {
            if (root != null) UnityEngine.Object.DestroyImmediate(root.gameObject);
        }
    }
}
