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

        public TakeSubject(Take take)
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
            List<int> steps = shot.anchor == "hit" ? take.hitSteps : shot.anchor == "episode" ? take.episodeStarts : null;
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
            var prefab = a.kind == ActorKind.Arm ? look.armPrefab : a.kind == ActorKind.Ball ? look.ballPrefab : look.targetPrefab;
            var p = new Puppet { go = new GameObject($"{a.kind}_{a.id}"), parts = new Transform[a.partPaths.Length] };
            p.go.transform.SetParent(root, false);
            var renderers = new Renderer[a.partPaths.Length];
            for (int i = 0; i < p.parts.Length; i++)
            {
                var source = string.IsNullOrEmpty(a.partPaths[i]) ? prefab.transform : prefab.transform.Find(a.partPaths[i]);
                var original = source != null && source.TryGetComponent<MeshFilter>(out var mf) ? mf.sharedMesh : null;
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
        }

        /// <summary>Poses every actor at take time t; dressings turn to face the viewer.</summary>
        public void Pose(float t, Vector3? viewer = null)
        {
            float step = t / take.stepSeconds;
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
                if (p.dressing != null && viewer is Vector3 v)
                {
                    var toViewer = v - p.dressing.position;
                    if (toViewer.sqrMagnitude > 1e-8f) p.dressing.rotation = Quaternion.FromToRotation(Vector3.up, toViewer.normalized);
                }
                if (a.kind == ActorKind.Target && a.hit) Flash(a, p, step, lingering);
            }
        }

        /// <summary>
        /// The hit: the target glows from within and pops, and a shell expands out of it and fades,
        /// so the moment reads in a still and in motion without blowing out to white.
        /// </summary>
        void Flash(TakeActor a, Puppet p, float step, bool lingering)
        {
            float k = lingering ? (step - a.LastStep) * take.stepSeconds / hitLinger : 0f;
            float glow = lingering ? Mathf.Exp(-4f * k) : 0f;
            var hitGlow = look.targetHit.GetColor(EmissionColor);
            foreach (var r in p.glow)
            {
                var mats = r.sharedMaterials;
                for (int m = 0; m < mats.Length; m++)
                {
                    if (!lingering) { r.SetPropertyBlock(null, m); continue; }
                    block.Clear();
                    var e = mats[m].IsKeywordEnabled("_EMISSION") ? mats[m].GetColor(EmissionColor) : Color.black;
                    block.SetColor(EmissionColor, Color.Lerp(e, hitGlow, glow));
                    r.SetPropertyBlock(block, m);
                }
            }
            if (lingering)
                foreach (var part in p.parts) part.localScale *= 1f + 0.12f * Mathf.Sin(Mathf.PI * Mathf.Min(1f, 2.5f * k));

            float grow = 1f - Mathf.Pow(1f - k, 3f); // fast out, settling
            float fade = Mathf.Pow(1f - k, 2f);
            foreach (var shell in p.shells)
            {
                shell.enabled = lingering;
                if (!lingering) continue;
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
