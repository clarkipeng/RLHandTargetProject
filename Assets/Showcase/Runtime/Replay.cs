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
        readonly Take take;
        readonly ShowcaseLook look;
        readonly Transform root;
        readonly Dictionary<int, (GameObject go, Transform[] parts, Renderer[] renderers)> puppets =
            new Dictionary<int, (GameObject, Transform[], Renderer[])>();

        /// <summary>Seconds a hit target lingers, flashing, before it disappears.</summary>
        public float hitLinger = 0.35f;

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

        (GameObject, Transform[], Renderer[]) Build(TakeActor a)
        {
            var prefab = a.kind == ActorKind.Arm ? look.armPrefab : a.kind == ActorKind.Ball ? look.ballPrefab : look.targetPrefab;
            var go = new GameObject($"{a.kind}_{a.id}");
            go.transform.SetParent(root, false);
            var parts = new Transform[a.partPaths.Length];
            var renderers = new Renderer[a.partPaths.Length];
            for (int i = 0; i < parts.Length; i++)
            {
                var source = string.IsNullOrEmpty(a.partPaths[i]) ? prefab.transform : prefab.transform.Find(a.partPaths[i]);
                var mesh = source != null && source.TryGetComponent<MeshFilter>(out var mf) ? mf.sharedMesh : null;
                if (mesh == null) Debug.LogWarning($"[showcase] no mesh for {a.kind} part '{a.partPaths[i]}' in {prefab.name}");
                var part = new GameObject(a.partPaths[i].Length > 0 ? a.partPaths[i].Replace('/', '.') : prefab.name);
                part.transform.SetParent(go.transform, false);
                part.AddComponent<MeshFilter>().sharedMesh = mesh;
                var r = part.AddComponent<MeshRenderer>();
                var mat = look.MaterialFor(a.kind, a.partPaths[i]);
                int subMeshes = mesh != null ? mesh.subMeshCount : 1;
                r.sharedMaterials = Enumerable.Repeat(mat, subMeshes).ToArray();
                parts[i] = part.transform;
                renderers[i] = r;
            }
            go.SetActive(false);
            return (go, parts, renderers);
        }

        public void Pose(float t)
        {
            float step = t / take.stepSeconds;
            foreach (var a in take.actors)
            {
                var (go, parts, renderers) = puppets[a.id];
                bool lingering = a.kind == ActorKind.Target && a.hit && step > a.LastStep && step <= a.LastStep + hitLinger / take.stepSeconds;
                bool visible = a.AliveAt(step) || lingering;
                if (go.activeSelf != visible) go.SetActive(visible);
                if (!visible) continue;
                for (int i = 0; i < parts.Length; i++)
                {
                    var p = a.Sample(i, step);
                    parts[i].SetPositionAndRotation(p.position, p.rotation);
                    parts[i].localScale = p.scale;
                }
                if (a.kind == ActorKind.Target && a.hit)
                {
                    // Flash on the hit: swap to the hit material for the linger window and swell slightly.
                    var mat = lingering ? look.targetHit : look.target;
                    foreach (var r in renderers) if (r.sharedMaterial != mat) r.sharedMaterial = mat;
                    if (lingering)
                    {
                        float k = (step - a.LastStep) * take.stepSeconds / hitLinger;
                        foreach (var part in parts) part.localScale *= 1f + 0.35f * Mathf.Sqrt(k);
                    }
                }
            }
        }

        public void Dispose()
        {
            if (root != null) UnityEngine.Object.DestroyImmediate(root.gameObject);
        }
    }
}
