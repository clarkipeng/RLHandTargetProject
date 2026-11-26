using System;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using UnityEngine;

namespace Showcase
{
    /// <summary>
    /// Records every physics step of a policy rollout into a <see cref="Take"/>.
    /// It needs no scene edits: start any build or play session with
    ///   -showcaseTake path/to/run.take [-showcaseSeconds 120] [-showcasePolicy name] [-showcaseSeed N]
    /// and it attaches itself, records, saves and quits.
    /// It finds actors by component, so it follows whatever the training scene spawns:
    /// the arm (SimCharController), balls (ProjectileScript) and targets (tag "target").
    /// </summary>
    [DefaultExecutionOrder(32000)] // after physics (570) and the agents' reward/teardown step (1100)
    public class TakeRecorder : MonoBehaviour
    {
        public string outputPath;
        public float seconds = 120f;

        readonly Take take = new Take();
        readonly Dictionary<int, (TakeActor actor, Transform root, Transform[] parts)> live =
            new Dictionary<int, (TakeActor, Transform, Transform[])>();
        readonly Dictionary<Agent, int> episodesSeen = new Dictionary<Agent, int>();
        readonly HashSet<int> ballsOnTarget = new HashSet<int>(); // balls whose hitTarget flag was set last sample
        int step;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void AttachFromCommandLine()
        {
            string path = Arg("-showcaseTake");
            if (string.IsNullOrEmpty(path)) return;
            var recorder = new GameObject("TakeRecorder").AddComponent<TakeRecorder>();
            recorder.outputPath = path;
            if (float.TryParse(Arg("-showcaseSeconds"), out float s)) recorder.seconds = s;
            recorder.take.policy = Arg("-showcasePolicy") ?? "";
            if (int.TryParse(Arg("-showcaseSeed"), out int seed)) UnityEngine.Random.InitState(seed); // same spawns for every policy
            DontDestroyOnLoad(recorder.gameObject);
            Debug.Log($"[showcase] recording {recorder.seconds}s to {path}");
        }

        static string Arg(string name)
        {
            var args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, name);
            return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
        }

        void Start() => take.stepSeconds = Time.fixedDeltaTime;

        void FixedUpdate()
        {
            TrackEpisodes();
            Discover();
            Sample();
            step++;
            take.stepCount = step;
            if (step * take.stepSeconds >= seconds) Finish();
        }

        void TrackEpisodes()
        {
            foreach (var agent in FindObjectsOfType<Agent>())
            {
                if (!agent.isActiveAndEnabled) continue;
                episodesSeen.TryGetValue(agent, out int seen);
                if (step == 0 || agent.CompletedEpisodes != seen) take.episodeStarts.Add(step);
                episodesSeen[agent] = agent.CompletedEpisodes;
            }
        }

        void Discover()
        {
            foreach (var arm in FindObjectsOfType<SimCharController>()) Track(arm.transform, ActorKind.Arm);
            foreach (var ball in FindObjectsOfType<ProjectileScript>())
                if (ball.transform.parent == null || ball.transform.parent.GetComponentInParent<ProjectileScript>() == null)
                    Track(ball.transform, ActorKind.Ball); // the prefab nests a second ProjectileScript; keep the outermost
            foreach (var target in GameObject.FindGameObjectsWithTag("target")) Track(target.transform, ActorKind.Target);
            foreach (var prop in FindObjectsOfType<Training.ShowcaseProp>()) Track(prop.transform, ActorKind.Prop); // the ball's pedestal
        }

        void Track(Transform root, ActorKind kind)
        {
            int id = root.GetInstanceID();
            if (live.ContainsKey(id)) return;
            var parts = root.GetComponentsInChildren<MeshRenderer>(true).Select(r => r.transform).ToArray();
            var actor = new TakeActor
            {
                id = id,
                kind = kind,
                partPaths = parts.Select(p => RelativePath(root, p)).ToArray(),
                birthStep = step,
            };
            take.actors.Add(actor);
            live[id] = (actor, root, parts);
        }

        void Sample()
        {
            var dead = new List<int>();
            foreach (var (id, entry) in live)
            {
                if (entry.root == null || !entry.root.gameObject.activeInHierarchy)
                {
                    entry.actor.deathStep = step;
                    dead.Add(id);
                    continue;
                }
                var frame = new PartPose[entry.parts.Length];
                for (int i = 0; i < frame.Length; i++)
                {
                    var t = entry.parts[i];
                    frame[i] = new PartPose { position = t.position, rotation = t.rotation, scale = t.lossyScale };
                }
                entry.actor.frames.Add(frame);

                // A hit is the flag's rising edge: training arenas reuse one ball for every episode.
                bool onTarget = entry.actor.kind == ActorKind.Ball && entry.root.GetComponent<ProjectileScript>().hitTarget;
                if (!onTarget) ballsOnTarget.Remove(id);
                else if (ballsOnTarget.Add(id))
                {
                    entry.actor.hit = true;
                    take.hitSteps.Add(step);
                    MarkNearestTargetHit(entry.root.position);
                }
            }
            foreach (var id in dead) live.Remove(id);
        }

        void MarkNearestTargetHit(Vector3 ballPosition)
        {
            var target = live.Values
                .Where(e => e.actor.kind == ActorKind.Target && e.root != null)
                .OrderBy(e => (e.root.position - ballPosition).sqrMagnitude)
                .FirstOrDefault();
            if (target.actor != null) target.actor.hit = true;
        }

        static string RelativePath(Transform root, Transform t)
        {
            var names = new List<string>();
            for (; t != root; t = t.parent) names.Add(t.name);
            names.Reverse();
            return string.Join("/", names);
        }

        void Finish()
        {
            take.Save(outputPath);
            int balls = take.actors.Count(a => a.kind == ActorKind.Ball);
            Debug.Log($"[showcase] saved {outputPath}: {take.stepCount} steps, {balls} balls, {take.hitSteps.Count} hits, {take.episodeStarts.Count} episodes");
            enabled = false;
#if UNITY_EDITOR
            UnityEditor.EditorApplication.Exit(0);
#else
            Application.Quit(0);
#endif
        }
    }
}
