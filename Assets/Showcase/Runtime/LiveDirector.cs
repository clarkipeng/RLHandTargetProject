using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using UnityEngine.SceneManagement;

namespace Showcase
{
    /// <summary>
    /// The live view: runs the training scene with its policy in inference, dressed in the showcase look,
    /// with a camera that cuts between cinematic shots. Built for WebGL and a wall screen.
    /// </summary>
    public class LiveDirector : MonoBehaviour
    {
        public ShowcaseLook look;
        public Stage stage;
        public Camera showcaseCamera;
        public string trainingScene = "Playground";
        public float shotSeconds = 9f;
        public List<Shot> shots = new List<Shot>
        {
            new Shot { name = "wide",   type = "Wide",   fov = 28, aperture = 8,   height = 10, azimuth = -12, sweep = 14 },
            new Shot { name = "orbit",  type = "Orbit",  fov = 34, aperture = 2.8f, height = 12, azimuth = 10,  sweep = 60 },
            new Shot { name = "close",  type = "Close",  fov = 30, aperture = 2f,  height = 16, azimuth = 35,  sweep = 20 },
            new Shot { name = "follow", type = "Follow", fov = 34, aperture = 4f,  height = 6,  azimuth = -20, sweep = 0 },
        };

        readonly HashSet<int> styled = new HashSet<int>();
        LiveSubject subject;
        DepthOfField dof;
        int shotIndex;
        float shotStart;
        bool fitted;

        void Start()
        {
            var volume = FindObjectsOfType<Volume>().FirstOrDefault(v => v.isGlobal);
            if (volume != null) volume.profile.TryGet(out dof);
            SceneManager.LoadSceneAsync(trainingScene, LoadSceneMode.Additive).completed += _ => Dress(SceneManager.GetSceneByName(trainingScene));
        }

        /// <summary>Keeps the training scene's logic and physics, drops its look.</summary>
        void Dress(Scene scene)
        {
            foreach (var root in scene.GetRootGameObjects())
            {
                foreach (var c in root.GetComponentsInChildren<Camera>(true)) c.enabled = false;
                foreach (var l in root.GetComponentsInChildren<AudioListener>(true)) l.enabled = false;
                foreach (var l in root.GetComponentsInChildren<Light>(true)) l.enabled = false;
                foreach (var v in root.GetComponentsInChildren<Volume>(true)) v.enabled = false;
                foreach (var r in root.GetComponentsInChildren<Renderer>(true)) r.enabled = false; // greybox, ground, templates
            }
            subject = new LiveSubject();
        }

        void Update()
        {
            if (subject == null) return;
            Restyle();
            if (!subject.Refresh()) return;
            if (!fitted)
            {
                stage.Fit(subject.ArmBase(0), new Bounds(subject.ArmBase(0), Vector3.one * subject.ArmLength * 6f), subject.ArmLength);
                fitted = true;
                shotStart = Time.time;
            }
            if (Time.time - shotStart > shotSeconds) { shotIndex = (shotIndex + 1) % shots.Count; shotStart = Time.time; }
            float u = (Time.time - shotStart) / shotSeconds;
            var pose = ShotEvaluator.Evaluate(shots[shotIndex], subject, Time.time, u, shotStart, shotStart + shotSeconds);
            showcaseCamera.transform.SetPositionAndRotation(pose.position, pose.rotation);
            showcaseCamera.fieldOfView = pose.fov;
            if (dof != null)
            {
                dof.focusDistance.Override(ShotEvaluator.LensDistance(pose.focusDistance, subject.ArmLength));
                dof.focalLength.Override(ShotEvaluator.FocalLengthMm(pose.fov));
                dof.aperture.Override(pose.aperture);
            }
        }

        void Restyle()
        {
            foreach (var arm in FindObjectsOfType<SimCharController>()) Style(arm.transform, ActorKind.Arm);
            foreach (var ball in FindObjectsOfType<ProjectileScript>()) Style(ball.transform, ActorKind.Ball);
            foreach (var target in GameObject.FindGameObjectsWithTag("target")) Style(target.transform, ActorKind.Target);
        }

        void Style(Transform root, ActorKind kind)
        {
            if (!styled.Add(root.GetInstanceID())) return;
            look.Restyle(root, kind);
            foreach (var r in root.GetComponentsInChildren<Renderer>(true)) r.enabled = true;
        }
    }

    /// <summary>
    /// Live positions for the shot framing. The live camera cannot see the future, so positions are
    /// exponentially smoothed from a short history instead of the replay's centered window.
    /// </summary>
    public class LiveSubject : ISubject
    {
        Transform arm, shoulder, palm;
        readonly SortedList<float, (Vector3 arm, Vector3 hand, Vector3? ball, Vector3? target)> history =
            new SortedList<float, (Vector3, Vector3, Vector3?, Vector3?)>();
        public float ArmLength { get; private set; } = 1f;

        public bool Refresh()
        {
            if (arm == null)
            {
                var sim = Object.FindObjectOfType<SimCharController>();
                if (sim == null) return false;
                arm = sim.transform;
                var parts = arm.GetComponentsInChildren<MeshRenderer>().Select(r => r.transform).ToArray();
                shoulder = arm; // the root part is the shoulder
                palm = parts.FirstOrDefault(p => p.name.ToLowerInvariant().Contains("palm")) ?? parts.Last();
                ArmLength = Mathf.Max(0.01f, parts.Max(p => Vector3.Distance(p.position, shoulder.position)));
            }
            var ball = Object.FindObjectOfType<ProjectileScript>();
            var target = GameObject.FindGameObjectWithTag("target");
            history[Time.time] = (shoulder.position, palm.position, ball ? ball.transform.position : (Vector3?)null,
                target ? target.transform.position : (Vector3?)null);
            while (history.Count > 0 && history.Keys[0] < Time.time - 2f) history.RemoveAt(0);
            return true;
        }

        // Evaluating at any t returns the latest smoothed state: the evaluator's window collapses onto "now".
        (Vector3 arm, Vector3 hand, Vector3? ball, Vector3? target) Latest => history.Values[history.Count - 1];

        public Vector3 ArmBase(float t) => Latest.arm;
        public Vector3 Hand(float t) => Damped(v => v.hand, t);
        public bool Ball(float t, out Vector3 p) { var b = Latest.ball; p = b ?? default; return b.HasValue; }
        public bool Target(float t, out Vector3 p) { var x = Latest.target; p = x ?? default; return x.HasValue; }

        Vector3 Damped(System.Func<(Vector3 arm, Vector3 hand, Vector3? ball, Vector3? target), Vector3> f, float t)
        {
            Vector3 sum = Vector3.zero; float w = 0f;
            foreach (var kv in history)
            {
                float k = Mathf.Exp(-(Time.time - kv.Key) / 0.3f);
                sum += f(kv.Value) * k; w += k;
            }
            return w > 0 ? sum / w : f(Latest);
        }
    }
}
