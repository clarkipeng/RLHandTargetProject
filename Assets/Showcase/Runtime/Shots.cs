using System;
using System.Collections.Generic;
using UnityEngine;

namespace Showcase
{
    /// <summary>Where the things worth framing are at time t (seconds).</summary>
    public interface ISubject
    {
        Vector3 ArmBase(float t);
        Vector3 Hand(float t);
        bool Ball(float t, out Vector3 position);
        bool Target(float t, out Vector3 position);
        /// <summary>Rough size of the arm, used to scale every shot.</summary>
        float ArmLength { get; }
    }

    public struct CameraPose
    {
        public Vector3 position;
        public Quaternion rotation;
        public float fov;           // vertical, degrees
        public float focusDistance; // world units
        public float aperture;      // f-stop for the bokeh
    }

    public enum ShotType { Wide, Orbit, Close, Follow, Impact }

    /// <summary>
    /// One camera shot of a reel. Shots are anchored to events in the take (a hit, an episode start)
    /// so a reel definition survives re-recording with a new policy.
    /// </summary>
    [Serializable]
    public class Shot
    {
        public string name = "";
        public string type = "Wide";   // Wide | Orbit | Close | Follow | Impact (a string: JsonUtility cannot read enum names)
        public ShotType Type => (ShotType)Enum.Parse(typeof(ShotType), type, true);
        public string anchor = "hit"; // hit | release | episode | time
        public int index;             // which hit or episode
        public float offset = -3f;    // seconds relative to the anchor
        public float duration = 5f;
        public float fov = 30f;
        public float aperture = 4f;
        public float distance = 1f;   // multiplier on the shot's natural distance
        public float height = 0f;     // camera elevation, degrees (positive looks down from above)
        public float azimuth = 0f;    // degrees around the arm, 0 = side-on to the throw
        public float sweep = 0f;      // degrees of orbit across the shot (only with move)
        public bool move = false;     // false (default): a locked camera, framed once on the whole shot, so the background
                                      // never drifts; true: a deliberate move (the sweep, and framing that follows the action)
        public float fit = 0f;        // Wide, locked: >0 sizes and centers the frame so the arm, the ball's whole path in the
                                      // shot and the target fill this fraction of the frame (0.7 = 70%); 0: the fixed span

        // Time: a shot follows one throw, the first whose hit is at or after the shot's start.
        public float slow = 1f;        // playback rate inside the slow windows (0.2 = five times slower)
        public float slowRelease = 0f; // seconds of the take, either side of the release, played slow
        public float slowHit = 0f;     // seconds of the take before the hit played slow
        public float hold = 0f;        // seconds the frame freezes on the hit while the impact plays out;
                                       // when set, the shot lasts until the hit plus the hold, not 'duration'
        public float trail = 0f;       // seconds of the ball's flight drawn as a glowing trail (0: none)
    }

    [Serializable]
    public class Reel
    {
        public int width = 3840;
        public int height = 2160;
        public int fps = 60;
        public int subframes = 16;    // accumulated renders per output frame (motion blur + antialiasing)
        public float shutter = 0.5f;  // fraction of the frame interval the shutter is open (0.5 = 180 degrees)
        public List<Shot> shots = new List<Shot>();
    }

    /// <summary>A subject whose time is clamped to [from, to]: camera smoothing never sees past a reset.</summary>
    public class ClampedSubject : ISubject
    {
        readonly ISubject s;
        readonly float from, to;
        public ClampedSubject(ISubject s, float from, float to) { this.s = s; this.from = from; this.to = Mathf.Max(from, to); }
        float C(float t) => Mathf.Clamp(t, from, to);
        public Vector3 ArmBase(float t) => s.ArmBase(C(t));
        public Vector3 Hand(float t) => s.Hand(C(t));
        public bool Ball(float t, out Vector3 p) => s.Ball(C(t), out p);
        public bool Target(float t, out Vector3 p) => s.Target(C(t), out p);
        public float ArmLength => s.ArmLength;
    }

    public static class ShotEvaluator
    {
        /// <summary>Camera pose of a shot at u in [0,1] across it, with t the subject time.</summary>
        public static CameraPose Evaluate(Shot shot, ISubject s, float t, float u, float shotStart, float shotEnd, float aspect = 16f / 9f)
        {
            float L = Mathf.Max(s.ArmLength, 0.01f);
            // Locked: evaluate once, at the shot's middle, on positions averaged over the whole shot; the pose is then the
            // same on every frame of the shot.
            float wArm = 0.6f, wHand = 0.25f, wBall = 0.35f;
            if (!shot.move && shotEnd > shotStart)
            {
                t = 0.5f * (shotStart + shotEnd); u = 0.5f;
                wArm = wHand = wBall = Mathf.Max(0.6f, 0.5f * (shotEnd - shotStart));
            }
            Vector3 armBase = Smooth(s.ArmBase, t, wArm, 17);
            Vector3 hand = Smooth(s.Hand, t, wHand, 17);
            Vector3 target = StableTarget(s, t, shotStart, shotEnd, armBase + Vector3.right * 3f * L);

            // The throw line, flattened: cameras are placed relative to it so every shot reads left to right.
            Vector3 throwDir = target - armBase; throwDir.y = 0;
            throwDir = throwDir.sqrMagnitude > 1e-6f ? throwDir.normalized : Vector3.right;
            Vector3 side = Vector3.Cross(Vector3.up, throwDir); // camera looks against this axis

            float az = shot.azimuth + (shot.move ? shot.sweep * (Ease(u) - 0.5f) : 0f);
            Quaternion around = Quaternion.AngleAxis(az, Vector3.up) * Quaternion.AngleAxis(shot.height, throwDir);
            Vector3 viewDir = around * side; // from the subject towards the camera

            Vector3 lookAt, focus;
            float dist;
            switch (shot.Type)
            {
                case ShotType.Close:
                    lookAt = hand; focus = hand;
                    dist = 1.1f * L * shot.distance;
                    break;
                case ShotType.Orbit:
                    lookAt = Vector3.Lerp(armBase, hand, 0.6f); focus = hand;
                    dist = 2.4f * L * shot.distance;
                    break;
                case ShotType.Impact:
                    // On the target, with room for the ball to fly in from the arm's side.
                    lookAt = Vector3.Lerp(target, hand, 0.12f); focus = target;
                    dist = 1.3f * L * shot.distance;
                    break;
                case ShotType.Follow:
                {
                    Vector3 subject = Smooth(t2 => s.Ball(t2, out var b) ? b : s.Hand(t2), t, wBall, 17);
                    lookAt = Vector3.Lerp(subject, target, 0.25f); focus = subject;
                    dist = 2.2f * L * shot.distance;
                    break;
                }
                default: // Wide: hold the arm and its target in frame together
                {
                    lookAt = Vector3.Lerp(armBase, target, 0.5f) + Vector3.up * 0.25f * L;
                    focus = Vector3.Lerp(armBase, hand, 0.5f);
                    float span = Vector3.Distance(armBase, target) + 1.1f * L;
                    dist = 0.5f * span / Mathf.Tan(0.5f * shot.fov * Mathf.Deg2Rad) * 1.05f * shot.distance;
                    if (shot.fit > 0f && !shot.move && shotEnd > shotStart)
                        FitFrame(s, shotStart, shotEnd, target, -viewDir, shot.fov, aspect, shot.fit, L, ref lookAt, ref dist);
                    break;
                }
            }

            Vector3 position = lookAt + viewDir * dist;
            var pose = new CameraPose
            {
                position = position,
                rotation = Quaternion.LookRotation(lookAt - position, Vector3.up),
                fov = shot.fov,
                aperture = shot.aperture,
            };
            pose.focusDistance = Mathf.Max(0.1f, Vector3.Dot(focus - position, pose.rotation * Vector3.forward));
            return pose;
        }

        /// <summary>
        /// Centers lookAt on everything the shot shows (the arm and its column, the hand's and the ball's paths, the target)
        /// and sets the distance so it fills 'fill' of the frame in its tighter dimension.
        /// </summary>
        static void FitFrame(ISubject s, float a, float b, Vector3 target, Vector3 fwd, float fov, float aspect, float fill, float L,
                             ref Vector3 lookAt, ref float dist)
        {
            var pts = new List<Vector3>();
            const int n = 48;
            for (int i = 0; i <= n; i++)
            {
                float t = Mathf.Lerp(a, b, i / (float)n);
                Vector3 arm = s.ArmBase(t);
                pts.Add(arm + Vector3.up * 0.1f * L); pts.Add(arm - Vector3.up * 1.0f * L); // the arm's column down to the floor
                pts.Add(s.Hand(t));
                if (s.Ball(t, out var ball)) pts.Add(ball);
            }
            // The ball's pedestal: a subject clamped to one throw returns the resting ball for any time before it.
            if (s.Ball(a - 1e3f, out var rest)) { pts.Add(rest + Vector3.up * 0.05f * L); pts.Add(rest - Vector3.up * 0.4f * L); }
            Vector3 right = Vector3.Cross(Vector3.up, fwd).normalized, up = Vector3.Cross(fwd, right);
            float r = 0.12f * L; // the target's face and the ball's radius
            foreach (var d in new[] { right, -right, up, -up }) pts.Add(target + d * r);

            // Center on the points' box in the image plane, then pull back until every point fits.
            float x0 = float.MaxValue, x1 = float.MinValue, y0 = float.MaxValue, y1 = float.MinValue, z = 0f;
            foreach (var p in pts)
            {
                float x = Vector3.Dot(p, right), y = Vector3.Dot(p, up);
                x0 = Mathf.Min(x0, x); x1 = Mathf.Max(x1, x); y0 = Mathf.Min(y0, y); y1 = Mathf.Max(y1, y); z += Vector3.Dot(p, fwd);
            }
            Vector3 c = right * 0.5f * (x0 + x1) + up * 0.5f * (y0 + y1) + fwd * (z / pts.Count);
            float tv = Mathf.Tan(0.5f * fov * Mathf.Deg2Rad) * fill, th = tv * aspect;
            float need = 0f;
            foreach (var p in pts)
            {
                Vector3 q = p - c;
                float depth = Vector3.Dot(q, fwd);
                need = Mathf.Max(need, Mathf.Abs(Vector3.Dot(q, right)) / th - depth, Mathf.Abs(Vector3.Dot(q, up)) / tv - depth);
            }
            lookAt = c; dist = Mathf.Max(need, 0.5f * L);
        }

        /// <summary>Box-filtered position over +-halfWindow seconds; keeps cameras from jittering with the physics.</summary>
        public static Vector3 Smooth(Func<float, Vector3> f, float t, float halfWindow, int taps = 9)
        {
            Vector3 sum = Vector3.zero;
            for (int i = 0; i < taps; i++)
                sum += f(t + halfWindow * (2f * i / (taps - 1) - 1f));
            return sum / taps;
        }

        /// <summary>
        /// The target most of the shot is about. Targets respawn after a hit; a shot keeps
        /// framing the one alive at its midpoint so the camera never jumps.
        /// </summary>
        static Vector3 StableTarget(ISubject s, float t, float start, float end, Vector3 fallback)
        {
            float mid = 0.5f * (start + end);
            if (s.Target(mid, out var p)) return p;
            if (s.Target(t, out p)) return p;
            if (s.Target(start, out p)) return p;
            return fallback;
        }

        static float Ease(float u) => u * u * (3f - 2f * u);

        /// <summary>
        /// The arm's size in meters as far as the lens is concerned. The training scene's arm is about 20 units
        /// long; at that scale a physical lens has almost no depth of field, so focus distances are converted
        /// to a real robot arm's scale before they reach the bokeh.
        /// </summary>
        public const float ArmMeters = 1f;
        public static float LensDistance(float worldDistance, float armLength) => worldDistance * ArmMeters / Mathf.Max(armLength, 1e-4f);

        /// <summary>Full-frame-equivalent focal length (mm) for a vertical FOV, for the bokeh.</summary>
        public static float FocalLengthMm(float verticalFov) => 12f / Mathf.Tan(0.5f * verticalFov * Mathf.Deg2Rad);
    }
}
