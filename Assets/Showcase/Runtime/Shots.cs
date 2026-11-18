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

    public enum ShotType { Wide, Orbit, Close, Follow }

    /// <summary>
    /// One camera shot of a reel. Shots are anchored to events in the take (a hit, an episode start)
    /// so a reel definition survives re-recording with a new policy.
    /// </summary>
    [Serializable]
    public class Shot
    {
        public string name = "";
        public string type = "Wide";   // Wide | Orbit | Close | Follow (a string: JsonUtility cannot read enum names)
        public ShotType Type => (ShotType)Enum.Parse(typeof(ShotType), type, true);
        public string anchor = "hit"; // hit | episode | time
        public int index;             // which hit or episode
        public float offset = -3f;    // seconds relative to the anchor
        public float duration = 5f;
        public float fov = 30f;
        public float aperture = 4f;
        public float distance = 1f;   // multiplier on the shot's natural distance
        public float height = 0f;     // camera elevation, degrees
        public float azimuth = 0f;    // degrees around the arm, 0 = side-on to the throw
        public float sweep = 0f;      // degrees of orbit across the shot
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

    public static class ShotEvaluator
    {
        /// <summary>Camera pose of a shot at u in [0,1] across it, with t the subject time.</summary>
        public static CameraPose Evaluate(Shot shot, ISubject s, float t, float u, float shotStart, float shotEnd)
        {
            float L = Mathf.Max(s.ArmLength, 0.01f);
            Vector3 armBase = Smooth(s.ArmBase, t, 0.6f);
            Vector3 hand = Smooth(s.Hand, t, 0.25f);
            Vector3 target = StableTarget(s, t, shotStart, shotEnd, armBase + Vector3.right * 3f * L);

            // The throw line, flattened: cameras are placed relative to it so every shot reads left to right.
            Vector3 throwDir = target - armBase; throwDir.y = 0;
            throwDir = throwDir.sqrMagnitude > 1e-6f ? throwDir.normalized : Vector3.right;
            Vector3 side = Vector3.Cross(Vector3.up, throwDir); // camera looks against this axis

            float az = shot.azimuth + shot.sweep * (Ease(u) - 0.5f);
            Quaternion around = Quaternion.AngleAxis(az, Vector3.up) * Quaternion.AngleAxis(-shot.height, throwDir);
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
                case ShotType.Follow:
                {
                    Vector3 subject = Smooth(t2 => s.Ball(t2, out var b) ? b : s.Hand(t2), t, 0.35f);
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

        /// <summary>Full-frame-equivalent focal length (mm) for a vertical FOV, for the bokeh.</summary>
        public static float FocalLengthMm(float verticalFov) => 12f / Mathf.Tan(0.5f * verticalFov * Mathf.Deg2Rad);
    }
}
