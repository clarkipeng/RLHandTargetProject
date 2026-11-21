using System.Collections.Generic;
using UnityEngine;

namespace Training
{
    /// <summary>
    /// The colliders the ball is touching. PhysX contact events flicker on a resting grip (enter/exit on alternate steps),
    /// so a collider counts as touching if it reported contact within the last few physics steps (25 ms at 240 Hz).
    /// </summary>
    public class BallContacts : MonoBehaviour
    {
        public const int HoldSteps = 6;
        readonly Dictionary<Collider, int> lastSeen = new Dictionary<Collider, int>();
        public readonly HashSet<Collider> Touching = new HashSet<Collider>();
        static int Step => Mathf.RoundToInt(Time.fixedTime / Time.fixedDeltaTime);

        public void Clear() { lastSeen.Clear(); Touching.Clear(); }

        void OnCollisionEnter(Collision c) => lastSeen[c.collider] = Step;
        void OnCollisionStay(Collision c) => lastSeen[c.collider] = Step;

        /// <summary>Called once per physics step by the agent, before it reads <see cref="Touching"/>.</summary>
        public void Refresh()
        {
            Touching.Clear();
            int now = Step;
            foreach (var kv in lastSeen)
                if (now - kv.Value <= HoldSteps && kv.Key != null) Touching.Add(kv.Key);
        }
    }
}
