using System.Collections.Generic;
using UnityEngine;

namespace Training
{
    /// <summary>The colliders the ball is touching right now, kept from enter/exit events.</summary>
    public class BallContacts : MonoBehaviour
    {
        readonly Dictionary<Collider, int> counts = new Dictionary<Collider, int>();
        public readonly HashSet<Collider> Touching = new HashSet<Collider>();

        public void Clear() { counts.Clear(); Touching.Clear(); }

        void OnCollisionEnter(Collision c)
        {
            counts.TryGetValue(c.collider, out int n);
            counts[c.collider] = n + 1;
            Touching.Add(c.collider);
        }

        void OnCollisionExit(Collision c)
        {
            if (!counts.TryGetValue(c.collider, out int n)) return;
            if (n <= 1) { counts.Remove(c.collider); Touching.Remove(c.collider); }
            else counts[c.collider] = n - 1;
        }
    }
}
