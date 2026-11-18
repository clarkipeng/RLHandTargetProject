using UnityEngine;

namespace Showcase
{
    /// <summary>
    /// A light with a physical size. The offline renderer accumulates many sub-frames per image; moving the
    /// light to a different point of its disc on each one turns hard shadow maps into true penumbras.
    /// </summary>
    [RequireComponent(typeof(Light))]
    [ExecuteAlways]
    public class StageLight : MonoBehaviour
    {
        [Tooltip("Angular radius in degrees (directional) of the light's disc.")]
        public float angularRadius = 3f;
        [Tooltip("Radius of the emitter in arm lengths (spot and point lights).")]
        public float size;
        [Tooltip("Range in arm lengths (spot and point lights); 0 keeps the light's range.")]
        public float range;
        [Tooltip("Spot and point lights: brightness at one arm length's distance; 0 keeps the light's intensity. " +
                 "Unity's lights fall off with the square of distance in world units, so this keeps the look at any scale.")]
        public float intensity;

        Vector3 basePosition;
        Quaternion baseRotation;
        bool captured;
        float armLength = 1f;

        public void Fit(float armLength)
        {
            this.armLength = armLength;
            var light = GetComponent<Light>();
            if (range > 0f && light.type != LightType.Directional) light.range = range * armLength;
            if (intensity > 0f && light.type != LightType.Directional) light.intensity = intensity * armLength * armLength;
            basePosition = transform.position;
            baseRotation = transform.rotation;
            captured = true;
        }

        public void Jitter(int index, int count)
        {
            if (!captured) return;
            // Stratified points on a unit disc (Vogel spiral): even coverage at any sample count.
            float r = Mathf.Sqrt((index + 0.5f) / Mathf.Max(1, count));
            float a = index * 2.39996323f;
            var disc = new Vector2(Mathf.Cos(a), Mathf.Sin(a)) * r;
            var right = baseRotation * Vector3.right;
            var up = baseRotation * Vector3.up;
            if (GetComponent<Light>().type == LightType.Directional)
            {
                transform.rotation = Quaternion.AngleAxis(disc.x * angularRadius, up)
                                     * Quaternion.AngleAxis(disc.y * angularRadius, right) * baseRotation;
            }
            else
            {
                transform.position = basePosition + (right * disc.x + up * disc.y) * size * armLength;
            }
        }
    }
}
