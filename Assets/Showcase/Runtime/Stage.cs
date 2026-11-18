using UnityEngine;

namespace Showcase
{
    /// <summary>
    /// The set the arm performs in: an optional round cyclorama, an optional environment authored in
    /// arm lengths, a pedestal under the shoulder, and the lights. Sized to the action so it works at any scene scale.
    /// </summary>
    [ExecuteAlways]
    public class Stage : MonoBehaviour
    {
        public Transform cove;        // unit-radius cyclorama mesh, scaled to fit (minimal look)
        public Transform environment; // authored with the arm length as unit, origin on the floor under the shoulder
        public Transform pedestal;    // unit cylinder (radius 0.5, height 1, pivot at its base)
        public float pedestalRadius = 0.11f; // in arm lengths
        public ReflectionProbe probe;
        public float floorY = -15f;   // top of the training scene's ground collider
        [Tooltip("Exponential-squared fog density times the arm length; 0 leaves fog off.")]
        public float fogDensityTimesArm;

        StageLight[] lights;

        /// <summary>Fits the set around the shoulder and everything the take does.</summary>
        public void Fit(Vector3 shoulder, Bounds action, float armLength)
        {
            float radius = armLength * 20f;
            if (cove != null)
            {
                var c = action.center; c.y = floorY;
                float reach = Mathf.Max(action.extents.x, action.extents.z) + armLength;
                // The cove is generous: wide shots pull the camera far back, and the wall must stay behind it.
                radius = Mathf.Max(reach * 6f, armLength * 20f);
                cove.position = c;
                cove.localScale = Vector3.one * radius;
            }
            if (environment != null)
            {
                environment.position = new Vector3(shoulder.x, floorY, shoulder.z);
                environment.localScale = Vector3.one * armLength;
            }

            float r = pedestalRadius * armLength;
            float top = shoulder.y - 0.04f * armLength;
            pedestal.position = new Vector3(shoulder.x, floorY, shoulder.z);
            pedestal.localScale = new Vector3(2f * r, Mathf.Max(0.01f, top - floorY), 2f * r);

            RenderSettings.fog = fogDensityTimesArm > 0f;
            if (RenderSettings.fog)
            {
                RenderSettings.fogMode = FogMode.ExponentialSquared;
                RenderSettings.fogDensity = fogDensityTimesArm / armLength;
            }

            lights = GetComponentsInChildren<StageLight>(true);
            foreach (var l in lights) l.Fit(armLength);

            if (probe != null)
            {
                probe.transform.position = shoulder + Vector3.up * armLength;
                probe.size = Vector3.one * radius * 2f;
                probe.RenderProbe();
            }
        }

        /// <summary>Moves every light to its sample of sub-frame index of count: accumulated, they cast soft shadows.</summary>
        public void Jitter(int index, int count)
        {
            if (lights == null) lights = GetComponentsInChildren<StageLight>(true);
            foreach (var l in lights) l.Jitter(index, count);
        }
    }
}
