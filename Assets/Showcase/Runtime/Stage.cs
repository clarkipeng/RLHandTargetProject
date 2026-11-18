using UnityEngine;

namespace Showcase
{
    /// <summary>
    /// The studio the arm performs in: a seamless round cyclorama (no corners or horizon from any angle),
    /// a pedestal under the shoulder, and the lights. Sized to the action so it works at any scene scale.
    /// </summary>
    [ExecuteAlways]
    public class Stage : MonoBehaviour
    {
        public Transform cove;        // unit-radius cyclorama mesh, scaled to fit
        public Transform pedestal;    // unit cylinder (radius 0.5, height 1, pivot at its base)
        public ReflectionProbe probe;
        public float floorY = -15f;   // top of the training scene's ground collider

        /// <summary>Fits the studio around the shoulder and everything the take does.</summary>
        public void Fit(Vector3 shoulder, Bounds action, float armLength)
        {
            var c = action.center; c.y = floorY;
            float reach = Mathf.Max(action.extents.x, action.extents.z) + armLength;
            // The cove is generous: wide shots pull the camera far back, and the wall must stay behind it.
            float radius = Mathf.Max(reach * 6f, armLength * 20f);
            cove.position = c;
            cove.localScale = Vector3.one * radius;

            float pedestalRadius = 0.11f * armLength;
            float top = shoulder.y - 0.12f * armLength;
            pedestal.position = new Vector3(shoulder.x, floorY, shoulder.z);
            pedestal.localScale = new Vector3(2f * pedestalRadius, Mathf.Max(0.01f, top - floorY), 2f * pedestalRadius);

            if (probe != null)
            {
                probe.transform.position = shoulder + Vector3.up * armLength;
                probe.size = Vector3.one * radius * 2f;
                probe.RenderProbe();
            }
        }
    }
}
