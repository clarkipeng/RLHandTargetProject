using UnityEngine;

namespace Training
{
    /// <summary>
    /// Where targets go, per lesson, in shoulder-relative coordinates (the arm rests pointing along -x; throws go toward -z).
    /// Azimuth is measured in the ground plane from -z, toward +x.
    /// Held-out: azimuths within 3 degrees of the band centres below never appear in training,
    /// and the evaluation set uses exactly those azimuths, at several distances and heights.
    /// </summary>
    public static class TargetSampler
    {
        public const int FinalLesson = 5;
        static readonly float[] HeldOutAzimuths = { -50f, -30f, -10f, 10f, 30f, 50f };
        const float HeldOutHalfWidth = 3f;

        //                                         azimuth±, dist min, dist max, y min, y max, radius
        static readonly float[,] Lessons =
        {
            { 0f, 0f, 0f, 0f, 0f, 0f },        // 0 grasp (no target)
            { 0f, 0f, 0f, 0f, 0f, 0f },        // 1 lift (no target)
            { 0f, 30f, 30f, -15f, -15f, 14f }, // 2 throw: land in a wide zone in front
            { 15f, 22f, 32f, -10f, -4f, 6f },  // 3 near targets
            { 35f, 22f, 42f, -12f, 0f, 5f },   // 4 mid
            { 60f, 22f, 55f, -12f, 6f, 4f },   // 5 far and varied (the final task)
        };

        public static (Vector3 position, float radius) Sample(int lesson)
        {
            lesson = Mathf.Clamp(lesson, 0, FinalLesson);
            float az;
            do az = Random.Range(-Lessons[lesson, 0], Lessons[lesson, 0]);
            while (lesson >= 3 && IsHeldOut(az));
            float d = Random.Range(Lessons[lesson, 1], Lessons[lesson, 2]);
            float y = Random.Range(Lessons[lesson, 3], Lessons[lesson, 4]);
            float r = Mathf.Max(Lessons[lesson, 5], 1f);
            return (At(az, d, y), r);
        }

        public static bool IsHeldOut(float azimuth)
        {
            foreach (var c in HeldOutAzimuths) if (Mathf.Abs(azimuth - c) < HeldOutHalfWidth) return true;
            return false;
        }

        public static int HeldOutCount => HeldOutAzimuths.Length * 4;

        /// <summary>The evaluation targets: 6 held-out azimuths x 4 (distance, height) pairs, final-lesson radius.</summary>
        public static (Vector3 position, float radius) HeldOut(int index)
        {
            index = ((index % HeldOutCount) + HeldOutCount) % HeldOutCount;
            float az = HeldOutAzimuths[index % HeldOutAzimuths.Length];
            int k = index / HeldOutAzimuths.Length;
            float[] dist = { 25f, 33f, 42f, 52f };
            float[] height = { -8f, -2f, -11f, 4f };
            return (At(az, dist[k], height[k]), Lessons[FinalLesson, 5]);
        }

        static Vector3 At(float azimuthDeg, float distance, float y)
        {
            float a = azimuthDeg * Mathf.Deg2Rad;
            return new Vector3(Mathf.Sin(a) * distance, y, -Mathf.Cos(a) * distance);
        }
    }
}
