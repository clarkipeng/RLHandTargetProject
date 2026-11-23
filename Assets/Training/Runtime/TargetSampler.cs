using UnityEngine;

namespace Training
{
    /// <summary>
    /// Where targets go, per lesson, in shoulder-relative coordinates (the arm rests pointing along -x; throws go toward -z).
    /// Azimuth is measured in the ground plane from -z, toward +x.
    /// Held-out: azimuths within 3 degrees of the band centres below never appear in training,
    /// and the evaluation set uses exactly those azimuths, at several distances and heights.
    ///
    /// Lesson 3 ("reach") is the distance ramp (see ReachRamp): targets out to the current reach, half of them in the
    /// frontier band just below it and half anywhere from MinDistance up, so near throws keep being practised.
    /// </summary>
    public static class TargetSampler
    {
        public const int FinalLesson = 3;
        public const float FrontierWidth = 4f, FrontierShare = 0.5f;
        public static float MinDistance = 22f;     // 22, or reach_start - FrontierWidth when the ramp starts nearer
        public static float MaxDistance = 72f;      // environment parameter reach_max: the far end of the distance ramp
        public const float AzimuthSpread = 60f, MinY = -12f, MaxY = 6f, FinalRadius = 4f;
        static readonly float[] HeldOutAzimuths = { -50f, -30f, -10f, 10f, 30f, 50f };
        const float HeldOutHalfWidth = 3f;
        public static readonly float[] HeldOutDistances = { 30f, 44f, 58f, 70f };   // PR #8: 25, 33, 42, 52
        static readonly float[] HeldOutHeights = { -8f, -2f, -11f, 4f };
        static readonly float[] DefaultDistances = (float[])HeldOutDistances.Clone(), DefaultHeights = (float[])HeldOutHeights.Clone();

        /// <summary>Environment parameters: reach_max, reach_start and reach_threshold (ReachRamp), and heldout_d0..3 / heldout_y0..3 for the held-out distances and heights.</summary>
        public static void Configure(Unity.MLAgents.EnvironmentParameters p)
        {
            MaxDistance = p.GetWithDefault("reach_max", 72f);
            ReachRamp.SetStart(p.GetWithDefault("reach_start", 26f));
            ReachRamp.Threshold = Mathf.Clamp01(p.GetWithDefault("reach_threshold", 0.6f));
            MinDistance = Mathf.Min(22f, ReachRamp.Start - FrontierWidth);
            for (int k = 0; k < HeldOutDistances.Length; k++)
            {
                HeldOutDistances[k] = p.GetWithDefault("heldout_d" + k, DefaultDistances[k]);
                HeldOutHeights[k] = p.GetWithDefault("heldout_y" + k, DefaultHeights[k]);
            }
        }

        /// <summary>Lessons 0-2 (grasp, lift, throw into a wide zone near the arm); lesson 3 samples up to <paramref name="reach"/>.</summary>
        public static (Vector3 position, float radius, bool frontier) Sample(int lesson, float reach)
        {
            lesson = Mathf.Clamp(lesson, 0, FinalLesson);
            if (lesson < 2) return (Vector3.zero, 1f, false);
            if (lesson == 2) return (At(0f, 22f, -15f), 12f, false); // land anywhere 10-34 out in front
            float az;
            do az = Random.Range(-AzimuthSpread, AzimuthSpread);
            while (IsHeldOut(az));
            reach = Mathf.Clamp(reach, MinDistance + FrontierWidth, MaxDistance);
            bool frontier = reach < MaxDistance && Random.value < FrontierShare;
            float d = frontier ? Random.Range(reach - FrontierWidth, reach) : Random.Range(MinDistance, reach);
            float y = Random.Range(MinY, MaxY);
            // Radius 6 at the first stage narrowing to the final 4 by reach 40.
            float r = Mathf.Lerp(6f, FinalRadius, Mathf.InverseLerp(MinDistance + FrontierWidth, 40f, reach));
            return (At(az, d, y), r, frontier);
        }

        public static bool IsHeldOut(float azimuth)
        {
            foreach (var c in HeldOutAzimuths) if (Mathf.Abs(azimuth - c) < HeldOutHalfWidth) return true;
            return false;
        }

        public static int HeldOutCount => HeldOutAzimuths.Length * HeldOutDistances.Length;
        public static float HeldOutDistance(int index) => HeldOutDistances[(((index % HeldOutCount) + HeldOutCount) % HeldOutCount) / HeldOutAzimuths.Length];

        /// <summary>The evaluation targets: 6 held-out azimuths x 4 (distance, height) pairs, final radius.</summary>
        public static (Vector3 position, float radius) HeldOut(int index)
        {
            index = ((index % HeldOutCount) + HeldOutCount) % HeldOutCount;
            float az = HeldOutAzimuths[index % HeldOutAzimuths.Length];
            int k = index / HeldOutAzimuths.Length;
            return (At(az, HeldOutDistances[k], HeldOutHeights[k]), FinalRadius);
        }

        static Vector3 At(float azimuthDeg, float distance, float y)
        {
            float a = azimuthDeg * Mathf.Deg2Rad;
            return new Vector3(Mathf.Sin(a) * distance, y, -Mathf.Cos(a) * distance);
        }
    }
}
