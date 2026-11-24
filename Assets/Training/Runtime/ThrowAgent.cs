using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace Training
{
    /// <summary>
    /// One arm, one resting ball, one target: reach, grasp, lift, wind up, release, hit.
    /// The curriculum (ml/config/throw.yaml) sets the environment parameter "lesson":
    ///   0 grasp (hold it off the pedestal), 1 lift, 2 throw into a wide zone near the arm, 3 reach: targets ramp out from 26 to 56
    ///   in small steps as the hit rate allows (ReachRamp), with near distances still sampled.
    /// The arm is weak on purpose (ArmLimits): torque and joint-speed caps, the wrist and fingers capped hardest,
    /// so far targets need a wind-up through the shoulder and elbow rather than a flick.
    /// Every lesson pays +1 for its own success and shares one bounded shaping potential
    /// (reach the ball, hold it, lift it; at most 0.4 per episode), so the reward measure means the same thing in every lesson.
    /// Throws also earn up to 0.5 for how close the ball flew to the target.
    /// Targets whose azimuth falls in the held-out bands (TargetSampler) are never used in training; "eval_target" picks one of them.
    /// </summary>
    public class ThrowAgent : Agent
    {
        // Set by ArenaSpawner before the agent is enabled.
        [HideInInspector] public GameObject armPrefab, ballPrefab, targetPrefab;
        [HideInInspector] public int armLayer, arenaIndex, arenaCount = 1;
        public const float Scale = 16f;            // the arm's length; observations are in arm lengths
        public const float FloorY = -15f;          // top of the ground, relative to the shoulder
        public static readonly Vector3 PedestalTop = new Vector3(-11.5f, -4f, -11.5f); // just under the hand of the start pose: the arm is counterbalanced, so it no longer sags onto the ball while exploring (PR #8: (-14, -9, 0))
        // Start pose, per drive in action units: the shoulder turned 45 degrees towards the targets, so a throw has little room
        // to accelerate unless the arm first swings back (ml/probe.py measures how far each kind of swing can reach).
        public static readonly float[] DefaultPose = { -0.5f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f };
        public const float PedestalHalf = 2f;
        public const int PhysicsStepsPerDecision = 8; // 240 Hz physics, 30 Hz decisions
        public const float MaxEpisodeSeconds = 12f;  // the longest episode_seconds can ask for (history buffers)
        float EpisodeSeconds = 10f;                // environment parameter episode_seconds
        public const float BallScale = 0.6f;       // radius 0.6: small enough for this hand to close around

        SimCharController arm;
        ArticulationBody[] bodies;
        readonly List<(ArticulationBody body, int axis, float lo, float hi)> drives = new List<(ArticulationBody, int, float, float)>();
        ArticulationBody[] dofBodies;
        Transform[] stateBones;
        Transform palm, thumbTip, middleTip;
        readonly HashSet<Collider> handColliders = new HashSet<Collider>();
        readonly HashSet<Collider> thumbOrPalm = new HashSet<Collider>();

        Rigidbody ball;
        float ballRadius;
        BallContacts contacts;
        GameObject target;
        Collider pedestal;
        Vector3 targetPos;   // arena-local
        // Showcase only (never set in training): target_motion > 0 sways the target sideways by that many units,
        // over target_period seconds, from a per-episode phase. The policy sees the target's current position every step.
        Vector3 targetBase;
        float targetMotion, targetPeriod, targetPhase;
        float targetRadius;

        float[] action, smoothed;
        float smoothing = 0.25f; // per physics step
        int lesson;
        int evalTarget = -1, evalEpisodes;

        // Strength limits (ArmLimits) and the throw measurements reported with every eval episode.
        int[] driveGroup;                       // ArmLimits group of each drive
        ArticulationBody[] groupBody;           // shoulder, elbow, wrist: the body each joint moves
        float[] torqueCap, speedCap;            // per drive; torqueCap <= 0: the uncapped arm of PR #6
        int[] armDrives;                        // drives of the shoulder and elbow (for the wind-up measure)
        float[,] armHistory;                    // their positions per physics step since the lift (radians)
        int historyLength, liftStep;
        float[,] recentVelocity;                // per drive, the last AccelWindow joint velocities (acceleration over 1/30 s)
        const int AccelWindow = 8;
        readonly float[] peakSpeed = new float[ArmLimits.Groups], peakAccel = new float[ArmLimits.Groups], peakTorque = new float[ArmLimits.Groups];
        readonly float[] share = new float[ArmLimits.Groups], heldShare = new float[ArmLimits.Groups];
        float heldSpeed, releaseSpeed, backswing, windupTime, forwardSwing, targetDistance;
        bool frontierTarget;
        // Speed caps enforced in physics (speed_enforce, default 1): PhysX's maxJointVelocity on each capped body,
        // backed by a clamp of the joint velocities after every physics step. Measure() reads the raw velocity before
        // the clamp, so a leak through PhysX still shows in the peaks; overCap counts the clamped steps.
        bool enforceSpeed, governor, predictiveClamp;
        readonly float[] clampOut = new float[32];   // each drive's joint velocity (rad/s) as the clamp left it last step
        int driveMode;              // drive_mode 1 (default): velocity servo; 0: PR #8's position drive with the explicit curve
        float servoGain;
        float[] baseStiffness, baseDamping;   // each drive's own gains (SimCharController sets them in Start), read at the first step
        int traceSteps;
        float governorBand = 0.95f;
        float[] previousVelocity = new float[32];
        readonly int[] governed = new int[ArmLimits.Groups];               // governor_band: fraction of the cap where the governor takes over
        const float GovernorDamping = 1e7f, GovernorForce = 1e9f;
        float lastDepen = -1f;
        float[] lastPosition;                     // per drive, the joint angle a step ago (sustained speed)
        float[,] recentStep;                      // per drive, the last SustainWindow per-step angle changes
        const int SustainWindow = 4;              // 1/60 s
        readonly float[] peakSustained = new float[ArmLimits.Groups];
        float[,] speedHistory;                    // per physics step since the lift, each group's fastest raw joint speed (deg/s)
        readonly float[] windowPeak = new float[ArmLimits.Groups]; // fastest raw speed from the backswing's turning point to the release             // trace_steps > 0: arena 0 logs the fingers and wrist every step of its first eval episode
        float[] defaultMaxJointVelocity;
        readonly int[] overCap = new int[ArmLimits.Groups];
        readonly int[] peakStep = new int[ArmLimits.Groups];
        readonly bool[] peakContact = new bool[ArmLimits.Groups], peakPushing = new bool[ArmLimits.Groups];
        // Start pose (pose_<i>: per drive, in action units; default the rest pose, straight out) and pedestal (ped_x/y/z).
        float[] startPose;
        Vector3 pedTop = PedestalTop;
        GameObject pedObject;
        // Wind-up shaping (windup_bonus, default 0): paid at release for a backswing followed by a forward swing,
        // annealed to zero as the reach ramp gets to windup_bonus_end.
        float windupBonus, windupBonusEnd;
        float touchShaping;                       // touch_shaping (default 0.06): see Potential
        bool poseRelative;                        // pose_relative (default 0): see OnActionReceived
        // start_held (default 0): every episode begins with the ball in the closed hand instead of on the pedestal. The ball is
        // pinned at the grip point while the fingers are driven shut (HoldSetupSteps), then it is a free body held by friction
        // alone, so the policy must keep its grip. Round 2 of the wind-up: with the capped arm the pedestal grasp collapsed into
        // pressing the ball onto the pedestal's edge, which never became a lift (windup08: 0 lifts in 2.3M steps of lesson 1).
        bool startHeld;
        public const int HoldSetupSteps = 48;     // 0.2 s
        float[] restAction = new float[0];        // each drive's start-pose action at the last reset (no noise)
        public const int HoldSettleSteps = 3;     // the first steps of the setup: the ball ignores the hand, the fingers stay open
        // Natural-motion variants (docs/WINDUP.md, round 2); each default leaves training as it was.
        float actionRateCost = 1e-4f;             // action_rate_cost: per decision, times the summed squared change of the drive targets
        float effortCost;                         // effort_cost: per second of held ball, times the mean (shoulder..wrist torque / cap)^2
        float sequenceBonus;                      // sequence_bonus: at release, times the share of proximal-to-distal peak orderings
        float heldClose = -1f;                    // held_close: finger and thumb targets while the hand closes (-1: their lower limit)
        // Probe (probe = 1): the ball starts fixed in the hand (no grasp), the hand ignores it, nothing is released;
        // every step records how far a release at that instant would carry (drag-free flight). The actions come from
        // ml/probe.py, which searches open-loop swings with and without a backswing.
        bool probe;
        ArticulationBody probePalm; int probeGlitches; float probePalmMass, probeDrift; Vector3 probePalmCom, probeOffset, probePalmInertia, probeShoulder;
        float probeBest, probeBestHigh, probeBestSpeed, probeSeconds;
        int probeBestStep, probeHistoryAtBest;
        readonly Vector3[] probeTrail = new Vector3[5];
        readonly float[] probePeakSpeed = new float[ArmLimits.Groups];

        // Episode state
        int steps, graspSteps, heldSteps, liftSteps, noContactSteps;
        bool everGrasped, everLifted, thrown, hit;
        float potential, minGap, maxThrowDistance;
        Vector3 releasePoint;

        public static int CountActions(GameObject armPrefab) => DofDrives(armPrefab.GetComponent<SimCharController>()).Count();
        public static int CountObservations(int actions) => 11 * 6 + 3 * actions + 9 + 4 + 6;

        static IEnumerable<(ArticulationBody, int, float, float)> DofDrives(SimCharController c)
        {
            for (int b = 1; b < SimCharController.nbones; b++)
            {
                var ab = c.boneToTransform[b].GetComponent<ArticulationBody>();
                var d = new[] { ab.xDrive, ab.yDrive, ab.zDrive };
                var locks = new[] { ab.twistLock, ab.swingYLock, ab.swingZLock };
                bool revolute = ab.jointType == ArticulationJointType.RevoluteJoint;
                for (int axis = 0; axis < 3; axis++)
                    if (d[axis].upperLimit - d[axis].lowerLimit != 0 && (revolute || locks[axis] != ArticulationDofLock.LockedMotion))
                        yield return (ab, axis, d[axis].lowerLimit, d[axis].upperLimit);
            }
        }

        public override void Initialize()
        {
            var root = transform;
            arm = Instantiate(armPrefab, root.position, Quaternion.identity, root).GetComponent<SimCharController>();
            foreach (var t in arm.GetComponentsInChildren<Transform>(true)) t.gameObject.layer = armLayer;
            bodies = arm.GetComponentsInChildren<ArticulationBody>();
            foreach (var b in bodies) { b.solverIterations = ConfigManager.Instance.solverIterations; b.solverVelocityIterations = ConfigManager.Instance.solverIterations; }
            drives.AddRange(DofDrives(arm));
            dofBodies = drives.Select(d => d.body).Distinct().ToArray();
            stateBones = MLAgent.stateBones.Select(b => arm.boneToTransform[(int)b]).ToArray();
            palm = arm.boneToTransform[(int)MLAgent.Bones.Bone_Palm];
            thumbTip = arm.boneToTransform[(int)MLAgent.Bones.Bone_Thumb2];
            middleTip = arm.boneToTransform[(int)MLAgent.Bones.Bone_FingerMiddle2];
            for (int b = (int)MLAgent.Bones.Bone_Palm; b < SimCharController.nbones; b++)
                foreach (var c in arm.boneToTransform[b].GetComponents<Collider>())
                {
                    handColliders.Add(c);
                    if (b <= (int)MLAgent.Bones.Bone_Thumb2) thumbOrPalm.Add(c);
                }

            var ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
            ground.name = "Ground";
            ground.transform.SetParent(root, false);
            ground.transform.localPosition = new Vector3(0, FloorY - 0.5f, -20f);
            ground.transform.localScale = new Vector3(260f, 1f, 260f);
            Destroy(ground.GetComponent<MeshRenderer>());

            var ped = GameObject.CreatePrimitive(PrimitiveType.Cube);
            ped.name = "Pedestal";
            ped.tag = "Untagged";
            ped.transform.SetParent(root, false);
            float h = PedestalTop.y - FloorY;
            ped.transform.localPosition = new Vector3(PedestalTop.x, FloorY + h / 2, PedestalTop.z);
            ped.transform.localScale = new Vector3(2 * PedestalHalf, h, 2 * PedestalHalf);
            pedestal = ped.GetComponent<Collider>();
            pedObject = ped;
            ped.AddComponent<ShowcaseProp>();

            ball = Instantiate(ballPrefab, root.position, Quaternion.identity, root).GetComponent<Rigidbody>();
            ball.mass = 2f;
            ball.transform.localScale = Vector3.one * BallScale;
            ball.GetComponent<SphereCollider>().sharedMaterial = new PhysicMaterial("Grippy")
            {
                dynamicFriction = 1f, staticFriction = 1.2f, bounciness = 0.1f,
                frictionCombine = PhysicMaterialCombine.Maximum, bounceCombine = PhysicMaterialCombine.Minimum,
            };
            ball.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            ball.interpolation = RigidbodyInterpolation.None;
            ballRadius = ball.GetComponent<SphereCollider>().radius * ball.transform.lossyScale.x;
            contacts = ball.gameObject.AddComponent<BallContacts>();

            target = Instantiate(targetPrefab, root.position, Quaternion.identity, root);

            driveGroup = drives.Select(d => ArmLimits.GroupOf(arm, d.body)).ToArray();
            groupBody = new[] { 1, 2, 3 }.Select(b => arm.boneToTransform[b].GetComponent<ArticulationBody>()).ToArray();
            torqueCap = new float[drives.Count];
            speedCap = new float[drives.Count];
            armDrives = Enumerable.Range(0, drives.Count).Where(i => driveGroup[i] <= 1).ToArray();
            armHistory = new float[Mathf.CeilToInt(MaxEpisodeSeconds / Time.fixedDeltaTime) + 2, armDrives.Length];
            defaultMaxJointVelocity = dofBodies.Select(b => b.maxJointVelocity).ToArray();

            startPose = new float[drives.Count];
            if (arenaIndex == 0) DumpArm();
            recentVelocity = new float[drives.Count, AccelWindow];
            recentStep = new float[drives.Count, SustainWindow];
            speedHistory = new float[armHistory.GetLength(0), ArmLimits.Groups];
            lastPosition = new float[drives.Count];
            ReachRamp.Init();

            action = new float[drives.Count];
            smoothed = new float[drives.Count];
            restAction = new float[drives.Count];
            if (drives.Count != GetComponent<Unity.MLAgents.Policies.BehaviorParameters>().BrainParameters.ActionSpec.NumContinuousActions)
                Debug.LogError($"[train] arm has {drives.Count} drives but the behavior expects a different action size");
        }

        Vector3 Local(Vector3 world) => world - transform.position;
        Vector3 Grip => (palm.position + thumbTip.position + middleTip.position) / 3f;
        float BallHeightAboveRest => Local(ball.position).y - ballRadius - pedTop.y;

        public override void OnEpisodeBegin()
        {
            var p = Academy.Instance.EnvironmentParameters;
            lesson = Mathf.RoundToInt(p.GetWithDefault("lesson", 0f));
            // eval_target >= 0: that held-out target; -2: every arena cycles through the whole held-out set.
            int e = Mathf.RoundToInt(p.GetWithDefault("eval_target", -1f));
            evalTarget = e == -2 ? (arenaIndex + evalEpisodes++ * arenaCount) % TargetSampler.HeldOutCount : e;
            if (evalTarget >= 0) lesson = TargetSampler.FinalLesson;
            ArmLimits.Read(p, driveGroup, torqueCap, speedCap);
            // joint_lo_<i> / joint_hi_<i> (degrees): a narrower physical range for drive i, e.g. a human wrist. Actions still map
            // over the prefab's range (a fine-tuned policy keeps its meaning); a target past the new limit presses on it.
            for (int i = 0; i < drives.Count; i++)
            {
                var (b, axis, lo, hi) = drives[i];
                float l = p.GetWithDefault("joint_lo_" + i, lo), h = p.GetWithDefault("joint_hi_" + i, hi);
                var d = axis == 0 ? b.xDrive : axis == 1 ? b.yDrive : b.zDrive;
                if (d.lowerLimit == l && d.upperLimit == h) continue;
                d.lowerLimit = l; d.upperLimit = h;
                if (axis == 0) b.xDrive = d; else if (axis == 1) b.yDrive = d; else b.zDrive = d;
            }
            // arm_gravity 0: a counterbalanced arm (its links feel no gravity; the ball still does), so the capped torque goes
            // into moving the arm and the ball rather than holding the arm up.
            bool armGravity = p.GetWithDefault("arm_gravity", 0f) > 0.5f;
            foreach (var b in bodies) b.useGravity = armGravity;
            float enforce = p.GetWithDefault("speed_enforce", 1f);
            enforceSpeed = enforce > 0.5f && torqueCap.Any(t => t > 0f);
            governor = enforce > 1.5f && enforceSpeed;   // speed_enforce 2: the in-step governor too (it costs the PR #8 policy its grasp)
            predictiveClamp = p.GetWithDefault("speed_predict", 0f) > 0.5f;   // speed_predict 1: the clamp leaves room for the next step's gain
            for (int k = 0; k < dofBodies.Length; k++)
            {
                float cap = float.MaxValue;
                for (int i = 0; i < drives.Count; i++) if (drives[i].body == dofBodies[k]) cap = Mathf.Min(cap, speedCap[i]);
                dofBodies[k].maxJointVelocity = enforceSpeed ? cap : defaultMaxJointVelocity[k];
            }
            driveMode = Mathf.RoundToInt(p.GetWithDefault("drive_mode", 1f));
            servoGain = p.GetWithDefault("servo_gain", 10f);
            governorBand = p.GetWithDefault("governor_band", 0.95f);
            traceSteps = arenaIndex == 0 && evalEpisodes <= 1 ? Mathf.RoundToInt(p.GetWithDefault("trace_steps", 0f)) : 0;
            // Contact onset: PhysX pushes a penetrating 0.05-mass finger link out of the 2-mass ball at whatever speed it takes
            // (one step at ~2000 deg/s). Bounding the depenetration speed of the hand links and the ball removes those kicks.
            float depen = p.GetWithDefault("depenetration", 0f); // 0: PhysX default (a bound did not remove the kicks)
            if (depen != lastDepen)
            {
                lastDepen = depen;
                float d = depen > 0f ? depen : 1e10f;
                foreach (var b in bodies) b.maxDepenetrationVelocity = d;
                ball.maxDepenetrationVelocity = d;
            }
            EpisodeSeconds = Mathf.Clamp(p.GetWithDefault("episode_seconds", 10f), 2f, MaxEpisodeSeconds);
            poseRelative = p.GetWithDefault("pose_relative", 0f) > 0.5f;   // off: the run that learned to grasp (windup05 -> windup08) used absolute targets
            touchShaping = p.GetWithDefault("touch_shaping", 0.06f);
            startHeld = p.GetWithDefault("start_held", 0f) > 0.5f;
            heldClose = Mathf.Clamp(p.GetWithDefault("held_close", -1f), -1f, 1f);
            actionRateCost = p.GetWithDefault("action_rate_cost", 1e-4f);
            effortCost = p.GetWithDefault("effort_cost", 0f);
            sequenceBonus = p.GetWithDefault("sequence_bonus", 0f);
            smoothing = Mathf.Clamp(p.GetWithDefault("action_smoothing", 0.25f), 0.01f, 1f); // per physics step: 0.25 ~ 13 ms, 0.05 ~ 80 ms
            windupBonus = p.GetWithDefault("windup_bonus", 0f);
            windupBonusEnd = p.GetWithDefault("windup_bonus_end", 60f);
            probe = p.GetWithDefault("probe", 0f) > 0.5f;
            probeSeconds = p.GetWithDefault("probe_seconds", 2.5f);
            TargetSampler.Configure(p);
            pedTop = new Vector3(p.GetWithDefault("ped_x", PedestalTop.x), p.GetWithDefault("ped_y", PedestalTop.y), p.GetWithDefault("ped_z", PedestalTop.z));
            float pedH = pedTop.y - FloorY;
            pedObject.transform.localPosition = new Vector3(pedTop.x, FloorY + pedH / 2, pedTop.z);
            pedObject.transform.localScale = new Vector3(2 * PedestalHalf, pedH, 2 * PedestalHalf);
            for (int i = 0; i < drives.Count; i++) startPose[i] = Mathf.Clamp(p.GetWithDefault("pose_" + i, i < DefaultPose.Length ? DefaultPose[i] : 0f), -1f, 1f);
            if (probePalm) { probePalm.mass = probePalmMass; probePalm.inertiaTensor = probePalmInertia; probePalm = null; }
            ball.isKinematic = false;
            foreach (var c in handColliders) Physics.IgnoreCollision(c, ball.GetComponent<Collider>(), probe);
            Physics.IgnoreCollision(pedestal, ball.GetComponent<Collider>(), probe);

            // Arm: start pose (rest = straight out, unless pose_<i> says otherwise), a little noise in the drive targets.
            arm.boneToTransform[0].GetComponent<ArticulationBody>().TeleportRoot(transform.position, Quaternion.identity);
            foreach (var b in bodies)
            {
                if (b.isRoot) continue;
                b.jointPosition = Zero(b.dofCount);
                b.jointVelocity = Zero(b.dofCount);
                b.velocity = Vector3.zero;
                b.angularVelocity = Vector3.zero;
            }
            for (int i = 0; i < drives.Count; i++)
            {
                var (_, _, lo, hi) = drives[i];
                float rest = lo < 0f && hi > 0f ? 0f : Mathf.Clamp((0f - (lo + hi) / 2f) / ((hi - lo) / 2f), -1f, 1f);
                if (startPose[i] != 0f) rest = startPose[i];
                restAction[i] = rest;
                smoothed[i] = action[i] = Mathf.Clamp(rest + (evalTarget >= 0 || probe ? 0f : Random.Range(-0.05f, 0.05f)), -1f, 1f);
            }
            SetJointPositions(smoothed);
            ApplyDrives();

            // Ball at rest on the pedestal.
            var rng = evalTarget >= 0 ? new System.Random(1000 + 97 * evalTarget + arenaIndex) : null;
            float R(float a, float b) => rng == null ? Random.Range(a, b) : a + (float)rng.NextDouble() * (b - a);
            float m = PedestalHalf - ballRadius * 0.6f;
            ball.position = transform.position + pedTop + new Vector3(R(-m, m), ballRadius + 0.01f, R(-m, m));
            ball.rotation = Quaternion.identity;
            ball.velocity = Vector3.zero;
            ball.angularVelocity = Vector3.zero;
            var script = ball.GetComponent<ProjectileScript>();
            if (script) { script.hitTarget = false; script.onHand = script.onArm = false; script.nOnHand = 0; }
            contacts.Clear();

            frontierTarget = false;
            if (evalTarget >= 0) (targetPos, targetRadius) = TargetSampler.HeldOut(evalTarget);
            else (targetPos, targetRadius, frontierTarget) = TargetSampler.Sample(lesson, ReachRamp.Reach);
            targetDistance = new Vector2(targetPos.x, targetPos.z).magnitude;
            target.transform.position = transform.position + targetPos;
            target.transform.localScale = Vector3.one * targetRadius;
            target.SetActive(lesson >= 2);
            targetBase = targetPos;
            targetMotion = p.GetWithDefault("target_motion", 0f);
            targetPeriod = Mathf.Max(0.5f, p.GetWithDefault("target_period", 6f));
            targetPhase = R(0f, 2f * Mathf.PI);

            steps = graspSteps = heldSteps = liftSteps = noContactSteps = 0;
            everGrasped = everLifted = thrown = hit = false;
            minGap = float.MaxValue;
            maxThrowDistance = 0f;
            historyLength = 0; liftStep = -1;
            System.Array.Clear(peakSpeed, 0, peakSpeed.Length); System.Array.Clear(peakAccel, 0, peakAccel.Length);
            System.Array.Clear(peakTorque, 0, peakTorque.Length); System.Array.Clear(share, 0, share.Length);
            System.Array.Clear(heldShare, 0, heldShare.Length);
            System.Array.Clear(recentVelocity, 0, recentVelocity.Length);
            System.Array.Clear(recentStep, 0, recentStep.Length); System.Array.Clear(windowPeak, 0, windowPeak.Length);
            System.Array.Clear(governed, 0, governed.Length); System.Array.Clear(previousVelocity, 0, previousVelocity.Length); System.Array.Clear(peakSustained, 0, peakSustained.Length);
            for (int i = 0; i < drives.Count; i++) lastPosition[i] = JointValue(drives[i].body.jointPosition, drives[i].body, drives[i].axis);
            heldSpeed = releaseSpeed = backswing = windupTime = forwardSwing = 0f;
            System.Array.Clear(overCap, 0, overCap.Length); System.Array.Clear(peakStep, 0, peakStep.Length);
            System.Array.Clear(peakContact, 0, peakContact.Length); System.Array.Clear(peakPushing, 0, peakPushing.Length);
            probeBest = probeBestHigh = probeBestSpeed = -1f; probeBestStep = probeHistoryAtBest = 0;
            System.Array.Clear(probePeakSpeed, 0, probePeakSpeed.Length);
            potential = Potential();
        }

        static ArticulationReducedSpace Zero(int dofs) =>
            dofs == 1 ? new ArticulationReducedSpace(0f) : dofs == 2 ? new ArticulationReducedSpace(0f, 0f) : new ArticulationReducedSpace(0f, 0f, 0f);

        int HandContacts(out bool thumbPalm)
        {
            int n = 0; thumbPalm = false;
            foreach (var c in contacts.Touching)
                if (handColliders.Contains(c)) { n++; if (thumbOrPalm.Contains(c)) thumbPalm = true; }
            return n;
        }

        bool Grasped => HandContacts(out bool tp) >= 2 && tp;
        bool OnPedestal => contacts.Touching.Contains(pedestal);

        float Potential()
        {
            float reach = 1f - Mathf.Clamp01((Grip - ball.position).magnitude / 25f);
            int n = HandContacts(out bool tp);
            bool g = n >= 2 && tp;
            float lift = g && !OnPedestal ? Mathf.Clamp01(BallHeightAboveRest / 4f) : 0f;
            // touch: each finger on the ball pays a little before the pinch does. With the counterbalanced arm the hand
            // hovers at the ball and never falls onto it, so without this the grasp has nothing to climb (PR #8: no touch term).
            float touch = touchShaping * Mathf.Min(n, 3) / 3f;
            return 0.2f * reach + touch + 0.1f * (g ? 1f : 0f) + 0.1f * lift;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            Vector3 shoulder = transform.position;
            foreach (var t in stateBones)
            {
                sensor.AddObservation((t.position - shoulder) / Scale);
                var ab = t.GetComponent<ArticulationBody>();
                sensor.AddObservation(ab.velocity / Scale);
            }
            foreach (var (body, axis, _, _) in drives) sensor.AddObservation(JointValue(body.jointPosition, body, axis));
            foreach (var (body, axis, _, _) in drives) sensor.AddObservation(0.1f * JointValue(body.jointVelocity, body, axis));
            sensor.AddObservation(smoothed);
            sensor.AddObservation(Local(ball.position) / Scale);
            sensor.AddObservation(ball.velocity / Scale);
            sensor.AddObservation((ball.position - Grip) / Scale);
            sensor.AddObservation(targetPos / Scale);
            sensor.AddObservation(targetRadius / Scale);
            int n = HandContacts(out bool tp);
            sensor.AddObservation(Mathf.Min(n, 4) / 4f);
            sensor.AddObservation(n >= 2 && tp ? 1f : 0f);
            sensor.AddObservation(everLifted ? 1f : 0f);
            sensor.AddObservation(thrown ? 1f : 0f);
            sensor.AddObservation(OnPedestal ? 1f : 0f);
            sensor.AddObservation(steps / (EpisodeSeconds / Time.fixedDeltaTime));
        }

        // Unlocked axes are packed in order (twist=x, swingY=y, swingZ=z) in the reduced coordinate space.
        static float JointValue(ArticulationReducedSpace s, ArticulationBody b, int axis)
        {
            int idx = DofIndex(b, axis);
            return idx < s.dofCount ? s[idx] : 0f;
        }

        static int DofIndex(ArticulationBody b, int axis)
        {
            if (b.jointType == ArticulationJointType.RevoluteJoint) return 0;
            int idx = 0;
            if (axis > 0 && b.twistLock != ArticulationDofLock.LockedMotion) idx++;
            if (axis > 1 && b.swingYLock != ArticulationDofLock.LockedMotion) idx++;
            return idx;
        }

        /// <summary>Puts every joint at the drive target of <paramref name="a"/> (action units), at rest.</summary>
        void SetJointPositions(float[] a)
        {
            foreach (var body in dofBodies)
            {
                var pos = Zero(body.dofCount);
                for (int i = 0; i < drives.Count; i++)
                {
                    var (b, axis, lo, hi) = drives[i];
                    if (b != body) continue;
                    int idx = DofIndex(b, axis);
                    if (idx < pos.dofCount) pos[idx] = DriveTarget(a[i], lo, hi) * Mathf.Deg2Rad;
                }
                body.jointPosition = pos;
                body.jointVelocity = Zero(body.dofCount);
            }
        }

        /// <summary>
        /// The physics-side speed cap: after PhysX's own maxJointVelocity, any joint velocity still past its drive's cap
        /// (contact impulses from the ball, the drive's explicit force limit overshooting on a light link) is set back to the cap.
        /// </summary>
        void ClampJointSpeeds()
        {
            foreach (var body in dofBodies)
            {
                var v = body.jointVelocity;
                bool changed = false;
                for (int i = 0; i < drives.Count; i++)
                {
                    var (b, axis, _, _) = drives[i];
                    if (b != body || torqueCap[i] <= 0f) continue;
                    int idx = DofIndex(b, axis);
                    if (idx >= v.dofCount) continue;
                    // Predictive: PhysX moves the joint with its end-of-step velocity, so a joint clamped to the cap still ran
                    // past it by whatever one step adds (passive whip from the shoulder: the wrist at up to 393 deg/s against
                    // 300 in windup13's throws). Leave room for the gain the last step added in the same direction.
                    float raw = v[idx], s = Mathf.Sign(raw);
                    float limit = speedCap[i];
                    if (predictiveClamp && steps > 1) limit = Mathf.Max(0.5f * speedCap[i], speedCap[i] - Mathf.Max(0f, (raw - clampOut[i]) * s));
                    if (Mathf.Abs(raw) > limit)
                    {
                        v[idx] = s * limit;
                        overCap[driveGroup[i]]++;
                        changed = true;
                    }
                    clampOut[i] = v[idx];
                }
                if (changed) body.jointVelocity = v;
            }
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            var a = actions.ContinuousActions;
            float jerk = 0f;
            for (int i = 0; i < action.Length; i++)
            {
                float v = Mathf.Clamp(a[i], -1f, 1f);
                // A drive with a start pose of its own (pose_<i>, e.g. the shoulder 45 degrees forward) takes actions relative to it:
                // 0 holds the start pose and +-1 still reach the limits. Otherwise a fresh policy (actions near 0) would swing
                // the arm straight out at once, away from the ball. The probe sends absolute targets.
                float s = startPose[i];
                // Held start: the fingers' origin is the closed grip, so a fresh policy (actions near 0) keeps hold of the ball
                // and +1 still opens the hand fully (windup12's first 1M steps: 0 meant half open, and 77% of balls dropped).
                if (startHeld && !probe && driveGroup[i] == ArmLimits.Groups - 1) s = heldClose;
                if (poseRelative && !probe && s != 0f) v = v >= 0f ? s + v * (1f - s) : s + v * (1f + s);
                // Held start: while the hand closes on the pinned ball the whole arm keeps its start pose. An arm that moved
                // then dragged the fingers against a ball that cannot move (wrist and finger spikes of 3000-13000 deg/s, and a
                // third of the balls dropped, in windup11's first evaluation).
                if (startHeld && !probe && steps <= HoldSetupSteps) v = driveGroup[i] == ArmLimits.Groups - 1 && steps > HoldSettleSteps ? heldClose : restAction[i];
                jerk += (v - action[i]) * (v - action[i]);
                action[i] = v;
            }
            AddReward(-actionRateCost * jerk);
        }

        /// <summary>Actions are relative to the rest pose: 0 holds the joint at rest (0 degrees), +1 and -1 reach its limits.</summary>
        static float DriveTarget(float a, float lo, float hi)
        {
            if (lo < 0f && hi > 0f) return a >= 0f ? a * hi : -a * lo;
            return (lo + hi) / 2f + a * (hi - lo) / 2f; // a joint whose range excludes rest: centred mapping
        }

        void ApplyDrives()
        {
            for (int i = 0; i < drives.Count; i++)
            {
                var (body, axis, lo, hi) = drives[i];
                float t = DriveTarget(smoothed[i], lo, hi);
                float f = ArmLimits.OldForceLimit;
                float stiffness = baseStiffness != null ? baseStiffness[i] : 0f, damping = baseDamping != null ? baseDamping[i] : 0f, targetVelocity = 0f;
                if (torqueCap[i] > 0f && driveMode == 1 && baseDamping != null)
                {
                    // Velocity servo: the drive chases a target speed proportional to the angle error and clamped to the cap.
                    // With gain stiffness/damping (10/s for every joint here) and the drive's own damping, this is exactly the
                    // position drive below whenever that target is under the cap. PhysX solves it implicitly, so no link (however
                    // light) is pushed past the cap by its own drive, which the explicit curve could not promise at 240 Hz
                    // (a 0.05-mass finger link reached ~2000 deg/s within one step). The torque-speed curve still limits the force.
                    float pos = JointValue(body.jointPosition, body, axis), v = JointValue(body.jointVelocity, body, axis);
                    float cap = speedCap[i] * Mathf.Rad2Deg;
                    targetVelocity = Mathf.Clamp(servoGain * (t - pos * Mathf.Rad2Deg), -cap, cap);
                    stiffness = 0f;
                    f = v * targetVelocity > 0f ? torqueCap[i] * Mathf.Clamp01(1f - Mathf.Abs(v) / speedCap[i]) : torqueCap[i];
                    f = Mathf.Max(f, 0.05f * torqueCap[i]); // enough to hold a joint still at speed against gravity
                    float vdeg = v * Mathf.Rad2Deg, predicted = vdeg + (vdeg - previousVelocity[i]);
                    previousVelocity[i] = vdeg;
                    bool pushing = targetVelocity * vdeg > 0f && Mathf.Abs(targetVelocity) > Mathf.Abs(vdeg);
                    bool over = Mathf.Abs(vdeg) >= cap, nearing = pushing && Mathf.Abs(predicted) >= governorBand * cap && predicted * vdeg >= 0f;
                    if (governor && (over || nearing))
                    {
                        // Governor: a stiff velocity constraint that PhysX solves within the step. Driving a joint towards the cap,
                        // it may reach the cap but not pass it; a joint already past the cap (passive whip, contacts) is pulled back
                        // to it. Below the cap a joint that is not driven is left alone, so contacts can still stop the fingers on
                        // the ball (a symmetric damper holding the current speed would push them through it).
                        targetVelocity = Mathf.Sign(vdeg) * Mathf.Min(cap, over ? cap : Mathf.Abs(targetVelocity));
                        damping = GovernorDamping; f = GovernorForce;
                        governed[driveGroup[i]]++;
                    }
                    lastForce[i] = f; lastTargetVelocity[i] = targetVelocity;
                }
                else if (torqueCap[i] > 0f)
                {
                    // Torque-speed curve: driving a joint further along its motion weakens linearly to nothing at the speed cap;
                    // braking keeps the full torque.
                    float v = JointValue(body.jointVelocity, body, axis), err = t * Mathf.Deg2Rad - JointValue(body.jointPosition, body, axis);
                    f = v * err > 0f ? torqueCap[i] * Mathf.Clamp01(1f - Mathf.Abs(v) / speedCap[i]) : torqueCap[i];
                    // Past the cap (gravity or the other joints can push it there), hold the current angle: the drive brakes.
                    if (v * err > 0f && Mathf.Abs(v) > speedCap[i]) { t = JointValue(body.jointPosition, body, axis) * Mathf.Rad2Deg; f = torqueCap[i]; }
                }
                var dr = axis == 0 ? body.xDrive : axis == 1 ? body.yDrive : body.zDrive;
                dr.target = t; dr.forceLimit = f;
                if (baseDamping != null) { dr.stiffness = stiffness; dr.damping = damping; dr.targetVelocity = targetVelocity; }
                if (axis == 0) body.xDrive = dr; else if (axis == 1) body.yDrive = dr; else body.zDrive = dr;
            }
        }

        /// <summary>start_held: pin the ball at the grip and close the hand on it; let go of the pin when the setup ends.</summary>
        void HoldSetup()
        {
            if (steps > HoldSetupSteps + 1) return;
            if (steps <= HoldSetupSteps)
            {
                // The hand's transforms only catch up with the reset articulation after its first physics step, so the grip
                // point of step 1 is the last episode's: a ball pinned there jumped across the fingers at step 2 (finger and
                // wrist joints at 9000-13000 deg/s, fingers bent past their limits, half the balls dropped).
                // For the first HoldSettleSteps the ball ignores the hand and the fingers stay open. Then the ball is set just
                // clear of the open hand on its palm side (the grip point of an open hand lies inside it, and contact switched on
                // there blew the hand apart the same way), pinned in the palm's frame, and the fingers close on it.
                bool settle = steps <= HoldSettleSteps;
                var ballCollider = ball.GetComponent<Collider>();
                foreach (var c in handColliders) Physics.IgnoreCollision(c, ballCollider, settle);
                for (int i = 0; i < drives.Count; i++)
                    action[i] = smoothed[i] = driveGroup[i] == ArmLimits.Groups - 1 && !settle ? heldClose : restAction[i];
                ball.collisionDetectionMode = CollisionDetectionMode.Discrete;
                ball.isKinematic = true;
                if (steps == HoldSettleSteps) heldLocal = palm.InverseTransformPoint(ClearOfHand(Grip, ballCollider));
                ball.position = settle ? Grip : palm.TransformPoint(heldLocal);
                ball.velocity = ball.angularVelocity = Vector3.zero;
                return;
            }
            ball.isKinematic = false;
            ball.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            ball.velocity = ball.angularVelocity = Vector3.zero;
        }

        Vector3 heldLocal;   // held start: the ball's place in the palm's frame while the fingers close

        /// <summary>
        /// The nearest point to p along the palm's normal (the side that faces down at the start pose, where the pedestal ball
        /// sits) at which the ball touches none of the hand's colliders.
        /// </summary>
        Vector3 ClearOfHand(Vector3 p, Collider ballCollider)
        {
            Vector3 n = Vector3.Cross(middleTip.position - palm.position, thumbTip.position - palm.position).normalized;
            if (Vector3.Dot(n, Vector3.down) < 0f) n = -n;
            for (int k = 0; k <= 60; k++)
            {
                Vector3 q = p + n * (k * 0.05f * ballRadius);
                bool overlaps = false;
                foreach (var c in handColliders)
                    if (Physics.ComputePenetration(ballCollider, q, ball.rotation, c, c.transform.position, c.transform.rotation, out _, out _))
                    { overlaps = true; break; }
                if (!overlaps) return q + n * 0.02f * ballRadius;
            }
            return p + n * 3f * ballRadius;
        }

        void FixedUpdate()
        {
            if (bodies == null) return;
            if (baseDamping == null)
            {
                baseStiffness = drives.Select(d => (d.axis == 0 ? d.body.xDrive : d.axis == 1 ? d.body.yDrive : d.body.zDrive).stiffness).ToArray();
                baseDamping = drives.Select(d => (d.axis == 0 ? d.body.xDrive : d.axis == 1 ? d.body.yDrive : d.body.zDrive).damping).ToArray();
                if (arenaIndex == 0) Debug.Log("[arm] stiffness " + string.Join(",", baseStiffness) + " damping " + string.Join(",", baseDamping));
            }
            steps++;
            Measure();                               // the raw joint velocities the last physics step left
            Trace();
            if (enforceSpeed) ClampJointSpeeds();
            for (int i = 0; i < smoothed.Length; i++) smoothed[i] += (action[i] - smoothed[i]) * smoothing;
            if (startHeld && !probe) HoldSetup();
            ApplyDrives();
            if (targetMotion > 0f) MoveTarget();
            if (probe) ProbeStep(); else Evaluate();
        }

        void MoveTarget()
        {
            var flat = new Vector3(targetBase.x, 0f, targetBase.z);
            var side = flat.sqrMagnitude > 1e-4f ? Vector3.Cross(Vector3.up, flat.normalized) : Vector3.forward;
            float w = 2f * Mathf.PI * steps * Time.fixedDeltaTime / targetPeriod + targetPhase;
            targetPos = targetBase + side * (targetMotion * Mathf.Sin(w)) + Vector3.up * (0.3f * targetMotion * Mathf.Sin(2f * w));
            target.transform.position = transform.position + targetPos;
        }

        // Runs every physics step, on the state left by the previous step.
        void Evaluate()
        {
            contacts.Refresh();
            int n = HandContacts(out bool tp);
            bool grasped = n >= 2 && tp;
            graspSteps = grasped ? graspSteps + 1 : Mathf.Max(0, graspSteps - 3); // contacts flicker; brief slips only cost a little
            if (graspSteps > 0) everGrasped = true;
            bool lifted = grasped && !OnPedestal && BallHeightAboveRest > 2f;
            liftSteps = lifted ? liftSteps + 1 : Mathf.Max(0, liftSteps - 3);
            if (liftSteps >= 24) everLifted = true;   // 0.1 s off the pedestal, held
            // Held start: the ball starts lifted; letting go at any time after the setup is a throw, however bad.
            if (startHeld && !probe && steps > HoldSetupSteps) everLifted = everGrasped = true;

            if (!thrown && effortCost > 0f)
            {
                // Effort: squared torque as a fraction of each arm joint's cap, the fingers excepted (they only hold the ball).
                float e = 0f; int n2 = 0;
                for (int i = 0; i < drives.Count; i++)
                {
                    if (driveGroup[i] >= ArmLimits.Groups - 1 || torqueCap[i] <= 0f) continue;
                    float f = JointValue(drives[i].body.driveForce, drives[i].body, drives[i].axis) / torqueCap[i];
                    e += f * f; n2++;
                }
                if (n2 > 0) AddReward(-effortCost * e / n2 * Time.fixedDeltaTime);
            }
            if (!thrown)
            {
                // Shaping potential, frozen once the ball is thrown (letting go must not cost reward).
                float phi = Potential();
                AddReward(phi - potential);
                potential = phi;
                noContactSteps = n == 0 ? noContactSteps + 1 : 0;
                if (everLifted && noContactSteps >= 12) // 50 ms without touching the hand after a held lift
                {
                    thrown = true;
                    releasePoint = Local(ball.position);
                    MeasureRelease();
                    Academy.Instance.StatsRecorder.Add("Task/ReleaseSpeed", ball.velocity.magnitude);
                    if (windupBonus > 0f && evalTarget < 0 && lesson >= 2)
                    {
                        // Shaping toward a wind-up, early in the ramp only: a backswing of up to 30 degrees on the arm axis that
                        // then swings forward furthest, scaled by how far it swings forward (full at 60). Anneals to zero by
                        // reach windup_bonus_end, so the final policy is judged on hits alone.
                        float anneal = Mathf.Clamp01((windupBonusEnd - ReachRamp.Reach) / Mathf.Max(1f, windupBonusEnd - ReachRamp.Start));
                        float bonus = windupBonus * anneal * Mathf.Clamp01(backswing / 30f) * Mathf.Clamp01(forwardSwing / 60f);
                        AddReward(bonus);
                        Academy.Instance.StatsRecorder.Add("Task/WindupBonus", bonus);
                        Academy.Instance.StatsRecorder.Add("Task/WindupBonusScale", windupBonus * anneal);
                    }
                    if (sequenceBonus > 0f && evalTarget < 0 && lesson >= 2 && forwardSwing > 20f)
                    {
                        // Proximal to distal: the shoulder's peak speed before the elbow's, the elbow's before the wrist's
                        // (and the shoulder's before the wrist's), as in a human throw. Only for a real forward swing.
                        int ordered = (peakStep[0] <= peakStep[1] ? 1 : 0) + (peakStep[1] <= peakStep[2] ? 1 : 0) + (peakStep[0] <= peakStep[2] ? 1 : 0);
                        AddReward(sequenceBonus * ordered / 3f);
                        Academy.Instance.StatsRecorder.Add("Task/Sequence", ordered / 3f);
                    }
                    if (lesson >= 2)
                    {
                        // Immediate feedback on the throw itself: how close the ball's ballistic flight will pass the target.
                        float predicted = PredictedGap(releasePoint, ball.velocity);
                        Academy.Instance.StatsRecorder.Add("Task/PredictedGap", predicted);
                        AddReward(0.5f * Mathf.Exp(-Mathf.Max(0f, predicted) / 8f));
                    }
                }
            }

            heldSteps = grasped && !OnPedestal ? heldSteps + 1 : Mathf.Max(0, heldSteps - 3);
            if (lesson == 0 && heldSteps >= 72) { Succeed(); return; }    // 0.3 s held off the pedestal
            if (lesson == 1 && liftSteps >= 72 && BallHeightAboveRest > 3f) { Succeed(); return; } // 0.3 s lifted 3 above rest

            Vector3 b = Local(ball.position);
            if (thrown)
            {
                float gap = (b - targetPos).magnitude - targetRadius - ballRadius;
                minGap = Mathf.Min(minGap, gap);
                maxThrowDistance = Mathf.Max(maxThrowDistance, new Vector2(b.x, b.z).magnitude);
                if (gap <= 0f)
                {
                    hit = true;
                    var script = ball.GetComponent<ProjectileScript>();
                    if (script) script.hitTarget = true;
                    AddReward(0.5f);
                    Succeed();
                    return;
                }
            }

            bool landed = b.y - ballRadius < FloorY + 0.05f;
            bool flewAway = b.magnitude > 120f;
            bool timeout = steps * Time.fixedDeltaTime >= EpisodeSeconds;
            if (landed || flewAway || timeout || (thrown && steps * Time.fixedDeltaTime >= EpisodeSeconds))
            {
                if (thrown && lesson >= 2) AddReward(0.25f * Mathf.Exp(-Mathf.Max(0f, minGap) / 8f));
                if (landed && !thrown) AddReward(-potential - 0.1f); // dropped: hand back the shaping (terminal potential 0) and a little more
                bool heldOn = timeout && !landed && !thrown && lesson >= 2;
                if (heldOn) AddReward(-potential); // throw lessons: holding on to the end is worth nothing; any throw beats it
                Finish(false, timeout && !landed && !heldOn);
            }
        }

        /// <summary>
        /// Every physics step until the release: peak joint speed (deg/s), acceleration (deg/s^2, the velocity change over
        /// 1/30 s, so contact impulses do not dominate it) and drive torque per joint group,
        /// the shoulder and elbow angles since the lift (for the wind-up), and, while the hand still touches the ball,
        /// how much of the ball's velocity each joint produces: joint j turning at w_j (relative to its parent) moves the ball
        /// at w_j x (ball - anchor_j), which is the velocity at the ball of the link it moves minus that of its parent link;
        /// the share is that velocity along the ball's direction over its speed, so the four shares sum to 1.
        /// The fingers' share is the ball's own velocity minus the palm's at the ball (finger joints, rolling and slip).
        /// </summary>
        void Measure()
        {
            if (thrown) return;
            float dt = Time.fixedDeltaTime;
            for (int i = 0; i < drives.Count; i++)
            {
                var (body, axis, _, _) = drives[i];
                int g = driveGroup[i];
                float v = JointValue(body.jointVelocity, body, axis);
                if (Mathf.Abs(v) * Mathf.Rad2Deg > peakSpeed[g])
                {
                    peakSpeed[g] = Mathf.Abs(v) * Mathf.Rad2Deg;
                    peakStep[g] = steps;
                    peakContact[g] = HandContacts(out _) > 0;
                    float target = (axis == 0 ? body.xDrive : axis == 1 ? body.yDrive : body.zDrive).target * Mathf.Deg2Rad;
                    peakPushing[g] = v * (target - JointValue(body.jointPosition, body, axis)) > 0f;
                }
                int slot = steps % AccelWindow;
                if (steps > AccelWindow) peakAccel[g] = Mathf.Max(peakAccel[g], Mathf.Abs(v - recentVelocity[i, slot]) / (AccelWindow * dt) * Mathf.Rad2Deg);
                recentVelocity[i, slot] = v;
                // Sustained speed: the angle actually covered over the last 1/60 s, so a one-step contact kick counts for a quarter.
                float pos = JointValue(body.jointPosition, body, axis);
                recentStep[i, steps % SustainWindow] = pos - lastPosition[i];
                lastPosition[i] = pos;
                if (steps > SustainWindow + 1)
                {
                    float sum = 0f;
                    for (int k = 0; k < SustainWindow; k++) sum += recentStep[i, k];
                    peakSustained[g] = Mathf.Max(peakSustained[g], Mathf.Abs(sum) / (SustainWindow * dt) * Mathf.Rad2Deg);
                }
                peakTorque[g] = Mathf.Max(peakTorque[g], Mathf.Abs(JointValue(body.driveForce, body, axis)));
            }
            if (everLifted)
            {
                if (liftStep < 0) liftStep = steps;
                if (historyLength < armHistory.GetLength(0))
                {
                    for (int k = 0; k < armDrives.Length; k++)
                    {
                        var (body, axis, _, _) = drives[armDrives[k]];
                        armHistory[historyLength, k] = JointValue(body.jointPosition, body, axis);
                    }
                    for (int g = 0; g < ArmLimits.Groups; g++) speedHistory[historyLength, g] = 0f;
                    for (int i = 0; i < drives.Count; i++)
                    {
                        float v = Mathf.Abs(JointValue(drives[i].body.jointVelocity, drives[i].body, drives[i].axis)) * Mathf.Rad2Deg;
                        speedHistory[historyLength, driveGroup[i]] = Mathf.Max(speedHistory[historyLength, driveGroup[i]], v);
                    }
                    historyLength++;
                }
            }
            if (HandContacts(out _) > 0)
            {
                ComputeShares();
                historyMark = historyLength;
                releaseStep = steps;
            }
        }

        void ComputeShares()
        {
            Vector3 vb = ball.velocity;
            heldSpeed = vb.magnitude;
            Vector3 dir = heldSpeed > 1e-3f ? vb / heldSpeed : Vector3.zero;
            // Telescoping: the velocity at the ball's centre of each link minus that of its parent link (the root is fixed).
            Vector3 inner = Vector3.zero;
            for (int g = 0; g < 4; g++)
            {
                Vector3 v = g < 3 ? groupBody[g].GetPointVelocity(ball.position) : vb;
                heldShare[g] = heldSpeed > 1e-3f ? Vector3.Dot(v - inner, dir) / heldSpeed : 0f;
                inner = v;
            }
        }
        int releaseStep;
        float[] lastForce = new float[32], lastTargetVelocity = new float[32];

        /// <summary>trace_steps: per step, the fastest wrist and finger drive (raw speed, its servo target, force limit, contact).</summary>
        void Trace()
        {
            if (traceSteps <= 0 || steps > traceSteps) return;
            var sb = new System.Text.StringBuilder($"[trace] {steps} c{HandContacts(out _)}");
            for (int g = 2; g < 4; g++)
            {
                int best = -1; float bv = 0f;
                for (int i = 0; i < drives.Count; i++)
                {
                    if (driveGroup[i] != g) continue;
                    float v = JointValue(drives[i].body.jointVelocity, drives[i].body, drives[i].axis) * Mathf.Rad2Deg;
                    if (best < 0 || Mathf.Abs(v) > Mathf.Abs(bv)) { best = i; bv = v; }
                }
                float pdeg = JointValue(drives[best].body.jointPosition, drives[best].body, drives[best].axis) * Mathf.Rad2Deg;
                sb.Append(System.FormattableString.Invariant($" {ArmLimits.Names[g]} d{best} v {bv:0} vt {lastTargetVelocity[best]:0} f {lastForce[best]:0} p {pdeg:0.0} lim {drives[best].lo:0}/{drives[best].hi:0}"));
            }
            Debug.Log(sb.ToString());
        }
        int historyMark; // history length at the last step the hand touched the ball

        /// <summary>
        /// At the release (the last step the hand touched the ball): the joint shares, and the wind-up. For each shoulder and
        /// elbow axis, the swing direction is the sign of its velocity at release; the backswing is how far that axis first
        /// went the other way from its angle at the lift (degrees), and the wind-up time runs from that turning point to the
        /// release. Reported for the axis that swings furthest forward.
        /// </summary>
        void MeasureRelease()
        {
            releaseSpeed = heldSpeed;
            System.Array.Copy(heldShare, share, share.Length);
            int end = Mathf.Min(historyMark, historyLength) - 1;
            backswing = windupTime = forwardSwing = 0f;
            if (end < 2) return;
            float bestSwing = -1f;
            for (int k = 0; k < armDrives.Length; k++)
            {
                float s = Mathf.Sign(armHistory[end, k] - armHistory[end - 1, k]);
                int turn = 0;
                for (int t = 1; t <= end; t++) if (s * armHistory[t, k] < s * armHistory[turn, k]) turn = t;
                float swing = s * (armHistory[end, k] - armHistory[turn, k]) * Mathf.Rad2Deg;
                if (swing > bestSwing)
                {
                    bestSwing = swing;
                    forwardSwing = swing;
                    for (int g = 0; g < ArmLimits.Groups; g++)
                    {
                        windowPeak[g] = 0f;
                        for (int t = turn; t < historyLength; t++) windowPeak[g] = Mathf.Max(windowPeak[g], speedHistory[t, g]);
                    }
                    backswing = Mathf.Max(0f, s * (armHistory[0, k] - armHistory[turn, k]) * Mathf.Rad2Deg);
                    windupTime = (end - turn) * Time.fixedDeltaTime;
                }
            }
        }

        /// <summary>Closest approach (surface to surface) of a drag-free flight from p with velocity v to the target, until the floor.</summary>
        float PredictedGap(Vector3 p, Vector3 v)
        {
            float best = float.MaxValue, dt = 1f / 60f;
            Vector3 g = Physics.gravity;
            for (float t = 0f; t < 6f; t += dt)
            {
                Vector3 x = p + v * t + 0.5f * g * t * t;
                best = Mathf.Min(best, (x - targetPos).magnitude - targetRadius - ballRadius);
                if (x.y - ballRadius < FloorY) break;
            }
            return best;
        }

        void Succeed()
        {
            AddReward(1f);
            Finish(true, false);
        }

        void Finish(bool success, bool interrupted)
        {
            var stats = Academy.Instance.StatsRecorder;
            stats.Add("Task/Grasp", everGrasped ? 1f : 0f);
            stats.Add("Task/Lift", everLifted ? 1f : 0f);
            stats.Add("Task/Throw", thrown ? 1f : 0f);
            stats.Add("Task/Hit", hit ? 1f : 0f);
            stats.Add("Task/Success", success ? 1f : 0f);
            stats.Add("Task/Lesson", lesson);
            if (lesson == TargetSampler.FinalLesson && evalTarget < 0)
            {
                stats.Add("Task/Reach", ReachRamp.Reach);
                stats.Add("Task/TargetDistance", targetDistance);
                if (frontierTarget) { stats.Add("Task/FrontierHit", hit ? 1f : 0f); ReachRamp.Record(hit); }
            }
            if (thrown)
            {
                stats.Add("Task/ThrowDistance", maxThrowDistance);
                stats.Add("Task/Backswing", backswing);
                stats.Add("Task/WindupTime", windupTime);
                for (int g = 0; g < ArmLimits.Groups; g++)
                {
                    stats.Add($"Task/Share/{ArmLimits.Names[g]}", share[g]);
                    stats.Add($"Task/PeakSpeed/{ArmLimits.Names[g]}", peakSpeed[g]);
                }
            }
            if (evalTarget >= 0)
            {
                string G(float[] v, string f) => string.Join(",", v.Select(x => x.ToString(f, System.Globalization.CultureInfo.InvariantCulture)));
                Debug.Log($"[eval] target {evalTarget} hit {(hit ? 1 : 0)} grasp {(everGrasped ? 1 : 0)} lift {(everLifted ? 1 : 0)} throw {(thrown ? 1 : 0)} gap {(thrown ? minGap : -1f):0.00}"
                    + System.FormattableString.Invariant($" dist {targetDistance:0.0} thrown {maxThrowDistance:0.00} vrel {releaseSpeed:0.00} backswing {backswing:0.0} windup {windupTime:0.000}")
                    + $" share {G(share, "0.000")} speed {G(peakSpeed, "0")} accel {G(peakAccel, "0")} torque {G(peakTorque, "0")}"
                    + $" pkc {G(peakContact.Select(c => c ? 1f : 0f).ToArray(), "0")} pkp {G(peakPushing.Select(c => c ? 1f : 0f).ToArray(), "0")}"
                    + $" pkt {G(peakStep.Select(t => (float)(t - releaseStep)).ToArray(), "0")} over {G(overCap.Select(c => (float)c).ToArray(), "0")}"
                    + System.FormattableString.Invariant($" fwd {forwardSwing:0.0}") + $" sspeed {G(peakSustained, "0")} wspeed {G(windowPeak, "0")} gov {G(governed.Select(c => (float)c).ToArray(), "0")}");
            }
            if (interrupted) EpisodeInterrupted(); else EndEpisode();
        }

        /// <summary>Probe mode, every physics step: fix the ball in the hand, then track the farthest carry of a release now.</summary>
        void ProbeStep()
        {
            if (steps == 2)
            {
                // The ball rides as a load on the palm (its mass added there, the inertia scaled with it; moving the centre of mass made it unstable),
                // and the carry follows the grip point. A FixedJoint from the ball to the articulation was unstable (100+ m/s).
                var pb = palm.GetComponent<ArticulationBody>();
                probePalm = pb; probePalmMass = pb.mass; probePalmCom = pb.centerOfMass;
                probeOffset = palm.InverseTransformPoint(Grip);
                pb.mass = probePalmMass + ball.mass;
                probePalmInertia = pb.inertiaTensor; pb.inertiaTensor = probePalmInertia * (pb.mass / probePalmMass);
                probeShoulder = arm.boneToTransform[1].position; probeDrift = 0f; probeGlitches = 0;
                ball.isKinematic = true; ball.position = transform.position + Vector3.down * 40f;
                everLifted = true; everGrasped = true;
                if (arenaIndex == 0)
                    Debug.Log(System.FormattableString.Invariant($"[pose] grip {Local(Grip).x:0.00} {Local(Grip).y:0.00} {Local(Grip).z:0.00} palm {Local(palm.position).x:0.00} {Local(palm.position).y:0.00} {Local(palm.position).z:0.00}"));
                return;
            }
            if (steps > 3)
            {
                // Velocity as the ball's displacement over the last 4 steps (1/60 s), so a one-step solver spike can't count.
                Vector3 grip = palm.TransformPoint(probeOffset), pos = Local(grip);
                probeDrift = Mathf.Max(probeDrift, (arm.boneToTransform[1].position - probeShoulder).magnitude);
                probeTrail[steps % 5] = grip;
                if (steps < 9) return;
                Vector3 moved = (grip - probeTrail[(steps - 4) % 5]) / (4f * Time.fixedDeltaTime);
                // The carry uses the palm body's own velocity at the grip point; a step where it disagrees with the grip's
                // actual displacement over the last 1/60 s (a spherical joint flipping at its limit teleports the hand) is skipped.
                Vector3 vel = probePalm.GetPointVelocity(grip);
                bool glitch = (moved - vel).magnitude > 0.25f * Mathf.Max(vel.magnitude, 4f);
                if (glitch) probeGlitches++;
                float low = glitch ? -1f : Carry(pos, vel, TargetSampler.MinY), high = glitch ? -1f : Carry(pos, vel, 4f);
                if (traceSteps > 0 && steps % 4 == 0)
                {
                    var jp = new System.Text.StringBuilder();
                    for (int i = 0; i < 6; i++) jp.Append(System.FormattableString.Invariant($" {JointValue(drives[i].body.jointPosition, drives[i].body, drives[i].axis) * Mathf.Rad2Deg:0}"));
                    Vector3 root = Local(arm.boneToTransform[0].position), sh = Local(arm.boneToTransform[1].position);
                    Debug.Log(System.FormattableString.Invariant($"[ptrace] {steps} grip {pos.x:0.0} {pos.y:0.0} {pos.z:0.0} v {vel.magnitude:0.0} moved {moved.magnitude:0.0} carry {low:0} root {root.x:0.0} {root.y:0.0} {root.z:0.0} upper {sh.x:0.0} {sh.y:0.0} {sh.z:0.0} joints") + jp);
                }
                if (low > probeBest)
                {
                    probeBest = low; probeBestStep = steps; probeHistoryAtBest = historyLength; probeBestSpeed = vel.magnitude;
                    System.Array.Copy(peakSpeed, probePeakSpeed, peakSpeed.Length);
                    ComputeShares(); System.Array.Copy(heldShare, share, share.Length);
                }
                probeBestHigh = Mathf.Max(probeBestHigh, high);
            }
            if (steps * Time.fixedDeltaTime >= probeSeconds)
            {
                historyMark = probeHistoryAtBest;
                var keep = (float[])share.Clone();
                MeasureRelease();
                System.Array.Copy(keep, share, share.Length);
                string G(float[] v, string f) => string.Join(",", v.Select(x => x.ToString(f, System.Globalization.CultureInfo.InvariantCulture)));
                Debug.Log(System.FormattableString.Invariant($"[probe] arena {arenaIndex} carry {probeBest:0.00} high {probeBestHigh:0.00} speed {probeBestSpeed:0.00} at {probeBestStep * Time.fixedDeltaTime:0.000} backswing {backswing:0.0} windup {windupTime:0.000} fwd {forwardSwing:0.0}")
                    + $" share {G(share, "0.000")} speed {G(probePeakSpeed, "0")} over {G(overCap.Select(c => (float)c).ToArray(), "0")} act {G(action, "0.0000")}" + System.FormattableString.Invariant($" drift {probeDrift:0.00} glitch {probeGlitches}"));
                EndEpisode();
            }
        }

        /// <summary>
        /// Horizontal distance from the shoulder at which a drag-free flight from p with velocity v comes down through height y,
        /// or -1 if it never does or lands outside the target azimuths (60 degrees either side of -z, plus 5).
        /// </summary>
        static float Carry(Vector3 p, Vector3 v, float y)
        {
            float g = -Physics.gravity.y, a = 0.5f * g, b = -v.y, c = y - p.y; // a t^2 + b t + c = 0
            float disc = b * b - 4f * a * c;
            if (disc < 0f) return -1f;
            float t = (-b + Mathf.Sqrt(disc)) / (2f * a);
            if (t <= 0f) return -1f;
            float x = p.x + v.x * t, z = p.z + v.z * t;
            float az = Mathf.Atan2(x, -z) * Mathf.Rad2Deg;
            if (Mathf.Abs(az) > TargetSampler.AzimuthSpread + 5f) return -1f;
            return new Vector2(x, z).magnitude;
        }

        void DumpArm()
        {
            var inv = System.Globalization.CultureInfo.InvariantCulture;
            for (int i = 0; i < drives.Count; i++)
            {
                var (b, axis, lo, hi) = drives[i];
                Debug.Log(string.Format(inv, "[arm] drive {0} {1} axis {2} limits {3:0} {4:0} group {5} mass {6:0.00} stiffness {7:0} damping {8:0} maxJointVelocity {9:0.0}",
                    i, b.name, axis, lo, hi, ArmLimits.Names[driveGroup[i]], b.mass, b.xDrive.stiffness, b.xDrive.damping, b.maxJointVelocity));
            }
            for (int k = 0; k < SimCharController.nbones; k++)
            {
                var t = arm.boneToTransform[k];
                var lp = t.position - transform.position;
                Debug.Log(string.Format(inv, "[arm] bone {0} {1} at {2:0.00} {3:0.00} {4:0.00}", k, t.name, lp.x, lp.y, lp.z));
            }
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var a = actionsOut.ContinuousActions;
            for (int i = 0; i < a.Length; i++) a[i] = action != null && i < action.Length ? action[i] : 0f;
        }
    }

    /// <summary>
    /// How strong the arm is. "arm_limits" 1 (the default, what training uses) caps each joint group's drive torque, and its
    /// speed through a torque-speed curve (ThrowAgent.ApplyDrives): the torque pushing a joint along its motion falls linearly
    /// to zero at the speed cap, so the joint can only build speed gradually and never past the cap by its own drive.
    /// 0 restores the arm of PR #6 (force limit 100000, no speed cap), for comparing old policies.
    /// Torques are in the drive's own units (PR #6's policy peaks at about 12000 shoulder, 6000 elbow, 4000 wrist, 1000 fingers);
    /// speeds in degrees per second. Any value can be overridden for sweeps: environment parameters torque_wrist, speed_elbow, ...
    /// The torque-speed curve alone leaked (PR #8: fingers to 2000 deg/s against 600): it is computed from the previous step's
    /// speed, so a light link overshoots within one step, and contact impulses from the ball are not limited at all.
    /// The caps are now enforced in physics as well (ThrowAgent.ClampJointSpeeds, speed_enforce).
    /// </summary>
    public static class ArmLimits
    {
        public const int Groups = 4;
        public static readonly string[] Names = { "shoulder", "elbow", "wrist", "fingers" };
        //                                          shoulder elbow  wrist  fingers
        // Shoulder and elbow were 4000 and 2000 in a first round (run windup02): a direct swing still threw 65 out, so no wind-up
        // was needed; at half that, the same direct swing falls short of the far band (mean 47 on targets 52 out).
        public static readonly float[] Torque =   { 500f,   250f,   100f,  150f };   // x0.25 of PR #8 (2000, 1000, 400, 150, with arm gravity); x0.2 could not lift the 2-kg ball
        public static readonly float[] Speed =    { 540f,   540f,   300f,  600f };
        public const float OldForceLimit = 100000f;

        public static int GroupOf(SimCharController arm, ArticulationBody body)
        {
            for (int b = 1; b <= 3; b++) if (arm.boneToTransform[b] == body.transform) return b - 1;
            return 3;
        }

        /// <summary>Per drive: torque cap (0 for the uncapped arm) and speed cap in radians per second.</summary>
        public static void Read(EnvironmentParameters p, int[] group, float[] torque, float[] speed)
        {
            bool on = p.GetWithDefault("arm_limits", 1f) > 0.5f;
            for (int i = 0; i < group.Length; i++)
            {
                string n = Names[group[i]];
                torque[i] = on ? p.GetWithDefault("torque_" + n, Torque[group[i]]) : 0f;
                speed[i] = Mathf.Max(1f, p.GetWithDefault("speed_" + n, Speed[group[i]])) * Mathf.Deg2Rad;
            }
        }
    }

    /// <summary>
    /// The distance ramp of the reach lesson, per training player: the reach (the farthest target distance) starts at 26
    /// (environment parameter reach_start) and moves out 1.5 at a time, up to reach_max, each time the last 64 frontier throws
    /// (targets within 4 of the reach) hit at least 60%.
    /// Half the targets sit in that frontier band, the other half anywhere from 22 out (or the start minus 4, if lower),
    /// so near throws stay in practice.
    /// It never moves back. A resumed run passes its last reach back in with -reachStart (tools/train/train.sh).
    /// </summary>
    public static class ReachRamp
    {
        public const float Step = 1.5f;
        public static float Threshold = 0.6f;   // reach_threshold
        public const int Window = 64;
        public static float Start = 26f;
        public static float Reach = Start;
        static int episodes, hits;
        static bool ready, started;
        static float resumeAt = -1f;

        public static void Init()
        {
            if (ready) return;
            ready = true;
            var args = System.Environment.GetCommandLineArgs();
            int i = System.Array.IndexOf(args, "-reachStart");
            if (i >= 0 && i + 1 < args.Length && float.TryParse(args[i + 1], System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture, out float r))
                resumeAt = r;
            Reach = Mathf.Max(Start, resumeAt);
        }

        /// <summary>The reach_start parameter; it takes effect only before the ramp has recorded its first throw.</summary>
        public static void SetStart(float start)
        {
            if (started || Mathf.Approximately(start, Start)) return;
            Start = Mathf.Clamp(start, 10f, 200f);
            Reach = Mathf.Max(Start, resumeAt);
        }

        public static void Record(bool hit)
        {
            started = true;
            episodes++;
            if (hit) hits++;
            if (episodes < Window) return;
            if (hits >= Threshold * episodes && Reach < TargetSampler.MaxDistance)
            {
                Reach = Mathf.Min(Reach + Step, TargetSampler.MaxDistance);
                Debug.Log($"[train] reach {Reach:0.0} (frontier hit rate {hits / (float)episodes:0.00})");
            }
            episodes = hits = 0;
        }
    }
}
