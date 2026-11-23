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
        public static readonly Vector3 PedestalTop = new Vector3(-14f, -9f, 0f); // ~10 from the moving shoulder joint at x = -8
        public const float PedestalHalf = 2f;
        public const int PhysicsStepsPerDecision = 8; // 240 Hz physics, 30 Hz decisions
        public const float EpisodeSeconds = 8f;
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
        float heldSpeed, releaseSpeed, backswing, windupTime, targetDistance;
        bool frontierTarget;

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
            armHistory = new float[Mathf.CeilToInt(EpisodeSeconds / Time.fixedDeltaTime) + 2, armDrives.Length];
            recentVelocity = new float[drives.Count, AccelWindow];
            ReachRamp.Init();

            action = new float[drives.Count];
            smoothed = new float[drives.Count];
            if (drives.Count != GetComponent<Unity.MLAgents.Policies.BehaviorParameters>().BrainParameters.ActionSpec.NumContinuousActions)
                Debug.LogError($"[train] arm has {drives.Count} drives but the behavior expects a different action size");
        }

        Vector3 Local(Vector3 world) => world - transform.position;
        Vector3 Grip => (palm.position + thumbTip.position + middleTip.position) / 3f;
        float BallHeightAboveRest => Local(ball.position).y - ballRadius - PedestalTop.y;

        public override void OnEpisodeBegin()
        {
            var p = Academy.Instance.EnvironmentParameters;
            lesson = Mathf.RoundToInt(p.GetWithDefault("lesson", 0f));
            // eval_target >= 0: that held-out target; -2: every arena cycles through the whole held-out set.
            int e = Mathf.RoundToInt(p.GetWithDefault("eval_target", -1f));
            evalTarget = e == -2 ? (arenaIndex + evalEpisodes++ * arenaCount) % TargetSampler.HeldOutCount : e;
            if (evalTarget >= 0) lesson = TargetSampler.FinalLesson;
            ArmLimits.Read(p, driveGroup, torqueCap, speedCap);

            // Arm: rest pose (straight out), a little noise in the drive targets.
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
                smoothed[i] = action[i] = Mathf.Clamp(rest + (evalTarget >= 0 ? 0f : Random.Range(-0.05f, 0.05f)), -1f, 1f);
            }
            ApplyDrives();

            // Ball at rest on the pedestal.
            var rng = evalTarget >= 0 ? new System.Random(1000 + 97 * evalTarget + arenaIndex) : null;
            float R(float a, float b) => rng == null ? Random.Range(a, b) : a + (float)rng.NextDouble() * (b - a);
            float m = PedestalHalf - ballRadius * 0.6f;
            ball.position = transform.position + PedestalTop + new Vector3(R(-m, m), ballRadius + 0.01f, R(-m, m));
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
            heldSpeed = releaseSpeed = backswing = windupTime = 0f;
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
            bool g = Grasped;
            float lift = g && !OnPedestal ? Mathf.Clamp01(BallHeightAboveRest / 4f) : 0f;
            return 0.2f * reach + 0.1f * (g ? 1f : 0f) + 0.1f * lift;
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
            int idx = 0;
            var locks = new[] { b.twistLock, b.swingYLock, b.swingZLock };
            if (b.jointType == ArticulationJointType.RevoluteJoint) return s.dofCount > 0 ? s[0] : 0f;
            for (int a = 0; a < axis; a++) if (locks[a] != ArticulationDofLock.LockedMotion) idx++;
            return idx < s.dofCount ? s[idx] : 0f;
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            var a = actions.ContinuousActions;
            float jerk = 0f;
            for (int i = 0; i < action.Length; i++)
            {
                float v = Mathf.Clamp(a[i], -1f, 1f);
                jerk += (v - action[i]) * (v - action[i]);
                action[i] = v;
            }
            AddReward(-0.0001f * jerk);
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
                if (torqueCap[i] > 0f)
                {
                    // Torque-speed curve: driving a joint further along its motion weakens linearly to nothing at the speed cap;
                    // braking keeps the full torque.
                    float v = JointValue(body.jointVelocity, body, axis), err = t * Mathf.Deg2Rad - JointValue(body.jointPosition, body, axis);
                    f = v * err > 0f ? torqueCap[i] * Mathf.Clamp01(1f - Mathf.Abs(v) / speedCap[i]) : torqueCap[i];
                    // Past the cap (gravity or the other joints can push it there), hold the current angle: the drive brakes.
                    if (v * err > 0f && Mathf.Abs(v) > speedCap[i]) { t = JointValue(body.jointPosition, body, axis) * Mathf.Rad2Deg; f = torqueCap[i]; }
                }
                if (axis == 0) { var d = body.xDrive; d.target = t; d.forceLimit = f; body.xDrive = d; }
                else if (axis == 1) { var d = body.yDrive; d.target = t; d.forceLimit = f; body.yDrive = d; }
                else { var d = body.zDrive; d.target = t; d.forceLimit = f; body.zDrive = d; }
            }
        }

        void FixedUpdate()
        {
            if (bodies == null) return;
            for (int i = 0; i < smoothed.Length; i++) smoothed[i] += (action[i] - smoothed[i]) * smoothing;
            ApplyDrives();
            steps++;
            if (targetMotion > 0f) MoveTarget();
            Measure();
            Evaluate();
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
                peakSpeed[g] = Mathf.Max(peakSpeed[g], Mathf.Abs(v) * Mathf.Rad2Deg);
                int slot = steps % AccelWindow;
                if (steps > AccelWindow) peakAccel[g] = Mathf.Max(peakAccel[g], Mathf.Abs(v - recentVelocity[i, slot]) / (AccelWindow * dt) * Mathf.Rad2Deg);
                recentVelocity[i, slot] = v;
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
                    historyLength++;
                }
            }
            if (HandContacts(out _) > 0)
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
                historyMark = historyLength;
            }
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
            backswing = windupTime = 0f;
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
                    + $" share {G(share, "0.000")} speed {G(peakSpeed, "0")} accel {G(peakAccel, "0")} torque {G(peakTorque, "0")}");
            }
            if (interrupted) EpisodeInterrupted(); else EndEpisode();
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
    /// (PhysX's own maxJointVelocity is not used: it did not hold joints near the cap.)
    /// </summary>
    public static class ArmLimits
    {
        public const int Groups = 4;
        public static readonly string[] Names = { "shoulder", "elbow", "wrist", "fingers" };
        //                                          shoulder elbow  wrist  fingers
        // Shoulder and elbow were 4000 and 2000 in a first round (run windup02): a direct swing still threw 65 out, so no wind-up
        // was needed; at half that, the same direct swing falls short of the far band (mean 47 on targets 52 out).
        public static readonly float[] Torque =   { 2000f,  1000f,  400f,  150f };
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
    /// The distance ramp of the reach lesson, per training player: the reach (the farthest target distance) starts at 26 and
    /// moves out 1.5 at a time, up to 56, each time the last 64 frontier throws (targets within 4 of the reach) hit at least 60%.
    /// Half the targets sit in that frontier band, the other half anywhere from 22 out, so near throws stay in practice.
    /// It never moves back. A resumed run passes its last reach back in with -reachStart (tools/train/train.sh).
    /// </summary>
    public static class ReachRamp
    {
        public const float Start = 26f, Step = 1.5f, Threshold = 0.6f;
        public const int Window = 64;
        public static float Reach = Start;
        static int episodes, hits;
        static bool ready;

        public static void Init()
        {
            if (ready) return;
            ready = true;
            var args = System.Environment.GetCommandLineArgs();
            int i = System.Array.IndexOf(args, "-reachStart");
            if (i >= 0 && i + 1 < args.Length && float.TryParse(args[i + 1], System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture, out float r))
                Reach = Mathf.Clamp(r, Start, TargetSampler.MaxDistance);
        }

        public static void Record(bool hit)
        {
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
