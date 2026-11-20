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
    ///   0 grasp (hold it off the pedestal), 1 lift, 2 throw toward a wide zone, 3 near targets, 4 mid targets, 5 far and varied targets.
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
        float targetRadius;

        float[] action, smoothed;
        float smoothing = 0.25f; // per physics step
        int lesson;
        int evalTarget = -1, evalEpisodes;

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

            (targetPos, targetRadius) = evalTarget >= 0 ? TargetSampler.HeldOut(evalTarget) : TargetSampler.Sample(lesson);
            target.transform.position = transform.position + targetPos;
            target.transform.localScale = Vector3.one * targetRadius;
            target.SetActive(lesson >= 2);

            steps = graspSteps = heldSteps = liftSteps = noContactSteps = 0;
            everGrasped = everLifted = thrown = hit = false;
            minGap = float.MaxValue;
            maxThrowDistance = 0f;
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
                if (axis == 0) { var d = body.xDrive; d.target = t; body.xDrive = d; }
                else if (axis == 1) { var d = body.yDrive; d.target = t; body.yDrive = d; }
                else { var d = body.zDrive; d.target = t; body.zDrive = d; }
            }
        }

        void FixedUpdate()
        {
            if (bodies == null) return;
            for (int i = 0; i < smoothed.Length; i++) smoothed[i] += (action[i] - smoothed[i]) * smoothing;
            ApplyDrives();
            steps++;
            Evaluate();
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
            if (thrown) stats.Add("Task/ThrowDistance", maxThrowDistance);
            if (evalTarget >= 0)
                Debug.Log($"[eval] target {evalTarget} hit {(hit ? 1 : 0)} grasp {(everGrasped ? 1 : 0)} lift {(everLifted ? 1 : 0)} throw {(thrown ? 1 : 0)} gap {(thrown ? minGap : -1f):0.00}");
            if (interrupted) EpisodeInterrupted(); else EndEpisode();
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var a = actionsOut.ContinuousActions;
            for (int i = 0; i < a.Length; i++) a[i] = action != null && i < action.Length ? action[i] : 0f;
        }
    }
}
