using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine;

namespace Training
{
    /// <summary>
    /// Builds N independent arenas at startup (command line: -arenas N, default 8), each an arm, a pedestal, a ball and a target,
    /// far enough apart that nothing crosses over. Sizes the behavior from the arm itself, so the scene needs no hand-kept numbers.
    /// </summary>
    [DefaultExecutionOrder(100)] // after ConfigManager.Awake
    public class ArenaSpawner : MonoBehaviour
    {
        public GameObject armPrefab, ballPrefab, targetPrefab;
        public int arenas = 8;
        public float spacing = 400f;
        public const string Behavior = "Throw";

        void Awake()
        {
            Application.targetFrameRate = -1;
            Time.maximumDeltaTime = 1f / 3f;
            int n = ArgInt("-arenas", arenas);
            int actions = ThrowAgent.CountActions(armPrefab);
            int obs = ThrowAgent.CountObservations(actions);
            Physics.defaultSolverIterations = ConfigManager.Instance.solverIterations;
            Physics.defaultSolverVelocityIterations = ConfigManager.Instance.solverIterations;
            int layer = LayerMask.NameToLayer("test_model");
            Debug.Log($"[train] {n} arenas, {obs} observations, {actions} actions, physics {1f / Time.fixedDeltaTime:0} Hz");
            for (int i = 0; i < n; i++)
            {
                var go = new GameObject($"Arena {i}");
                go.SetActive(false);
                go.transform.position = new Vector3(i * spacing, 0f, 0f);
                var bp = go.AddComponent<BehaviorParameters>();
                bp.BehaviorName = Behavior;
                bp.BrainParameters.VectorObservationSize = obs;
                bp.BrainParameters.NumStackedVectorObservations = 1;
                bp.BrainParameters.ActionSpec = ActionSpec.MakeContinuous(actions);
                var agent = go.AddComponent<ThrowAgent>();
                agent.armPrefab = armPrefab;
                agent.ballPrefab = ballPrefab;
                agent.targetPrefab = targetPrefab;
                agent.armLayer = layer;
                var dr = go.AddComponent<DecisionRequester>();
                dr.DecisionPeriod = ThrowAgent.PhysicsStepsPerDecision;
                dr.TakeActionsBetweenDecisions = true;
                go.SetActive(true);
            }
        }

        static int ArgInt(string name, int fallback)
        {
            var args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, name);
            return i >= 0 && i + 1 < args.Length && int.TryParse(args[i + 1], out int v) ? v : fallback;
        }
    }
}
