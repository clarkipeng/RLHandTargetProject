using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MultiMLAgentsDirector : MonoBehaviour
{
    public int numAgents = 5;
    private int solverIterations = 32;
    public GameObject modelDirector;
    private MLAgentsDirector[] directors;
    public int reportMeanRewardEveryNSteps = 10000;
    private int curStep = 0;
    public int targetFrameRate = -1;
    private float meanReward;
    public int fps = 60;
    public bool projectileTraining = true;
    public float LAUNCH_FREQUENCY = 1f;
    public float LAUNCH_RADIUS = .66f;
    public float LAUNCH_SPEED = 5f;
    // Start is called before the first frame update
    void Awake()
    {
        if (modelDirector == null)
            return;
        Time.fixedDeltaTime = (1f / (float)fps);
        Physics.defaultSolverIterations = solverIterations;
        Physics.defaultSolverVelocityIterations = solverIterations;
        directors = new MLAgentsDirector[numAgents];
        for (int i = 0; i < numAgents; i++)
            directors[i] = createMLAgent();
        Application.targetFrameRate = targetFrameRate;
        Physics.autoSimulation = false;
    }

    private void Start()
    {
        for(int i = 0; i < numAgents; i++)
            for(int j = + 1; j < numAgents; j++)
            {
                var colliders1 = directors[i].simulated_char.GetComponentsInChildren<Collider>();
                var projCollider1 = directors[i].projectileCollider;
                var colliders2 = directors[j].simulated_char.GetComponentsInChildren<Collider>();
                var projCollider2 = directors[j].projectileCollider;
                Physics.IgnoreCollision(projCollider1, projCollider2);
                foreach (var c1 in colliders1)
                    foreach (var c2 in colliders2)
                    {
                        Physics.IgnoreCollision(c1, c2);
                        Physics.IgnoreCollision(projCollider1, c2);
                        Physics.IgnoreCollision(c1, projCollider2);
                    }

            }
        Physics.autoSimulation = true;
    }
    private MLAgentsDirector createMLAgent()
    {
        GameObject obj =  Instantiate(modelDirector);
        obj.SetActive(true);
        MLAgentsDirector director = obj.GetComponent<MLAgentsDirector>();
        director.LAUNCH_FREQUENCY = LAUNCH_FREQUENCY;
        director.LAUNCH_RADIUS = LAUNCH_RADIUS;
        director.LAUNCH_SPEED = LAUNCH_SPEED;
        return director;
    }
    void FixedUpdate()
    {
        curStep++;
        if (curStep % reportMeanRewardEveryNSteps == 0)
        {
            Debug.Log($"Step {curStep} mean reward last {reportMeanRewardEveryNSteps} is: {meanReward}");
            meanReward = 0f;
        }
        float curStepReward = 0f;
        foreach (var director in directors)
            curStepReward += director.final_reward / (float) directors.Length;
        meanReward += (curStepReward / (float)reportMeanRewardEveryNSteps);
    }
}
