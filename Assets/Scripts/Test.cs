using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class Test : MonoBehaviour
{
    private void Start()
    {
        Academy.Instance.AutomaticSteppingEnabled = false;
    }
    private void FixedUpdate()
    {
        Academy.Instance.EnvironmentStep();
    }

    // // Update is called once per frame
    // void Update()
    // {
        
    // }
}
