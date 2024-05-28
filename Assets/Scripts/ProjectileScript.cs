using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ProjectileScript : MonoBehaviour
{
    public bool hitTarget = false;
    public bool onArm = false;
    public bool onHand = false;
    public int nOnHand = 0;
    void Start()
    {
        
    }
    void OnCollisionEnter(Collision other)
    {
        if (other.gameObject.tag == "target")
        { 
            hitTarget = true;
            return;
        }
        if (other.gameObject.tag == "Hand")
        { 
            onHand = true;
            nOnHand++;
        }
        if (other.gameObject.tag == "Arm")
        { 
            onArm = true;
        }

    }
    void OnCollisionExit(Collision other)
    {
        if (other.gameObject.tag == "Hand")
        { 
            onHand = false;
            nOnHand--;
        }
        if (other.gameObject.tag == "Arm")
        { 
            onArm = false;
        }
    }
    // Update is called once per frame
    void Update()
    {
        
    }
}
