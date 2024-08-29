using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ProjectileScript : MonoBehaviour
{
    public bool hitTarget = false;
    public bool onArm = false;
    public bool onHand = false;
    public int nOnHand;
    void Start()
    {
        nOnHand = 0;
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
            nOnHand++;
            onHand = true;
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
            nOnHand--;
            if (nOnHand ==0){
                onHand = false;
            }
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
