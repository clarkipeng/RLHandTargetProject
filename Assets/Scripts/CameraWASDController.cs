using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;

public class CameraWASDController : MonoBehaviour
{
    //public float velocityPerSecond = 2f;
    public float horizontalVelPerSecond = 10f;
    public float verticalVelPerSec = 10f;
    Keyboard kb;
    KeyControl forwardKey, backwardKey, rightKey, leftKey;
    ButtonControl upKey, downKey;
    public float xSens = 50f;
    public float ySens = 50f;
    public float screenSensitivity = 50f;
    float xRot, yRot;

    Vector2 screenVel, screenAcc;

    public float screenRotHalflife= .1f;

    private bool locked = false;
    private bool clocked = false;
    public float rotVelPerSecond = 10f;
    public GameObject target;//the coord to the point where the camera looks at

    void Start()
    {
        //Cursor.lockState = CursorLockMode.Locked;
        kb = Keyboard.current;
        forwardKey = kb.wKey;
        backwardKey = kb.sKey;
        leftKey = kb.aKey;
        rightKey = kb.dKey;
        upKey = kb.spaceKey;
        downKey = kb.ctrlKey;
        locked = true;
        clocked = false;
    }

    // Update is called once per frame
    void Update()
    {

        if (Mouse.current.leftButton.wasPressedThisFrame)
            clocked = !clocked;
            Cursor.lockState = clocked ? CursorLockMode.Locked : CursorLockMode.None;
        if (kb.qKey.wasPressedThisFrame)
            locked = !locked;
        if (!locked)
        {
            Vector3 positionDelta = transform.forward * (forwardKey.isPressed ? 1f: 0f);
            positionDelta += -transform.forward * (backwardKey.isPressed ? 1f : 0f);
            positionDelta += transform.right * (rightKey.isPressed ? 1f : 0f);
            positionDelta += -transform.right * (leftKey.isPressed ? 1f : 0f);
            positionDelta = positionDelta.normalized * (horizontalVelPerSecond * Time.deltaTime);

            float yDelta = upKey.isPressed ? 1f : 0f;
            yDelta += downKey.isPressed ? -1f : 0f;
            positionDelta.y = yDelta * verticalVelPerSec * Time.deltaTime;

            transform.position += positionDelta;
            if (kb.capsLockKey.isPressed)
                transform.position = new Vector3(0f, 1f, 1f);

            if (Cursor.lockState == CursorLockMode.None)
                return;
            SpringUtils.spring_character_update(screenVel, screenAcc, Mouse.current.delta.ReadValue() * screenSensitivity * Time.deltaTime, screenRotHalflife, Time.deltaTime , out screenVel, out screenAcc);
            Vector2 mouseDelta = screenVel;
            yRot += mouseDelta.x;
            xRot -= mouseDelta.y;
            xRot = Mathf.Clamp(xRot, -90f, 90f);
            transform.rotation = Quaternion.Euler(xRot, yRot, 0f);
            
        }    
        if (locked)
        {
            
            if (target == null)
                target = GameObject.Find("Projectile(Clone)");
            if (target == null)
                return;
            Vector3 positionDelta = transform.forward * (forwardKey.isPressed ? 1f: 0f);
            positionDelta += -transform.forward * (backwardKey.isPressed ? 1f : 0f);
            positionDelta += transform.right * (rightKey.isPressed ? 1f : 0f);
            positionDelta += -transform.right * (leftKey.isPressed ? 1f : 0f);
            positionDelta += transform.up * (upKey.isPressed ? 1f : 0f);
            positionDelta += -transform.up * (downKey.isPressed ? 1f : 0f);

            positionDelta = positionDelta.normalized * (rotVelPerSecond * Time.deltaTime);

            transform.position += positionDelta;
            if (kb.capsLockKey.isPressed)
                transform.position = new Vector3(0f, 1f, 1f);

            // float up = 0, right = 0;
            // up += (forwardKey.isPressed ? 1f: 0f);
            // up += (backwardKey.isPressed ? 1f : 0f);
            // right += (rightKey.isPressed ? 1f : 0f);
            // right += (leftKey.isPressed ? 1f : 0f);
            // up = up / Mathf.Sqrt(up*up+right*right);
            // right = right / Mathf.Sqrt(up*up+right*right);
            Vector3 point = target.transform.position;//get target's coords
            transform.LookAt(point);//makes the camera look to it
            // transform.RotateAround(point,new Vector3(0.0f,up,right),(rotVelPerSecond * Time.deltaTime));
            xRot = transform.eulerAngles[0];
            yRot = transform.eulerAngles[1];
        }
    }
}
