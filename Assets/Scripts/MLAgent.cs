using System;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Text;
using Cinemachine;

public struct CharInfo
{
    public Transform[]  boneToTransform;
    public GameObject[] boneToCollider;
    public GameObject charObj;
    public Transform  trans;
    public ArticulationBody root;
    public Vector3 cm; // prev center of mass
    public Vector3 cmVel;
    public Vector3[] boneWorldPos;
    public Vector3[][] boneSurfacePts;
    public Vector3[][] boneSurfacePtsWorldSpace;
    public Vector3[][] boneSurfaceVels;
    public ArticulationBody[] boneToArtBody;
    public float[] boneState;
    public CharInfo(int nbodies, int numStateBones) : this()
    {
        boneSurfacePts = new Vector3[nbodies][];
        boneSurfacePtsWorldSpace = new Vector3[nbodies][];
        boneSurfaceVels = new Vector3[nbodies][];
        boneWorldPos = new Vector3[numStateBones];
        boneToCollider = new GameObject[nbodies];
        // From DreCon paper, contains: 
        // { LeftToe, RightToe, Spine, Head, LeftForeArm, RightForeArm },
        // we compute positions and velocities then concatenate these
        boneState = new float[6*11 + 9];
    }
    public CharInfo(CharInfo other) : this()
    {
        boneToTransform = (Transform[]) other.boneToTransform.Clone();
        root = other.root;
        trans = other.trans;

        boneSurfacePts = (Vector3[][]) other.boneSurfacePts.Clone();
        boneSurfacePtsWorldSpace = (Vector3[][]) other.boneSurfacePtsWorldSpace.Clone();
        boneSurfaceVels = (Vector3[][]) other.boneSurfaceVels.Clone();
        boneWorldPos = (Vector3[]) other.boneWorldPos.Clone();
        boneToCollider = (GameObject[]) other.boneToCollider.Clone();
        boneState = (float[]) other.boneState.Clone();
    }
}
public class MLAgent : Agent
{
    private ConfigManager _config;
    private SixtyFPSSyncOracle _sixtyFPSSyncOracle;
    CharInfo simChar, defaultChar;

    GameObject CharObj;

    public BoxCollider groundCollider;
    private float groundColliderY = -20f;
    // private float toeColliderRadius = 0f;
    public GameObject simulated_prefab;
    public int reportMeanRewardEveryNSteps = 10000;
    private SimCharController SimulationCharController;
    public static int nbodies = 12;

    private int curFixedUpdate = -1;
    private int lastSimCharTeleportFixedUpdate = -1;

    private bool releasedOnce = false;
    private bool touchedOnce = false;
    private float timeSinceTouch = 0, timeSinceRelease = 0;

    float[] prevActionOutput;
    float[] smoothedActions;
    int numActions;
    int numObservations;
    // MocapDB motionDB;
    public bool kinUseDebugMats = false;
    public bool simUseDebugMats = false;
    public CinemachineVirtualCamera thirdPersonCam;

    private Vector3 lastKinRootPos = Vector3.zero;

    public enum Bones
    {
        Bone_Shoulder = 0,
        Bone_UpperArm = 1, 
        Bone_LowerArm = 2,
        Bone_Palm = 3,
        Bone_Thumb1 = 4,
        Bone_Thumb2 = 5,
        Bone_FingerIndex1 = 6,
        Bone_FingerIndex2 = 7,
        Bone_FingerMiddle1 = 8,
        Bone_FingerMiddle2 = 9,
        Bone_FingerPinky1 = 10,
        Bone_FingerPinky2 = 11,
    };
    internal Quaternion[] bone_rotations = new Quaternion[nbodies];

    [HideInInspector]
    public static Bones[] stateBones = new Bones[]
    {  Bones.Bone_UpperArm, Bones.Bone_LowerArm, Bones.Bone_Palm, Bones.Bone_Thumb1, Bones.Bone_Thumb2, Bones.Bone_FingerIndex1, Bones.Bone_FingerIndex2, Bones.Bone_FingerMiddle1, Bones.Bone_FingerMiddle2, Bones.Bone_FingerPinky1, Bones.Bone_FingerPinky2 };

    [HideInInspector]
    public static Bones[] DOFBones = new Bones[]
    {  Bones.Bone_UpperArm, Bones.Bone_LowerArm, Bones.Bone_Palm, Bones.Bone_Thumb1, Bones.Bone_Thumb2, Bones.Bone_FingerIndex1, Bones.Bone_FingerIndex2, Bones.Bone_FingerMiddle1, Bones.Bone_FingerMiddle2, Bones.Bone_FingerPinky1, Bones.Bone_FingerPinky2,};
    
    [HideInInspector]
    public static Bones[] openloopBones = new Bones[]
      {};

    
    // [HideInInspector]
    // public static Bones[] extendedfullDOFBones = new Bones[]
    // {  Bones.Bone_LeftUpLeg, Bones.Bone_RightUpLeg, Bones.Bone_LeftFoot, Bones.Bone_RightFoot, Bones.Bone_LeftArm, Bones.Bone_RightArm, Bones.Bone_Hips, Bones.Bone_Spine,  Bones.Bone_Spine1, Bones.Bone_Spine2, Bones.Bone_LeftShoulder, Bones.Bone_RightShoulder};
    // [HideInInspector]
    // public static Bones[] extendedLimitedDOFBones = new Bones[]
    // {  Bones.Bone_LeftLeg, Bones.Bone_RightLeg, Bones.Bone_LeftForeArm, Bones.Bone_RightForeArm};
    // [HideInInspector]
    // public static Bones[] alwaysOpenloopBones = new Bones[]
    // { Bones.Bone_Neck, Bones.Bone_Head, Bones.Bone_LeftHand, Bones.Bone_RightHand, Bones.Bone_LeftToe, Bones.Bone_RightToe};


    // private GameObject[] projectiles;
    private GameObject curProjectile;
    private GameObject curTarget;

    private int projectileIdx = 0;
    public GameObject projectilePrefab;
    public GameObject targetPrefab;
    private float lastProjectileLaunchtime = 0f;
    public bool debug = false;
    public bool updateVelOnTeleport = true;
    private Unity.MLAgents.Policies.BehaviorParameters behaviorParameters;

    float period = 1f / 60f;



    public override void CollectObservations(VectorSensor sensor)
    {
        if (debug)
            Debug.Log("OBS COLLECTED");
        sensor.AddObservation(getState());
    }
    private void applyActions(bool applyLastAction)
    {
        if (debug)
            Debug.Log(smoothedActions);
        if (applyLastAction)
            for (int i = 0; i < numActions; i++)
                smoothedActions[i] = (1 - _config.ACTION_STIFFNESS_HYPERPARAM) * smoothedActions[i] + _config.ACTION_STIFFNESS_HYPERPARAM * prevActionOutput[i];
        if (debug)
            debugPrintActions();
        Quaternion[] curRotations = bone_rotations;
        Bones[] DOFBonesToUse = DOFBones;
        int actionIdx = 0;

        applyActionsAsEulerRotations(smoothedActions, curRotations, DOFBonesToUse, ref actionIdx);
        // switch (_config.actionRotType)
        // {
        //     case ActionRotationType.Euler:
        //         applyActionsAsEulerRotations(smoothedActions, curRotations, DOFBonesToUse, ref actionIdx);
        //         break;
        //     // case ActionRotationType.AxisAngle:
        //     //     applyActionsAsAxisAngleRotations(smoothedActions, curRotations, DOFBonesToUse, ref actionIdx);
        //     //     break;
        //     // case ActionRotationType.SixD:
        //     //     applyActionsWith6DRotations(smoothedActions, curRotations, DOFBonesToUse, ref actionIdx);
        //     //     break; 
        //     // case ActionRotationType.Exp:
        //     //     applyActionsAsExp(smoothedActions, curRotations, DOFBonesToUse, ref actionIdx);
        //     //     break;
        // }

        Bones[] openloopBonesBonesToUse = openloopBones;
        for (int i = 0; i < openloopBonesBonesToUse.Length; i++)
        {
            int boneIdx = (int)openloopBonesBonesToUse[i];
            Quaternion final = curRotations[boneIdx];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            ab.SetDriveRotation(final);
        }
        if (actionIdx != numActions)
        {
            throw new Exception($"Actions may not be properly intialized - length is {actionIdx} after copying everything");
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (debug)
            Debug.Log("RECIEVED ACTION");
        prevActionOutput = actionBuffers.ContinuousActions.Array;
        applyActions(true);
    }
    private void applyActionsAsEulerRotations(float[] finalActions, Quaternion[] curRotations, Bones[] DOFBonesToUse, ref int actionIdx)
    {
        for (int i = 0; i < DOFBonesToUse.Length; i++)
        {
            int boneIdx = (int)DOFBonesToUse[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            ArticulationDrive[] drives = new ArticulationDrive[]{ab.xDrive, ab.yDrive, ab.zDrive};
            for (int j = 0; j < 3; j++)
            {
                ArticulationDrive drive = drives[j];
                float range = drive.upperLimit - drive.lowerLimit;
                if (range == 0)
                    continue;

                float output = smoothedActions[actionIdx];
                float target;

                if (_config.setRotsDirectly)
                {
                    var midpoint = drive.lowerLimit + (range/2);
                    target = (output * (range/2)) + midpoint;
                }
                else
                {
                    float angle = output * range;
                    Vector3 targetRotationInJointSpace = ab.ToTargetRotationInReducedSpace(curRotations[boneIdx], true);
                    target = targetRotationInJointSpace[j] + angle;
                }
                drives[j].target = target;

                actionIdx++;
            }
            ab.xDrive = drives[0];
            ab.yDrive = drives[1];
            ab.zDrive = drives[2];
        }
    }

    public override void Heuristic(in ActionBuffers actionsout)
    {

        // Quaternion[] curRotations = bone_rotations;
        // for (int i = 0; i < fullDOFBones.Length; i++)
        // {
        //     int bone_idx = (int)fullDOFBones[i];
        //     simChar.boneToArtBody[bone_idx].SetDriveRotation(curRotations[bone_idx]);
        // }
        // for (int i = 0; i < limitedDOFBones.Length; i++)
        // {
        //     Bones bone =  limitedDOFBones[i];
        //     ArticulationBody ab = simChar.boneToArtBody[(int)bone];
        //     Vector3 target = ab.ToTargetRotationInReducedSpace(curRotations[(int) bone], true);
        //     ArticulationDrive drive = ab.zDrive;
        //     drive.target = target.z;
        //     ab.zDrive = drive;
        // }
        // for (int i = 0; i < openloopBones.Length; i++)
        // {
        //     int bone_idx = (int)openloopBones[i];
        //     simChar.boneToArtBody[bone_idx].SetDriveRotation(curRotations[bone_idx]);
        // }
    }
    private Vector3 feetBozSize;
    private Vector3 leftfootColliderCenter;
    private Vector3 rightfootColliderCenter;

    public GameObject camController;
    private CameraWASDController playerController;
    void customInit()
    {
        if (kinUseDebugMats || simUseDebugMats)
        {
            Material RedMatTransparent, WhiteMatTransparent;
#if UNITY_EDITOR
            RedMatTransparent = (Material)AssetDatabase.LoadAssetAtPath("Assets/Resources/RedMatTransparent.mat", typeof(Material));
            WhiteMatTransparent = (Material)AssetDatabase.LoadAssetAtPath("Assets/Resources/WhiteMatTransparent.mat", typeof(Material));
#else
            RedMatTransparent = Resources.Load<Material>("RedMatTransparent");
            WhiteMatTransparent = Resources.Load<Material>("WhiteMatTransparent");
#endif
            if (kinUseDebugMats)
                UnityObjUtils.setAllChildrenRenderersMaterial(CharObj.transform, WhiteMatTransparent);
        }
        if (_config.useSkinnedMesh)
        {
            foreach (var rend in CharObj.GetComponentsInChildren<Renderer>())
                rend.enabled = false;
            var skin = CharObj.GetComponentInChildren<SkinnedMeshRenderer>(true);
            if (skin != null)
            {
                skin.gameObject.SetActive(true);
                skin.enabled = true;
            }
        }

        SimulationCharController = CharObj.GetComponent<SimCharController>();
        SimulationCharController.debug = false;
        SimulationCharController.enabled = true;
        simChar = new CharInfo(nbodies, stateBones.Length);
        simChar.trans = CharObj.transform;
        simChar.boneToTransform = SimulationCharController.boneToTransform;
        simChar.root = SimulationCharController.boneToArtBody[(int)Bones.Bone_Shoulder];
        defaultChar = new CharInfo(simChar);

        foreach (var body in CharObj.GetComponentsInChildren<ArticulationBody>())
        {
            body.solverIterations = _config.solverIterations;
            body.solverVelocityIterations = _config.solverIterations;
        }
        simChar.charObj = CharObj;
        simChar.boneToArtBody = SimulationCharController.boneToArtBody;
        // foreach (Bones bone in fullDOFBones){
        //     ArticulationBody body = simChar.boneToArtBody[(int) bone];

        //     ArticulationDrive drive = body.xDrive;
        //     drive.stiffness = _config.boneToStiffness[(int) bone];
        //     drive.damping = _config.damping;
        //     drive.forceLimit = _config.forceLimit;
        //     simChar.boneToArtBody[(int) bone].xDrive = drive;
            
        //     drive = body.yDrive;
        //     drive.stiffness = _config.boneToStiffness[(int) bone];
        //     drive.damping = _config.damping;
        //     drive.forceLimit = _config.forceLimit;
        //     simChar.boneToArtBody[(int) bone].yDrive = drive;
            
        //     drive = body.zDrive;
        //     drive.stiffness = _config.boneToStiffness[(int) bone];
        //     drive.damping = _config.damping;
        //     drive.forceLimit = _config.forceLimit;
        //     simChar.boneToArtBody[(int) bone].zDrive = drive;
        // }
        // foreach (Bones bone in limitedDOFBones){
        //     ArticulationBody body = simChar.boneToArtBody[(int) bone];

        //     ArticulationDrive drive = body.xDrive;
        //     drive.stiffness = _config.boneToStiffness[(int) bone];
        //     drive.damping = _config.damping;
        //     drive.forceLimit = _config.forceLimit;
        //     simChar.boneToArtBody[(int) bone].xDrive = drive;
            
        //     drive = body.yDrive;
        //     drive.stiffness = _config.boneToStiffness[(int) bone];
        //     drive.damping = _config.damping;
        //     drive.forceLimit = _config.forceLimit;
        //     simChar.boneToArtBody[(int) bone].yDrive = drive;
            
        //     drive = body.zDrive;
        //     drive.stiffness = _config.boneToStiffness[(int) bone];
        //     drive.damping = _config.damping;
        //     drive.forceLimit = _config.forceLimit;
        //     simChar.boneToArtBody[(int) bone].zDrive = drive;
        // }

        // simChar.boneToCollider = SimulationCharController.boneToCollider;
        for (int i = 0; i < nbodies; i++)
        {
            // simChar.boneToCollider[i] = UnityObjUtils.getChildCapsuleCollider(simChar.boneToTransform[i].gameObject);
            simChar.boneToCollider[i] = simChar.boneToTransform[i].gameObject;
            bone_rotations[i] = simChar.boneToTransform[i].rotation;
            // Debug.Log(simChar.boneToArtBody[i]);
        }
        groundColliderY = groundCollider.bounds.max.y;
        // toeColliderRadius = simChar.boneToCollider[(int)Bones.Bone_RightToe].GetComponent<CapsuleCollider>().radius;
        // projectiles = new GameObject[_config.maxNumProjectiles];

        behaviorParameters = GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();
        numObservations = behaviorParameters.BrainParameters.VectorObservationSize;
        numActions = behaviorParameters.BrainParameters.ActionSpec.NumContinuousActions;
        if (debug)
        {
            Debug.Log(numObservations);
            Debug.Log(numActions);
        }
        bool isInference = behaviorParameters.BehaviorType == Unity.MLAgents.Policies.BehaviorType.InferenceOnly;
        if (isInference)
        {
            // GameObject camController = new GameObject("CameraWASDController");
            // var camera = camController.AddComponent<Camera>();
            // camera.enabled = true;
            playerController = camController.GetComponent<CameraWASDController>();
            playerController.target = curProjectile;
            // thirdPersonCam.gameObject.SetActive(true);
            // thirdPersonCam.Follow = camTarget.transform;
            // MMScript.playerCamTarget = playerCamTarget;
            // if (_config.userControl)
            //     MMScript.gen_inputs = false;
        }
        curFixedUpdate = _config.EVALUATE_EVERY_K_STEPS - 1;
        resetData();
    }

    private void resetData()
    {
        for (int i = 0; i < nbodies; i++)
        {
            simChar.boneSurfacePts[i] = new Vector3[6];
            simChar.boneSurfacePtsWorldSpace[i] = new Vector3[6];
            simChar.boneSurfaceVels[i] = new Vector3[6];
        }
        prevActionOutput = new float[numActions];
        smoothedActions = new float[numActions];
        simChar.boneState = new float[6*11 + 9];
        UpdateCMData(false, period);
        UpdateBoneObsState(false, period, true);
        UpdateBoneSurfacePts(false, period);
    }
    public void Awake()
    {
        // base.Initialize();
    // }
    // public void OnEnable()
    // {
        Debug.Log("Start MLAgent Setup");
        // if (motionDB == null)
        //     motionDB = MocapDB.Instance;
        _config = ConfigManager.Instance;
        _sixtyFPSSyncOracle = SixtyFPSSyncOracle.Instance;
        // Debug.Log("1");
        CharObj = Instantiate(simulated_prefab , Vector3.zero, Quaternion.identity);
        // Debug.Log("2");
        debug = debug && !Academy.Instance.IsCommunicatorOn;
        if (Academy.Instance.IsCommunicatorOn)
        {
            int numStepsPerSecond = (int)Mathf.Ceil(1f / period);
            MaxStep = numStepsPerSecond * _config.MAX_EPISODE_LENGTH_SECONDS;
        }
        // Debug.Log("3");
        customInit();
        Debug.Log("Finished MLAgent Setup");
    }
    public override void OnEpisodeBegin()
    {
        //Debug.Log($"{Time.frameCount}: Begin Episode on: {curFixedUpdate}, lasted {curFixedUpdate - lastEpisodeEndingFrame} frames ({(curFixedUpdate - lastEpisodeEndingFrame) / 60f} sec)");
        lastEpisodeEndingFrame = curFixedUpdate;
        // float verticalOffset = _config.ARM_VERT_OFFSET;
        SimCharController.teleportSimChar(simChar, defaultChar, 0, updateVelOnTeleport);
        lastSimCharTeleportFixedUpdate = curFixedUpdate;
        Physics.Simulate(.00001f);
        resetData();
        simChar.cmVel = Vector3.zero;
    }

    bool updateVelocity;
    public void FixedUpdate()
    {
        //teleporting 
        // if (MMScript.teleportedThisFixedUpdate)
        // {
        //     Vector3 preTeleportSimCharPosOffset = lastKinRootPos - simChar.root.transform.position;
        //     SimulationCharController.teleportSimCharRoot(simChar, MMScript.origin, preTeleportSimCharPosOffset);
        //     applyActions(false);
        //     lastSimCharTeleportFixedUpdate = curFixedUpdate;
        // }

        if (!_sixtyFPSSyncOracle.isSyncFrame)
            return;

        curFixedUpdate++;
        updateVelocity = lastSimCharTeleportFixedUpdate + 1 < curFixedUpdate;

        UpdateProjectileTarget();
        UpdateBoneObsState(updateVelocity, period);
        
        if (curFixedUpdate % _config.EVALUATE_EVERY_K_STEPS == 0)
            RequestDecision();
            if (debug)
                Debug.Log("DECISION REQUESTED");
        else 
            applyActions(_config.applyActionOverMultipleTimeSteps);
        
        // if (_config.projectileTraining)
        //     FireProjectile();
    }
    Vector3 prevProjPos, curProjPos, projVel, curTargetPos;
    private void UpdateProjectileTarget()
    {
        
        if (Time.time - lastProjectileLaunchtime < _config.LAUNCH_FREQUENCY)
            return;
        lastProjectileLaunchtime = Time.time;
        if (curProjectile == null || curTarget == null){
            projectileIdx++;
            if (projectileIdx == _config.maxNumProjectiles){
                prevProjPos = new Vector3();
                curProjPos = new Vector3();
                projectileIdx = 0;
                if (debug)
                    Debug.Log("EPISODE END");
                EndEpisode();
            }
            curProjectile = Instantiate(projectilePrefab, Vector3.zero, Quaternion.identity);
            curTarget = Instantiate(targetPrefab, Vector3.zero, Quaternion.identity);

            releasedOnce = touchedOnce = false;
            timeSinceTouch = timeSinceRelease = 0;

            bool isInference = behaviorParameters.BehaviorType == Unity.MLAgents.Policies.BehaviorType.InferenceOnly;
            if (isInference)
                playerController.target = curProjectile;

            var projectileRB = curProjectile.GetComponent<Rigidbody>();
            curProjectile.transform.localScale = Vector3.one * UnityEngine.Random.Range(_config.PROJECTILE_MIN_SCALE, _config.PROJECTILE_MAX_SCALE);
            projectileRB.mass = UnityEngine.Random.Range(_config.PROJECTILE_MIN_WEIGHT, _config.PROJECTILE_MAX_WEIGHT);
            float projectile_h = UnityEngine.Random.Range(_config.PROJECTILE_MIN_HEIGHT, _config.PROJECTILE_MAX_HEIGHT);
            Vector2 randomUnitCircle = UnityEngine.Random.insideUnitCircle.normalized;
            curProjectile.transform.position = simChar.trans.position + new Vector3(_config.PROJECTILE_SPAWNCENTER_X + randomUnitCircle.x * _config.PROJECTILE_RADIUS, projectile_h, randomUnitCircle.y * _config.PROJECTILE_RADIUS);
            
            var targetCollider = curTarget.GetComponent<SphereCollider>();
            curTarget.transform.localScale = Vector3.one * _config.TARGET_RADIUS;
            // targetCollider.radius = _config.TARGET_RADIUS;
            Vector3 targetPos = UnityEngine.Random.insideUnitSphere.normalized*_config.TARGET_SPAWN_RADIUS;
            targetPos += new Vector3(_config.TARGET_SPAWNCENTER_X, _config.TARGET_SPAWNCENTER_Y, _config.TARGET_SPAWNCENTER_Z);
            targetPos[1] = Mathf.Max(0, targetPos[1]);
            curTarget.transform.position = targetPos;
            

            
            prevProjPos = curProjPos = curProjectile.transform.position;
        }else{
            prevProjPos = curProjPos;
            curProjPos = curProjectile.transform.position;
            projVel = (curProjPos - prevProjPos) / period;
            curTargetPos = curTarget.transform.position;
        }
    }
    
    private void CheckProjectileTarget(out bool groundHit, out bool targetHit, out bool handHit, out bool armHit, out int nHandHit)
    {
        groundHit=false;
        targetHit=false;
        handHit=false;
        armHit=false;
        nHandHit = 0;
        if (!curProjectile)
            return;

        if (curProjectile.transform.position.y < groundColliderY){
            groundHit=true;
            Destroy(curProjectile);
            Destroy(curTarget);
        }
        var targetScript = curProjectile.GetComponent<ProjectileScript>();
        if (targetScript.hitTarget){
            targetHit=true;
            Destroy(curTarget);
            Destroy(curProjectile);
        } 
        else if (targetScript.onArm){
            armHit=true;
        }
        else if (targetScript.onHand){
            handHit=true;
            nHandHit= targetScript.nOnHand;
        }
    }
    // private void FireProjectile()
    // {
    //     float timeToTravel = _config.LAUNCH_RADIUS / _config.LAUNCH_SPEED;
    //     float YTarget = UnityEngine.Random.Range(simChar.cm.y - .5f, simChar.cm.y + .5f);
    //     float changeInY = YTarget - simChar.cm.y;
    //     float totalAcceleration = timeToTravel * -9.8f; 
    //     float curYVelocity = -totalAcceleration + (changeInY / timeToTravel);
    //     projectileRB.AddForce(-randomUnitCircle.x * _config.LAUNCH_SPEED, curYVelocity, -randomUnitCircle.y * _config.LAUNCH_SPEED, ForceMode.VelocityChange);
    // }
    private void UpdateCMData(bool updateVelocity, float dt)
    {
        Vector3 newSimCM = getCM(simChar.boneToTransform);
        simChar.cmVel = updateVelocity ? (newSimCM - simChar.cm) / dt : simChar.cmVel;
        simChar.cm = newSimCM;
    }

    private void UpdateBoneSurfacePts(bool updateVelocity, float dt)
    {
        for (int i = 1; i < nbodies; i++)
        {
            var charInfo = simChar;
            Vector3[] newSurfacePts = new Vector3[6];
            Vector3[] newWorldSurfacePts = new Vector3[6];
            UnityObjUtils.getSixPointsOnCollider(charInfo.boneToCollider[i], ref newWorldSurfacePts, (Bones)i);
            Vector3[] prevWorldSurfacePts = charInfo.boneSurfacePtsWorldSpace[i];

            for (int j = 0; j < 6; j++)
            {
                newSurfacePts[j] = resolvePosInRefFrame(newWorldSurfacePts[j]);
                if (updateVelocity)
                {
                    Vector3 surfaceVel = (newWorldSurfacePts[j] - prevWorldSurfacePts[j]) / dt;
                    charInfo.boneSurfaceVels[i][j] = resolveVelInRefFrame(surfaceVel);
                }
            }
            charInfo.boneSurfacePtsWorldSpace[i] = newWorldSurfacePts;
            charInfo.boneSurfacePts[i] = newSurfacePts;
        }
    }
// update bonestate
    private void UpdateBoneObsState(bool updateVelocity, float dt, bool zeroVelocity = false, bool updateKinOnly = false)
    {
        CharInfo curInfo = simChar;
        float[] copyInto = curInfo.boneState;
        int copyIdx = 0;
        for (int j = 0; j < stateBones.Length; j++)
        {
            Bones bone = stateBones[j];
            Vector3 boneWorldPos = curInfo.boneToTransform[(int)bone].position;
            Vector3 boneLocalPos = resolvePosInRefFrame(boneWorldPos);
            Vector3 prevBonePos = curInfo.boneWorldPos[j];
            Vector3 boneVel = (boneWorldPos - prevBonePos) / dt;
            boneVel = zeroVelocity ? Vector3.zero : resolveVelInRefFrame(boneVel);

            ArrayUtils.copyVecIntoArray(ref copyInto, ref copyIdx, boneLocalPos);
            if (updateVelocity || zeroVelocity)
                ArrayUtils.copyVecIntoArray(ref copyInto, ref copyIdx, boneVel);
            else
                copyIdx += 3;
            curInfo.boneWorldPos[j] = boneWorldPos;
        }

        if (curProjectile is not null){
            projVel = (curProjPos - prevProjPos) / dt;
            ArrayUtils.copyVecIntoArray(ref copyInto, ref copyIdx, curTargetPos);
            ArrayUtils.copyVecIntoArray(ref copyInto, ref copyIdx, curProjPos);
            ArrayUtils.copyVecIntoArray(ref copyInto, ref copyIdx, projVel);
        }
        else
            copyIdx += 9;
    }

    public void AssignLayer(int layer)
    {
        // Debug.Log(CharObj);
        CharObj.layer = layer;
        foreach (var child in CharObj.GetComponentsInChildren<Transform>())
        // foreach (var child in CharObj.GetComponent<Transform>())
            child.gameObject.layer = layer;
    }


    internal float finalReward = 0f;
    internal int lastEpisodeEndingFrame = 0;
    private bool shouldEndThisFrame = false;
    public void LateFixedUpdate()
    {
        if (!_sixtyFPSSyncOracle.isSyncFrame)
            return;
        CheckProjectileTarget(out bool groundHit, out bool targetHit, out bool handHit, out bool armHit, out int nHandHit);
        calcAndSetRewards(groundHit, targetHit, handHit, armHit, nHandHit);
        
        // for (int i = 0;i<GetComponents(typeof(Component)).Length;i++){
        //     Debug.Log(GetComponents(typeof(Component))[i]);
        // }
        if (behaviorParameters == null){
            Debug.Log("not initialized");
            customInit();
        }
        bool inInferenceMode = behaviorParameters.BehaviorType == Unity.MLAgents.Policies.BehaviorType.InferenceOnly;
        if (!inInferenceMode)
            return;
        UpdateBoneObsState(false, period, false, true);
        return;
    }
    // returns TRUE if episode ended
    private void calcProjectileTargetReward(out double posReward){
        if (!curTarget || !curProjectile){
            posReward = 0;
            return;
        }
        Vector3 distance = curTarget.transform.position - curProjectile.transform.position;
        posReward = 1000/distance.magnitude;
        // velReward = (Vector3.Dot(distance,projVel)) / (distance.magnitude+1e-5) / (projVel.magnitude+1e-5);
    }
    private void calcProjectileAgentReward(bool handHit, bool armHit, int nHandHit, out double agentReward){
        agentReward = 0;
        if (debug)
            Debug.Log(nHandHit);
        
        if (handHit){
            agentReward = 5 + 10 * nHandHit;
        }
        else if (armHit){
            agentReward = 5;
        }
    }
    private void calcRotationVelReward(out double reward){
        reward = 0;
        reward -= 5 * simChar.boneToArtBody[(int) Bones.Bone_UpperArm].angularVelocity.magnitude;
        reward -= 0.5 * simChar.boneToArtBody[(int) Bones.Bone_LowerArm].angularVelocity.magnitude;
        // reward -= 0.1 * simChar.boneToArtBody[(int) Bones.Bone_Palm].angularVelocity.magnitude;
        if (debug)
            Debug.Log(reward);
    }
    private void calcRotationPosReward(out double reward){
        reward = 0;

        float rot = 0;
        var rotation = simChar.boneToArtBody[(int) Bones.Bone_UpperArm].jointPosition;
        for(int i = 0; i<rotation.dofCount;i++){
            float r = rotation[i];
            rot += r*r;
        }
        reward -= 10 * Mathf.Sqrt(rot);

        
        rot = 0;
        rotation = simChar.boneToArtBody[(int) Bones.Bone_LowerArm].jointPosition;
        for(int i = 0; i<rotation.dofCount;i++){
            float r = rotation[i];
            rot += r*r;
        }
        reward -= 1 * Mathf.Sqrt(rot);

        if (debug)
            Debug.Log(reward);
    }
    // for holding only
    public bool calcAndSetRewards(bool groundHit, bool targetHit, bool handHit, bool armHit, int nHandHit)
    {
        double posReward, rotReward, velReward, agentReward, timeReward;
        if (handHit)
        {
            if (!touchedOnce)
                timeSinceTouch = 0;
            touchedOnce = true;
            timeSinceTouch += Time.fixedDeltaTime;
        }
        else if (touchedOnce)
            if (!releasedOnce)
                timeSinceRelease = 0;
            releasedOnce = true;
            timeSinceRelease += Time.fixedDeltaTime;

        calcProjectileAgentReward(handHit, armHit, nHandHit, out agentReward);
        calcRotationVelReward(out velReward);
        calcRotationPosReward(out posReward);
        finalReward = (float) (agentReward + velReward + posReward);
        finalReward = finalReward/60f;

        if (targetHit){
            if (debug)
                Debug.Log("TARGET HIT");
            finalReward = 0;
            SetReward(finalReward);
            return true;
        }
        if (groundHit){
            if (debug)
                Debug.Log("GROUND HIT");
            finalReward = 0;
            SetReward(finalReward);
            return true;
        }
        AddReward(finalReward);
        return false;
    }
    // public bool calcAndSetRewards(bool groundHit, bool targetHit, bool handHit, bool armHit, int nHandHit)
    // {
    //     double posReward, rotReward, velReward, agentReward, timeReward;
        // float timeSinceTouch, timeSinceRelease;
        // if (handHit)
        // {
        //     if (!touchedOnce)
        //         timeSinceTouch = 0;
        //     touchedOnce = true;
        //     timeSinceTouch += Time.fixedDeltaTime;
        // }
        // else if (touchedOnce)
        //     if (!releasedOnce)
        //         timeSinceRelease = 0;
        //     releasedOnce = true;
        //     timeSinceRelease += Time.fixedDeltaTime;

    //     // calcProjectileAgentReward(handHit, armHit, nHandHit, out agentReward);
    //     // calcRotationVelReward(out velReward);
    //     // finalReward = (float) (agentReward + velReward);
    //     // finalReward = finalReward/60f;

    //     // timeReward = touchedOnce ? -10 * Mathf.Pow(timeSinceTouch,1.5f) : 0f;
    //     // finalReward = (float) (agentReward + velReward + timeReward);

    //     calcProjectileTargetReward(out posReward);

    //     calcProjectileAgentReward(handHit, armHit, nHandHit, out agentReward);
    //     calcRotationVelReward(out velReward);
    //     calcRotationPosReward(out rotReward);
    //     finalReward = (float) (posReward);
    //     if (timeSinceTouch < 10)
    //     {
    //         finalReward += (float) (agentReward + velReward/2 + rotReward/5);
    //     }
    //     else if (timeSinceTouch>10 && timeSinceRelease==0) // just released
    //     {
    //         finalReward += 1;
    //     }
    //     finalReward = finalReward/60f;
    //     if (debug)
    //         Debug.Log(finalReward);
    //     finalReward = Mathf.Clamp(finalReward, -10, 10);
    //     if (targetHit){
    //         if (debug)///
    //             Debug.Log("TARGET HIT");
    //         finalReward = 100 * Mathf.Min(timeSinceTouch, 10)/10;
    //         SetReward(finalReward);
    //         return true;
    //     }
    //     if (groundHit){
    //         if (debug)
    //             Debug.Log("GROUND HIT");
    //         finalReward = 0;
    //         SetReward(finalReward);
    //         return true;
    //     }
    //     AddReward(finalReward);
    //     return false;
    // }
    float[] getState()
    {
        float[] state = new float[numObservations];
        int state_idx = 0;
        
        // if (curProjectile is not null){
        //     int copyIdx = 0;
        //     float[] copyInto = new float[11*6];
        //     ArrayUtils.copyVecIntoArray(ref copyInto, ref copyIdx, curProjPos);
        //     ArrayUtils.copyVecIntoArray(ref copyInto, ref copyIdx, (curProjPos - prevProjPos) / dt);
        //     for (int i = 0; i < 11*6; i++){
        //         state[state_idx++] = simChar.boneState[i] - copyInto;
        //     }

        // } else {
        //     for (int i = 0; i < 11*6; i++){
        //         state[state_idx++] = 0;
        //     }
        // }
        // Vector3 palmPos = simChar.boneToTransform[(int) Bones.Bone_Palm].transform;

        for (int i = 0; i < 11*6 + 9; i++)
            state[state_idx++] = simChar.boneState[i];
        for (int i = 0; i < numActions; i++)
            state[state_idx++] = smoothedActions[i];

        if (_config.ADD_PROJECTILE_TOUCH)
        {
            state[state_idx++] = (touchedOnce ? 1f : 0f);
            state[state_idx++] = (releasedOnce ? 1f : 0f);
            state[state_idx++] = (touchedOnce ? timeSinceTouch : -1f);
            state[state_idx++] = (releasedOnce ? timeSinceRelease : -1f);
        }
   
        if (state_idx != numObservations)
            throw new Exception($"State may not be properly intialized - length is {state_idx} after copying everything");

        //if (debug)
        //    ArrayUtils.debugArray(state, $"{curFixedUpdate} state: ");
        return state;
    }

    // Gets CoM in world position
    public static Vector3 getCM(Transform[] boneToTransform, Vector3[] globalBonePositions = null)
    {
        // We start at 1 because 0 is the root bone with no colliders
        float totalMass = 0f;
        Vector3 CoM = Vector3.zero;
        for (int i = 1; i < boneToTransform.Length; i++)
        {
            Transform t = boneToTransform[i];
            var ab = t.GetComponent<ArticulationBody>();
            float mass = t.GetComponent<ArticulationBody>().mass;
            Vector3 childCenter = globalBonePositions == null ? UnityObjUtils.getChildColliderCenter(t.gameObject) : globalBonePositions[i];
            CoM += mass * childCenter;
            totalMass += ab.mass;

        }
        return CoM / totalMass;
    }

    // Velocity is different in that we only need to make its rotation
    // local to the kinematic character, whereas with pos we also need to
    // position at the character's CM position
    // Vector3 resolveVelInKinematicRefFrame(Vector3 vel)
    // {
    //     // using same logic as in desired_velocity_update
    //     return MathUtils.quat_inv_mul_vec3(kinChar.trans.rotation, vel);
    // }
    Vector3 resolveVelInRefFrame(Vector3 vel)
    {
        // using same logic as in desired_velocity_update
        return MathUtils.quat_inv_mul_vec3(simChar.trans.rotation, vel);
    }
    // Vector3 resolvePosInKinematicRefFrame(Vector3 pos)
    // {
    //     // using same logic as in desired_velocity_update
    //     return MathUtils.quat_inv_mul_vec3(kinChar.trans.rotation, pos - kinChar.cm);
    // }
    Vector3 resolvePosInRefFrame(Vector3 pos)
    {
        // using same logic as in desired_velocity_update
        return MathUtils.quat_inv_mul_vec3(simChar.trans.rotation, pos - simChar.cm);
    }

    public void processCollision(Collision collision)
    {
        foreach (ContactPoint contact in collision.contacts)
        {
            string colliderName = contact.thisCollider.gameObject.name;
            if (!colliderName.ToLower().Contains("toe") && !colliderName.ToLower().Contains("foot") && !colliderName.ToLower().Contains("leg_") && contact.otherCollider.gameObject.name == "Ground")
            {
                Debug.Log($"Collider name: {colliderName} other collider name: {contact.otherCollider.gameObject.name}");
                shouldEndThisFrame = true;
            }
        }
    }

    // private float getBottomMostPointOnFoot(Transform foot, Vector3 center)
    // {
    //     float x = feetBozSize.x / 2;
    //     float y = feetBozSize.y / 2;
    //     float z = feetBozSize.z / 2;
    //     Vector3 topLeft = foot.TransformPoint(center + new Vector3(x, -y, z));
    //     Vector3 topRight = foot.TransformPoint(center + new Vector3(x, -y, -z));
    //     Vector3 bottomLeft = foot.TransformPoint(center + new Vector3(-x, -y, z));
    //     Vector3 bottomRight = foot.TransformPoint(center + new Vector3(-x, -y, -z));
    //     return Mathf.Min(topLeft.y, topRight.y, bottomLeft.y, bottomRight.y);
    // }

    // private float getVerticalOffset()
    // {
    //     // ClearGizmos();
    //     Transform leftFoot = kinChar.boneToCollider[(int)Bone_LeftFoot].transform;
    //     Transform rightFoot = kinChar.boneToCollider[(int)Bone_RightFoot].transform;
    //     float minPointOnFoot = Mathf.Min(getBottomMostPointOnFoot(leftFoot, leftfootColliderCenter), getBottomMostPointOnFoot(rightFoot, rightfootColliderCenter));
    //     Transform leftToe = kinChar.boneToTransform[(int)Bone_LeftToe];
    //     Transform rightToe = kinChar.boneToTransform[(int)Bone_RightToe];
    //     float minToeY = Mathf.Min(leftToe.position.y, rightToe.position.y) - toeColliderRadius;
    //     float maxGroundPenetration = Mathf.Max(0f, groundColliderY - Mathf.Min(minPointOnFoot , minToeY));
    //     //Debug.Log($"minToeY: {minToeY} minPointOnFoot: {minPointOnFoot} maxGroundPenetration: {maxGroundPenetration}");
    //     return maxGroundPenetration;
    // }

    public float gizmoSphereRad = .01f;

    List<(Vector3 pos, Color color)> gizmoSpheres;
    List<(Vector3 a, Vector3 b, Color color)> gizmoLines;
    private void AddGizmoSphere(Vector3 v, Color c)
    {
        if (gizmoSpheres == null)
            gizmoSpheres = new List<(Vector3, Color)>();
        gizmoSpheres.Add((v, c));
    }
    private void AddGizmoLine(Vector3 a, Vector3 b, Color color)
    {
        if (gizmoLines == null)
            gizmoLines = new List<(Vector3, Vector3, Color)>();
        gizmoLines.Add((a, b, color));
    }
    [ContextMenu("Reset gizmos")]
    private void ClearGizmos()
    {
        if (gizmoSpheres != null)
            gizmoSpheres.Clear();
        if (gizmoLines != null)
            gizmoLines.Clear();
    }
    public bool drawGizmos = false;
    private void OnDrawGizmos()
    {
        if (!drawGizmos)
            return;
        if (gizmoSpheres != null)
        {
            foreach ((Vector3 v, Color c) in gizmoSpheres)
            {
                Gizmos.color = c;
                Gizmos.DrawSphere(v, gizmoSphereRad);
            }
        }

        if (gizmoLines != null)
        {
            foreach ((Vector3 a, Vector3 b, Color color) in gizmoLines)
            {
                Gizmos.color = color;
                Gizmos.DrawLine(a, b);
            }
        }
    }
    private void debugPrintActions()
    {
        // bool actionsAre6D = _config.actionRotType == ActionRotationType.SixD;
        StringBuilder debugStr = new StringBuilder();
        int actionIdx = 0;
        Bones[] DOFBonesToUse = DOFBones;
        for (int i = 0; i < DOFBonesToUse.Length; i++)
        {
            Bones bone = DOFBonesToUse[i];
            int boneIdx = (int) bone;
            StringBuilder boneString =  new StringBuilder();
            boneString.Append($"{bone.ToString().Substring(5)}: ");

            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            ArticulationDrive[] drives = new ArticulationDrive[]{ab.xDrive, ab.yDrive, ab.zDrive};
            for (int j = 0; j < 3; j++)
            {
                ArticulationDrive drive = drives[j];
                float range = drive.upperLimit - drive.lowerLimit;
                if (range == 0)
                    continue;

                float output = smoothedActions[actionIdx];
                boneString.Append($"{output:0.00} ");
                actionIdx++;
            }
            // Vector3 output = new Vector3(prevActionOutput[actionIdx], prevActionOutput[actionIdx + 1], prevActionOutput[actionIdx + 2]);
            debugStr.Append(boneString.ToString());
            // actionIdx += actionsAre6D ? 6 : 3;
        }
        // Bones[] limitedDOFBonesToUse = limitedDOFBones;
        // for (int i = 0; i < limitedDOFBonesToUse.Length; i++)
        // {
        //     Bones bone = limitedDOFBonesToUse[i];
        //     debugStr.Append($"{bone.ToString().Substring(5)}: {prevActionOutput[actionIdx]} | {smoothedActions[actionIdx]} ");
        //     actionIdx += 1;
        // }
        Debug.Log(debugStr.ToString());
    }
    private void OnGUI()
    {
        if (!_config.rewardsInGUI)
            return;
        GUIStyle fontSize = new GUIStyle(GUI.skin.GetStyle("label"));
        fontSize.fontSize = 16;
        GUI.Label(new Rect(100, 175, 600, 100), "Last Reward: " + finalReward.ToString(), fontSize);
    }
}
