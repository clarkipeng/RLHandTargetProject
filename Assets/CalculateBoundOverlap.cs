using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static mm_v2.Bones;
public class CalculateBoundOverlap : MonoBehaviour
{

    public float INC = .001f;
    public BoxCollider[] boxes;
    public GameObject[] bone_to_collider;
    public Transform[] bone_to_transform;

    [ContextMenu("Calculate bounding overlaps")]
    void calculateBoundingOverlaps()
    {
        init_bone_colliders();
        int n = 23; // num bones
        float [][] bone_to_points = new float[n][];
        gizmo_points = new List<Vector3>();

        // idx 0 = points only in this bone, 1 = points shared bt 1 other bone, 2 = points shared bt 2 other bones, 3 etc 4 etc
        for (int i = 0; i < n; i++)
            bone_to_points[i] = new float[5];

        int[] bone_collided = new int[n];
        foreach (BoxCollider box in boxes)
        {
            //float min_x, max_x, min_y, max_y, min_z, max_z;
            Vector3 mins = box.transform.TransformPoint(box.center - box.size/2);
            Vector3 maxes = box.transform.TransformPoint(box.center + box.size / 2);
            int num_points_checked = 0;
            for (float x = mins.x; x < maxes.x; x += INC)
                for (float y = mins.y; y < maxes.y; y += INC)
                    for (float z = mins.z; z < maxes.z; z += INC)
                    {
                        Vector3 point = new Vector3(x, y, z);
                        int num_collisions = 0;
                        for (int i = 1; i < n; i++)
                        {
                            int collided = check_collision(point, (mm_v2.Bones)i) ? 1 : 0;
                            num_collisions += collided;
                            bone_collided[i] = collided;
                        }
                        if (num_collisions > 5)
                            Debug.Log($"Point with {num_collisions} collisions found");
                        if (num_collisions > 0 && num_collisions <= 5)
                            for (int i = 1; i < n; i++)
                                bone_to_points[i][num_collisions - 1] += bone_collided[i];
                        num_points_checked += 1;
                        gizmo_points.Add(point);
                    }
            Debug.Log($"Num points checked: {num_points_checked}");
        }
        for (int i = 1; i < n; i++)
        {
            float[] weight_dist = bone_to_points[i];
            float total_collisions = weight_dist[0] + weight_dist[1] + weight_dist[2] + weight_dist[3] + weight_dist[4];
            float full_weight = get_full_weight((mm_v2.Bones)i);
            float final_weight =  (weight_dist[0] / total_collisions) * full_weight +
                                  (weight_dist[1] / total_collisions) * .5f * full_weight +
                                  (weight_dist[2] / total_collisions) * .33f * full_weight +
                                  (weight_dist[3] / total_collisions) * .25f * full_weight +
                                  (weight_dist[4] / total_collisions) * .2f * full_weight;

            Debug.Log($"Bone {(mm_v2.Bones)i} total_collisions: {total_collisions}  full weight: {full_weight} | final_weight: {final_weight}");
            Debug.Log($"| weight_dist[0]: {weight_dist[0]} weight_dist[1] : {weight_dist[1]}  weight_dist[2]: {weight_dist[2]} | weight_dist[3]: {weight_dist[3]} | weight_dist[4]: {weight_dist[4]}");

        }
    }
    public int AVG_HUMAN_DENSITY = 985; // kg / m^3

    float get_full_weight(mm_v2.Bones bone)
    {
        float volume;
        if (bone == Bone_LeftFoot || bone == Bone_RightFoot)
        {
            BoxCollider box = bone_to_collider[(int)bone].GetComponent<BoxCollider>();
            volume = box.size.x * box.size.y * box.size.z;
        }
        else
        {
            CapsuleCollider caps = bone_to_collider[(int)bone].GetComponent<CapsuleCollider>();
            volume = GeoUtils.GetCapsuleVolume(caps);
        }
        return volume * AVG_HUMAN_DENSITY;

    }


    void init_bone_colliders()
    {
        int nbodies = 23;
        bone_to_collider = new GameObject[nbodies];
        for (int i = 0; i < nbodies; i++)
        {
            if (i == (int)Bone_LeftFoot || i == (int)Bone_RightFoot)  
                bone_to_collider[i] = UpdateJointPositions.getChildBoxCollider(bone_to_transform[i].gameObject);
            else
                bone_to_collider[i] = UpdateJointPositions.getChildCapsuleCollider(bone_to_transform[i].gameObject);
            
        }

    }

    private bool check_collision(Vector3 point, mm_v2.Bones bone)
    {
        if (bone == Bone_LeftFoot || bone == Bone_RightFoot)
        {
            BoxCollider box = bone_to_collider[(int)bone].GetComponent<BoxCollider>();
            return box.collision_check(point);
        }
        else
        {
            CapsuleCollider caps = bone_to_collider[(int)bone].GetComponent<CapsuleCollider>();
            return caps.collision_check(point);
        }
    }
    public bool draw_gizmos = false;
    List<Vector3> gizmo_points;
    private void OnDrawGizmos()
    {
        if (gizmo_points == null || !draw_gizmos || gizmo_points.Count > 10000)
            return;
        Gizmos.color = Color.red;
        foreach (Vector3 point in gizmo_points)
            Gizmos.DrawSphere(point, .01f);
    }

}

public static class CapsuleColliderUtils
{
    // Returns true if point is inside the capsule
    public static bool collision_check(this CapsuleCollider cap, Vector3 point)
    {
        Vector3 center = cap.center;
        float halfHeight = cap.height / 2 - cap.radius;
        if (halfHeight < 0 || Mathf.Approximately(halfHeight, 0f))
            return cap.radius >= (point - cap.transform.TransformPoint(center)).magnitude;
        var top = cap.transform.TransformPoint(center + halfHeight * Vector3.right);
        var bottom = cap.transform.TransformPoint(center - halfHeight * Vector3.right);
        return cap.radius >= GeoUtils.dist_bt_point_and_seg(point, top, bottom);
    }
}


public static class BoxColliderUtils
{
    // Returns true if point is inside the capsule
    public static bool collision_check(this BoxCollider box, Vector3 point)
    {
        point = box.transform.InverseTransformPoint(point) - box.center;

        float halfX = (box.size.x * 0.5f);
        float halfY = (box.size.y * 0.5f);
        float halfZ = (box.size.z * 0.5f);
        if (point.x < halfX && point.x > -halfX &&
           point.y < halfY && point.y > -halfY &&
           point.z < halfZ && point.z > -halfZ)
            return true;
        else
            return false;
    }
}