using UnityEngine;
using System.Collections.Generic;
using System.Collections;
using UnityEditor.PackageManager.UI;

public class ProceduralWaypointGenerator : MonoBehaviour
{
    public GameObject waypointPrefab;

    int rayCount = 31;
    float raycastHeight = 5f;
    float raycastDistance = 10f;

    float scanRadius = 7f;
    float spawnDistance = 5f; 
    public int maxWaypoints = 50;

    private List<GameObject> WAYPOINTS = new List<GameObject>();

    public GameObject CURRENT_TARGET;

    //Exclude self
    int roadLayerMask;
    void Start()
    {
        roadLayerMask = ~LayerMask.GetMask("Car");

        //StartCoroutine(DelayedGenerate());
        PlaceWaypointByScanning();
    }

    void Update()
    {
        if (WAYPOINTS.Count == 0)
        {
            Debug.Log("No waypoints available");
            return;
        }
        else
        {
            for (int i = 0; i < WAYPOINTS.Count; i++)
            {
                CURRENT_TARGET = WAYPOINTS[i];

                Vector3 carFront = transform.position + transform.forward * 2.4f;//Works for now
                float distToNextWP = Vector3.Distance(carFront, WAYPOINTS[i].transform.position);

                Debug.DrawLine(transform.position, WAYPOINTS[i].transform.position, Color.yellow);

                if (distToNextWP < 1.8f)
                {
                    Debug.Log($"Waypoint: {i} reached !");
                    WAYPOINTS.RemoveAt(i);
                    PlaceWaypointByScanning();
                }
                else
                {
                    Debug.Log("Moving To Waypoint No: " + i);

                }
            }
        }
    }

    public void PlaceWaypointByScanning()
    {
        List<Vector3> hitPoints = new List<Vector3>();

        // Arc Scan
        float angleStep = 180f / (rayCount - 1); // 180° arc
        float startAngle = -90f;

        for (int i = 0; i < rayCount; i++)
        {
            float angle = startAngle + i * angleStep;
            Quaternion rotation = Quaternion.AngleAxis(angle, Vector3.up);
            Vector3 dir = rotation * transform.forward;

            Vector3 rayOrigin = transform.position + dir.normalized * scanRadius + Vector3.up * raycastHeight;
            Debug.DrawRay(rayOrigin, Vector3.down * raycastDistance, Color.cyan, 1.5f);

            if (Physics.Raycast(rayOrigin, Vector3.down, out RaycastHit hit, raycastDistance, roadLayerMask))
            {
                if (hit.collider.CompareTag("Road"))
                {
                    hitPoints.Add(hit.point + Vector3.up * 0.1f);
                }
            }
        }

        //Find the Right Lane
        if (hitPoints.Count > 0)
        {
            Vector3 leftMost = hitPoints[0];
            Vector3 rightMost = hitPoints[0];
            float minDot = float.MaxValue;
            float maxDot = float.MinValue;

            foreach (var point in hitPoints)
            {
                Vector3 dir = (point - transform.position).normalized;
                float dot = Vector3.Dot(transform.right, dir); // How "right" the point is

                if (dot > maxDot)
                {
                    maxDot = dot;
                    rightMost = point;
                }
                if (dot < minDot)
                {
                    minDot = dot;
                    leftMost = point;
                }
            }

            Vector3 middle = (leftMost + rightMost) / 2f + Vector3.up * 0.1f;

            Vector3 rightLane = (middle + rightMost) / 2f;
            GameObject go = Instantiate(waypointPrefab, rightLane, Quaternion.identity);

            WAYPOINTS.Add(go);
            Debug.Log("Whaypoint Saved");
        }
        else
        {
            Debug.LogError("No road found in Machine Vision.");
        }
    }

    //Lets TagManager finish its tagging first (For Map Box)
    IEnumerator DelayedGenerate()
    {
        // Wait until AutoTagMapboxRoads is done
        yield return new WaitUntil(() => TagManager.RoadsTagged);

        // Now generate waypoints
        PlaceWaypointByScanning();
    }
}

