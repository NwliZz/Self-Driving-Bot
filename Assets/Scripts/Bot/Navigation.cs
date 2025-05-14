using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(SimulationHandler))]
public class Navigation : MonoBehaviour
{
    [Header("Debug")]
    public bool DEBUG = false;

    [Header("Scanning Settings")]
    //Initial radius for raycast origin offset
    public float scanRadius = 7f;

    //Maximum radius multiplier when adapting
    public float maxScanRadiusMultiplier = 2f;

    //Height above ground to start raycasts
    public float raycastHeight = 5f;

    //Maximum distance for downward raycast
    public float raycastDistance = 10f;

    //Number of rays in the arc (covering 180 degrees)
    public int rayCount = 31;

    //Number of waypoints to generate 
    public int waypointCount = 5;

    //Minimum distance (m) from rear axle to waypoint to consider reached
    public float waypointReachThreshold = 1.8f;

    [Header("Waypoint Prefab")]
    public GameObject waypointPrefab;

    // Waypoints sliding window
    private List<GameObject> waypoints = new List<GameObject>();
    private GameObject currentTarget;

    private SimulationHandler simHandler;

    // If true Corutine is Running
    public bool isScanning = false;

    // Last Scan State
    private Vector3 lastScanPos;
    private Vector3 lastScanDir;


    // Expose current target publicly
    public GameObject CurrentTarget => currentTarget;

    private IEnumerator Start()
    {
        simHandler = GetComponent<SimulationHandler>();
        if (waypointPrefab == null)
        {
            Debug.LogError("Navigation | Start: waypointPrefab is not assigned!");
        }

        // Seed lastScanPos/lastScanDir
        lastScanPos = simHandler.GetRearAxlePosition();
        lastScanDir = transform.forward;

        // Initialize waypoints

        for (int i = 0; i < waypointCount; i++)
            yield return StartCoroutine(GenerateWaypoint());
        isScanning = false;
        currentTarget = waypoints[0];

    }

    void Update()
    {
        // If we’re still scanning, don’t touch the list
        if (isScanning) return;

        if (waypoints.Count > 0)
        {
            // Get the rear axle position, the current target and check distance to the first waypoint
            Vector3 rearAxle = simHandler.GetRearAxlePosition();
            currentTarget = waypoints[0];
            float dist = Vector3.Distance(rearAxle, currentTarget.transform.position);

            Debug.DrawLine(rearAxle, currentTarget.transform.position, Color.magenta);

            if (DEBUG) Debug.Log($"Navigation | Update: Dist to Waypoint: {dist}");


            if (dist < waypointReachThreshold)
            {
                // Destroy only the one we just hit
                Destroy(currentTarget);
                waypoints.RemoveAt(0);

                if (DEBUG) Debug.Log("Navigation | Update: Waypoint reached");

                StartCoroutine(GenerateWaypoint());
                isScanning = false;
            }
            return;
        }

        // If we’ve run out of waypoints, do a full rescan
        if (waypoints.Count == 0)
        {
            if (DEBUG) Debug.LogError("Navigation | Update: No Waypoints available, regenerating...");


            //StartCoroutine(GenerateWaypoints());
        }
    
    }

    private IEnumerator GenerateWaypoint()
    {
        isScanning = true;
        float radius = scanRadius;
        List<Vector3> roadHits = new List<Vector3>();
        bool retry;
        int retries = 0, maxRetries = 3;

        //Adaptive Scan
        do
        {
            if (DEBUG) Debug.Log($"Navigation | GenerateWaypointsAsync: Scan Started...");

            roadHits.Clear();

            //True means the scan starts again with larger radius
            bool firstEdgeHit = false, lastEdgeHit = false;

            float startAngle = -70f, angleStep = 140f / (rayCount - 1);

            for (int i = 0; i < rayCount; i++)
            {
                float angle = startAngle + i * angleStep;

                Vector3 dir = Quaternion.AngleAxis(angle, Vector3.up) * lastScanDir;
                Vector3 origin = lastScanPos + dir.normalized * radius + Vector3.up * raycastHeight;

                if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, raycastDistance))
                {
                    Debug.DrawRay(origin, Vector3.down * raycastDistance, Color.cyan, 1f);

                    if (hit.collider.CompareTag("Road"))
                    {
                        roadHits.Add(hit.point);

                        //Check if the hit point is on the edge of the road, if so, retry.
                        if (i == 0) firstEdgeHit = true;
                        if (i == rayCount - 1) lastEdgeHit = true;
                    }
                }
            }

            retry = (firstEdgeHit || lastEdgeHit) && retries < maxRetries;
            if (retry)
            {
                radius = Mathf.Min(radius + scanRadius, scanRadius * maxScanRadiusMultiplier);
                retries++;
                if (DEBUG) Debug.Log($"Navigation | GenerateWaypoint: Edge hit, increasing radius to {radius:F2} and retrying...");
            }

        } while (retry);


        //----
        // Lane Detection
        //----
        if (roadHits.Count == 0)
        {
            Debug.LogError("Navigation | GenerateWaypoint: No road hits detected, stopping generation.");
            yield return null;
        }

        // Evaluate leftmost/rightmost relative to lastDir
        Vector3 rightAxis = Vector3.Cross(Vector3.up, lastScanDir).normalized;

        
        Vector3 leftMost = roadHits[0], rightMost = roadHits[0];

        float minDot = float.MaxValue, maxDot = float.MinValue;

        foreach (var pt in roadHits)
        {
            float d = Vector3.Dot(rightAxis, (pt - lastScanPos).normalized);

            //Hit is Left related to lastDir
            if (d < minDot) { minDot = d; leftMost = pt; }

            //Hit is Right -//-
            if (d > maxDot) { maxDot = d; rightMost = pt; }
        }

        Vector3 middle = (leftMost + rightMost) * 0.5f;
        Vector3 rightLaneCenter = (middle + rightMost) * 0.5f;
        Vector3 wpPos = rightLaneCenter + Vector3.up * 0.1f;


        GameObject wp = Instantiate(waypointPrefab, wpPos, Quaternion.identity);
        waypoints.Add(wp);
        if (DEBUG) Debug.Log($"Navigation | GenerateWaypointsAsync: Placed waypoint at {wpPos} & Added to list");


        // Update for next iteration
        lastScanDir = (wpPos - lastScanPos).normalized;
        lastScanPos = wpPos;

        if (DEBUG) Debug.Log($"Navigation | GenerateWaypointsAsync: Scan Complete.");

        yield return null;     
    }
}
