using Mapbox.Directions;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(SimulationHandler))]
public class Navigation : MonoBehaviour
{
    [Header("Scanning Settings")]
    public float scanRadius = 7f;
    private float raycastDistance = 10f;
    private float raycastHeight = 5f;
    private float maxScanRadiusMultiplier = 2f;
    private int rayCount = 31;

    [Header("Waypoint PlaceMent Settings")]
    public int waypointCount = 5;
    public float laneOffset = Mathf.Clamp01(0.1f);




    private SimulationHandler simHandler;
    private WaypointManager waypointManager;
    [HideInInspector]public SplineManager splineManager;

    // True if performing an ArcScan
    [HideInInspector] public bool isScanning = false;

    //Debug
     public bool DEBUG = true;

    private Vector3 rearAxle;

    private Vector3 wpPos;
    private Quaternion wpRotation;

    private Vector3 lastScanPos;
    private Vector3 lastScanDir;

    private void Awake()
    {
        simHandler = GetComponent<SimulationHandler>();
        waypointManager = GetComponent<WaypointManager>();
        splineManager = GetComponent<SplineManager>();
    }

    private IEnumerator Start()
    {
        if (waypointManager.waypointPrefab == null)
        {
            Debug.LogError("Navigation | Start: waypointPrefab is not assigned!");
            yield break;
        }

        lastScanPos = simHandler.GetRearAxlePosition();
        lastScanDir = transform.forward;

        StartCoroutine(CreatePath());
    }

    private IEnumerator CreatePath()
    {
        isScanning = true;
        for (int i = 0; i < waypointCount; i++)
            yield return StartCoroutine(GenerateSingleWaypointAsync());
        isScanning = false;

        splineManager.RegenerateSpline(rearAxle, waypointManager.Waypoints);

        
        Debug.Log($"Navigation | Start: seeded {waypointCount} waypoints and generated spline");
    }

    private void Update()
    {
        if (isScanning) return;

        rearAxle = simHandler.GetRearAxlePosition();

        if (waypointManager.Waypoints.Count > 0)
        {
            Debug.Log($"Navigation | Update: Dist to WP[0] = {Vector3.Distance(rearAxle, waypointManager.Waypoints[0].transform.position):F2}");
            Debug.DrawLine(rearAxle, waypointManager.Waypoints[0].transform.position, Color.magenta);

            if (waypointManager.HasReachedWaypoint(rearAxle))
            {
                waypointManager.RemoveFirstWaypoint();

                Debug.Log($"Navigation | Update: Waypoint reached, {waypointManager.Waypoints.Count} left");

                StartCoroutine(GenerateSingleWaypointAsync());
                return;
            }
            return;
        }

        //Not Likely
        if (waypointManager.Waypoints.Count == 0)
        {
            Debug.Log("Navigation | Update: No waypoints—regenerating full batch");
            StartCoroutine(CreatePath());
        }
    }



    private IEnumerator GenerateSingleWaypointAsync()
    {
        isScanning = true;

        //Scan and find position for wp
        GetLanePosition(ArcScan());

        //Place wp
        waypointManager.CreateWaypoint(wpPos, wpRotation);

        //Regenerate the Spline
        splineManager.RegenerateSpline(rearAxle, waypointManager.Waypoints);

        isScanning = false;
        yield return null;
    }
    private List<Vector3> ArcScan()
    {
        //START ARCSCAN


        float radius = scanRadius;
        List<Vector3> roadHits = new List<Vector3>();
        bool retry;
        int retries = 0, maxRetries = 3;

        do
        {
            roadHits.Clear();
            bool firstEdgeHit = false, lastEdgeHit = false;
            float startAngle = -90f;
            float angleStep = 180f / (rayCount - 1);

            for (int i = 0; i < rayCount; i++)
            {
                Vector3 dir = Quaternion.AngleAxis(startAngle + i * angleStep, Vector3.up) * lastScanDir;

                Vector3 origin = lastScanPos + dir.normalized * radius + Vector3.up * raycastHeight;
                Debug.DrawRay(origin, Vector3.down * raycastDistance, Color.cyan, 1f);
                if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, raycastDistance)
                    && hit.collider.CompareTag("Road"))
                {
                    roadHits.Add(hit.point);
                    if (i == 0) firstEdgeHit = true;
                    if (i == rayCount - 1) lastEdgeHit = true;
                }
            }

            retry = (firstEdgeHit || lastEdgeHit) && retries < maxRetries;
            if (retry)
            {
                radius = Mathf.Min(radius + scanRadius, scanRadius * maxScanRadiusMultiplier);
                retries++;
                Debug.Log($"Navigation | ArcScan retry {retries}, radius {radius:F2}");
            }
        } while (retry);

        if (roadHits.Count == 0)
        {
            Debug.LogError("Navigation | GenerateSingleWaypointAsync: No road hits detected");
            isScanning = false;
            return roadHits;
        }
        return roadHits;
        //END - ARCSCAN

    }

    private void GetLanePosition(List<Vector3> roadHits)
    {

        // Reference vector that points to the right of the vehicle's forward direction
        Vector3 rightAxis = Vector3.Cross(Vector3.up, lastScanDir).normalized;

        Vector3 leftMost = roadHits[0], rightMost = roadHits[0];

        // These will be used to find the leftmost and rightmost points
        float minDot = float.MaxValue, maxDot = float.MinValue;

        foreach (var pt in roadHits)
        {
            float d = Vector3.Dot(rightAxis, (pt - lastScanPos).normalized);
            if (d < minDot)
            {
                minDot = d;
                leftMost = pt;
            }

            if (d > maxDot)
            {
                maxDot = d;
                rightMost = pt;
            }
        }

        Vector3 middle = (leftMost + rightMost) * 0.5f;
        Vector3 laneCenter = (middle + rightMost) * 0.5f;
        wpPos = laneCenter + Vector3.up * 0.1f - transform.right * laneOffset;

        Debug.Log($"Navigation | Placing waypoint at {wpPos}");

        Vector3 wpDirection = (wpPos - lastScanPos).normalized;
        wpRotation = Quaternion.LookRotation(wpDirection, Vector3.up);

        //Update scanning state
        Vector3 prevPos = lastScanPos;
        lastScanPos = wpPos;
        lastScanDir = (wpPos - prevPos).normalized;
    }


   
}