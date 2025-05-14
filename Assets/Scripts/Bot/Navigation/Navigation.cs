using Mapbox.Directions;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(SimulationHandler))]
public class Navigation : MonoBehaviour
{
    [Header("Debug")]
    public bool DEBUG = false;

    [Header("Scanning Settings")]
    public float scanRadius = 7f;
    public int waypointCount = 5;

    private float raycastDistance = 10f;
    private float raycastHeight = 5f;
    private  float maxScanRadiusMultiplier = 2f;
    private int rayCount = 31;



    [Header("Look Ahead Settings")]
    private float minLookAheadDistance = 2f;
    private float maxLookAheadDistance = 10f;
    public float curvatureThreshold = 0.5f; // Adjust this value to fine-tune sensitivity to curves

    private SimulationHandler simHandler;
    private WaypointManager waypointManager;
    private SplineGenerator splineGenerator;

    public bool isScanning = false;

    private Vector3 lastScanPos;
    private Vector3 lastScanDir;

    private void Awake()
    {
        simHandler = GetComponent<SimulationHandler>();
        waypointManager = GetComponent<WaypointManager>();
        splineGenerator = GetComponent<SplineGenerator>();
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

        isScanning = true;
        for (int i = 0; i < waypointCount; i++)
            yield return StartCoroutine(GenerateSingleWaypointAsync());
        isScanning = false;

        RegenerateSpline();

        if (DEBUG)
            Debug.Log($"Navigation | Start: seeded {waypointCount} waypoints and generated spline");
    }

    private void Update()
    {
        if (isScanning) return;

        Vector3 rearAxle = simHandler.GetRearAxlePosition();

        if (waypointManager.Waypoints.Count > 0)
        {
            if (DEBUG) Debug.Log($"Navigation | Update: Dist to WP[0] = {Vector3.Distance(rearAxle, waypointManager.Waypoints[0].transform.position):F2}");
            Debug.DrawLine(rearAxle, waypointManager.Waypoints[0].transform.position, Color.magenta);

            if (waypointManager.HasReachedWaypoint(rearAxle))
            {
                waypointManager.RemoveFirstWaypoint();
                if (DEBUG) Debug.Log($"Navigation | Update: Waypoint reached, {waypointManager.Waypoints.Count} left");
                StartCoroutine(GenerateSingleWaypointAsync());
                return;
            }
            return;
        }

        if (waypointManager.Waypoints.Count == 0)
        {
            if (DEBUG) Debug.Log("Navigation | Update: No waypoints—regenerating full batch");
            StartCoroutine(Start());
        }
    }

    private IEnumerator GenerateSingleWaypointAsync()
    {
        isScanning = true;

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
                if (DEBUG) Debug.Log($"Navigation | ArcScan retry {retries}, radius {radius:F2}");
            }
        } while (retry);

        if (roadHits.Count == 0)
        {
            Debug.LogError("Navigation | GenerateSingleWaypointAsync: No road hits detected");
            isScanning = false;
            yield break;
        }

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
        Vector3 wpPos = laneCenter + Vector3.up * 0.1f;

        if (DEBUG) Debug.Log($"Navigation | Placing waypoint at {wpPos}");

        Vector3 wpDirection = (wpPos - lastScanPos).normalized;
        Quaternion wpRotation = Quaternion.LookRotation(wpDirection, Vector3.up);

        waypointManager.CreateWaypoint(wpPos, wpRotation);

        Vector3 prevPos = lastScanPos;
        lastScanPos = wpPos;
        lastScanDir = (wpPos - prevPos).normalized;

        RegenerateSpline();

        isScanning = false;
        yield return null;
    }

    private void RegenerateSpline()
    {
        List<Vector3> waypointPositions = new List<Vector3>();
        foreach (var wp in waypointManager.Waypoints)
        {
            waypointPositions.Add(wp.transform.position);
        }

        Vector3 rearAxle = simHandler.GetRearAxlePosition();
        splineGenerator.GenerateSpline(rearAxle, waypointPositions);
    }

    public Vector3 GetLookAheadPoint(float baseDistanceAhead)
    {
        Vector3 rear = simHandler.GetRearAxlePosition();
        float accumulated = 0f;

        // Find the closest point on the spline
        int startIndex = FindClosestSplinePoint(rear);

        // Calculate curvature and adjust look-ahead distance
        float curvature = CalculateCurvature(startIndex);
        float adjustedDistance = Mathf.Lerp(minLookAheadDistance, maxLookAheadDistance, 1 - curvature / curvatureThreshold);
        adjustedDistance = Mathf.Clamp(adjustedDistance, minLookAheadDistance, baseDistanceAhead);

        // Walk forward along the spline
        Vector3 currentPoint = splineGenerator.SplinePoints[startIndex];
        for (int i = startIndex; i < splineGenerator.SplinePoints.Count - 1; i++)
        {
            Vector3 nextPoint = splineGenerator.SplinePoints[i + 1];
            float segmentDist = Vector3.Distance(currentPoint, nextPoint);

            accumulated += segmentDist;

            if (accumulated >= adjustedDistance)
                return nextPoint;

            currentPoint = nextPoint;
        }

        // If we run out of spline, return the last point
        return splineGenerator.SplinePoints[^1];
    }

    private int FindClosestSplinePoint(Vector3 position)
    {
        int closestIndex = 0;
        float closestDistance = float.MaxValue;

        for (int i = 0; i < splineGenerator.SplinePoints.Count; i++)
        {
            float distance = Vector3.Distance(position, splineGenerator.SplinePoints[i]);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    private float CalculateCurvature(int index)
    {
        if (index <= 0 || index >= splineGenerator.SplinePoints.Count - 1)
            return 0f;

        Vector3 prev = splineGenerator.SplinePoints[index - 1];
        Vector3 current = splineGenerator.SplinePoints[index];
        Vector3 next = splineGenerator.SplinePoints[index + 1];

        Vector3 v1 = current - prev;
        Vector3 v2 = next - current;

        float angle = Vector3.Angle(v1, v2);
        return angle / Vector3.Distance(prev, next);
    }
}