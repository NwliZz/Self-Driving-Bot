using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(SimulationHandler))]
public class EnchansedNavigation : MonoBehaviour
{
    [Header("Debug")]
    public bool DEBUG = false;

    [Header("Scanning Settings")]
    public float scanRadius = 7f;
    public float maxScanRadiusMultiplier = 2f;
    public float raycastHeight = 5f;
    public float raycastDistance = 10f;
    public int rayCount = 31;
    public int waypointCount = 5;
    public float waypointReachThreshold = 1.8f;

    [Header("Spline Settings")]
    public int subdivisionsPerSegment = 8;

    [Header("Waypoint Prefab")]
    public GameObject waypointPrefab;

    private List<GameObject> waypoints = new List<GameObject>();
    private List<Vector3> splinePoints = new List<Vector3>();
    public IReadOnlyList<Vector3> SplinePoints => splinePoints;

    private SimulationHandler simHandler;
    public bool isScanning = false;

    private Vector3 lastScanPos;
    private Vector3 lastScanDir;

    private void Awake()
    {
        simHandler = GetComponent<SimulationHandler>();
    }

    private IEnumerator Start()
    {
        if (waypointPrefab == null)
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
        if (waypoints.Count > 0)
        {
            float dist = Vector3.Distance(rearAxle, waypoints[0].transform.position);
            if (DEBUG) Debug.Log($"Navigation | Update: Dist to WP[0] = {dist:F2}");
            Debug.DrawLine(rearAxle, waypoints[0].transform.position, Color.magenta);

            if (dist < waypointReachThreshold)
            {
                Destroy(waypoints[0]);
                waypoints.RemoveAt(0);
                if (DEBUG) Debug.Log($"Navigation | Update: Waypoint reached, {waypoints.Count} left");
                StartCoroutine(GenerateSingleWaypointAsync());
                return;
            }
            return;
        }

        if (waypoints.Count == 0)
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
        GameObject wp = Instantiate(waypointPrefab, wpPos, wpRotation);
        waypoints.Add(wp);

        Vector3 prevPos = lastScanPos;
        lastScanPos = wpPos;
        lastScanDir = (wpPos - prevPos).normalized;

        RegenerateSpline();

        isScanning = false;
        yield return null;
    }

    private void RegenerateSpline()
    {
        splinePoints.Clear();
        List<Vector3> control = new List<Vector3>();
        Vector3 rearAxle = simHandler.GetRearAxlePosition();
        control.Add(rearAxle);
        foreach (var wp in waypoints)
            control.Add(wp.transform.position);

        control.Insert(0, control[0]);
        control.Add(control[control.Count - 1]);

        for (int i = 0; i < control.Count - 3; i++)
        {
            Vector3 p0 = control[i];
            Vector3 p1 = control[i + 1];
            Vector3 p2 = control[i + 2];
            Vector3 p3 = control[i + 3];
            for (int j = 0; j <= subdivisionsPerSegment; j++)
            {
                float t = j / (float)subdivisionsPerSegment;
                splinePoints.Add(CatmullRom(p0, p1, p2, p3, t));
            }
        }
    }

    private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        return 0.5f * (
            (2f * p1) +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * t * t +
            (-p0 + 3f * p1 - 3f * p2 + p3) * t * t * t
        );
    }

    public Vector3 GetLookAheadPoint(float distanceAhead)
    {
        Vector3 rear = simHandler.GetRearAxlePosition();
        float accumulated = 0f;

        // Step 1: Find the first spline point ahead of the rear axle
        int startIndex = -1;
        float closestDist = float.MaxValue;

        for (int i = 0; i < splinePoints.Count; i++)
        {
            Vector3 toPoint = splinePoints[i] - rear;
            float dot = Vector3.Dot(transform.forward, toPoint.normalized);

            // Skip points behind or nearly perpendicular
            if (dot < 0.3f)
                continue;

            float dist = toPoint.magnitude;
            if (dist < closestDist)
            {
                closestDist = dist;
                startIndex = i;
            }
        }

        // Safety check if no forward point was found
        if (startIndex == -1)
            return splinePoints.Count > 0 ? splinePoints[^1] : rear;

        // Step 2: Walk forward along the spline accumulating distances
        Vector3 currentPoint = splinePoints[startIndex];

        for (int i = startIndex; i < splinePoints.Count - 1; i++)
        {
            Vector3 nextPoint = splinePoints[i + 1];
            float segmentDist = Vector3.Distance(currentPoint, nextPoint);

            accumulated += segmentDist;

            if (accumulated >= distanceAhead)
                return nextPoint;

            currentPoint = nextPoint;
        }

        // If we run out of spline, return the last point
        return splinePoints[^1];
    }


}
