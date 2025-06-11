using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Scan : MonoBehaviour
{
    [Header("Scanning Settings")]
    public bool DEBUG;
    public float debugRayDuration = 1.0f;
    //Data for the lasr placment of the wp
    private Vector3 lastScanPos;
    private Vector3 lastScanDir;

    [Header("Scanning Settings")]
    //Scacing to each wp
    public float wpSpacing = 3f;

    public float scanRadius = 7f;
    private float raycastDistance = 10f;
    private float raycastHeight = 5f;
    private float maxScanRadiusMultiplier = 2f;

    //Rays of the scan
    private int rayCount = 31;

    // True if performing an ArcScan
    [HideInInspector] public bool isScanning = false;

    //Lane boundries
    Vector3 leftMost, rightMost;
    [Header("H-Scan Settings")]
    public bool hScan = false;

    public float scanDistanceInFront = 8f;

    public float horisontalLength = 3;
    public int horizontalRayCount = 12;

    public float Vlength = 4;
    public int verticalRayCount = 8;

    private SimulationHandler simulationHandler;

    private void Start()
    {
        simulationHandler = GetComponent<SimulationHandler>();
        lastScanPos = simulationHandler.GetRearAxlePosition();
        lastScanDir = transform.forward;
    }

    private void Update()
    {
        Hscannow();
    }

    public void Hscannow()
    {
        if (hScan)
        {
            HScan();
            hScan = false;
        }
    }


    private void HScan()
    {
        List<Vector3> horizontalPoints = new List<Vector3>();
        List<Vector3> leftVerticalPoints = new List<Vector3>();
        List<Vector3> rightVerticalPoints = new List<Vector3>();

        float currentLength = horisontalLength;
        bool leftHit, rightHit;

        Vector3 scanOrigin = lastScanPos + lastScanDir * scanDistanceInFront;

        //Get the road center and center the scan
        Vector3 detectedRoadCenter = FindRoadCenter(scanOrigin, lastScanDir);
        Vector3 leftEnd = detectedRoadCenter - transform.right * (currentLength / 2f);
        Vector3 rightEnd = detectedRoadCenter + transform.right * (currentLength / 2f);

        // ---- Draw the entire horizontal line for visualization ----
        Debug.DrawLine(leftEnd + Vector3.up * raycastHeight, rightEnd + Vector3.up * raycastHeight, Color.magenta, debugRayDuration);

        Vector3 direction = (rightEnd - leftEnd).normalized;
        float step = Vector3.Distance(leftEnd, rightEnd) / (horizontalRayCount - 1);

        // ---- Shoot rays along the crossbar ----
        horizontalPoints.Clear();
        Vector3 sentinel = new Vector3(float.NaN, float.NaN, float.NaN);

        for (int i = 0; i <= horizontalRayCount; i++)
        {
            //float t = (float)i / (horizontalRayCount -1);
            //Vector3 origin = Vector3.Lerp(leftEnd, rightEnd, t) + Vector3.up * raycastHeight;
            Vector3 origin = leftEnd + direction * (i * step) + Vector3.up * raycastHeight;

            if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, raycastDistance) && hit.collider.CompareTag("Road"))
            {
                Debug.DrawLine(origin, hit.point, Color.green, debugRayDuration);
                horizontalPoints.Add(hit.point);
            }
            else
            {
                horizontalPoints.Add(sentinel);
            }
        }

        leftHit = horizontalPoints.Exists(p => Vector3.Distance(p, leftEnd) < 0.5f);
        rightHit = horizontalPoints.Exists(p => Vector3.Distance(p, rightEnd) < 0.5f);

        // Vertical line scanning at left end
        Vector3 leftStart = leftEnd - transform.forward * (Vlength / 2f);
        Vector3 leftEndVert = leftEnd + transform.forward * (Vlength / 2f);
        Vector3 vertDir = (leftEndVert - leftStart).normalized;
        float vertStep = Vector3.Distance(leftStart, leftEndVert) / (verticalRayCount - 1);

        for (int i = 0; i < verticalRayCount; i++)
        {
            Vector3 origin = leftStart + vertDir * (i * vertStep) + Vector3.up * raycastHeight;
            Debug.DrawRay(origin, Vector3.down * raycastDistance, Color.blue, debugRayDuration);
            if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, raycastDistance) && hit.collider.CompareTag("Road"))
            {
                Debug.DrawLine(origin, hit.point, Color.green, debugRayDuration);
                leftVerticalPoints.Add(hit.point);
            }
            else
            {
                leftVerticalPoints.Add(sentinel);
            }
        }

        // Vertical line scanning at right end
        Vector3 rightStart = rightEnd - transform.forward * (Vlength / 2f);
        Vector3 rightEndVert = rightEnd + transform.forward * (Vlength / 2f);
        vertDir = (rightEndVert - rightStart).normalized;
        vertStep = Vector3.Distance(rightStart, rightEndVert) / (verticalRayCount - 1);

        for (int i = 0; i < verticalRayCount; i++)
        {
            Vector3 origin = rightStart + vertDir * (i * vertStep) + Vector3.up * raycastHeight;
            Debug.DrawRay(origin, Vector3.down * raycastDistance, Color.yellow, debugRayDuration);
            if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, raycastDistance) && hit.collider.CompareTag("Road"))
            {
                Debug.DrawLine(origin, hit.point, Color.green, debugRayDuration);
                rightVerticalPoints.Add(hit.point);
            }
            else
            {
                rightVerticalPoints.Add(sentinel);
            }
        }


        lastScanPos = detectedRoadCenter;

        LaneHandler(horizontalPoints, leftVerticalPoints, rightVerticalPoints); 
    }
    private void LaneHandler(List<Vector3>Center, List<Vector3>Left, List<Vector3>Right)
    {
        bool c = false, l = false, r = false;

        c = IsLaneAvailable(Center);
        l = IsLaneAvailable(Left);
        r = IsLaneAvailable(Right);

        Debug.Log($"Straight is available: {c} |#| Left Turn is Available: {l} |#| Right turn is available: {r}");

    }

    bool IsLaneAvailable(List<Vector3> points)
    {
        // Helper to check for your sentinel value (all NaN)
        bool IsSentinel(Vector3 v) => float.IsNaN(v.x) && float.IsNaN(v.y) && float.IsNaN(v.z);

        if (points.Count == 0) return false;

        int firstAvailable = -1;
        int lastAvailable = -1;

        // Find first and last valid (non-sentinel) indices
        for (int i = 0; i < points.Count; i++)
        {
            if (!IsSentinel(points[i]))
            {
                if (firstAvailable == -1) firstAvailable = i;
                lastAvailable = i;
            }
        }

        // If no valid points at all, or only one, lane is blocked
        if (firstAvailable == -1 || lastAvailable == -1 || lastAvailable == firstAvailable)
            return false;

        // If the first available is not at start and last available is not at end, we have a "gap" in the middle
        if (firstAvailable > 0 && lastAvailable < points.Count - 1)
        {
            // There are at least two available points in the middle, surrounded by sentinels
            return (lastAvailable - firstAvailable) >= 1; // You can adjust this threshold if needed
        }

        // If available region touches either end, the lane is NOT currently available for entry
        return false;
    }

    //Scans in a Arc shape scanRadius whide, raycastDistance far made of rayCount vertical Raycasts which try to hit the road. If the first or last does not hit the scan whidens maxScanRadiusMultiplier times. and returns that as a List of Vector3 positions
    private List<Vector3> ArcScan()
    {
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

                Vector3 origin = lastScanPos + dir.normalized * wpSpacing + Vector3.up * raycastHeight;

                Debug.DrawRay(origin, Vector3.down * raycastDistance, Color.cyan, 0.1f);

                if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, raycastDistance) && hit.collider.CompareTag("Road"))
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
                //Debug.Log($"WaypointManager | ArcScan retry {retries}, radius {radius:F2}");
            }
        } while (retry);

        if (roadHits.Count == 0)
        {
            //Debug.LogError("WaypointManager | ArcScan: No road hits detected");
            isScanning = false;
            return roadHits;
        }
        return roadHits;
    }

    //Scans and finds road center - Todo: retries.
    public Vector3 FindRoadCenter(Vector3 scanOrigin, Vector3 scanDir)
    {
        Vector3 leftEnd = scanOrigin - transform.right * (15 / 2f);
        Vector3 rightEnd = scanOrigin + transform.right * (15 / 2f);

        List<Vector3> roadHits = new List<Vector3>();
        for (int i = 0; i < horizontalRayCount; i++)
        {
            float t = (float)i / (horizontalRayCount - 1);
            Vector3 origin = Vector3.Lerp(leftEnd, rightEnd, t) + Vector3.up * raycastHeight;

            if (DEBUG) Debug.DrawRay(origin, Vector3.down * raycastDistance, Color.cyan, debugRayDuration);

            if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, raycastDistance) && hit.collider.CompareTag("Road"))
            {
                if (DEBUG) Debug.DrawLine(origin, hit.point, Color.green, debugRayDuration);
                roadHits.Add(hit.point);
            }
        }

        if (roadHits.Count < 2)
        {
            // Not enough hits, return the original scan origin as fallback
            return scanOrigin;
        }

        // Find leftmost and rightmost hit along the local right axis
        Vector3 rightAxis = transform.right;
        Vector3 leftMost = roadHits[0];
        Vector3 rightMost = roadHits[0];
        float minProj = Vector3.Dot(rightAxis, leftMost);
        float maxProj = minProj;

        foreach (var pt in roadHits)
        {
            float proj = Vector3.Dot(rightAxis, pt);
            if (proj < minProj) { minProj = proj; leftMost = pt; }
            if (proj > maxProj) { maxProj = proj; rightMost = pt; }
        }

        Vector3 center = (leftMost + rightMost) * 0.5f;

        Debug.DrawLine(leftMost + Vector3.up * 2, rightMost + Vector3.up * 2, Color.yellow, 2f);
        Debug.DrawRay(center + Vector3.up * 2, Vector3.down * 2, Color.magenta, 2f); // Center marker

        return center;
    }
}
