using System.Collections.Generic;
using UnityEngine;

public class WaypointManager : MonoBehaviour
{
    private List<GameObject> waypoints = new List<GameObject>();
    public IReadOnlyList<GameObject> Waypoints => waypoints;
    [Header("General Settings")]
    public bool DEBUG;

    [Header("Scanning Settings")]
    public float debugRayDuration = 1.0f;

    //Scacing to each wp
    public float wpSpacing = 3f;
    public float scanRadius = 7f;

    private readonly float raycastDistance = 10f;
    private readonly float raycastHeight = 5f;
    private readonly float maxScanRadiusMultiplier = 2f;

    //Rays of the scan
    private readonly int rayCount = 31;

    // True if performing an ArcScan
    [HideInInspector] public bool isScanning = false;


    [Header("Waypoint Instantiation Settings")]

    //The waypoint wich is consindered the target of the Waypoints list.(It will start removing from the 3rd wp of the list)
    public readonly int targetWaypointIdx = 2;

    //How many wps will be in waypoint sliding window
    public int waypointCount = 5;

    //- for right, + for left
    public float laneOffset;

    //Lane boundries
    Vector3 leftMost, rightMost;

    //Data for the current placment of the wp
    private Vector3 wpPos;
    private Quaternion wpRotation;

    //Data for the lasr placment of the wp
    private Vector3 lastScanPos;
    private Vector3 lastScanDir;

    private Vector3 rearAxle;

    ////WP Arival 
    public float waypointReachThreshold = 1.8f;



    //Refrences
    private SplineManager splineManager;
    private SimulationHandler simulationHandler;
    public GameObject waypointPrefab;


    private void Start()
    {
        simulationHandler = GetComponent<SimulationHandler>();
        splineManager = GetComponent<SplineManager>();
        rearAxle = simulationHandler.GetRearAxlePosition();
        
    }

    private void FixedUpdate()
    {
        rearAxle = simulationHandler.GetRearAxlePosition();

        //Not Likely
        if (waypoints.Count == 0)
        {
            Debug.Log("Navigation | Update: No waypoints—regenerating full batch");
            CreatePath(rearAxle);
        }

        if (HasReachedWaypoint(rearAxle))
        {
            UpdatePath();
        }
    }

    //Creates a path of waypointCount Waypoints
    public void CreatePath(Vector3 rearAxle)
    {
        isScanning = true;

        //Don't like it, nothing i could do...
        splineManager = GetComponent<SplineManager>();

        if (DEBUG) Debug.Log("Creating Path of :" + waypointCount + " Waypoints...");
        if (waypointPrefab == null )
        {
            Debug.LogError("WaypointManager | Start: waypointPrefab is not assigned!");
            return;
        }

        //Initial scan orogin
        lastScanPos = rearAxle;
        lastScanDir = transform.forward;

        for (int i = 0; i < waypointCount; i++)
            AddWaypoint();

        splineManager.GenerateSplinePath(Waypoints);

        isScanning = false;

        if (DEBUG) Debug.Log("WaypointManager | CreatePath: Path instantiated.");
    }

    private void UpdatePath()
    {
        if (DEBUG) Debug.Log($"Navigation | UpdatePath: Updating path...");

        isScanning = true;

        splineManager.RemoveSegment();

        RemoveFirstWaypoint();

        AddWaypoint();

        splineManager.AddSegment(Waypoints);

        isScanning = false;
    }

    //Starts the proces of creating a waypoint
    public void AddWaypoint()
    {
        //Scan and find position for wp
        GetLanePosition(ArcScan());

        //Place wp
        CreateAndSaveWaypoint(wpPos, wpRotation);
    }

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

                if(DEBUG) Debug.DrawRay(origin, Vector3.down * raycastDistance, Color.cyan, 0.1f);

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

    //Find the right lane from Arc scan list
    private void GetLanePosition(List<Vector3> roadHits)
    {

        // Reference vector that points to the right of the vehicle's forward direction
        Vector3 rightAxis = Vector3.Cross(Vector3.up, lastScanDir).normalized;

        leftMost = roadHits[0];
        rightMost = roadHits[0];

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

        //Debug.Log($"WaypointManager | GetLanePosition: Placing waypoint at {wpPos}");

        Vector3 wpDirection = (wpPos - lastScanPos).normalized;
        wpRotation = Quaternion.LookRotation(wpDirection, Vector3.up);

        //Update scanning state
        Vector3 prevPos = lastScanPos;
        lastScanPos = wpPos;
        lastScanDir = (wpPos - prevPos).normalized;
    }

    // Removes first item in Waypoints list
    public bool RemoveFirstWaypoint()
    {
        if (waypoints.Count > 0)
        {
            Destroy(waypoints[0]);
            waypoints.RemoveAt(0);

            return true;
        }
        return false;
    }

    //Checks if the go Has Reached the next Waypoint in the list
    public bool HasReachedWaypoint(Vector3 position)
    {
        if (waypoints.Count > 0)
        {
            float distanceToWP = Vector3.Distance(position, waypoints[targetWaypointIdx].transform.position);
            if (distanceToWP <= waypointReachThreshold)
            {
                return true;
            }
        }
        return false;
    }

    //Instantiates a WP go and adds it to the list
    public void CreateAndSaveWaypoint(Vector3 position, Quaternion rotation)
    {
        GameObject wp = Instantiate(waypointPrefab, position, rotation);
        waypoints.Add(wp);
    }

    public float GetLaneWidth() => (Vector3.Distance((leftMost+rightMost)*0.5f, rightMost))*0.5f;
    
    //Clears the hole list

    public void ClearWaypoints()
    {
        foreach (var wp in waypoints)
        {
            Destroy(wp);
        }
        waypoints.Clear();
    }
}