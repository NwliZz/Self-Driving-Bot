using System.Collections.Generic;
using UnityEngine;

public class NavigationSystem : MonoBehaviour
{
    [Header("Scanning Settings")]
    public float scanRadius = 7f;
    public float raycastHeight = 5f;
    public float raycastDistance = 10f;
    public int rayCount = 31;

    [Header("Obstacle Detection")]
    public LayerMask roadLayerMask;
    public LayerMask obstacleLayerMask;

    [Header("Waypoint")]
    public GameObject waypointPrefab;
    public GameObject currentTarget;

    private List<GameObject> waypoints = new List<GameObject>();

    //Road detection
    private List<Vector3> roadHits = new List<Vector3>();


    //Obstacle detection
    private List<Collider> obstacles = new List<Collider>();
    float closestObstacle;

    public float obstacleProximity01 { get; private set; }
    public float laneClarity01 { get; private set; }

    //SimulationHandler
    SimulationHandler simulationHandler;
    Vector3 rearAxlePos;


    private void Start()
    {
        //roadLayerMask = LayerMask.GetMask("Default");

        //obstacleLayerMask = LayerMask.GetMask("Obstacle");

        simulationHandler = GetComponent<SimulationHandler>();


        ArcScan();// Initialize the first waypoint

        //PlaceWaypointByScanning();
    }

    private void Update()
    {
        if (waypoints.Count == 0) return;

        //Vector3 carFront = transform.position + transform.forward * 2.4f;

        currentTarget = waypoints[0];

        rearAxlePos = simulationHandler.GetRearAxlePosition();

        float distToTarget = Vector3.Distance(rearAxlePos, currentTarget.transform.position);
        Debug.DrawLine(rearAxlePos, currentTarget.transform.position, Color.magenta);

        if (distToTarget < 1.8f)
        {
            Destroy(currentTarget);
            waypoints.RemoveAt(0);

            ArcScan();
        }
    }

    private void ArcScan()
    {
        float startAngle = -90f;
        float angleStep = 180f / (rayCount - 1);

        for (int i = 0; i < rayCount; i++)
        {
            float angle = startAngle + i * angleStep;
            Quaternion rotation = Quaternion.AngleAxis(angle, Vector3.up);
            Vector3 dir = rotation * transform.forward;

            Vector3 rayOrigin = transform.position + dir * scanRadius + Vector3.up * raycastHeight;
            Debug.DrawRay(rayOrigin, Vector3.down * raycastDistance, Color.cyan);

            
            if (Physics.Raycast(rayOrigin, Vector3.down, out RaycastHit hit, raycastDistance))
                HandleTarget(hit);
            else
                Debug.LogError("NavigationSystem | ArcScan: No hit detected!");
        }
        WaypointPlacement();
    }

    private void HandleTarget(RaycastHit hit)
    {
        switch (hit.collider.tag)
        {
            case "Road":
                roadHits.Add(hit.point);
                break;
            case "Obstacle":
                obstacles.Add(hit.collider);
                Debug.Log("Obstacle detected in hit and saved Collider");
                HandleObstacles();
                break;
            default:
                break;
        }
    }

    private void WaypointPlacement()
    {
        if (roadHits.Count > 0)
        {
            Vector3 leftMost = roadHits[0];
            Vector3 rightMost = roadHits[0];
            float minDot = float.MaxValue;
            float maxDot = float.MinValue;

            foreach (var point in roadHits)
            {
                Vector3 dir = (point - transform.position).normalized;
                float dot = Vector3.Dot(transform.right, dir);

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

            Vector3 middle = (leftMost + rightMost) / 2f;
            Vector3 rightLane = (middle + rightMost) / 2f;

            GameObject wp = Instantiate(waypointPrefab, rightLane + Vector3.up * 0.1f, Quaternion.identity);
            waypoints.Add(wp);

            roadHits.Clear(); // Clear road hits after placing a waypoint

            laneClarity01 = Mathf.Clamp01(roadHits.Count / (float)rayCount);
            //Debug.Log($"Lane clarity updated: {laneClarity01}");
        }

    }

    private void HandleObstacles()
    {
        float dist = Vector3.Distance(transform.position, obstacles[obstacles.Count -1].transform.position);
        if (dist < closestObstacle)
        {
            closestObstacle = dist;

            obstacleProximity01 = Mathf.Clamp01(1f - (closestObstacle / scanRadius));
            //Debug.Log($"Obstacle proximity updated: {closestObstacle}");

        }
    }
}









//_____________-----------------______________________--------------------_____________
//private float CalculateCurvature(Vector3 start, Vector3 end)
//{

//    Vector3 toEnd = end - start;
//    float distance = toEnd.magnitude;
//    Vector3 direction = toEnd / distance;
//    float angle = Vector3.Angle(transform.forward, direction);
//    return angle / distance;
//}