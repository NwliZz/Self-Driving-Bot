using UnityEngine;

[RequireComponent(typeof(SimulationHandler))]
public class Navigation : MonoBehaviour
{
    //Refrences
    private SimulationHandler simHandler;
    [HideInInspector] public WaypointManager waypointManager;
    [HideInInspector]public SplineManager splineManager;

    //Debug
    public bool DEBUG = true;

    private Vector3 rearAxle;

    private void Awake()
    {
        simHandler = GetComponent<SimulationHandler>();
        waypointManager = GetComponent<WaypointManager>();
        splineManager = GetComponent<SplineManager>();
    }

    private void Start()
    {
        if (waypointManager == null || simHandler == null || splineManager == null)
        {
            Debug.LogError($"{gameObject.name} | waypointPrefab IS NOT ASSIGNED!");
            return;
        }

        rearAxle = simHandler.GetRearAxlePosition();

        waypointManager.CreatePath(rearAxle);
    }

    private void Update()
    {
        rearAxle = simHandler.GetRearAxlePosition();

        if (waypointManager.isScanning) return;

        if (waypointManager.Waypoints.Count > 2 && DEBUG)
        {
            //TargetWaypoint
            Debug.DrawLine(rearAxle, waypointManager.Waypoints[waypointManager.targetWaypointIdx].transform.position, Color.magenta);
            Debug.Log($"Navigation | Distance to Waypoint = {Vector3.Distance(rearAxle, waypointManager.Waypoints[waypointManager.targetWaypointIdx].transform.position):F2}");
        }
    }

    public Vector3 GetLookAheadPoint()
    {
        if (waypointManager.isScanning)
        {
            if (DEBUG) Debug.LogWarning("Navigation | GetLookAheadPoint: Cannot get look ahead point, Waypoint Manager is scanning...");

            return rearAxle;
        }
        else
        {
            if (DEBUG) Debug.LogWarning("Navigation | GetLookAheadPoint: Look ahead point updated...");
            
            return splineManager.GetLookAheadPoint(rearAxle, simHandler.CURRENT_SPEED);
        }
    }
}