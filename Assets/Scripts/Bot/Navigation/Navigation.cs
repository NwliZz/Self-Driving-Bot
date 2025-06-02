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
        if (waypointManager.waypointPrefab == null)
        {
            Debug.LogError($"{gameObject.name} | waypointPrefab IS NOT ASSIGNED!");
            return;
        }
        rearAxle = simHandler.GetRearAxlePosition();

        //Create a start path of waypoints
        waypointManager.CreatePath(rearAxle);

        //Build a spline path on top of the waypoints
        splineManager.GenerateSplinePath();
    }

    private void Update()
    {
        //In case the spline path did not created
        if(splineManager.SplinePoints.Count < 4)
        {
            splineManager.GenerateSplinePath();
        }

        rearAxle = simHandler.GetRearAxlePosition();

        //Update the re rear position
        waypointManager.rearAxle = rearAxle;
        splineManager.rearAxle = rearAxle;

        if (waypointManager.isScanning) return;

        if (waypointManager.Waypoints.Count > 2 && DEBUG)
        {
            //TargetWayoint
            Debug.DrawLine(rearAxle, waypointManager.Waypoints[2].transform.position, Color.magenta);
            Debug.Log($"Navigation | Distance to Waypoint = {Vector3.Distance(rearAxle, waypointManager.Waypoints[1].transform.position):F2}");
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