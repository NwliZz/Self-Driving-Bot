using System.Collections;
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

        rearAxle = simHandler.GetRearAxlePosition();

        waypointManager.CreatePath(rearAxle);
        splineManager.GenerateSplinePath();

    }

    private void Update()
    {
        rearAxle = simHandler.GetRearAxlePosition();

        waypointManager.rearAxle = rearAxle;
        splineManager.rearAxle = rearAxle;

        if (waypointManager.isScanning) return;


        if (waypointManager.Waypoints.Count > 2)
        {
            //TargetWayoint
            Debug.DrawLine(rearAxle, waypointManager.Waypoints[2].transform.position, Color.magenta);

            if (DEBUG) Debug.Log($"Navigation | Update: Dist to WP[0] = {Vector3.Distance(rearAxle, waypointManager.Waypoints[1].transform.position):F2}");

            //if (waypointManager.wpCounter >= 4)
            //{
            //    RegenSpline();
            //    waypointManager.wpCounter = 0;
            //}
        }


    }

    private void UpdateSplinePath()
    {
        //- SPLINE PATH LOGIC
        //splineManager.RemoveSegment();
        //splineManager.AddSegment(waypointManager.Waypoints);



    }

    public Vector3 GetLookAheadPoint()
    {
        if (waypointManager.isScanning)
        {
            Debug.LogWarning("Navigation | GetLookAheadPoint: Cannot get point, Waypoint Manager is scanning...");
            return rearAxle;
        }
        else
        {
            return splineManager.GetLookAheadPoint(rearAxle, simHandler.CURRENT_SPEED);
        }
    }
}