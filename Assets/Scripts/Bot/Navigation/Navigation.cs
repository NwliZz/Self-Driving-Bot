using Mapbox.Directions;
using System.Collections;
using System.Collections.Generic;
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

    private int wpCounter;

    //private bool isFirstscan = true;

    private void Awake()
    {
        simHandler = GetComponent<SimulationHandler>();
        waypointManager = GetComponent<WaypointManager>();
        splineManager = GetComponent<SplineManager>();
    }

    private IEnumerator Start()
    {

        rearAxle = simHandler.GetRearAxlePosition();

        yield return StartCoroutine(waypointManager.CreatePath(rearAxle));
        RegenSpline();

        
        yield return null;

    }

    private void Update()
    {

        if (waypointManager.isScanning) return;


        //Curent WP Managment
        rearAxle = simHandler.GetRearAxlePosition();
        if (waypointManager.Waypoints.Count > 0)
        {

            if (DEBUG) Debug.Log($"Navigation | Update: Dist to WP[0] = {Vector3.Distance(rearAxle, waypointManager.Waypoints[0].transform.position):F2}");
            Debug.DrawLine(rearAxle, waypointManager.Waypoints[0].transform.position, Color.magenta);


            if (waypointManager.HasReachedWaypoint(rearAxle))
            {
                if (DEBUG) Debug.Log($"Navigation | Update: Waypoint reached, regenerating for que...");

                StartCoroutine(AddWP());
                RegenSpline();


                return;
            }
            return;
        }

       



        //Not Likely
        if (waypointManager.Waypoints.Count == 0)
        {
            Debug.Log("Navigation | Update: No waypoints—regenerating full batch");
            StartCoroutine(waypointManager.CreatePath(rearAxle));
        }
    }

    private IEnumerator AddWP()
    {
        yield return StartCoroutine(waypointManager.GenerateWaypoint());

    }

    public void RegenSpline()
    {
        splineManager.RegenerateSpline(rearAxle, waypointManager.Waypoints);
    }
}