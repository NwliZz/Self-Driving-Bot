using UnityEngine;

[RequireComponent(typeof(Navigation))]
[RequireComponent(typeof(SimulationHandler))]
public class BasicWaypointFollower : MonoBehaviour
{
    [Header("Settings")]
    public float desiredSpeed = 15f; // target speed (km/h)
    public float waypointReachThreshold = 1.8f; // stopping distance threshold

    private  EnchansedNavigation navigation;
    //private Navigation navigation;
    private SimulationHandler simHandler;

    private void Start()
    {
        navigation = GetComponent<EnchansedNavigation>();
        //navigation = GetComponent<Navigation>();
        simHandler = GetComponent<SimulationHandler>();
    }

    private void FixedUpdate()
    {
        if (navigation.isScanning)
        {
            Debug.Log("BasicWaypointFollower | FixedUpdate: Waiting for Navigation Scan");
            simHandler.SetThrottlePercent(0f);
            simHandler.SetBrakePercent(100f);
            return;
        }

//        GameObject targetWaypoint = navigation.CurrentTarget;


        Vector3 rearAxlePos = simHandler.GetRearAxlePosition();
        Vector3 targetWaypoint = Vector3.zero; // e.g. 5f
        float distance = Vector3.Distance(rearAxlePos, targetWaypoint);
        Vector3 directionToWaypoint = (targetWaypoint - rearAxlePos).normalized;
        if(distance < waypointReachThreshold)
        {
            Debug.Log("BasicWaypointFollower | FixedUpdate: Waypoint reached");
            return;
        }


        // Steering towards waypoint
        float steerAngle = Vector3.SignedAngle(transform.forward, directionToWaypoint, Vector3.up);
        simHandler.SetSteeringAngle(steerAngle);

        // Speed control

        if (simHandler.CURRENT_SPEED < desiredSpeed)
        {
            simHandler.SetThrottlePercent(60f); // Moderate throttle
            simHandler.SetBrakePercent(0f);
        }
        else if (simHandler.CURRENT_SPEED > desiredSpeed)
        {
            simHandler.SetThrottlePercent(0f);
            simHandler.SetBrakePercent(10f); // Slight braking if overspeed
        }
    }
}
