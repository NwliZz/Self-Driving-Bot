using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Generates a smooth path through the next N waypoints using a Catmull-Rom spline,
/// then drives the car along that curved line for smoother steering.
/// </summary>
[RequireComponent(typeof(Navigation))]
[RequireComponent(typeof(SimulationHandler))]
public class CurvedWaypointFollower : MonoBehaviour
{
    [Header("Settings")]
    [Tooltip("Target cruise speed (km/h)")]
    public float desiredSpeed = 15f;
    [Tooltip("Distance threshold to each spline sample before advancing")]
    public float sampleReachThreshold = 1.5f;
    [Tooltip("Number of subdivisions per segment (higher = smoother)")]
    public int subdivisionsPerSegment = 8;

    private Navigation navigation;
    private SimulationHandler simHandler;

    // Full list of sampled spline points
    private List<Vector3> splinePoints = new List<Vector3>();
    private int currentSplineIndex = 0;

    private void Start()
    {
        navigation = GetComponent<Navigation>();
        simHandler = GetComponent<SimulationHandler>();
        RegenerateSplinePath();
    }

    private void FixedUpdate()
    {
        // Wait until navigation scanning is done
        //if (navigation.IsScanning)
        {
            simHandler.SetThrottlePercent(0f);
            simHandler.SetBrakePercent(100f);
            return;
        }

        // If spline is empty or we've reached the end, regenerate
        if (splinePoints.Count == 0 || currentSplineIndex >= splinePoints.Count)
        {
            RegenerateSplinePath();
            return;
        }

        // Current target on the spline
        Vector3 targetPos = splinePoints[currentSplineIndex];
        Vector3 rearAxle = simHandler.GetRearAxlePosition();
        Vector3 toTarget = targetPos - rearAxle;
        float dist = toTarget.magnitude;
        Vector3 dir = toTarget.normalized;

        // Advance index if close enough to this sample
        if (dist < sampleReachThreshold)
        {
            currentSplineIndex++;
            return;
        }

        // Steering: point toward the next spline sample
        float steerAngle = Vector3.SignedAngle(transform.forward, dir, Vector3.up);
        simHandler.SetSteeringAngle(steerAngle);

        // Throttle/Brake to maintain desired speed
        float currentSpeed = simHandler.CURRENT_SPEED;
        if (currentSpeed < desiredSpeed)
        {
            simHandler.SetThrottlePercent(60f);
            simHandler.SetBrakePercent(0f);
        }
        else
        {
            simHandler.SetThrottlePercent(0f);
            simHandler.SetBrakePercent(10f);
        }
    }

    /// <summary>
    /// Builds a Catmull-Rom spline through the upcoming waypoints plus the car's current position.
    /// </summary>
    public void RegenerateSplinePath()
    {
        // Clear old data
        splinePoints.Clear();
        currentSplineIndex = 0;

        // Gather control points: rear axle, then next N waypoints
        List<Vector3> control = new List<Vector3>();
        Vector3 rearAxle = simHandler.GetRearAxlePosition();
        control.Add(rearAxle);

        //var wps = navigation.GetAllWaypoints(); // assume returns List<GameObject>
        //if (wps == null || wps.Count == 0)
            return;

        // Add waypoint positions
        //foreach (var wp in wps)
            //control.Add(wp.transform.position);

        // Duplicate first and last to satisfy Catmull-Rom endpoint conditions
        control.Insert(0, control[0]);
        control.Add(control[control.Count - 1]);

        // Generate spline samples for each segment
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

    /// <summary>
    /// Catmull-Rom interpolation between P1 and P2 with tangents from P0 and P3.
    /// </summary>
    private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        // Standard Catmull-Rom formula
        return 0.5f * (
            (2f * p1) +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * t * t +
            (-p0 + 3f * p1 - 3f * p2 + p3) * t * t * t
        );
    }
}
