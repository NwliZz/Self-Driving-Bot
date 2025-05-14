using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Navigation))]
[RequireComponent(typeof(SimulationHandler))]
public class SplineFollower : MonoBehaviour
{
    [Header("Follow Settings")]
    [Tooltip("Brake while nav is scanning or out of points")]
    public float holdBrakePercent = 100f;
    [Tooltip("Desired cruising speed (km/h)")]
    public float desiredSpeed = 15f;
    [Tooltip("How close to each spline sample before we advance")]
    public float sampleReachThreshold = 1.0f;

    [Tooltip("How far ahead to look for the next sample")]
    public float lookAheadDistance = 5f; // e.g. 5f

    EnchansedNavigation navigation;
    SimulationHandler simHandler;
    IReadOnlyList<Vector3> spline;
    int idx;

    void Start()
    {
        navigation = GetComponent<EnchansedNavigation>();
        simHandler = GetComponent<SimulationHandler>();
        idx = 0;
    }

    void FixedUpdate()
    {
        // 1) Pause whenever nav is still scanning
        if (navigation.isScanning)
        {
            simHandler.SetThrottlePercent(0f);
            simHandler.SetBrakePercent(holdBrakePercent);
            return;
        }

        // 2) Grab the current spline (may have been regenerated)
        spline = navigation.SplinePoints;
        if (spline == null || spline.Count == 0)
        {
            // no path yet: hold brakes
            simHandler.SetThrottlePercent(0f);
            simHandler.SetBrakePercent(holdBrakePercent);
            return;
        }

        // 3) Ensure idx in range
        if (idx >= spline.Count) idx = spline.Count - 1;

        //Target
        Vector3 target = navigation.GetLookAheadPoint(lookAheadDistance); // e.g. 5f

        // Compute direction to current sample
        Vector3 rearPos = simHandler.GetRearAxlePosition();

        Vector3 toT = target - rearPos;
        float dist = toT.magnitude;

        // 5) If we're close enough, advance to the next sample
        if (dist < sampleReachThreshold)
        {
            idx = Mathf.Min(idx + 1, spline.Count - 1);
            return;
        }

        // 6) STEER: angle toward the sample
        Vector3 dir = (target - rearPos).normalized;
        float steerDeg = Vector3.SignedAngle(transform.forward, dir, Vector3.up);
        simHandler.SetSteeringAngle(steerDeg);

        // 7) THROTTLE/BRAKE Control
        float speed = simHandler.CURRENT_SPEED;
        if (speed < desiredSpeed)
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
    // Draw the spline path in the Scene view
    private void OnDrawGizmos()
    {
        if (navigation == null || navigation.SplinePoints == null) return;

        Vector3 rear = simHandler.GetRearAxlePosition();
        Vector3 lookAhead = navigation.GetLookAheadPoint(lookAheadDistance);

        Gizmos.color = Color.red;
        Gizmos.DrawSphere(lookAhead, 0.3f);
        Gizmos.DrawLine(rear, lookAhead);
    }
}
