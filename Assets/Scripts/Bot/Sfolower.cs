using UnityEngine;

[RequireComponent(typeof(Navigation))]
[RequireComponent(typeof(SimulationHandler))]
public class SFollower : MonoBehaviour
{
    [Header("Follow Settings")]
    [Tooltip("Brake while nav is scanning or out of points")]
    public float holdBrakePercent = 100f;
    [Tooltip("Desired cruising speed (km/h)")]
    public float desiredSpeed = 15f;
    [Tooltip("How far ahead to look for the next sample")]
    //public float lookAheadDistance = 5f;

    [Header("Adaptive Speed Control")]
    public float minSpeedMultiplier = 0.5f;
    [Tooltip("Maximum speed multiplier for straight paths")]
    public float maxSpeedMultiplier = 1.0f;

    private Navigation navigation;
    private SimulationHandler simHandler;

    void Start()
    {
        navigation = GetComponent<Navigation>();
        simHandler = GetComponent<SimulationHandler>();
    }

    void FixedUpdate()
    {
        if (navigation.waypointManager.isScanning)
        {
            simHandler.SetThrottlePercent(0f);
            simHandler.SetBrakePercent(holdBrakePercent);
            return;
        }

        Vector3 rearPos = simHandler.GetRearAxlePosition();
        Vector3 target = navigation.splineManager.GetLookAheadPoint(rearPos);

        // Steering
        Vector3 dirToTarget = (target - rearPos).normalized;
        float steerAngle = Vector3.SignedAngle(transform.forward, dirToTarget, Vector3.up);
        simHandler.SetSteeringAngle(steerAngle);

        // Speed control
        float curvature = navigation.splineManager.CalculateCurvature(navigation.splineManager.FindClosestSplinePoint(rearPos));
        float normalized = Mathf.Clamp01(1f - (curvature / Mathf.Max(0.0001f, navigation.splineManager.curvatureThreshold)));
        float speedMultiplier = Mathf.Lerp(minSpeedMultiplier, maxSpeedMultiplier, normalized);
        float adjustedDesiredSpeed = desiredSpeed * speedMultiplier;

        float currentSpeed = simHandler.CURRENT_SPEED;

        if (currentSpeed < adjustedDesiredSpeed)
        {
            float throttlePercent = Mathf.Lerp(20f, 60f, (adjustedDesiredSpeed - currentSpeed) / adjustedDesiredSpeed);
            simHandler.SetThrottlePercent(throttlePercent);
            simHandler.SetBrakePercent(0f);
        }
        else
        {
            simHandler.SetThrottlePercent(0f);
            float brakePercent = Mathf.Lerp(5f, 100f, (currentSpeed - adjustedDesiredSpeed) / adjustedDesiredSpeed);
            simHandler.SetBrakePercent(brakePercent);
        }
    }



    private void OnDrawGizmos()
    {
        if (navigation == null || simHandler == null) return;

        Vector3 rear = simHandler.GetRearAxlePosition();
        Vector3 lookAhead = navigation.splineManager.GetLookAheadPoint(rear);

        Gizmos.color = Color.red;
        Gizmos.DrawSphere(lookAhead, 0.3f);
        Gizmos.DrawLine(rear, lookAhead);
    }
}