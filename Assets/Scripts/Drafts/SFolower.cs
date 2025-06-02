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
        Vector3 target = navigation.GetLookAheadPoint();

        // Steering
        Vector3 dirToTarget = (target - rearPos).normalized;
        float steerAngle = Vector3.SignedAngle(transform.forward, dirToTarget, Vector3.up);
        simHandler.SetSteeringAngle(steerAngle);


        Vector3 lookAheadPoint = navigation.GetLookAheadPoint();
        float distanceToLookAhead = Vector3.Distance(rearPos, lookAheadPoint);
        Vector3 toTarget = (lookAheadPoint - rearPos).normalized;
        float approachSpeed = Vector3.Dot(simHandler.carRigidbody.velocity, toTarget); // velocity toward the target
        approachSpeed = Mathf.Max(0f, approachSpeed); // only forward motion

        // Predictive time-to-reach
        float timeToReach = distanceToLookAhead / Mathf.Max(approachSpeed, 0.1f);

        // Dynamic desired speed based on curvature
        int splineIndex = navigation.splineManager.FindClosestSplinePoint(rearPos);
        float curvature = navigation.splineManager.CalculateCurvature(splineIndex);
        float normalizedCurvature = Mathf.Clamp01(1f - curvature / Mathf.Max(0.0001f, navigation.splineManager.curvatureThreshold));
        float curveSpeed = desiredSpeed * Mathf.Lerp(minSpeedMultiplier, maxSpeedMultiplier, normalizedCurvature);

        // Optional: scale down further if approaching too fast
        if (timeToReach < 1.0f && approachSpeed > curveSpeed)
        {
            curveSpeed *= 0.7f;
        }

        float currentSpeed = simHandler.CURRENT_SPEED;

        if (currentSpeed < curveSpeed)
        {
            float throttlePercent = Mathf.Lerp(20f, 60f, (curveSpeed - currentSpeed) / curveSpeed);
            simHandler.SetThrottlePercent(throttlePercent);
            simHandler.SetBrakePercent(0f);
        }
        else
        {
            // Make curvature affect brake aggressiveness
            float curvatureBoost = Mathf.Clamp01(curvature / navigation.splineManager.curvatureThreshold); // 0–1
            float baseBrake = Mathf.Lerp(10f, 100f, (currentSpeed - curveSpeed) / curveSpeed);

            // Boost brake % by curvature
            float brakePercent = baseBrake * Mathf.Lerp(1f, 100f, curvatureBoost); // up to 150% brake in sharp turns
            
            simHandler.SetBrakePercent(brakePercent);
        }

    }



    private void OnDrawGizmos()
    {
        if (navigation == null || simHandler == null) return;

        Vector3 rear = simHandler.GetRearAxlePosition();
        Vector3 lookAhead = navigation.GetLookAheadPoint();

        Gizmos.color = Color.red;
        Gizmos.DrawSphere(lookAhead, 0.3f);
        Gizmos.DrawLine(rear, lookAhead);
    }
}