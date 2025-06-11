using UnityEngine;
using System.Collections.Generic;

public class PurePursuitController : MonoBehaviour
{
    [Header("Pure Pursuit Settings")]
    public float lookaheadDistance = 5.0f;  // Tune as needed
    public bool pathFollowingActive = false;

    private float lastSteeringAngle = 0f;
    public float steeringSmooth = 0.8f;

    [Header("PID Settings")]
    public float desiredSpeed = 10f;
    public float proportionalGain = 1.0f;
    public float integralGain = 0f;
    public float derivativeGain = 0f;
    private float integral;
    private float lastError;

    private SimulationHandler simHandler;
    private SplineManager splineManager;

    //Debug
    private Vector3 lastRearAxle;
    private Vector3 lastLookaheadPoint;
    //

    // Initialization
    void Start()
    {
        simHandler = GetComponent<SimulationHandler>();
        splineManager = GetComponent<SplineManager>();
    }

    // Call this to activate path following
    public void StartPathFollowing()
    {
        pathFollowingActive = true;
    }

    // Call this to stop path following (e.g., when stopping for obstacles)
    public void StopPathFollowing()
    {
        pathFollowingActive = false;
        simHandler.SetSteeringAngle(0);
        simHandler.SetThrottlePercent(0);
        simHandler.SetBrakePercent(100);
    }

    // Main Unity loop
    void FixedUpdate()
    {
        if (!pathFollowingActive) return;

        DoPurePursuitSteering();
        DoPIDSpeedControl();
    }

    // --- STEERING: Pure Pursuit ---
    private void DoPurePursuitSteering()
    {
        Vector3 rearAxle = simHandler.GetRearAxlePosition();

        Vector3 lookaheadPoint = FindLookaheadPoint(rearAxle, lookaheadDistance);

        lastRearAxle = rearAxle;
        lastLookaheadPoint = lookaheadPoint;

        Vector3 toLookahead = lookaheadPoint - rearAxle;
        float ld = toLookahead.magnitude;
        if (ld < 0.1f) return;

        float angleToLookahead = Vector3.SignedAngle(transform.forward, toLookahead, Vector3.up) * Mathf.Deg2Rad;
        float L = simHandler.wheelbase;

        float newSteeringAngle = Mathf.Atan2(2 * L * Mathf.Sin(angleToLookahead), ld) * Mathf.Rad2Deg;
        newSteeringAngle = Mathf.Clamp(newSteeringAngle, -simHandler.maxSteerAngle, simHandler.maxSteerAngle);

        // Lowpass filter the steering
        float steeringAngle = steeringSmooth * lastSteeringAngle + (1 - steeringSmooth) * newSteeringAngle;
        lastSteeringAngle = steeringAngle;
            simHandler.SetSteeringAngle(steeringAngle);
        Debug.DrawLine(rearAxle, lookaheadPoint, Color.magenta);
    }

    // --- SPEED: PID Control ---
    private void DoPIDSpeedControl()
    {
        float currentSpeed = simHandler.CURRENT_SPEED / 3.6f; // Convert to m/s if needed
        float speedControl = PIDControl(desiredSpeed, currentSpeed);

        if (speedControl >= 0)
        {
            simHandler.SetThrottlePercent(Mathf.Clamp(speedControl, 0, 100));
            simHandler.SetBrakePercent(0);
        }
        else
        {
            simHandler.SetThrottlePercent(0);
            simHandler.SetBrakePercent(Mathf.Clamp(-speedControl, 0, 100));
        }
    }

    private float PIDControl(float setpoint, float measured)
    {
        float error = setpoint - measured;
        integral += error * Time.deltaTime;
        float derivative = (error - lastError) / Time.deltaTime;
        lastError = error;
        return proportionalGain * error + integralGain * integral + derivativeGain * derivative;
    }

    // --- PATH LOGIC: Find Lookahead Point ---
    private Vector3 FindLookaheadPoint(Vector3 rearAxle, float lookaheadDist)
    {
        var points = splineManager.SplinePoints;
        for (int i = 0; i < points.Count; i++)
        {
            if (Vector3.Distance(rearAxle, points[i]) > lookaheadDist)
                return points[i];
        }
        // If not found (e.g., end of path), return last
        return points[points.Count - 1];
    }

    private void OnDrawGizmos()
    {
        // Draw rear axle and lookahead point only if initialized
        if (lastLookaheadPoint != Vector3.zero && lastRearAxle != Vector3.zero)
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawLine(lastRearAxle, lastLookaheadPoint);

            Gizmos.color = Color.green;
            Gizmos.DrawSphere(lastLookaheadPoint, 0.3f);

            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(lastRearAxle, 0.2f);
        }

        // Optional: Draw the entire spline
        if (splineManager != null && splineManager.SplinePoints != null && splineManager.SplinePoints.Count > 1)
        {
            Gizmos.color = Color.yellow;
            var points = splineManager.SplinePoints;
            for (int i = 0; i < points.Count - 1; i++)
                Gizmos.DrawLine(points[i], points[i + 1]);
        }
    }
}

//Vector3 currentPos = simHandler.GetRearAxlePosition();
//float currentSpeed = simHandler.CURRENT_SPEED / 3.6f; // convert to m/s
//Vector3 forwardDir = transform.forward;

// --- STEERING ---
//List<Vector3> predictedPoints = new List<Vector3>();
//float dt = predictionHorizon / steps;
//float simulatedSpeed = currentSpeed;

//Vector3 pos = currentPos;
//Vector3 dir = forwardDir;

//// Prediction loop
//for (int i = 0; i < steps; i++)
//{
//    float lookAheadDistance = simulatedSpeed * dt;
//    Vector3 targetPoint = splineManager.GetLookAheadPoint(pos, simulatedSpeed);

//    Vector3 toTarget = (targetPoint - pos).normalized;
//    float angleToTarget = Vector3.SignedAngle(dir, toTarget, Vector3.up) * Mathf.Deg2Rad;

//    float steerAngle = Mathf.Atan2(2f * wheelbase * Mathf.Sin(angleToTarget), lookAheadDistance);

//    // Simple update using bicycle model
//    dir = Quaternion.Euler(0, steerAngle * Mathf.Rad2Deg * dt, 0) * dir;
//    pos += dir * simulatedSpeed * dt;

//    predictedPoints.Add(pos);
//}

//// Compute immediate steering
//Vector3 immediateTarget = predictedPoints[0];
//Vector3 immediateDirection = (immediateTarget - currentPos).normalized;
//float immediateAngle = Vector3.SignedAngle(forwardDir, immediateDirection, Vector3.up);