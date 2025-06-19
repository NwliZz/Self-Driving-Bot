using System.Collections.Generic;
using UnityEngine;

public class Actions : MonoBehaviour
{
    [Header("Object Detection")]
    public float stopDistanceHysteresis = 1.0f; // meters, tune as needed

    public float maxDetectDistance = 10f;
    public float maxAcceleration = 2.0f;
    public float maxBreaking = 5.0f;

    [Header("Path")]
    // ---Speed---
    public float desiredSpeed = 15f;
    public float minSpeedMultiplier = 0.5f;
    public float maxSpeedMultiplier = 1.0f;
    private float currentSpeed;
    [HideInInspector] public PIDController pathSpeedPID;

    // ---Vision---
    public float lookaheadGain = 0.5f;
    public float minLookAheadDistance = 3.7f;
    public float maxLookAheadDistance = 12f;
    private Vector3 lastLookAheadPoint;
    private Vector3 lookAheadPoint;

    //Turn aknowlagment curvature threshold
    public float curvatureThreshold;

    //PID waights
    public float motorPropGain = 2f, motorIntegralGain = 0.3f, motorDerivativeGain = 0.15f;

    private int clostestSplinePointIndx;

    // ---PurePursuit---
    private float lastSteeringAngle = 0f;
    public float steeringSmoothness = 0.8f;



    [Header("General")]
    public bool DEBUG;
    //General
    [HideInInspector] public Vector3 rearAxle;
    private float laneWidth;

    //To the end of the spline path
    [HideInInspector] public float detectionDistanceLength;


    //// ---Refrences---
    private SimulationHandler simHandler;
    private SplineManager splineManager;
    private WaypointManager waypointManager;
    private Calculations calculations;

    // Start is called before the first frame update
    void Start()
    {
        splineManager = GetComponent<SplineManager>();
        simHandler = GetComponent<SimulationHandler>();
        waypointManager = GetComponent<WaypointManager>();

        calculations = new Calculations();

        pathSpeedPID = new PIDController(motorPropGain, motorIntegralGain, motorDerivativeGain);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (splineManager.SplinePoints.Count < 10) return;

        UpdateVariables();
    }

    public ControlCommand GetCar(Car target)
    {
        if (target == null) return ControlCommand.Default;

        List<Vector3> crdr = calculations.GetDetectionCorridor(splineManager.SplinePoints, clostestSplinePointIndx);

        bool inCorridor = false;
        foreach (var point in crdr)
        {
            float lateralDist = Vector3.Distance(
                new Vector3(target.Position.x, 0, target.Position.z),
                new Vector3(point.x, 0, point.z));
            if (lateralDist < laneWidth)
            {
                inCorridor = true;
                break;
            }
        }

        if (inCorridor) return HandleCar(target);
        

        return ControlCommand.Default;
    }

    private ControlCommand HandleCar(Car target)
    {
        ControlCommand result = ControlCommand.Default;


        float dist = Vector3.Distance(rearAxle, target.Position);

        // Your car's speed
        float v_ego = simHandler.carRigidbody.velocity.magnitude;
        // The front car's speed projected in your direction
        float v_front = Vector3.Dot(target.Velocity, transform.forward);

        // IDM computes desired acceleration
        // Prevent division by zero if you are exactly at the car's position (shouldn't happen, but for safety)
        float s = Mathf.Max(dist, 0.1f);

        float accel = CalculateIDMAcceleration(v_ego, v_front, s);

        // Convert acceleration to throttle/brake percent
        // You may need to tune these scale factors for your simulator
        float throttle = Mathf.Clamp01(accel / 2.0f); // assuming 2 m/s^2 is full throttle
        float brake = Mathf.Clamp01(-accel / 4.0f);   // assuming -4 m/s^2 is full brake

        //Dynamic Prioritization as the risk is higher
        float basePriority = 35f;
        float maxPriority = 90f;

        // Calculate weights
        float decelWeight = Mathf.Clamp01(-accel / maxBreaking);
        float gapWeight = Mathf.Clamp01((detectionDistanceLength - (s - stopDistanceHysteresis)) / detectionDistanceLength);

        // Use the maximum of deceleration need or gap closeness
        float dynamicPriority = basePriority + (maxPriority - basePriority) * Mathf.Max(decelWeight, gapWeight);

        result.ThrottlePercent = throttle * 100f;
        result.BrakePercent = brake * 100f;
        result.SteeringAngle = 0; // No steering correction here (just following)
        result.Priority = Mathf.RoundToInt(dynamicPriority); // Boost priority if braking

        // Visual debug
        Debug.DrawLine(rearAxle, target.Position, accel < 0 ? Color.red : Color.yellow);

        return result;
    }

    public ControlCommand EvaluateTrafficLight(TrafficLight target)
    {
        ControlCommand result = ControlCommand.Default;

        if (target != null)
        {
            float dist = Vector3.Distance(rearAxle, target.Position);

            float distToStop = Mathf.Max(dist - stopDistanceHysteresis, 0);

            if (target.CurrentState == TrafficLight.State.Red || target.CurrentState == TrafficLight.State.Yellow)
            {
                float accel = CalculateIDMAcceleration(simHandler.carRigidbody.velocity.magnitude, 0, distToStop);

                Debug.DrawLine(rearAxle, target.Position, target.CurrentState == TrafficLight.State.Red ? Color.red : Color.yellow);

                float basePriority = 35f; 
                float maxPriority = 80f; 

                float decelWeight = Mathf.Clamp01(-accel / maxBreaking);
                float gapWeight = Mathf.Clamp01((detectionDistanceLength - (distToStop - stopDistanceHysteresis)) / detectionDistanceLength);

                // Priority increases as stopping becomes urgent
                float dynamicPriority = basePriority + (maxPriority - basePriority) * Mathf.Max(decelWeight, gapWeight);

                result.ThrottlePercent = Mathf.Clamp01(accel / maxAcceleration) * 100;
                result.BrakePercent = Mathf.Clamp01(-accel / maxAcceleration) * 100;
                result.SteeringAngle = 0;

                result.Priority = Mathf.RoundToInt(dynamicPriority);
            }
        }
        return result;
    }

    public ControlCommand EvaluatePath()
    {

        Vector3 toLookahead = lookAheadPoint - rearAxle;
        float ld = toLookahead.magnitude;

        float angleToLookahead = Vector3.SignedAngle(transform.forward, toLookahead, Vector3.up) * Mathf.Deg2Rad;
        float L = simHandler.wheelbase;

        //float newSteeringAngle = Mathf.Atan2(2 * L * Mathf.Sin(angleToLookahead), ld) * Mathf.Rad2Deg;
        //newSteeringAngle = Mathf.Clamp(newSteeringAngle, -simHandler.maxSteerAngle, simHandler.maxSteerAngle);

        //// Lowpass filter the steering
        //float steeringAngle = steeringSmoothness * lastSteeringAngle + (1 - steeringSmoothness) * newSteeringAngle;
        //lastSteeringAngle = steeringAngle;

        // Adaptive speed on curve
        float distanceToLookAhead = Vector3.Distance(rearAxle, lookAheadPoint);

        Vector3 toTarget = (lookAheadPoint - rearAxle).normalized;
        float approachSpeed = Mathf.Max(0.1f, Vector3.Dot(simHandler.carRigidbody.velocity, toTarget));

        float timeToReach = distanceToLookAhead / approachSpeed;

        float curvature = splineManager.CalculateCurvature(clostestSplinePointIndx);

        float curvatureSensitivity = Mathf.Lerp(1.0f, 2.0f, Mathf.Clamp01(curvature / curvatureThreshold));

        float newSteeringAngle = curvatureSensitivity * Mathf.Atan2(2 * L * Mathf.Sin(angleToLookahead), ld) * Mathf.Rad2Deg;

        float normalizedCurvature = Mathf.Clamp01(1f - curvature / curvatureThreshold);
        float curveSpeed = desiredSpeed * Mathf.Lerp(minSpeedMultiplier, maxSpeedMultiplier, normalizedCurvature);

        if (timeToReach < 1.0f && approachSpeed > curveSpeed)
        {
            curveSpeed *= 0.7f;
        }

        // --- Throtle ---
        float speedOutput = pathSpeedPID.UpdatePID(curveSpeed, currentSpeed, Time.deltaTime);
        float throttlePercent = Mathf.Clamp(speedOutput, 0, 100);
        float brakePercent = Mathf.Clamp(-speedOutput, 0, 100);

        return new ControlCommand
        {
            ThrottlePercent = throttlePercent,
            BrakePercent = brakePercent,
            SteeringAngle = newSteeringAngle,
            Priority = 30
        };
    }

    public Vector3 GetLookAheadPoint(float vehicleSpeed)
    {

        //// Safty Check
        //if (clostestSplinePointIndx < 0 || clostestSplinePointIndx >= SplineManager.SplinePoints.Count)
        //{
        //    Debug.LogWarning("SplineManager | GetLookAheadPoint: closestIndex invalid!");
        //    return rearAxle;
        //}


        if (splineManager.SplinePoints == null || splineManager.SplinePoints.Count == 0 || waypointManager.isScanning == true)
        {
            Debug.LogWarning("SplineManager | GetLookAheadPoint: could not get lookAhead");
            return lastLookAheadPoint; // or some safe fallback value
        }

        float adjustedDistance = minLookAheadDistance + lookaheadGain * vehicleSpeed;
        adjustedDistance = Mathf.Clamp(adjustedDistance, minLookAheadDistance, maxLookAheadDistance);

        float accumulated = 0f;
        Vector3 currentPoint = splineManager.SplinePoints[clostestSplinePointIndx];

        for (int i = clostestSplinePointIndx; i < splineManager.SplinePoints.Count - 1; i++)
        {
            Vector3 nextPoint = splineManager.SplinePoints[i + 1];
            accumulated += Vector3.Distance(currentPoint, nextPoint);

            if (accumulated >= adjustedDistance)
            {
                lastLookAheadPoint = nextPoint;
                return nextPoint;
            }

            currentPoint = nextPoint;
        }

        return splineManager.SplinePoints[splineManager.SplinePoints.Count - 1];
    }

    void UpdateVariables()
    {
        rearAxle = simHandler.GetRearAxlePosition();

        currentSpeed = simHandler.CURRENT_SPEED;

        lookAheadPoint = GetLookAheadPoint(currentSpeed);

        laneWidth = waypointManager.GetLaneWidth();

        clostestSplinePointIndx = splineManager.FindClosestSplinePoint(rearAxle);

        detectionDistanceLength = Vector3.Distance(rearAxle, splineManager.SplinePoints[splineManager.SplinePoints.Count - 2]);

    }

    float CalculateIDMAcceleration(float v, float v_front, float s)
    {
        // IDM parameters (tune as needed)
        float delta = 4.0f;        // Acceleration exponent (usually 4)
        float v_desired = desiredSpeed; // Your desired cruise speed (from inspector)
        float s0 = 8.0f;           // Minimum gap (m)
        float T = 1.5f;            // Desired time headway (s)

        float delta_v = v - v_front; // Approach rate (positive if closing)

        float s_star = s0 + Mathf.Max(0, v * T + v * delta_v / (2 * Mathf.Sqrt(maxAcceleration * maxBreaking)));
        float accel = maxAcceleration * (1 - Mathf.Pow(v / v_desired, delta) - Mathf.Pow(s_star / s, 2));

        return accel;
    }

    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying || waypointManager.isScanning) return;

        float accumulatedDistance = 0f;

        Gizmos.color = Color.green;

        // Visualization toggle - set to true to enable
        if (true)
        {
            for (int i = clostestSplinePointIndx; i < splineManager.SplinePoints.Count - 1 && accumulatedDistance < detectionDistanceLength; i++)
            {
                Vector3 current = splineManager.SplinePoints[i];
                Vector3 next = splineManager.SplinePoints[i + 1];

                accumulatedDistance += Vector3.Distance(current, next);

                // Draw center line
                Gizmos.DrawLine(current + Vector3.up * 0.2f, next + Vector3.up * 0.2f);

                // Draw lane boundaries
                Vector3 dir = (next - current).normalized;
                Vector3 right = Vector3.Cross(Vector3.up, dir).normalized;

                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(current + right * laneWidth, next + right * laneWidth);
                Gizmos.DrawLine(current - right * laneWidth, next - right * laneWidth);

                // Connect lane edges vertically
                Gizmos.DrawLine(current + right * laneWidth, current - right * laneWidth);
                Gizmos.DrawLine(next + right * laneWidth, next - right * laneWidth);
            }
        }

        // Highlight look-ahead point
        Gizmos.color = Color.cyan;
        Gizmos.DrawSphere(lookAheadPoint, 0.3f);
    }

}

