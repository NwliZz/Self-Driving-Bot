using System.Collections;
using UnityEngine;

[RequireComponent(typeof(Navigation))]
[RequireComponent(typeof(SimulationHandler))]
public class SFollowerOD : MonoBehaviour
{
    public struct ControlCommand
    {
        public float ThrottlePercent; // 0-100
        public float BrakePercent;    // 0-100
        public float SteeringAngle;   // degrees
        public int Priority;          // higher wins (ObjectDetection > PathFollowing)

        public static ControlCommand Default => new ControlCommand
        {
            ThrottlePercent = 0,
            BrakePercent = 0,
            SteeringAngle = 0,
            Priority = 0
        };
    }

    [Header("Drive - Settings")]
    public float desiredSpeed = 15f;
    public float minSpeedMultiplier = 0.5f;
    public float maxSpeedMultiplier = 1.0f;

    [Header("OD - Settings")]
    public float stopDistanceHysteresis = 1.0f; // meters, tune as needed
    private bool isBrakingForObstacle = false;

    public float maxDetectDistance = 10f;
    public float maxAccel = 2.0f;
    public float maxBreaking = 5.0f;
    public float reactionTime = 1.0f;
    public float lateralStandstillGap = 1.0f;
    public float detectionBuffer = 5f;

    private WorldModel worldModel;
    private Navigation navigation;
    private SimulationHandler simHandler;
    private CarMono selfObstacle;

    void Start()
    {
        navigation = GetComponent<Navigation>();
        simHandler = GetComponent<SimulationHandler>();
        worldModel = FindObjectOfType<WorldModel>();
        selfObstacle = GetComponent<CarMono>();
    }

    void FixedUpdate()
    {
        if (navigation.waypointManager.isScanning) return;
        
        ////Does not work Make EvaluateCar Evaluate TrafficLight etc..
        ///Short them in a list and pick the first
        ///:)


        // 1. Object Detection & RSS Logic
        ControlCommand odCmd = EvaluateObjectDetection();

        // 2. Path Following (Default Behavior)
        ControlCommand pfCmd = EvaluatePathFollowing();

        // 3. Final Arbitration (higher priority wins)
        ControlCommand cmd = (odCmd.Priority > pfCmd.Priority) ? odCmd : pfCmd;

        ApplyControls(cmd);
    }

    // --- Module: Object Detection (RSS, Traffic Light, etc.) ---
    ControlCommand EvaluateObjectDetection()
    { // -- Expand: Add lateral RSS  --
        ControlCommand result = ControlCommand.Default;

        if (worldModel == null) return result;

        Vector3 rearPos = simHandler.GetRearAxlePosition();
        Vector3 lookAhead = navigation.GetLookAheadPoint();

        foreach (var obj in worldModel.GetAllWorldObjects())
        {

            float dist = Vector3.Distance(transform.position, obj.Position);
            Vector3 toObj = (obj.Position - transform.position).normalized;


            // -- CAR OBSTACLE --
            if (obj is Car car)
            {
                if (car == selfObstacle.self)
                    continue;

                float dynamicDetDist = Mathf.Max(maxDetectDistance, Vector3.Distance(transform.position, lookAhead) + detectionBuffer);
                float lateralDist = Mathf.Abs(Vector3.Dot(car.Position - transform.position, transform.right));
                if (dist < dynamicDetDist && lateralDist < navigation.waypointManager.GetLaneWidth())
                {
                   result = HandleCar(car, dist);
                }
            }

            // -- TRAFFIC LIGHT --
             if (obj is TrafficLight tl)
            {
                if (dist < 20f)
                {
                    if (tl.CurrentState == TrafficLight.State.Red)
                    {
                        Debug.DrawLine(rearPos, tl.Position, Color.red);
                        result = new ControlCommand { ThrottlePercent = 0, BrakePercent = 100, SteeringAngle = 0, Priority = 80 };

                    }
                    else if (tl.CurrentState == TrafficLight.State.Yellow)
                    {
                        if(dist<20/2f)
                        {
                            Debug.DrawLine(rearPos, tl.Position, Color.yellow);
                            result = new ControlCommand { ThrottlePercent = 0, BrakePercent = 100, SteeringAngle = 0, Priority = 60 };
                        }
                        else
                        {
                            Debug.DrawLine(rearPos, tl.Position, Color.yellow);
                            result = new ControlCommand { ThrottlePercent = 0, BrakePercent = 70, SteeringAngle = 0, Priority = 9 };
                        }
                    }
                   
                    // Lower priority than obstacle, but can add logic to merge, etc.
                }
            }

           
        }
        return result;
    }

    // --- Module: Path Following (Spline/Waypoint) ---
    ControlCommand EvaluatePathFollowing()
    {
        Vector3 rearPos = simHandler.GetRearAxlePosition();
        Vector3 target = navigation.GetLookAheadPoint();

        // Steering
        Vector3 dirToTarget = (target - rearPos).normalized;
        float steerAngle = Vector3.SignedAngle(transform.forward, dirToTarget, Vector3.up);

        // Adaptive speed on curve
        Vector3 lookAheadPoint = navigation.GetLookAheadPoint();
        float distanceToLookAhead = Vector3.Distance(rearPos, lookAheadPoint);
        Vector3 toTarget = (lookAheadPoint - rearPos).normalized;
        float approachSpeed = Vector3.Dot(simHandler.carRigidbody.velocity, toTarget);
        approachSpeed = Mathf.Max(0f, approachSpeed);

        float timeToReach = distanceToLookAhead / Mathf.Max(approachSpeed, 0.1f);

        int splineIndex = navigation.splineManager.FindClosestSplinePoint(rearPos);
        float curvature = navigation.splineManager.CalculateCurvature(splineIndex);
        float normalizedCurvature = Mathf.Clamp01(1f - curvature / Mathf.Max(0.0001f, navigation.splineManager.curvatureThreshold));
        float curveSpeed = desiredSpeed * Mathf.Lerp(minSpeedMultiplier, maxSpeedMultiplier, normalizedCurvature);

        if (timeToReach < 1.0f && approachSpeed > curveSpeed)
        {
            curveSpeed *= 0.7f;
        }

        float currentSpeed = simHandler.CURRENT_SPEED;
        float throttlePercent = 0f, brakePercent = 0f;

        if (currentSpeed < curveSpeed)
        {
            throttlePercent = Mathf.Lerp(20f, 60f, (curveSpeed - currentSpeed) / curveSpeed);
            brakePercent = 0f;
        }
        else
        {
            throttlePercent = 0f;
            float curvatureBoost = Mathf.Clamp01(curvature / navigation.splineManager.curvatureThreshold);
            float baseBrake = Mathf.Lerp(10f, 100f, (currentSpeed - curveSpeed) / curveSpeed);
            brakePercent = baseBrake * Mathf.Lerp(1f, 100f, curvatureBoost);
        }

        return new ControlCommand
        {
            ThrottlePercent = throttlePercent,
            BrakePercent = brakePercent,
            SteeringAngle = steerAngle,
            Priority = 10
        };
    }

    // --- Apply Controls ---
    void ApplyControls(ControlCommand cmd)
    {
        Debug.Log(cmd.Priority);
        simHandler.SetThrottlePercent(cmd.ThrottlePercent);
        simHandler.SetBrakePercent(cmd.BrakePercent);
        simHandler.SetSteeringAngle(cmd.SteeringAngle);
    }

    // --- RSS Safe Distance Calculation ---
    public float CalcRSSSafeDistance(float v_ego, float v_front)
    {
        float term1 = v_ego * reactionTime;
        float term2 = 0.5f * maxAccel * reactionTime * reactionTime;
        float v_ego_after_reaction = v_ego + reactionTime * maxAccel;
        float term3 = (v_ego_after_reaction * v_ego_after_reaction) / (2f * maxBreaking);
        float term4 = (v_front * v_front) / (2f * maxAccel);
        float d_min = term1 + term2 + term3 - term4;
        return Mathf.Max(d_min, 0f);
    }

    private ControlCommand HandleCar( Car car, float dist)
    {
        float v_ego = simHandler.carRigidbody.velocity.magnitude;
        float v_front = Vector3.Dot(car.Velocity, transform.forward);

        float safeDistance = CalcRSSSafeDistance(v_ego, v_front);
        float minRequiredDistance = Mathf.Max(safeDistance, stopDistanceHysteresis);

        Vector3 toOther = car.Position - transform.position;
        float forwardDot = Vector3.Dot(transform.forward, toOther.normalized);

        if (dist < safeDistance)
        {
            Debug.DrawLine(transform.position, car.Position, Color.yellow);
            return new ControlCommand { ThrottlePercent = 0, BrakePercent = 80, SteeringAngle = 0, Priority = 50 };
        }

        else if (forwardDot > 0.7f && dist < minRequiredDistance)
        {
            Debug.DrawLine(transform.position, car.Position, Color.red);
            return new ControlCommand { ThrottlePercent = 0, BrakePercent = 100, SteeringAngle = 0, Priority = 41 };
        }

        ////Keep the priority just in case
        //else if (dist > safeDistance)
        //{
        //    Debug.DrawLine(transform.position, car.Position, Color.green);
        //    return new ControlCommand { ThrottlePercent = 0, BrakePercent = 0, SteeringAngle = 0, Priority = 0 };
        //}
        return new ControlCommand { ThrottlePercent = 0, BrakePercent= 0, SteeringAngle = 0, Priority= 0 };
    }
}

//using UnityEngine;
//using System.Linq;

//[RequireComponent(typeof(Navigation))]
//[RequireComponent(typeof(SimulationHandler))]
//public class SFollowerOD : MonoBehaviour
//{
//    [Header("Follow Settings")]
//    public float holdBrakePercent = 100f;
//    public float desiredSpeed = 15f;

//    [Header("Adaptive Speed Control")]
//    public float minSpeedMultiplier = 0.5f;
//    public float maxSpeedMultiplier = 1.0f;

//    [Header("Responsibility-Sensitive Safety Parameters")]
//    public float reactionTime = 1.0f;      // ?, in seconds
//    public float maxAccel = 2.0f;          // ?_max, m/s^2
//    /*    public float minBraking = 4.0f; */       // ?_min, m/s^2
//    /*    public float maxBraking = 8.0f; */       // ?_max, m/s^2

//    [Header("Vision - Obs Det")]
//    public float maxDetectDistance = 10f;

//    private WorldModel worldModel;
//    private Navigation navigation;
//    private SimulationHandler simHandler;

//    private bool stopForObstacle = false;
//    private bool stopForLight = false;

//    private bool OdHandlesSteer = false;

//    private CarObstacleMono selfObstacle;

//    void Start()
//    {
//        navigation = GetComponent<Navigation>();
//        simHandler = GetComponent<SimulationHandler>();
//        worldModel = FindObjectOfType<WorldModel>();
//        selfObstacle = GetComponent<CarObstacleMono>();
//    }

//    void FixedUpdate()
//    {
//        if (navigation.waypointManager.isScanning)
//        {
//            SetControls(0f, holdBrakePercent);
//            return;
//        }

//        Vector3 leftBoundary = transform.position - transform.right * (navigation.waypointManager.GetLaneWidth());
//        Vector3 rightBoundary = transform.position + transform.right * (navigation.waypointManager.GetLaneWidth());
//        Vector3 rearPos = simHandler.GetRearAxlePosition();



//        // --- RESET STOP FLAGS ---
//        stopForObstacle = false;
//        stopForLight = false;

//        OdHandlesSteer = false;

//        // --- DETECTION  ---
//        if (worldModel != null)
//        {

//            foreach (var obj in worldModel.GetAllWorldObjects())
//            {
//                float dist = Vector3.Distance(transform.position, obj.Position);
//                Vector3 toObj = (obj.Position - transform.position).normalized;
//                float dot = Vector3.Dot(transform.forward, toObj);

//                // Only consider objects in front (within a 45-degree cone)
//                if (dot <= 0.5f) continue;

//                switch (obj)
//                {
//                    case Car obs:
//                        if ((object)obj == selfObstacle) break;

//                        Vector3 toObstacle = obs.Position - transform.position;
//                        dist = toObstacle.magnitude;

//                        float forwardDot = Vector3.Dot(transform.forward, toObstacle.normalized);

//                        float laneWidth = navigation.waypointManager.GetLaneWidth();

//                        float lateralDist = Mathf.Abs(Vector3.Dot(toObstacle, transform.right));

//                        // --- Dynamic Look Ahead For Obstacle Detection --- 
//                        Vector3 lah = navigation.GetLookAheadPoint();

//                        float distToLah =Vector3.Distance(transform.position, lah);

//                        float dynamicDetDist = distToLah + 5f;

//                        // ---  VISUALIZATION ---
//                        Vector3 start = transform.position;
//                        Vector3 end = start + transform.forward * dynamicDetDist;

//                        Debug.DrawLine(start, end, Color.green);
//                        Debug.DrawLine(leftBoundary, leftBoundary + transform.forward * dynamicDetDist, Color.blue);
//                        Debug.DrawLine(rightBoundary, rightBoundary + transform.forward * dynamicDetDist, Color.blue);


//                        if (dist < dynamicDetDist && forwardDot > 0.6f && lateralDist < laneWidth)
//                        {
//                            float v_ego = simHandler.carRigidbody.velocity.magnitude;
//                            float v_front = 0f;

//                            // Project the front car's velocity onto ego car's forward direction
//                            v_front = Vector3.Dot(obs.Velocity, transform.forward);

//                            float safeDistance = CalcRSSSafeDistance(v_ego,v_front);

//                            float minRequiredDistance = Mathf.Max(safeDistance, 6f);

//                            if (dist <= minRequiredDistance)
//                            {
//                                stopForObstacle = true;
//                                Debug.DrawLine(transform.position, obs.Position, Color.red);
//                            }
//                            else
//                            {
//                                Debug.DrawLine(transform.position, obs.Position, Color.yellow);
//                            }
//                        }

//                        break;

//                    case TrafficLight tl:
//                        if (dist < 15f && (tl.CurrentState == TrafficLight.State.Red || tl.CurrentState == TrafficLight.State.Yellow))
//                        {
//                            stopForLight = true;
//                            // Draw debug line to traffic light
//                            Debug.DrawLine(rearPos, tl.Position, Color.yellow);
//                        }
//                        break;

//                        // Add other cases as you add more types, e.g., RoadFeature
//                }
//            }
//        }

//        // --- CONTROL LOGIC ---
//        if (stopForObstacle || stopForLight)
//        {
//            SetControls(0f, 100f);
//            return;
//        }

//// --- PATH FOLLOWING LOGIC  ---
//        Vector3 target = navigation.GetLookAheadPoint();

//        // Steering
//        Vector3 dirToTarget = (target - rearPos).normalized;
//        float steerAngle = Vector3.SignedAngle(transform.forward, dirToTarget, Vector3.up);
//        simHandler.SetSteeringAngle(steerAngle);


//        Vector3 lookAheadPoint = navigation.GetLookAheadPoint();
//        float distanceToLookAhead = Vector3.Distance(rearPos, lookAheadPoint);
//        Vector3 toTarget = (lookAheadPoint - rearPos).normalized;
//        float approachSpeed = Vector3.Dot(simHandler.carRigidbody.velocity, toTarget);
//        approachSpeed = Mathf.Max(0f, approachSpeed);

//        float timeToReach = distanceToLookAhead / Mathf.Max(approachSpeed, 0.1f);

//        int splineIndex = navigation.splineManager.FindClosestSplinePoint(rearPos);
//        float curvature = navigation.splineManager.CalculateCurvature(splineIndex);
//        float normalizedCurvature = Mathf.Clamp01(1f - curvature / Mathf.Max(0.0001f, navigation.splineManager.curvatureThreshold));
//        float curveSpeed = desiredSpeed * Mathf.Lerp(minSpeedMultiplier, maxSpeedMultiplier, normalizedCurvature);

//        if (timeToReach < 1.0f && approachSpeed > curveSpeed)
//        {
//            curveSpeed *= 0.7f;
//        }

//        float currentSpeed = simHandler.CURRENT_SPEED;

//        if (true)//(!stopForObstacle && !stopForLight)
//        {

//            if (currentSpeed < curveSpeed)
//            {
//                float throttlePercent = Mathf.Lerp(20f, 60f, (curveSpeed - currentSpeed) / curveSpeed);
//                SetControls(throttlePercent, 0f);
//            }
//            else
//            {
//                float curvatureBoost = Mathf.Clamp01(curvature / navigation.splineManager.curvatureThreshold);
//                float baseBrake = Mathf.Lerp(10f, 100f, (currentSpeed - curveSpeed) / curveSpeed);
//                float brakePercent = baseBrake * Mathf.Lerp(1f, 100f, curvatureBoost);
//                SetControls(0f, brakePercent);
//            }
//        }
//    }

//    private void SetControls(float throttle, float brake)
//    {
//        simHandler.SetThrottlePercent(throttle);
//        simHandler.SetBrakePercent(brake);
//    }

//    /// <summary>
//    /// Calculate the minimum safe longitudinal distance (RSS) between two vehicles.
//    /// </summary>
//    /// <param name="v_ego">Ego (rear) vehicle speed (m/s, positive forward)</param>
//    /// <param name="v_front">Front vehicle speed (m/s, positive forward)</param>
//    /// <param name="reactionTime">Reaction time ρ (seconds)</param>
//    /// <param name="maxAccel">Max acceleration α_max (m/s^2)</param>
//    /// <param name="minBraking">Min braking β_min (m/s^2, positive)</param>
//    /// <param name="maxBraking">Max braking β_max (m/s^2, positive)</param>
//    /// <returns>Safe longitudinal distance (meters)</returns>
//    public float CalcRSSSafeDistance(float v_ego, float v_front)
//    {
//        // Terms of the formula
//        float term1 = v_ego * reactionTime;
//        float term2 = 0.5f * maxAccel * reactionTime * reactionTime;
//        float v_ego_after_reaction = v_ego + reactionTime * maxAccel;
//        float term3 = (v_ego_after_reaction * v_ego_after_reaction) / (2f * minSpeedMultiplier);
//        float term4 = (v_front * v_front) / (2f * maxSpeedMultiplier);

//        float d_min = term1 + term2 + term3 - term4;

//        // Clamp to 0 if negative, as per [·]+ notation
//        return Mathf.Max(d_min, 0f);
//    }
//}
