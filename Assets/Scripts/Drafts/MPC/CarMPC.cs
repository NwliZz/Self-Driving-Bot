using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(SimulationHandler))]
[RequireComponent(typeof(SplineManager))]
public class CarMPC : MonoBehaviour
{
    // MPC parameters
    [Header("MPC Settings")]
    public int horizonSteps = 10;          // Number of prediction steps
    public float dt = 0.15f;               // Time per step (seconds)
    public float refSpeed = 12f;           // Desired speed (m/s)

    [Header("Weights (Cost Function)")]
    public float wPath = 1.0f;
    public float wSpeed = 0.5f;
    public float wSteer = 0.1f;
    public float wAccel = 0.1f;
    public float wObstacle = 2.0f;

    [Header("Vehicle Limits")]
    public float maxSteeringAngle = 30f;   // degrees
    public float maxThrottle = 1.0f;
    public float maxBrake = 1.0f;

    private SimulationHandler simHandler;
    private SplineManager splineManager;
    private WorldModel worldModel;
    private Rigidbody rb;
    private CarMono selfCarMono;
    private Car carData;

    void Start()
    {
        simHandler = GetComponent<SimulationHandler>();
        splineManager = GetComponent<SplineManager>();
        worldModel = FindObjectOfType<WorldModel>();
        selfCarMono = GetComponent<CarMono>();
        
        rb = simHandler.carRigidbody;
    }

    void FixedUpdate()
    {
        

        // 1. Get current state
        Vector3 position = rb.position;
        float yaw = rb.rotation.eulerAngles.y * Mathf.Deg2Rad;
        float speed = rb.velocity.magnitude;

        // 2. Run MPC optimization
        MPCResult result = RunMPC(position, yaw, speed);

        // 3. Apply first control (open-loop, receding horizon)
        simHandler.SetSteeringAngle(result.firstSteering);
        simHandler.SetThrottlePercent(result.firstThrottle * 100f);
        simHandler.SetBrakePercent(result.firstBrake * 100f);

        // Optional: draw predicted trajectory
        for (int i = 1; i < result.predictedPositions.Count; i++)
            Debug.DrawLine(result.predictedPositions[i - 1], result.predictedPositions[i], Color.cyan);
    }

    // -------- MPC Core Logic Below --------

    // Result struct
    struct MPCResult
    {
        public float firstSteering, firstThrottle, firstBrake;
        public List<Vector3> predictedPositions;
    }

    // Main optimization loop (simplified: random shooting/heuristic search)
    MPCResult RunMPC(Vector3 pos, float yaw, float speed)
    {
        int N = horizonSteps;
        float bestCost = float.MaxValue;
        MPCResult bestResult = new MPCResult();

        // Try K random or grid action sequences (for real MPC, use optimizer library!)
        int K = 30; // Number of candidate plans (keep this small for performance, or use a better optimizer)
        for (int k = 0; k < K; k++)
        {
            // Generate random/simple sequence
            float[] steerSeq = new float[N];
            float[] throttleSeq = new float[N];
            float[] brakeSeq = new float[N];
            for (int i = 0; i < N; i++)
            {
                steerSeq[i] = Random.Range(-maxSteeringAngle, maxSteeringAngle);
                throttleSeq[i] = maxThrottle * 0.7f;
                brakeSeq[i] = Random.Range(0f, maxBrake * 0.3f); // Rarely full brake
            }

            // Simulate this candidate
            float cost;
            List<Vector3> positions = SimulateTrajectory(pos, yaw, speed, steerSeq, throttleSeq, brakeSeq, out cost);

            // If better, save
            if (cost < bestCost)
            {
                bestCost = cost;
                bestResult.firstSteering = steerSeq[0];
                bestResult.firstThrottle = throttleSeq[0];
                bestResult.firstBrake = brakeSeq[0];
                bestResult.predictedPositions = positions;
            }
        }
        return bestResult;
    }

    // Simulate car motion over horizon
    List<Vector3> SimulateTrajectory(Vector3 pos, float yaw, float speed, float[] steerSeq, float[] throttleSeq, float[] brakeSeq, out float totalCost)
    {
        int N = steerSeq.Length;
        List<Vector3> positions = new List<Vector3> { pos };
        float s = speed;
        Vector3 p = pos;
        float angle = yaw;

        totalCost = 0f;
        for (int i = 0; i < N; i++)
        {
            // Vehicle update: simple kinematic bicycle model
            float steerRad = steerSeq[i] * Mathf.Deg2Rad;
            float v = s + throttleSeq[i] * dt * 2.0f - brakeSeq[i] * dt * 4.0f; // crude accel/brake (tune these factors!)

            float wheelbase = 2.5f; // Tune for your car
            float dx = v * Mathf.Cos(angle) * dt;
            float dz = v * Mathf.Sin(angle) * dt;
            float dyaw = v / wheelbase * Mathf.Tan(steerRad) * dt;

            p += new Vector3(dx, 0f, dz);
            angle += dyaw;
            s = Mathf.Max(v, 0f);

            positions.Add(p);

            // Cost: path tracking (find closest spline point)
            Vector3 closest = FindClosestSplinePoint(p);
            float cte = (p - closest).magnitude;

            // Cost: speed error
            float speedError = refSpeed - s;

            // Cost: control smoothness (penalize big steer/accel changes)
            float steerCost = Mathf.Abs(steerSeq[i]);
            float accelCost = Mathf.Abs(throttleSeq[i] - brakeSeq[i]);

            // Cost: obstacle avoidance
            float obsCost = 0f;
            foreach (var car in worldModel.Cars)
            {
                if (car == selfCarMono.self) continue;

                float d = Vector3.Distance(p, car.Position);
                if (d < 4f)
                {
                    Debug.Log("here");
                    obsCost += 1f / Mathf.Max(0.1f, d - 1f);
                }// stronger penalty if too close
            }
            //foreach (var obs in worldModel.Obstacles)
            //{
            //    float d = Vector3.Distance(p, obs.Position);
            //    if (d < obs.Radius + 1f) obsCost += 1f / Mathf.Max(0.1f, d - obs.Radius);
            //}

            totalCost += wPath * cte * cte
                         + wSpeed * speedError * speedError
                         + wSteer * steerCost * steerCost
                         + wAccel * accelCost * accelCost
                         + wObstacle * obsCost;
        }
        return positions;
    }

    // Helper: find closest spline point for path tracking
    Vector3 FindClosestSplinePoint(Vector3 p)
    {
        return splineManager.SplinePoints[splineManager.FindClosestSplinePoint(p)];
    }
    void OnDrawGizmos()
    {
        // Draw the spline path (yellow)
        if (splineManager != null && splineManager.SplinePoints != null)
        {
            Gizmos.color = Color.yellow;
            var points = splineManager.SplinePoints;
            for (int i = 1; i < points.Count; i++)
                Gizmos.DrawLine(points[i - 1], points[i]);
        }

        // Draw obstacles (red)
        if (worldModel != null)
        {
            Gizmos.color = Color.red;
            foreach (var car in worldModel.Cars)
                Gizmos.DrawWireSphere(car.Position, 1.2f);
            foreach (var obs in worldModel.Obstacles)
                Gizmos.DrawWireSphere(obs.Position, obs.Radius);
        }

        // Draw car's current forward
        if (simHandler != null && simHandler.carRigidbody != null)
        {
            Gizmos.color = Color.green;
            Vector3 p = simHandler.carRigidbody.position;
            Vector3 fwd = simHandler.carRigidbody.transform.forward;
            Gizmos.DrawLine(p, p + fwd * 3f);
        }
    }

}
