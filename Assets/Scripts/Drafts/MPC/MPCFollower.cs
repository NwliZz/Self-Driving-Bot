using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(Navigation))]
[RequireComponent(typeof(SimulationHandler))]
public class MPCFollower : MonoBehaviour
{
    [Header("MPC Settings")]
    public int horizon = 5;
    public float timestep = 0.2f;
    public float desiredSpeed = 15f;
    public float maxSteerAngle = 30f;
    public float steeringRatio = 13.4f;
    public float wheelBase = 2.6f;

    [Header("Cost Function")]
    public float steeringPenalty = 0.1f;

    [Header("Safety")]
    public float lateralSafeDistance = 0.7f;

    private Navigation navigation;
    private SimulationHandler simHandler;
    private WorldModel worldModel;

    private Vector3 currentPos;

    void Start()
    {
        navigation = GetComponent<Navigation>();
        simHandler = GetComponent<SimulationHandler>();
        worldModel = FindObjectOfType<WorldModel>();
    }

    void FixedUpdate()
    {
        currentPos = simHandler.GetRearAxlePosition();

        int closest = navigation.splineManager.FindClosestSplinePoint(currentPos);

        //int idx = Mathf.Min(closest + 5, navigation.splineManager.SplinePoints.Count - 1);

        Vector3 target = navigation.splineManager.SplinePoints[closest+2];

        Vector3 direction = target - currentPos;
        float angle = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;



        Vector3 toTarget = (target - currentPos).normalized;
        float targetTheta = Mathf.Atan2(toTarget.x, toTarget.z);

        float carTheta = Mathf.Atan2(transform.forward.x, transform.forward.z);

        float steer = Mathf.Rad2Deg * Mathf.DeltaAngle(Mathf.Rad2Deg * carTheta, Mathf.Rad2Deg * targetTheta);

       // float steer = Mathf.Rad2Deg * (targetTheta - carTheta); // in degrees

        simHandler.SetSteeringAngle(-steer); // maybe clamp to maxSteerAngle
        simHandler.SetThrottlePercent(40f);
        simHandler.SetBrakePercent(0f);
        //Vector3 lookAhead = navigation.GetLookAheadPoint();
        //float distanceToTarget = Vector3.Distance(currentPos, lookAhead);

        //float velocity = simHandler.CURRENT_SPEED / 3.6f;
        //float theta = Mathf.Atan2(transform.forward.x, transform.forward.z);


        //float[] x0 = { currentPos.x, currentPos.z, theta, velocity };

        //List<float[]> trajectory = GenerateTrajectory();

        //float[] bestSteerSequence = FindBestSteering(x0, trajectory);

        //ApplyControl(bestSteerSequence[0], desiredSpeed);
    }

    List<float[]> GenerateTrajectory()
    {
        List<float[]> traj = new List<float[]>();
        var splinePoints = navigation.splineManager.SplinePoints;
        int closestIdx = navigation.splineManager.FindClosestSplinePoint(currentPos);

        for (int i = 1; i <= horizon; i++)
        {
            int idx = Mathf.Min(closestIdx + i * 3, splinePoints.Count - 1); // spread points ahead
            Vector3 point = splinePoints[idx];
            traj.Add(new float[] { point.x, point.z, 0, desiredSpeed });
        }
        return traj;
    }

    float[] BicycleModel(float[] state, float steer)
    {
        float delta = Mathf.Deg2Rad * steer / steeringRatio;
        float thetaNext = state[2] + (state[3] / wheelBase) * Mathf.Tan(delta) * timestep;
        float xNext = state[0] + state[3] * Mathf.Cos(thetaNext) * timestep;
        float yNext = state[1] + state[3] * Mathf.Sin(thetaNext) * timestep;
        float vNext = state[3];

        return new float[] { xNext, yNext, thetaNext, state[3] };
    }

    float ComputeCost(float[] steerSequence, float[] x0, List<float[]> traj)
    {
        float cost = 0f;
        float[] state = (float[])x0.Clone();

        for (int i = 0; i < horizon; i++)
        {
            state = BicycleModel(state, steerSequence[i]);

            float dx = state[0] - traj[i][0];
            float dy = state[1] - traj[i][1];

            cost += dx * dx + dy * dy + steeringPenalty * Mathf.Pow(steerSequence[i] - lastSteerAngle, 2);

            if (!CheckSafety(new Vector3(state[0], 0, state[1])))
            {
                cost += 1e6f;
            }
        }

        return cost;
    }

    bool CheckSafety(Vector3 position)
    {
        foreach (var obj in worldModel.GetAllWorldObjects())
        {
            if (Vector3.Distance(position, obj.Position) < lateralSafeDistance)
                return false;
        }
        return true;
    }

    private float lastSteerAngle = 0f;

    float[] FindBestSteering(float[] x0, List<float[]> traj)
    {
        int samples = 50;
        float bestCost = float.MaxValue;
        float[] bestSequence = new float[horizon];
        System.Random rand = new System.Random();

        for (int s = 0; s < samples; s++)
        {
            float[] steerSeq = new float[horizon];

            for (int i = 0; i < horizon; i++)
            {
                float baseSteer = (i == 0) ? lastSteerAngle : steerSeq[i - 1];
                float steerNoise = (float)(rand.NextDouble() * 10f - 5f);  // limit noise to ±5 degrees per step
                steerSeq[i] = Mathf.Clamp(baseSteer + steerNoise, -maxSteerAngle, maxSteerAngle);
            }

            float cost = ComputeCost(steerSeq, x0, traj);

            if (cost < bestCost)
            {
                bestCost = cost;
                steerSeq.CopyTo(bestSequence, 0);
            }
        }

        lastSteerAngle = bestSequence[0]; // Store last steer angle for continuity

        return bestSequence;
    }

    void ApplyControl(float steerAngle, float desiredSpeed)
    {
        float currentSpeed = simHandler.CURRENT_SPEED;
        float speedError = desiredSpeed - currentSpeed;

        float throttlePercent = Mathf.Clamp(speedError * 10f, 0f, 100f);  // simple proportional controller
        float brakePercent = speedError < 0 ? Mathf.Clamp(-speedError * 20f, 0f, 100f) : 0f;

        simHandler.SetSteeringAngle(steerAngle);
        simHandler.SetThrottlePercent(throttlePercent);
        simHandler.SetBrakePercent(brakePercent);
    }
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying || simHandler == null || navigation == null)
            return;

        // Car's real position and heading
        Vector3 carPos = simHandler.GetRearAxlePosition();
        Vector3 fwd = transform.forward;
        float theta = Mathf.Atan2(transform.forward.x, transform.forward.z);
        float v0 = simHandler.CURRENT_SPEED / 3.6f;
        if (v0 < 0.5f) v0 = desiredSpeed / 3.6f;

        // Draw the car's forward direction (yellow)
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(carPos, carPos + fwd * 5f);

        // Draw the lookahead point (blue)
        Vector3 lookAhead = navigation.GetLookAheadPoint();
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(carPos, lookAhead);
        Gizmos.DrawSphere(lookAhead, 0.3f);

        // Draw a safe zone (green)
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(carPos, lateralSafeDistance);

        //// Draw the reference trajectory (cyan)
        //var traj = GenerateTrajectory();
        //Gizmos.color = Color.cyan;
        //foreach (var p in traj)
        //{
        //    Gizmos.DrawSphere(new Vector3(p[0], carPos.y, p[1]), 0.12f);
        //}

        // Draw the predicted MPC trajectory (magenta)
        //float[] stateDraw = { carPos.x, carPos.z, theta, v0 };
        //float[] bestSteerSequence = FindBestSteering(stateDraw, GenerateTrajectory());
        //Gizmos.color = Color.magenta;
        //Vector3 prevPoint = carPos;
        //for (int i = 0; i < horizon; i++)
        //{
        //    stateDraw = BicycleModel(stateDraw, bestSteerSequence[i]);
        //    Vector3 point = new Vector3(stateDraw[0], carPos.y, stateDraw[1]);
        //    Gizmos.DrawSphere(point, 0.15f);
        //    Gizmos.DrawLine(prevPoint, point);
        //    prevPoint = point;
        //}

        // Optionally: Draw debug log of car heading (yellow) and first magenta vector
        Vector3 magForward = new Vector3(Mathf.Cos(theta + Mathf.PI), 0, Mathf.Sin(theta + Mathf.PI));
        Debug.DrawLine(carPos, carPos + magForward * 4f, Color.magenta, 0, false);
        Debug.DrawLine(carPos, carPos + fwd * 4f, Color.yellow, 0, false);

        // Extra: Draw a direct line from car to next reference (if you want to see geometric targeting)
        //Gizmos.color = Color.white;
        //if (traj.Count > 0)
        //    Gizmos.DrawLine(carPos, new Vector3(traj[0][0], carPos.y, traj[0][1]));
    }

    //private void OnDrawGizmos()
    //{
    //    if (!Application.isPlaying || simHandler == null || navigation == null)
    //        return;
    //    Gizmos.color = Color.yellow;
    //    Gizmos.DrawLine(currentPos, currentPos + transform.forward * 5f);
    //    Gizmos.color = Color.green;
    //    Gizmos.DrawWireSphere(currentPos, lateralSafeDistance);

    //    // Draw lookahead (target) point
    //    Vector3 lookAhead = navigation.GetLookAheadPoint();
    //    Gizmos.color = Color.blue;
    //    Gizmos.DrawLine(currentPos, lookAhead);
    //    Gizmos.DrawSphere(lookAhead, 0.3f);

    //    Vector3 fwd = transform.forward;
    //    float theta = Mathf.Atan2(fwd.x, fwd.z);
    //    // Draw predicted MPC trajectory
    //    //float v0 = simHandler.CURRENT_SPEED > 0.1f ? simHandler.CURRENT_SPEED / 3.6f : desiredSpeed / 3.6f;
    //    float v0 = simHandler.CURRENT_SPEED / 3.6f;
    //    if (v0 < 0.5f) v0 = desiredSpeed / 3.6f; // default to desired speed if nearly stopped
    //    float[] state = { currentPos.x, currentPos.z, theta, v0 };

    //    float[] bestSteerSequence = FindBestSteering(state, GenerateTrajectory());
    //    Gizmos.color = Color.magenta;
    //    Vector3 prevPoint = currentPos;
    //    float[] stateDraw = { currentPos.x, currentPos.z, theta, v0 };
    //    for (int i = 0; i < horizon; i++)
    //    {
    //        stateDraw = BicycleModel(stateDraw, bestSteerSequence[i]);
    //        Vector3 point = new Vector3(stateDraw[0], currentPos.y, stateDraw[1]);
    //        Gizmos.DrawSphere(point, 0.15f);
    //        Gizmos.DrawLine(prevPoint, point);
    //        prevPoint = point;
    //    }
    //    var traj = GenerateTrajectory();
    //    Gizmos.color = Color.cyan;
    //    foreach (var p in traj)
    //    {
    //        Gizmos.DrawSphere(new Vector3(p[0], currentPos.y, p[1]), 0.12f);
    //    }
    //}
}