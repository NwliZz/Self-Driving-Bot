using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Driver : MonoBehaviour
{
    [Header("Refrence - Settings")]
    private TrackingManager trackingManager;
    private SimulationHandler simHandler;
    private CarMono selfObstacle;
    private Actions actions;
    private Calculations calculate;

    //Internal Variables
    private int lastPriority;

    void Start()
    {
        simHandler = GetComponent<SimulationHandler>();
        selfObstacle = GetComponent<CarMono>();
        trackingManager = GetComponent<TrackingManager>();
        actions = GetComponent<Actions>();
        calculate = new Calculations();
    }

    void FixedUpdate()
    {

        ControlCommand pathCommand = EvaluatePath();
        ControlCommand trafficCommand = EvaluateTrafficLight();
        ControlCommand carCommand = EvaluateCar();

        List<ControlCommand> allCommands = new List<ControlCommand>
        {
            trafficCommand,
            carCommand,
            pathCommand
        };

        // Final Arbitration (higher priority wins)
        ControlCommand chosen = allCommands.OrderByDescending(cmd => cmd.Priority).First();

        chosen.SteeringAngle = pathCommand.SteeringAngle;

        //if priority changes update uninformed Controllers
        HasChangedPriority(chosen.Priority);

        ApplyControls(chosen);

    }

    ControlCommand EvaluateTrafficLight()
    {
        TrafficLight target = trackingManager.ReturnClosestObstacle<TrafficLight>(actions.detectionDistanceLength, actions.rearAxle);

        return actions.EvaluateTrafficLight(target);
    }

    ControlCommand EvaluatePath()
    {
       return actions.EvaluatePath();
    }

    ControlCommand EvaluateCar()
    {
        Car target = trackingManager.ReturnClosestObstacle<Car>(actions.detectionDistanceLength, actions.rearAxle, selfObstacle.self);

        return actions.GetCar(target);
    }

    void ApplyControls(ControlCommand cmd)
    {
        lastPriority = cmd.Priority;
        simHandler.SetThrottlePercent(cmd.ThrottlePercent);
        simHandler.SetBrakePercent(cmd.BrakePercent);
        simHandler.SetSteeringAngle(cmd.SteeringAngle);
    }

    void ResetPIDControllers()
    {
        actions.pathSpeedPID.Reset();
    }

    void HasChangedPriority(int curentPriotity)
    {
        if (lastPriority != curentPriotity) ResetPIDControllers();
    }
}
