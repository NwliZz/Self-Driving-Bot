//Priority execution logic for steering
public struct ControlCommandSteer
{
    public float SteeringAngle;
    public int Priority;
    public static ControlCommandSteer Default => new ControlCommandSteer
    {
        SteeringAngle = 0,
        Priority = 0
    };
}

//Priority execution logic for Throtle and Brakes
public struct ControlCommandMotor
{
    public float ThrottlePercent;
    public float BrakePercent;
    public int Priority;
    public static ControlCommandMotor Default => new ControlCommandMotor
    {
        ThrottlePercent = 0,
        BrakePercent = 0,
        Priority = 0
    };
}

//Priority execution logic for Throtle and Steering
public struct ControlCommand
{
    public float ThrottlePercent;
    public float BrakePercent;
    public float SteeringAngle;
    public int Priority;

    public static ControlCommand Default => new ControlCommand
    {
        ThrottlePercent = 0,
        BrakePercent = 0,
        SteeringAngle = 0,
        Priority = 0
    };
}
