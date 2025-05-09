using TMPro;
using UnityEngine;


public class SimulationHandler : MonoBehaviour
{
    [Header("Wheel Colliders")]
    [SerializeField] private WheelCollider frontLeftWheel;
    [SerializeField] private WheelCollider frontRightWheel;
    [SerializeField] private WheelCollider rearLeftWheel;
    [SerializeField] private WheelCollider rearRightWheel;

    [Header("Car Parameters")]
    [SerializeField] private float topSpeed = 30f;            // km/h
    [SerializeField] private float maxMotorTorque = 1500f;
    [SerializeField] private float maxBrakeForce = 3000f;
    [SerializeField] private float maxSteerAngle = 30f;

    [Header("Engine Braking")]
    [Range(0f, 1f)] public float engineBrakingPercent = 0.09f; // 9% of max brake force

    [Header("Debug Info")]
    [SerializeField] private Rigidbody carRigidbody;

    [Header("Drivetrain Settings")]
    public DriveType driveType;

    //Input values
    private float throttleInput = 0f; //%
    private float brakeInput = 0f;    //%
    private float steerInput = 0f;    //Degrees

    public float CURRENT_SPEED;

    [Header("UI")]
    public TextMeshProUGUI vehicleSpeedText;

    //Drive train type
    public enum DriveType
    {
        FWD,
        RWD,
        AWD
    }

    private void FixedUpdate()
    {
        // Car speed & UI
        CURRENT_SPEED = Mathf.Round(carRigidbody.velocity.magnitude * 3.6f);
        vehicleSpeedText.text = "Speed: " + CURRENT_SPEED.ToString("F1") + " km/h";


        ApplySteering(steerInput);

        ApplyBrakes(brakeInput);

        ApplyMotor(throttleInput);

    }

    public void SetThrottlePercent(float percent)
    {
        //percent = Mathf.Clamp(percent, 0f, 100f);
        throttleInput = percent / 100f;
    } 

    public void SetBrakePercent(float percent)
    {
        percent = Mathf.Clamp(percent, 0f, 100f);
        brakeInput = percent / 100f;
    }

    public void SetSteeringAngle(float angleDegrees)
    {
        steerInput = Mathf.Clamp(angleDegrees, -maxSteerAngle, maxSteerAngle);
    }

    private void ApplyMotor(float throttle)
    {
        if (CURRENT_SPEED < topSpeed)
        {
            float torque = throttle * maxMotorTorque;

            switch (driveType)
            {
                case DriveType.FWD:
                    frontLeftWheel.motorTorque = torque;
                    frontRightWheel.motorTorque = torque;
                    rearLeftWheel.motorTorque = 0f;
                    rearRightWheel.motorTorque = 0f;
                    break;

                case DriveType.RWD:
                    rearLeftWheel.motorTorque = torque;
                    rearRightWheel.motorTorque = torque;
                    frontLeftWheel.motorTorque = 0f;
                    frontRightWheel.motorTorque = 0f;
                    break;

                case DriveType.AWD:
                    // Devide torque by half and distributing evenly to all wheels 
                    frontLeftWheel.motorTorque = torque * 0.5f;
                    frontRightWheel.motorTorque = torque * 0.5f;
                    rearLeftWheel.motorTorque = torque * 0.5f;
                    rearRightWheel.motorTorque = torque * 0.5f;
                    break;
            }
        }
        else
        {
            frontLeftWheel.motorTorque = 0f;
            frontRightWheel.motorTorque = 0f;
            rearLeftWheel.motorTorque = 0f;
            rearRightWheel.motorTorque = 0f;
        }
    }

    private void ApplyBrakes(float brake)
    {
        float totalBrakeForce = brake * maxBrakeForce;

        // Add engine braking only if throttle is fully released
        if (throttleInput <= 0.01f)
        {
            totalBrakeForce += engineBrakingPercent * maxBrakeForce;
        }

        // Distribute brake force between front and rear wheels
        float frontBrake = totalBrakeForce * 0.7f; 
        float rearBrake = totalBrakeForce * 0.3f;

        frontLeftWheel.brakeTorque = frontBrake;
        frontRightWheel.brakeTorque = frontBrake;

        rearLeftWheel.brakeTorque = rearBrake;
        rearRightWheel.brakeTorque = rearBrake;
        Debug.Log($"Front brake: {frontBrake}, Rear brake: {rearBrake}");
    }

    private void ApplySteering(float angle)
    {
        frontLeftWheel.steerAngle = angle;
        frontRightWheel.steerAngle = angle;
    }
}
