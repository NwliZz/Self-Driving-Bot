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
    [SerializeField] public float maxSteerAngle = 35f;
    [SerializeField] private float trackWidth = 1.5f;
    [SerializeField] public float wheelbase = 2.5f;

    [Header("Car Parts State")]
    [SerializeField] private float frontBrake;
    [SerializeField] private float rearBrake;
    [SerializeField] private float torque;
    [SerializeField] private float steeringDegrees;


   [Header("Engine Braking")]
    [Range(0f, 1f)] public float engineBrakingPercent = 0.09f; // 9% of max brake force

    [Header("Debug Info")]
    [SerializeField] public Rigidbody carRigidbody;

    [Header("Drivetrain Settings")]
    public DriveType driveType;

    //Input values
    private float throttleInput = 0f; //%
    private float brakeInput = 0f;    //%
    private float steerInput = 0f;    //Degrees

    public float CURRENT_SPEED;

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
        //vehicleSpeedText.text = "Speed: " + CURRENT_SPEED.ToString("F1") + " km/h";


        ApplySteering(steerInput);

        ApplyBrakes(brakeInput);

        ApplyMotor(throttleInput);

    }

    public void SetThrottlePercent(float percent)
    {
        percent = Mathf.Clamp(percent, 0f, 100f);
        throttleInput = percent / 100f;
    } 

    public void SetBrakePercent(float percent)
    {
        percent = Mathf.Clamp(percent, 0f, 100f);
        brakeInput = percent / 100f;
    }

    /// <summary>
    /// SetSteeringAngle In degrees
    /// </summary>
    /// <param name="deg"></param>
    public void SetSteeringAngle(float deg)
    {
        steerInput = Mathf.Clamp(deg, -maxSteerAngle, maxSteerAngle);
    }

    private void ApplyMotor(float throttle)
    {
        if (CURRENT_SPEED < topSpeed)
        {
            torque = throttle * maxMotorTorque;

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

        // Add engine braking - Not working well* 
        //if (throttleInput <= 0.01f)
        //{
        //    totalBrakeForce += engineBrakingPercent * maxBrakeForce;
        //}


        // Distribute brake force between front and rear wheels
        frontBrake = totalBrakeForce * 0.7f; 
        rearBrake = totalBrakeForce * 0.3f;

        frontLeftWheel.brakeTorque = frontBrake;
        frontRightWheel.brakeTorque = frontBrake;

        rearLeftWheel.brakeTorque = rearBrake;
        rearRightWheel.brakeTorque = rearBrake;
    }

    private void ApplySteering(float angle)
    {
        // Early out if nearly straight
        if (Mathf.Abs(angle) < 0.01f)
        {
            frontLeftWheel.steerAngle = 0f;
            frontRightWheel.steerAngle = 0f;
            return;
        }

        // Steer angle limit
        angle = Mathf.Clamp(angle, -maxSteerAngle, maxSteerAngle);

        // Compute turning radius
        float absAngleRad = Mathf.Deg2Rad * Mathf.Abs(angle);
        float turningRadius = wheelbase / Mathf.Tan(absAngleRad);


        float halfTrack = trackWidth * 0.5f;

        // Safety, don't divide by zero or negative radius
        turningRadius = Mathf.Max(turningRadius, halfTrack + 0.01f);

        // 5) Compute individual wheel angles
        float innerAngleRad = Mathf.Atan(wheelbase / (turningRadius - halfTrack));
        float outerAngleRad = Mathf.Atan(wheelbase / (turningRadius + halfTrack));

        steeringDegrees = Mathf.Atan(wheelbase / turningRadius);

        float innerDeg = Mathf.Rad2Deg * innerAngleRad;
        float outerDeg = Mathf.Rad2Deg * outerAngleRad;

        // 6) Assign to wheel colliders based on turn direction
        if (angle > 0) // turning right -> inner is right wheel
        {
            frontLeftWheel.steerAngle = outerDeg;
            frontRightWheel.steerAngle = innerDeg;
        }
        else // turning left -> inner is left wheel
        {
            frontLeftWheel.steerAngle = -innerDeg;
            frontRightWheel.steerAngle = -outerDeg;
        }

        // debug
        Vector3 rearAxle = GetRearAxlePosition();
        Debug.DrawRay(rearAxle, transform.forward * 5f, Color.green);
    }

    public Vector3 GetRearAxlePosition()
    {
        return (rearLeftWheel.transform.position + rearRightWheel.transform.position) / 2f;
    }

    private void OnDrawGizmosSelected()
    {
        if (Mathf.Abs(steerInput) < 0.01f) return;

        // 1) Get the true pivot: the midpoint between the two rear wheels
        Vector3 rearAxleMid = GetRearAxlePosition();

        // 3) Turning radius from your current steer angle
        float radius = wheelbase / Mathf.Tan(Mathf.Deg2Rad * Mathf.Abs(steerInput));

        // 4) Offset sideways from the rear axle toward the center
        Vector3 pivotOffset = steerInput < 0 ? -transform.right * radius : transform.right * radius;

        // 5) Final center of rotation in world space
        Vector3 centerOfRotation = rearAxleMid + pivotOffset;

        // 6) Draw
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(centerOfRotation, 0.2f);
        Gizmos.DrawLine(rearAxleMid, centerOfRotation);
    }
}
