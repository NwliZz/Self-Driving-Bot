using UnityEngine;
using TMPro;

public class WaypointFollower : MonoBehaviour
{
    public Transform[] waypoints;
    public WheelCollider frontLeftWheel;
    public WheelCollider frontRightWheel;
    public WheelCollider rearLeftWheel;
    public WheelCollider rearRightWheel;

    public float maxMotorTorque = 1500f;
    public float maxBrakeForce = 2000;

    public float maxSteerAngle = 30f;
    public float waypointThreshold = 3f;

    private int currentWaypointIndex = 0;

    float motor = 1000;

    float curentSpeed;
    float speedLimit = 20;

    public Rigidbody carRigidbody;
    public TextMeshProUGUI vehicleSpeedText;

    void FixedUpdate()
    {
  

        //Curent Vehicle Speed
        curentSpeed = carRigidbody.velocity.magnitude * 3.6f;
        
        //Geting WP, Setting it to local target, and calculating steer
        Transform target = waypoints[currentWaypointIndex];
        Vector3 localTarget = transform.InverseTransformPoint(target.position);
        float steer = Mathf.Clamp(localTarget.x / localTarget.magnitude, -1f, 1f);

        if (Vector3.Distance(transform.position, target.position) < waypointThreshold) 
        {
            if(currentWaypointIndex < waypoints.Length - 1)
            {
                currentWaypointIndex++;
                EngineRunning(true);
                ApllyBrakes(false);
            }
            else
            {
                EngineRunning(false);
                ApllyBrakes(true);
            }
        }

        //Aply Sterring
        frontLeftWheel.steerAngle = steer * maxSteerAngle;
        frontRightWheel.steerAngle = steer * maxSteerAngle;

        //Update UI
        vehicleSpeedText.text = "Speed: " + curentSpeed.ToString("F1") + " km/h";

        //Aply Motor Torque
        if (curentSpeed > speedLimit)
        {
            EngineRunning(false);
        }
        else
        {
            EngineRunning(true);
        }

        // Advance to next waypoint if close enough
        //if (Vector3.Distance(transform.position, target.position) < waypointThreshold)
        //{
        //    currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
        //}
        
        
    }

    private void EngineRunning(bool b)
    {
        if (b)
        {
            frontLeftWheel.motorTorque = motor;
            frontRightWheel.motorTorque = motor;
        }
        else
        {
            frontLeftWheel.motorTorque = 0f;
            frontRightWheel.motorTorque = 0f;
        }
    }

    private void ApllyBrakes(bool b)
    {
        if (b) 
        {
            frontLeftWheel.brakeTorque = maxBrakeForce;
            frontRightWheel.brakeTorque = maxBrakeForce;
            rearLeftWheel.brakeTorque = maxBrakeForce;
            rearRightWheel.brakeTorque = maxBrakeForce;
        }
        else
        {
            frontLeftWheel.brakeTorque = 0f;
            frontRightWheel.brakeTorque = 0f;
            rearLeftWheel.brakeTorque = 0f;
            rearRightWheel.brakeTorque = 0f;
        }
    }
}
