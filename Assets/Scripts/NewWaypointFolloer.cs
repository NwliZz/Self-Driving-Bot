using System.Collections.Generic;
using UnityEngine;
using TMPro;

[RequireComponent(typeof(ProceduralWaypointGenerator))]
public class NewWaypointFollower : MonoBehaviour
{
    private ProceduralWaypointGenerator waypointGenerator;

    [Header("Wheel Colliders")]
    public WheelCollider frontLeftWheel;
    public WheelCollider frontRightWheel;
    public WheelCollider rearLeftWheel;
    public WheelCollider rearRightWheel;

    
    [Header("Driving Settings")]
    public float maxMotorTorque = 1500f;
    public float maxBrakeForce = 2000f;
    public float maxSteerAngle = 30f;
    public float speedLimit = 5f;
    
    [Header("UI")]
    public Rigidbody carRigidbody;
    public TextMeshProUGUI vehicleSpeedText;

    private float motor = 1000f;
    private float currentSpeed;

    void Start()
    {
        waypointGenerator = GetComponent<ProceduralWaypointGenerator>();
    }

    void FixedUpdate()
    {

        // Get current target 
        GameObject target = waypointGenerator.CURRENT_TARGET;
        Vector3 localTarget = transform.InverseTransformPoint(target.transform.position);
        float steer = Mathf.Clamp(localTarget.x / localTarget.magnitude, -1f, 1f);

        // Apply steering
        frontLeftWheel.steerAngle = steer * maxSteerAngle;
        frontRightWheel.steerAngle = steer * maxSteerAngle;

        // Car speed & UI
        currentSpeed = carRigidbody.velocity.magnitude * 3.6f;
        vehicleSpeedText.text = "Speed: " + currentSpeed.ToString("F1") + " km/h";

        //SPEED LIMIT
        float targetSpeed = speedLimit;
        float speedDiff = targetSpeed - currentSpeed;

        // If too fast, coast or brake gently
        if (speedDiff < -2f) // Over by more than 2 km/h
        {
            EngineRunning(false);
            ApplyBrakes(true);
        }
        else if (speedDiff < 0f) // Slightly over
        {
            EngineRunning(false);
            ApplyBrakes(false); // Coast
        }
        else
        {
            ApplyBrakes(false);

            // Apply proportional torque based on how far we are from limit
            float torqueFactor = Mathf.Clamp01(speedDiff / 10f); // 10 km/h buffer
            EngineRunning(true, torqueFactor);
        }

        //if (currentSpeed > speedLimit)
        //{
        //    EngineRunning(false);
        //    ApplyBrakes(true);
        //}
        //else
        //{
        //    EngineRunning(true);
        //    ApplyBrakes(false);
        //}

    }

    private void EngineRunning(bool active, float power = 1f)
    {
        float torque = active ? motor * power : 0f;

        frontLeftWheel.motorTorque = torque;
        frontRightWheel.motorTorque = torque;
    }


    private void ApplyBrakes(bool active)
    {
        float brakeForce = active ? maxBrakeForce : 0f;

        frontLeftWheel.brakeTorque = brakeForce;
        frontRightWheel.brakeTorque = brakeForce;
        rearLeftWheel.brakeTorque = brakeForce;
        rearRightWheel.brakeTorque = brakeForce;
    }
}
