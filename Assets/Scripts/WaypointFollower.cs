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
    public float maxSteerAngle = 30f;
    public float waypointThreshold = 3f;

    private int currentWaypointIndex = 0;

    float motor;

    float curentSpeed;
    float speedLimit = 20;

    public Rigidbody carRigidbody;
    public TextMeshProUGUI vehicleSpeedText;

    void FixedUpdate()
    {
        curentSpeed = carRigidbody.velocity.magnitude * 3.6f;
        
        if(curentSpeed > speedLimit)
        {
            motor = 0.1f;
        }
        else
        {
            motor = maxMotorTorque;
        }
        Transform target = waypoints[currentWaypointIndex];
        Vector3 localTarget = transform.InverseTransformPoint(target.position);
        float steer = Mathf.Clamp(localTarget.x / localTarget.magnitude, -1f, 1f);
        

        vehicleSpeedText.text = "Speed: " + curentSpeed.ToString("F1") + " km/h";

        frontLeftWheel.steerAngle = steer * maxSteerAngle;
        frontRightWheel.steerAngle = steer * maxSteerAngle;

        rearLeftWheel.motorTorque = motor;
        rearRightWheel.motorTorque = motor;

        // Advance to next waypoint if close enough
        if (Vector3.Distance(transform.position, target.position) < waypointThreshold)
        {
            currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
        }
    }
}
