using UnityEngine;

[RequireComponent(typeof(SimulationHandler))]
public class ManualCarController : MonoBehaviour
{
    private SimulationHandler sim;

    [Header("Manual Control Settings")]
    public float throttleSpeed = 100f; // % when holding W
    public float brakeSpeed = 100f;    // % when holding S
    public float steeringSpeed = 30f;  // Degrees max steering

    private void Start()
    {
        sim = GetComponent<SimulationHandler>();
    }

    private void Update()
    {
        float throttle = Input.GetKey(KeyCode.W) ? throttleSpeed : 0f;
        sim.SetThrottlePercent(throttle);

        float brake = Input.GetKey(KeyCode.S) ? brakeSpeed : 0f;
        sim.SetBrakePercent(brake);


        float steerInput = 0f;
        if (Input.GetKey(KeyCode.A)) steerInput = -steeringSpeed;
        if (Input.GetKey(KeyCode.D)) steerInput = steeringSpeed;
        sim.SetSteeringAngle(steerInput);


    }
}
