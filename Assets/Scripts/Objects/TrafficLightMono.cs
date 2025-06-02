using UnityEngine;

public class TrafficLightMono : MonoBehaviour
{
    private float greenDuration = 10f;
    private float yellowDuration = 2f;
    private float redDuration = 8f;

    [SerializeField] private float timer;
    [SerializeField] private TrafficLight.State currentState = TrafficLight.State.Red;
    private WorldModel worldModel;
    private TrafficLight trafficLightData;

    public Renderer rend;

    void Start()
    {
        rend.material.color = Color.red;
        worldModel = FindObjectOfType<WorldModel>();

        // Register this traffic light in the WorldModel
        trafficLightData = new TrafficLight
        {
            Position = transform.position,
            CurrentState = currentState,
            TimeToNextChange = redDuration
        };
        worldModel.TrafficLights.Add(trafficLightData);

        timer = redDuration;
    }

    void Update()
    {
        timer -= Time.deltaTime;
        if (timer <= 0)
        {
            // Change state
            switch (currentState)
            {
                case TrafficLight.State.Red:
                    rend.material.color = Color.green;
                    currentState = TrafficLight.State.Green;
                    timer = greenDuration;
                    break;
                case TrafficLight.State.Green:
                    rend.material.color = Color.yellow;
                    currentState = TrafficLight.State.Yellow;

                    timer = yellowDuration;
                    break;
                case TrafficLight.State.Yellow:

                    rend.material.color = Color.red;
                    currentState = TrafficLight.State.Red;


                    timer = redDuration;
                    break;
            }

            // Update WorldModel info
            trafficLightData.CurrentState = currentState;
            trafficLightData.TimeToNextChange = timer;
        }
        else
        {

            // Just update the time left
            trafficLightData.TimeToNextChange = timer;
        }
    }
}
