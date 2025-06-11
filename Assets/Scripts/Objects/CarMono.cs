using UnityEngine;

public class CarMono : MonoBehaviour
{
    private WorldModel worldModel;
    [HideInInspector] public Car self;
    public float radius;
    private Vector3 lastPosition;
    [SerializeField]private Vector3 velocity;
    [SerializeField]private Vector3 heading;

    private SimulationHandler simulationHandler;

    void Start()
    {
        worldModel = FindObjectOfType<WorldModel>();
        simulationHandler = GetComponent<SimulationHandler>();
        self = new Car
        {
            Name = gameObject.name,
            Position = transform.position,
            IsDynamic = true,
            Radius = radius,
            Direction = transform.forward,
            Velocity = (transform.position - lastPosition) / Time.deltaTime

        };
        worldModel.Cars.Add(self);

        lastPosition = transform.position;
    }

    void FixedUpdate()
    {
        // Update position, velocity, heading

        self.Position = transform.position;
        self.Velocity = (transform.position - lastPosition) / Time.deltaTime;
        self.Direction = transform.forward;

        lastPosition = transform.position;
    }
}
