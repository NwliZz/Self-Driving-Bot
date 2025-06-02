using UnityEngine;

public class CarMono : MonoBehaviour
{
    private WorldModel worldModel;
    [HideInInspector] public Car self;
    public float radius;
    private Vector3 lastPosition;
    [SerializeField]private Vector3 velocity;
    [SerializeField]private Vector3 heading;

    void Start()
    {
        worldModel = FindObjectOfType<WorldModel>();
        self = new Car
        {
            Name = gameObject.name,
            Position = transform.position,
            IsDynamic = true,
            Radius = radius,
            Direction = transform.forward,
            Velocity = Vector3.zero
            
        };
        worldModel.Cars.Add(self);
    }

    void Update()
    {
        // Update position, velocity, heading
        Vector3 currentPosition = transform.position;
        velocity = (currentPosition - lastPosition) / Time.deltaTime;
        heading = transform.forward;

        self.Position = currentPosition;
        self.Velocity = velocity;
        self.Direction = heading;

        lastPosition = currentPosition;
    }
}
