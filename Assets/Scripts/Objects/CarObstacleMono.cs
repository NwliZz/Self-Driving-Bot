using UnityEngine;

public class CarObstacleMono : MonoBehaviour
{
    private WorldModel worldModel;
    [HideInInspector] public Obstacle selfObstacle; 
    public float radius = 1.2f;

    void Start()
    {
        worldModel = FindObjectOfType<WorldModel>();
        selfObstacle = new Obstacle
        {
            Name = gameObject.name,
            Position = transform.position,
            IsDynamic = true,
            Radius = radius
        };
        worldModel.Obstacles.Add(selfObstacle);
    }

    void Update()
    {
        selfObstacle.Position = transform.position;
        // If you want velocity, set it here as well
    }
}