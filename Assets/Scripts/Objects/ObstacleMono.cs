using UnityEngine;

public class ObstacleMono : MonoBehaviour
{
    public float radius = 0.5f;
    private WorldModel worldModel;
    private Obstacle linkedObstacle;

    void Start()
    {
        worldModel = FindObjectOfType<WorldModel>();

        // Create and add obstacle to world model
        linkedObstacle = new Obstacle
        {
            Name = gameObject.name,
            Position = transform.position,
            IsDynamic = false, // or true if it moves
            Radius = radius
        };
        worldModel.Obstacles.Add(linkedObstacle);
    }

    void Update()
    {
        // Update the Obstacle's data every frame
        linkedObstacle.Position = transform.position;
        // Update other properties as needed
    }
}