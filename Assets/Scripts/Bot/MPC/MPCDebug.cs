using UnityEngine;
using System.Linq;

public class MPCDebug : MonoBehaviour
{
    private WorldModel worldModel;

    void Start()
    {
        // Find the single WorldModel in the scene (make sure there's only one!)
        worldModel = FindObjectOfType<WorldModel>();
        if (worldModel == null)
        {
            Debug.LogError("WorldModel not found in the scene!");
        }
    }

    void Update()
    {
        if (worldModel == null)
            return;

        // Get all obstacles within 20 meters of this car
        var closeObstacles = worldModel.GetAllWorldObjects().Where(obs => Vector3.Distance(obs.Position, transform.position) < 20f).ToList();

        foreach (var obs in closeObstacles)
        {
            Debug.Log(GetWorldObjectDebugInfo(obs));
        }

    }

    string GetWorldObjectDebugInfo(IWorldObject obj)
    {
        switch (obj)
        {
            case Obstacle o:
                return $"[MPC DEBUG] | Obstacle name: {o.Name}, at: {o.Position}, Dynamic: {o.IsDynamic}, Radius: {o.Radius}";
            case TrafficLight tl:
                return $"[MPC DEBUG] | TrafficLight at: {tl.Position}, State: {tl.CurrentState}, TimeToNextChange: {tl.TimeToNextChange:F1}s";
            case RoadFeature rf:
                return $"[MPC DEBUG] | RoadFeature: {rf.FeatureType} at {rf.Position}";
            case Car car:
                return $"[MPC DEBUG] | Car name: {car.Name}, Position: {car.Position}, Dynamic: {car.IsDynamic}, Radius: {car.Radius}, Direction: {car.Direction}, Velocity: {car.Velocity}"; 
            default:
                return $"[MPC DEBUG] | Unknown object at: {obj.Position}, Dynamic: {obj.IsDynamic}";
        }
    }
}
