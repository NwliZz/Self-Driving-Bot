using System.Collections.Generic;
using UnityEngine;

// Base interface for all detectable world objects
public interface IWorldObject
{
    Vector3 Position { get; }
    bool IsDynamic { get; }
}

// Obstacle Example (cars, pedestrians, cones, etc.)
public class Obstacle : IWorldObject
{
    public Vector3 Position { get; set; }
    public bool IsDynamic { get; set; }
    public float Radius { get; set; }
}

// Traffic Light Example
public class TrafficLight : IWorldObject
{
    public Vector3 Position { get; set; }
    public bool IsDynamic { get; set; } = false;
    public enum State { Red, Yellow, Green }
    public State CurrentState { get; set; }
    public float TimeToNextChange { get; set; }
}

// Road Feature Example (intersections, crosswalks)
public class RoadFeature : IWorldObject
{
    public Vector3 Position { get; set; }
    public bool IsDynamic { get; set; } = false;
    public string FeatureType { get; set; }
}

// Centralized World Model for MPC
public class WorldModel : MonoBehaviour
{
    public List<Obstacle> Obstacles { get; private set; } = new List<Obstacle>();
    public List<TrafficLight> TrafficLights { get; private set; } = new List<TrafficLight>();
    public List<RoadFeature> RoadFeatures { get; private set; } = new List<RoadFeature>();

    // Unified accessor if needed
    public IEnumerable<IWorldObject> GetAllWorldObjects()
    {
        foreach (var o in Obstacles) yield return o;
        foreach (var t in TrafficLights) yield return t;
        foreach (var f in RoadFeatures) yield return f;
    }

    // Example: Update with new detections each frame
    void Update()
    {
        // Populate/update the lists here using raycasts, sensors, or scene info
    }
}
