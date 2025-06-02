using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Base interface for all detectable world objects.
/// Every world object must have a Position and say whether it is dynamic (can move) or not.
/// </summary>
public interface IWorldObject
{
    Vector3 Position { get; }   // Where is this object?
    bool IsDynamic { get; }     // Does this object move?
}

/// <summary>
/// Data structure for obstacles, such as vehicles, pedestrians, cones, etc.
/// Implements IWorldObject so it can be handled generically.
/// </summary>
public class Obstacle : IWorldObject
{
    public string Name { get; set; }
    public Vector3 Position { get; set; } // Position of the obstacle in the world
    public bool IsDynamic { get; set; }   // Is it moving? (true for cars, false for cones)
    public float Radius { get; set; }     // Size, useful for collision checks
}

public class Car : IWorldObject
{
    public string Name { get; set; }
    public Vector3 Position { get; set; }
    public bool IsDynamic { get; set; } = true;
    public float Radius { get; set; }     // Size, useful for collision checks

    public Vector3 Velocity { get; set; }
    public Vector3 Direction { get; set; }


}

/// <summary>
/// Data structure for a traffic light.
/// Implements IWorldObject for general handling in lists.
/// </summary>
public class TrafficLight : IWorldObject
{
    public Vector3 Position { get; set; }         // Position of the traffic light
    public bool IsDynamic { get; set; } = false;  // Traffic lights are static (don't move)

    // Enum to represent the state of the light
    public enum State { Red, Yellow, Green }
    public State CurrentState { get; set; }       // The current light color
    public float TimeToNextChange { get; set; }   // Time until the next state switch
}

/// <summary>
/// Data structure for road features such as intersections and crosswalks.
/// Implements IWorldObject for generic access.
/// </summary>
public class RoadFeature : IWorldObject
{
    public Vector3 Position { get; set; }         // Where is this feature (center of intersection, etc.)
    public bool IsDynamic { get; set; } = false;  // Usually static features
    public string FeatureType { get; set; }       // E.g. "Intersection", "Crosswalk"
}

/// <summary>
/// Centralized world model to keep track of all objects relevant to self-driving logic.
/// Only one instance per scene!
/// </summary>
public class WorldModel : MonoBehaviour
{
    // List of all detected obstacles in the scene
    public List<Obstacle> Obstacles { get; private set; } = new List<Obstacle>();

    // List of all detected traffic lights in the scene
    public List<TrafficLight> TrafficLights { get; private set; } = new List<TrafficLight>();

    // List of all detected road features in the scene
    public List<RoadFeature> RoadFeatures { get; private set; } = new List<RoadFeature>();

    // List of all detected cars in the scene
    public List<Car> Cars { get; private set; } = new List<Car>();

    /// <summary>
    /// Optional: Get every world object (obstacles, lights, features) in a single list for easy querying.
    /// This makes it easy for AI/MPC to process all relevant objects together.
    /// </summary>
    /// <returns>IEnumerable of IWorldObject (so you don't care about their concrete type)</returns>
    public IEnumerable<IWorldObject> GetAllWorldObjects()
    {
        foreach (var o in Obstacles) yield return o;         // Yield each obstacle
        foreach (var t in TrafficLights) yield return t;     // Yield each traffic light
        foreach (var f in RoadFeatures) yield return f;      // Yield each road feature
        foreach (var c in Cars) yield return c;
    }

    /// <summary>
    /// This method runs every Unity frame (60x/sec by default).
    /// In here you would update/populate the lists by checking the scene,
    /// sensors, raycasts, or other detection logic.
    /// For now, this is just a placeholder.
    /// </summary>
    void Update()
    {
        // Populate/update the lists here using raycasts, sensors, or scene info
        // For example, scan for new obstacles, update traffic light states, etc.
    }
}
