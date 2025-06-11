using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class TrackingManager : MonoBehaviour
{
    private WorldModel worldModel;
    private Transform egoTransform; 
    private float lookDistance = 20f;

    void Awake()
    {
        if (worldModel == null)
            worldModel = FindObjectOfType<WorldModel>();
    }

    /// <summary>
    /// Finds and returns the closest world object of the specified type <typeparamref name="T"/> 
    /// that is in front of the ego vehicle and within a given look-ahead distance.
    /// </summary>
    /// <typeparam name="T">
    /// The type of world object to search for. Must implement <see cref="IWorldObject"/>.
    /// </typeparam>
    /// <param name="lookDistance">
    /// Maximum distance ahead of the ego vehicle to consider for obstacle detection (in meters). Default is 20.
    /// </param>
    /// <param name="egoPosition">
    /// The position of the ego vehicle. If null, uses the GameObject's <c>transform.position</c>.
    /// </param>
    /// <param name="egoForward">
    /// The forward direction of the ego vehicle. If null, uses the GameObject's <c>transform.forward</c>.
    /// </param>
    /// <returns>
    /// The closest object of type <typeparamref name="T"/> that is in front of the ego vehicle and within the specified distance, 
    /// or <c>null</c> if no such object is found.
    /// </returns>
    public T ReturnClosestObstacle<T>(float lookDistance, Vector3? egoPosition, T exclude = null) where T : class, IWorldObject
    {
        Vector3 pos = egoPosition ?? transform.position;
        Vector3 forward = transform.forward;

        T closest = null;
        float closestDist = float.MaxValue;

        foreach (var obj in worldModel.GetAllWorldObjects())
        {
            if (obj is not T typedObj)
                continue;
            if (exclude != null && typedObj == exclude)
                continue;
            float dist = Vector3.Distance(pos, obj.Position);
            if (dist > lookDistance)
                continue;
            Vector3 toObj = (obj.Position - pos).normalized;
            float dot = Vector3.Dot(forward, toObj);
            if (dot < 0.4f)
                continue;
            if (dist < closestDist)
            {
                closestDist = dist;
                closest = typedObj;
            }
        }
        return closest;
    }



    /// <summary>
    /// Returns all objects of specified types in front and within distance
    /// </summary>
    public List<IWorldObject> ReturnAllObstacles(params Type[] acceptedTypes)
    {
        Vector3 pos = egoTransform.position;
        Vector3 forward = egoTransform.forward;

        return worldModel.GetAllWorldObjects()
            .Where(obj => acceptedTypes.Any(type => type.IsInstanceOfType(obj)))
            .Where(obj => Vector3.Distance(pos, obj.Position) < lookDistance)
            .Where(obj => Vector3.Dot(forward, (obj.Position - pos).normalized) > 0.4f)
            .ToList();
    }
}