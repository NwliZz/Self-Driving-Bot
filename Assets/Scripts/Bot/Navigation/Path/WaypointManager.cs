using System.Collections.Generic;
using UnityEngine;

public class WaypointManager : MonoBehaviour
{
    public GameObject waypointPrefab;
    public float waypointReachThreshold = 1.8f;

    private List<GameObject> waypoints = new List<GameObject>();
    public IReadOnlyList<GameObject> Waypoints => waypoints;

    public void CreateWaypoint(Vector3 position, Quaternion rotation)
    {
        GameObject wp = Instantiate(waypointPrefab, position, rotation);
        waypoints.Add(wp);
    }

    public bool RemoveFirstWaypoint()
    {
        if (waypoints.Count > 0)
        {
            Destroy(waypoints[0]);
            waypoints.RemoveAt(0);
            return true;
        }
        return false;
    }

    public bool HasReachedWaypoint(Vector3 position)
    {
        if (waypoints.Count > 0)
        {
            return Vector3.Distance(position, waypoints[0].transform.position) < waypointReachThreshold;
        }
        return false;
    }

    public void ClearWaypoints()
    {
        foreach (var wp in waypoints)
        {
            Destroy(wp);
        }
        waypoints.Clear();
    }
}