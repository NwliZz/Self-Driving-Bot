using System.Collections.Generic;
using UnityEngine;

public class PathBuilder : MonoBehaviour
{
    public GameObject waypointPrefab;
    public int maxWaypointCount = 100;
    public float waypointSpacing = 5f;
    public float raycastHeight = 5f;
    public float raycastDistance = 10f;
    public LayerMask roadLayerMask;

    private List<Vector3> pathPoints = new List<Vector3>();

    private void Start()
    {
        GeneratePath();
    
    }

    public void GeneratePath()
    {
        pathPoints.Clear();

        Vector3 currentPos = transform.position;
        Vector3 lastDirection = transform.forward;

        for (int i = 0; i < maxWaypointCount; i++)
        {
            Vector3 rayOrigin = currentPos + Vector3.up * raycastHeight;
            Vector3 scanPoint = currentPos + lastDirection * waypointSpacing;

            // Cast down at scan point
            if (Physics.Raycast(scanPoint + Vector3.up * raycastHeight, Vector3.down, out RaycastHit hit, raycastDistance, roadLayerMask))
            {
                Vector3 wpPos = hit.point + Vector3.up * 0.1f;
                pathPoints.Add(wpPos);
                Instantiate(waypointPrefab, wpPos, Quaternion.identity);

                // Calculate new direction
                if (pathPoints.Count >= 2)
                {
                    lastDirection = (pathPoints[^1] - pathPoints[^2]).normalized;
                }

                currentPos = wpPos;
            }
            else
            {
                Debug.Log("No road found. Ending path.");
                break;
            }
        }

        Debug.Log($"Generated {pathPoints.Count} waypoints.");
    }

    public List<Vector3> GetPath()
    {
        return pathPoints;
    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               