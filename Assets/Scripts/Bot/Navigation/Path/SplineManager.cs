using System.Collections.Generic;
using UnityEngine;


// This script is responsible for generating a spline path for the bot to follow.
public class SplineManager : MonoBehaviour
{
    private WaypointManager waypointManager;

    [Header("Look Ahead Settings")]
    public float curvatureThreshold = 0.5f; //fine-tune sensitivity to curves !! ONLY FOR SFOLOWER
    public float lookaheadGain = 0.5f;
    public float minLookAheadDistance = 3.7f;
    public float maxLookAheadDistance = 12f;
    private Vector3 lastLookAhead;

    [Header("Spline Path Settings")]
    private List<Vector3> splinePoints = new List<Vector3>();
    public IReadOnlyList<Vector3> SplinePoints => splinePoints;
    public int subdivisionsPerSegment = 8;

    private void Start()
    {
        waypointManager = GetComponent<WaypointManager>();
    }

    public void GenerateSplinePath(IReadOnlyList<GameObject> Waypoints)
    {
        splinePoints.Clear();
        List<Vector3> control = new List<Vector3> ();

        foreach (var wp in Waypoints)
        {
            control.Add(wp.transform.position);
        }

        // Add extra control points for a smoother start and end
        control.Insert(0, control[0]);
        // TODO: ADD EXTRA TO TH END TOO

        for (int i = 0; i < control.Count - 3; i++)
        {
            Vector3 p0 = control[i];
            Vector3 p1 = control[i + 1];
            Vector3 p2 = control[i + 2];
            Vector3 p3 = control[i + 3];

            for (int j = 0; j <= subdivisionsPerSegment; j++)
            {
                float t = j / (float)subdivisionsPerSegment;
                splinePoints.Add(CatmullRom(p0, p1, p2, p3, t));
            }
        }
    }

    public bool HasReachedCurrentSegment(Vector3 rearAxle)
    {
        float distToPoint = Vector3.Distance(rearAxle, SplinePoints[subdivisionsPerSegment*2]);

        if (distToPoint < 5f)
        {
            Debug.Log("Here");
            return true;
        }
        return false;
    }

    public void RemoveSegment()
    {
        if (SplinePoints.Count > subdivisionsPerSegment + 1)
        {
            splinePoints.RemoveRange(0, subdivisionsPerSegment + 1);
        }
        // Optional: Keep 1-2 points at the front for interpolation stability
    }

    public void AddSegment(IReadOnlyList<GameObject> Waypoints)
    {

        if (waypointManager.Waypoints.Count < 4 || SplinePoints.Count < 1)
        {
            Debug.LogWarning("SplineManager | Not enough control or spline points to add segment.");
            return;
        }

        int n = Waypoints.Count;

        Vector3 p0 = SplinePoints[SplinePoints.Count-1];
        Vector3 p1 = waypointManager.Waypoints[n - 3].transform.position;
        Vector3 p2 = waypointManager.Waypoints[n - 2].transform.position;
        Vector3 p3 = waypointManager.Waypoints[n - 1].transform.position;

        for (int j = 0; j <= subdivisionsPerSegment; j++) // j=1 to avoid duplicate of last point
        {
            float t = j / (float)subdivisionsPerSegment;
            splinePoints.Add(CatmullRom(p0, p1, p2, p3, t));
        }
    }

    private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        return 0.5f * (
            (2f * p1) +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * Mathf.Pow(t,2) +
            (-p0 + 3f * p1 - 3f * p2 + p3) * Mathf.Pow(t,3)
        );
    }

    public int FindClosestSplinePoint(Vector3 position)
    {
        if (SplinePoints == null || SplinePoints.Count == 0)
            return -1;

        int closestIndex = 0;
        float closestDistance = float.MaxValue;

        for (int i = 0; i < SplinePoints.Count; i++)
        {
            float distance = Vector3.Distance(position, SplinePoints[i]);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestIndex = i;
            }
        }

        return closestIndex;
    }
    public float CalculateCurvature(int index)
    {
        if (index <= 0 || index >= SplinePoints.Count - 1)
            return 0f;

        Vector3 prev = SplinePoints[index - 1];
        Vector3 current = SplinePoints[index];
        Vector3 next = SplinePoints[index + 1];

        Vector3 v1 = current - prev;
        Vector3 v2 = next - current;

        float angle = Vector3.Angle(v1, v2);
        return angle / Vector3.Distance(prev, next);
    }
    public Vector3 GetLookAheadPoint(Vector3 rearAxle, float vehicleSpeed)
    {

        if (SplinePoints == null || SplinePoints.Count == 0)
        {
            Debug.LogWarning("SplineManager | GetLookAheadPoint: splinePoints is null or empty!");
            return rearAxle; // or some safe fallback value
        }

        float adjustedDistance = minLookAheadDistance + lookaheadGain * vehicleSpeed;
        adjustedDistance = Mathf.Clamp(adjustedDistance, minLookAheadDistance, maxLookAheadDistance);

        float accumulated = 0f;

        // Safty Check
        int closestIndex = FindClosestSplinePoint(rearAxle);
        if (closestIndex < 0 || closestIndex >= SplinePoints.Count)
        {
            Debug.LogWarning("SplineManager | GetLookAheadPoint: closestIndex invalid!");
            return rearAxle;
        }
        //
        Vector3 currentPoint = SplinePoints[closestIndex];


        for (int i = FindClosestSplinePoint(rearAxle); i < SplinePoints.Count - 1; i++)
        {
            Vector3 nextPoint = SplinePoints[i + 1];
            accumulated += Vector3.Distance(currentPoint, nextPoint);

            if (accumulated >= adjustedDistance)
            {
                return nextPoint;
            }

            currentPoint = nextPoint;
        }

        return SplinePoints[SplinePoints.Count - 1];
    }

    private void OnDrawGizmosSelected()
    {
        if (SplinePoints == null || SplinePoints.Count < 2)
            return;

        // Draw the spline segments
        Gizmos.color = Color.yellow;
        for (int i = 0; i < SplinePoints.Count - 1; i++)
        {
            Gizmos.DrawLine(SplinePoints[i], SplinePoints[i + 1]);
        }

        // Draw a small sphere at each sample point
        Gizmos.color = Color.red;
        foreach (var pt in SplinePoints)
        {
            Gizmos.DrawSphere(pt, 0.1f);
        }
    }
}
    