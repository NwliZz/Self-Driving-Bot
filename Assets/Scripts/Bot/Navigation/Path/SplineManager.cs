using System.Collections.Generic;
using UnityEngine;


// This script is responsible for generating a spline path for the bot to follow.
public class SplineManager : MonoBehaviour
{
    [Header("Look Ahead Settings")]
    public float curvatureThreshold = 0.5f; //fine-tune sensitivity to curves
    private float minLookAheadDistance = 3.7f;
    private float maxLookAheadDistance = 12f;

    public int subdivisionsPerSegment = 8;

    private List<Vector3> splinePoints = new List<Vector3>();
    public IReadOnlyList<Vector3> SplinePoints => splinePoints;

    Vector3 rearAxle;


    public void RegenerateSpline(Vector3 rAxle, IReadOnlyList<GameObject>Waypoints)
    {
        List<Vector3> waypointPositions = new List<Vector3>();
        foreach (var wp in Waypoints)
        {
            waypointPositions.Add(wp.transform.position);
        }

        Vector3 rearAxle = rAxle;
        GenerateSpline(rearAxle, waypointPositions);
    }

    private void GenerateSpline(Vector3 startPoint, List<Vector3> waypoints)
    {
        splinePoints.Clear();
        List<Vector3> control = new List<Vector3> { startPoint };
        control.AddRange(waypoints);

        // Add extra control points for a smoother start and end
        control.Insert(0, control[0]);
        control.Add(control[control.Count - 1]);

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

    private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        return 0.5f * (
            (2f * p1) +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * t * t +
            (-p0 + 3f * p1 - 3f * p2 + p3) * t * t * t
        );
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

    public int FindClosestSplinePoint(Vector3 position)
    {
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

    public Vector3 GetLookAheadPoint(float baseDistanceAhead, Vector3 rearAxle)
    {

        float accumulated = 0f;

        // Find the closest point on the spline
        int startIndex = FindClosestSplinePoint(rearAxle);

        // Calculate curvature and adjust look-ahead distance
        float curvature = CalculateCurvature(startIndex);
        float adjustedDistance = Mathf.Lerp(minLookAheadDistance, maxLookAheadDistance, 1 - curvature / curvatureThreshold);
        adjustedDistance = Mathf.Clamp(adjustedDistance, minLookAheadDistance, baseDistanceAhead);

        // Walk forward along the spline
        Vector3 currentPoint = SplinePoints[startIndex];
        for (int i = startIndex; i < SplinePoints.Count - 1; i++)
        {   
            Vector3 nextPoint = SplinePoints[i + 1];
            float segmentDist = Vector3.Distance(currentPoint, nextPoint);

            accumulated += segmentDist;

            if (accumulated >= adjustedDistance)
                return nextPoint;

            currentPoint = nextPoint;
        }

        // If we run out of spline, return the last point
        return SplinePoints[^1];
    }


}