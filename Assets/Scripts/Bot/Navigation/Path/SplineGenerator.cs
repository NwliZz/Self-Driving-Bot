using System.Collections.Generic;
using UnityEngine;


// This script is responsible for generating a spline path for the bot to follow.
public class SplineGenerator : MonoBehaviour
{
    public int subdivisionsPerSegment = 8;

    private List<Vector3> splinePoints = new List<Vector3>();
    public IReadOnlyList<Vector3> SplinePoints => splinePoints;

    public void GenerateSpline(Vector3 startPoint, IEnumerable<Vector3> waypoints)
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
}