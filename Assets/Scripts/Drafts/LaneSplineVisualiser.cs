using System.Collections.Generic;
using UnityEngine;

public class LaneSplineVisualizer : MonoBehaviour
{
    [Header("Spline Settings")]
    public int subdivisionsPerSegment = 8;
    private List<Vector3> controlPoints = new List<Vector3>();
    private List<Vector3> splinePoints = new List<Vector3>();

    public void SetControlPoints(List<Vector3> points)
    {
        controlPoints = points;
        GenerateSpline();
    }

    public void GenerateSpline()
    {
        splinePoints.Clear();
        if (controlPoints == null || controlPoints.Count < 4)
            return;

        // Duplicate the first and last points for proper spline start/end
        List<Vector3> ctrl = new List<Vector3>(controlPoints);
        ctrl.Insert(0, ctrl[0]);
        ctrl.Add(ctrl[ctrl.Count - 1]);

        for (int i = 0; i < ctrl.Count - 3; i++)
        {
            Vector3 p0 = ctrl[i];
            Vector3 p1 = ctrl[i + 1];
            Vector3 p2 = ctrl[i + 2];
            Vector3 p3 = ctrl[i + 3];
            for (int j = 0; j <= subdivisionsPerSegment; j++)
            {
                float t = j / (float)subdivisionsPerSegment;
                splinePoints.Add(CatmullRom(p0, p1, p2, p3, t));
            }
        }
    }

    private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        // Standard Catmull-Rom spline interpolation
        return 0.5f * (
            2f * p1 +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * t * t +
            (-p0 + 3f * p1 - 3f * p2 + p3) * t * t * t
        );
    }

    private void OnDrawGizmos()
    {
        if (splinePoints == null || splinePoints.Count < 2)
            return;

        Gizmos.color = Color.yellow;
        for (int i = 0; i < splinePoints.Count - 1; i++)
        {
            Gizmos.DrawLine(splinePoints[i], splinePoints[i + 1]);
        }
        Gizmos.color = Color.red;
        foreach (var pt in splinePoints)
        {
            Gizmos.DrawSphere(pt, 0.07f);
        }
    }
}
