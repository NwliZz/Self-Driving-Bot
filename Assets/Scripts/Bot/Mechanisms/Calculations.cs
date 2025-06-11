using System.Collections.Generic;
using UnityEngine;

public class Calculations 
{
    public List<Vector3> GetDetectionCorridor(IReadOnlyList<Vector3> splinePoints, int clostestSplinePointIndx)
    {
        List<Vector3> detectionCorridor = new List<Vector3>();

        // Use full spline instead of limited distance
        for (int i = clostestSplinePointIndx; i < splinePoints.Count - 1; i++)
        {
            detectionCorridor.Add(splinePoints[i]);
        }

        return detectionCorridor;
    }
}
