using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Newtonsoft.Json;


[System.Serializable]
public class LanePointList
{
    public List<LaneTestData> lanes;
}

[System.Serializable]
public class LaneTestData
{
    public int id;
    public List<List<int>> points;  // [[x, y], ...]
}

public class LaneJsonLoader : MonoBehaviour
{
    public Camera referenceCamera; // The camera that matches your image
    public string jsonFileName = "lanes_output.json";
    public GameObject waypointPrefab;
    public LaneSplineVisualizer splineVisualizer;
    public int laneToShow = 0;  // Change to see other lanes if needed

    private List<GameObject> waypointObjects = new List<GameObject>();

    [ContextMenu("Load And Create Waypoints")]

    private void Start()
    {
        LoadAndCreateWaypoints();
    }

    public void LoadAndCreateWaypoints()
    {
        // Clean up any old waypoints
        foreach (var go in waypointObjects)
            DestroyImmediate(go);
        waypointObjects.Clear();

        // Load JSON
        string filePath = Path.Combine(Application.streamingAssetsPath, jsonFileName);
        if (!File.Exists(filePath))
        {
            Debug.LogError("LaneJsonLoader: JSON file not found at " + filePath);
            return;
        }
        string json = File.ReadAllText(filePath);

        //var lanesData = JsonUtility.FromJson<LanePointList>(json);

        var lanesData = JsonConvert.DeserializeObject<LanePointList>(json);


        if (lanesData == null || lanesData.lanes == null || lanesData.lanes.Count == 0)
        {
            Debug.LogError("LaneJsonLoader: JSON did not parse or no lanes found.");
            return;
        }

        int laneIndex = Mathf.Clamp(laneToShow, 0, lanesData.lanes.Count - 1);
        var lane = lanesData.lanes[laneIndex];

        // Create waypoint objects & prepare point list
        List<Vector3> points = new List<Vector3>();
        foreach (var pt in lane.points)
        {
            int x = pt[0]; // pixel X
            int y = pt[1]; // pixel Y

            // Map to viewport (0-1) coordinates
            float u = x / (float)referenceCamera.pixelWidth;
            float v = y / (float)referenceCamera.pixelHeight;

            // ScreenToWorldPoint needs Z distance from camera
            float rayDistance = 1000f; // arbitrary large value

            // Project ray from camera through that pixel
            Ray ray = referenceCamera.ViewportPointToRay(new Vector3(u, v, 0)); // 1-v for Unity's y-axis direction
            Debug.DrawRay(ray.origin, ray.direction * rayDistance, Color.yellow, 1000f);
            if (Physics.Raycast(ray, out RaycastHit hit, rayDistance))
            {
                // Place your waypoint at hit.point
                Instantiate(waypointPrefab, hit.point, Quaternion.identity);
            }
            else
            {
                Debug.LogWarning($"Raycast missed at lane point ({x},{y})");
            }
        }

        // Feed to spline visualizer (optional)
        if (splineVisualizer != null)
        {
            splineVisualizer.SetControlPoints(points);
        }
    }
}
