using UnityEngine;
using System.Collections.Generic;
using System.IO;
using Mapbox.Json;

public class LaneWaypointLoader : MonoBehaviour
{
    public GameObject waypointPrefab;
    public string pointsFilePath; // Place this in Assets/StreamingAssets for easy loading
    public Camera laneCamera; // Reference to your lane camera

    public class PointsData
    {
        public List<List<int>> points;
    }

    void Start()
    {

        string filePath = Path.Combine(Application.streamingAssetsPath, "waypoints.json");


        if (File.Exists(filePath))
        {
            string jsonString = File.ReadAllText(filePath); // Read file CONTENT, not path
            Debug.Log(jsonString);

            List<List<int>> listOfPoints = JsonConvert.DeserializeObject<List<List<int>>>(jsonString);
            //PointsData pointsData = JsonUtility.FromJson<PointsData>(jsonString);

            foreach (var point in listOfPoints)
            {
                Vector3 worldPos = CameraToWorld(point[0], point[1]);
                Instantiate(waypointPrefab, worldPos, Quaternion.identity);
                Debug.Log($"[{point[0]}, {point[1]}]");
            }


            //foreach (var pt in pointsData.points)
            //{
            //    Vector3 worldPos = CameraToWorld(pt[0], pt[1]);
            //    Instantiate(waypointPrefab, worldPos, Quaternion.identity);
            //}
        }
        else
        {
            Debug.LogError("Lane points file not found: " + filePath);
        }
    }

    // Helper to convert from image coordinates to world
    Vector3 CameraToWorld(float px, float py)
    {
        float imgWidth = 512f;
        float imgHeight = 256f;

        float normX = px / imgWidth;
        float normY = py / imgHeight;

        Ray ray = laneCamera.ViewportPointToRay(new Vector3(normX, 1f - normY, 0));

        // Raycast down to hit the road (must have collider on the road mesh)
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit, 100f))
        {
            return hit.point; // This is the exact world position on the road mesh
        }
        else
        {
            // fallback: fixed distance
            return ray.GetPoint(20f);
        }
    }
}
