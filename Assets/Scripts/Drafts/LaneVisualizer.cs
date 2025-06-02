using UnityEngine;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;

public class LaneVisualizer : MonoBehaviour
{
    [System.Serializable]
    public class Lane
    {
        public int id;
        public float[] coefficients;
        public List<float[]> points; // Deserialize to float arrays first

        [JsonIgnore] public List<Vector2> vectorPoints;

        public void ConvertPoints()
        {
            vectorPoints = new List<Vector2>();
            foreach (var p in points)
                vectorPoints.Add(new Vector2(p[0], p[1]));
        }
    }

    [System.Serializable]
    public class LaneData
    {
        public List<Lane> lanes;
        public int[] image_size;
    }

    public string jsonFileName = "lanes_output.json";
    public Color[] laneColors;

    private LaneData laneData;

    private void Awake()
    {
        LoadLaneData();
    }

    void LoadLaneData()
    {
        string path = Path.Combine(Application.streamingAssetsPath, jsonFileName);

        if (File.Exists(path))
        {
            string jsonContent = File.ReadAllText(path);
            laneData = JsonConvert.DeserializeObject<LaneData>(jsonContent);

            foreach (var lane in laneData.lanes)
                lane.ConvertPoints();

            Debug.Log("Lane data loaded successfully.");
        }
        else
        {
            Debug.LogError($"Lane data file not found: {path}");
        }
    }

    
}
