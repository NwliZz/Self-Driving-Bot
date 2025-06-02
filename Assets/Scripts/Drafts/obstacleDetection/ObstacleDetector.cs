using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class ObstacleDetector : MonoBehaviour
{
    public float detectionRange = 50;
    public List<GameObject> Obstacles = new List<GameObject>();
    public List<TrafficLight> traficLights = new List<TrafficLight>();

    private WorldModel wm;
    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
       asd();
       ScanForObstacles();
    }

    private void ScanForObstacles()
    {
        Collider[] targets = Physics.OverlapSphere(transform.position, detectionRange);

        targets = targets.Where(t => t.transform != transform).ToArray();

        if (targets.Length > 0)
        {
            foreach (Collider c in targets)
            {
                SaveObstacle(c);
            }
        }
    }

    private void SaveObstacle(Collider c)
    {
        switch (c.gameObject.tag)
        {
            case "Obstacle":
                Obstacles.Add(c.gameObject);
                break;

            case "TrafficLight":
                //TODO...
                break;
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.blue;
        // Draw a wire sphere at the transform's position with the detection range
        Gizmos.DrawWireSphere(transform.position, detectionRange);
    }
    private void asd()
    {
        for(int i = 0; i < Obstacles.Count; i++)
        {
            Debug.Log(Obstacles[i].transform.position);
        }
    }

 
}
