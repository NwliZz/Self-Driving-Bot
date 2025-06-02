using UnityEngine;

public class TagManager : MonoBehaviour
{
    public float delay = 1.0f; // Wait a bit to let Mapbox generate tiles

    public static bool RoadsTagged = false;

    void Start()
    {
        Invoke(nameof(TagRoadMeshes), delay);
    }

    void TagRoadMeshes()
    {
        int count = 0;
        GameObject[] allObjects = FindObjectsOfType<GameObject>();

        foreach (GameObject obj in allObjects)
        {
            if (obj.GetComponent<MeshCollider>() && obj.name.StartsWith("Untitled"))
            {
                obj.tag = "Road";
                count++;
            }
        }

        Debug.Log("Tagged " + count + " road mesh(es) as 'Road'");
        RoadsTagged = true;
    }
}
