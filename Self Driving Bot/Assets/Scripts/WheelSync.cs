using UnityEngine;

public class WheelSync : MonoBehaviour
{
    [System.Serializable]
    public class Wheel
    {
        public WheelCollider collider;
        public Transform visual;
    }

    public Wheel[] wheels;

    void Update()
    {
        foreach (var wheel in wheels)
        {
            UpdateVisualWheel(wheel.collider, wheel.visual);
        }
    }

    void UpdateVisualWheel(WheelCollider collider, Transform visual)
    {
        Vector3 pos;
        Quaternion rot;

        // Get world position and rotation from the collider
        collider.GetWorldPose(out pos, out rot);

        // Apply to the visual wheel
        visual.position = pos;
        visual.rotation = rot;
    }
}
