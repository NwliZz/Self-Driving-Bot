using UnityEngine;
using System.IO;

public class CameraCapture : MonoBehaviour
{
    public Camera cam;
    public string imageSavePath = "CapturedImages";
    public int captureWidth = 1280;
    public int captureHeight = 720;

    private void Start()
    {
        if (cam == null)
            cam = GetComponent<Camera>();

        if (!Directory.Exists(imageSavePath))
            Directory.CreateDirectory(imageSavePath);
    }
    private void Update()
    {
        // Example trigger (you can call this in Update or via key press for testing)
        if (Input.GetKeyDown(KeyCode.C))
        {
            CaptureImage("frame.png");
        }

    }

    public void CaptureImage(string filename = "frame.png")
    {
        RenderTexture rt = new RenderTexture(captureWidth, captureHeight, 24);
        cam.targetTexture = rt;

        Texture2D screenShot = new Texture2D(captureWidth, captureHeight, TextureFormat.RGB24, false);
        cam.Render();
        RenderTexture.active = rt;
        screenShot.ReadPixels(new Rect(0, 0, captureWidth, captureHeight), 0, 0);
        screenShot.Apply();

        cam.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);

        byte[] bytes = screenShot.EncodeToPNG();
        File.WriteAllBytes(Path.Combine(imageSavePath, filename), bytes);
    }
}
