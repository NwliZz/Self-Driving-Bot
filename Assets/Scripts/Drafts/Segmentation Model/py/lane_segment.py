import cv2
import numpy as np

def segment_lanes(image_path):
    # Load the Unity camera frame
    image = cv2.imread(image_path)
    if image is None:
        print("Error: image not found at", image_path)
        return None

    height, width, _ = image.shape

    # Create a dummy mask: draw a white vertical lane center
    mask = np.zeros((height, width), dtype=np.uint8)
    lane_center_x = width // 2
    mask[:, lane_center_x-4:lane_center_x+4] = 255  # 8-pixel-wide center line

    return mask

if __name__ == "__main__":
    input_path = "D:/Documents/GitHub/Self-Driving-Bot/CapturedImages/frame.png"  # Adjust if needed
    mask = segment_lanes(input_path)

    if mask is not None:
        # Show it in a window
        cv2.imshow("Lane Mask", mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Optional: Save to file
        output_path = input_path.replace(".png", "_mask.png")
        cv2.imwrite(output_path, mask)
        print("Saved mask to:", output_path)
