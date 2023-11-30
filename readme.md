
---

# Object Detection and Distance Measurement

This Python script integrates the Intel RealSense camera with the YOLO (You Only Look Once) object detection model to detect objects in the camera's field of view and calculate their distances and positions relative to the camera.

## Features

- **Object Detection:** Utilizes the YOLO model to detect objects in real-time.
- **Distance Calculation:** Measures the distance from the camera to detected objects using depth information.
- **Positional Information:** Calculates the 3D position of objects and their relative direction and angle from the camera's perspective.
- **Visual Feedback:** Draws bounding boxes and displays information on the video feed.

## Setup

1. **Dependencies:**
    - OpenCV (`cv2`)
    - NumPy
    - YOLO object detection model (Ultralytics implementation)
    - Intel RealSense SDK

2. **Camera Configuration:**
    - The script is configured for the Intel RealSense camera. Ensure the camera is connected and set up correctly.
    - Modify the `CAMERA_HEIGHT` constant in the script to match the height of your camera from the ground.

3. **Object Detection Model:**
    - The script uses a pre-trained YOLO model. Ensure the model file is available at `"train6/weights/best.pt"` or update the path accordingly.

## Usage

1. Run the script using a Python interpreter.
2. The script will start the video stream from the Intel RealSense camera and begin detecting objects.
3. Detected objects will be highlighted with bounding boxes, and relevant information (distance, position, etc.) will be displayed on the screen.
4. Press `ESC` to exit the script.

## Functions

- `get_vals`: Processes the depth and color images from the camera.
- `calculate_depth_info_box`: Calculates average depth within a bounding box.
- `calculate_depth_info_mask`: Calculates average depth for a given mask area.
- `deproject_and_calculate`: Converts pixel coordinates to 3D space and calculates additional positional information.
- `draw_and_print_info`: Draws bounding boxes and prints object information.
- `process_mask`: Processes detected masks for depth and position calculation.
- `process_box`: Processes detected bounding boxes for depth and position calculation.

## Customization

- Adjust `CONFIDENCE_THRESHOLD` to change the sensitivity of the object detection.

## Note

- Ensure all dependencies are installed and the camera is properly configured before running the script.
- The script is designed for educational and experimental purposes and may require adjustments for different environments or use cases.

---

