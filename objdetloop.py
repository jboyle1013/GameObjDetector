"""
Old Test Loop Version of objdetclass.py
"""


import math
import traceback

import cv2
import numpy as np
from ultralytics import YOLO  # Import YOLO object detection model

from NoMaskFoundException import NoMaskFound  # Custom exception class
from realsense_obj import DepthCamera  # Import DepthCamera class for depth sensing

# Constants for camera setup and object detection
CAMERA_HEIGHT = 63.5  # Camera height from the ground in mm
# Names of Classes for labeling images
CLASS_NAMES = ['BigBox', 'Nozzle', 'Rocket', 'SmallBox']
# Colors for bounding boxes to be drawn on images
CLASS_COLORS = {
    'BigBox': (235, 82, 52),
    'Nozzle': (235, 217, 52),
    'Rocket': (52, 235, 73),
    'SmallBox': (230, 46, 208)
}
CONFIDENCE_THRESHOLD = 0.6  # Minimum confidence threshold for detection
MM_TO_INCHES = 25.4  # Conversion factor from millimeters to inches
model = YOLO("train6/weights/best.pt")  # Load trained YOLO model

dc = DepthCamera()  # Initialize the depth camera


def get_vals(depth_image, color_image, depth_frame):
    """
    Processes the depth and color images to detect objects and calculate their positions.
    """
    robot_Vals = []
    results = model(color_image, stream=True)  # Object detection

    # coordinates
    for r in results:
        boxes = r.boxes  # Detected bounding boxes
        masks = r.masks  # Detected masks
        try:
            for mask, box in zip(masks, boxes):
                class_name = CLASS_NAMES[int(box.cls[0])]
                if box.conf[0] > CONFIDENCE_THRESHOLD:
                    try:
                        robot_Vals = process_mask(mask, class_name, color_image, depth_image)
                    except Exception as e:
                        print("An error occurred:", e)
                        print("Traceback:", traceback.format_exc())
                        robot_Vals = process_box(box, class_name, color_image, depth_image)
                    finally:
                        draw_and_print_info(class_name, box.conf[0], robot_Vals, box, color_image)
        except Exception as e:
            print("An error occurred:", e)
            print("Traceback:", traceback.format_exc())
            for box in boxes:
                class_name = CLASS_NAMES[int(box.cls[0])]
                robot_Vals = process_box(box, class_name, color_image, depth_image)
                draw_and_print_info(class_name, box.conf[0], robot_Vals, box, color_image)


# Calculates average depth information within a bounding box in the depth image
# Calculates average depth information within a bounding box in the depth image
def calculate_depth_info_box(depth_image, bbox):
    # Extract coordinates of the bounding box
    x1, y1, x2, y2 = map(int, bbox.xyxy[0])

    # Sort coordinates for iteration
    bx1, bx2 = sorted([x1, x2])
    by1, by2 = sorted([y1, y2])

    depth_list = []
    # Iterate through each pixel within the bounding box to accumulate depth values
    for i in range(by1, by2):  # rows
        for j in range(bx1, bx2):  # columns
            depth_value = depth_image[i, j]
            depth_list.append(depth_value)

    # Calculate average depth
    depth = sum(depth_list) / len(depth_list)

    return depth, depth / MM_TO_INCHES  # Return depth in both mm and inches


# Calculates average depth information for a given mask area
def calculate_depth_info_mask(depth_image, mask_area):
    # Extract depth values where the mask is present
    depth_values = depth_image[mask_area]
    # Calculate average depth, excluding zero values
    depth = np.mean(depth_values[depth_values > 0])
    return depth, depth / MM_TO_INCHES


# Deprojects pixel coordinates to 3D space and calculates additional info
def deproject_and_calculate(centerx, centery, depth):
    # Deprojects pixel to point in 3D space
    deproj = dc.deproject([centery, centerx], depth)
    # Calculate height with respect to camera height
    height = CAMERA_HEIGHT - deproj[1]
    # Calculate the direct distance from the camera to the point
    calcdistfromdproj = math.sqrt(deproj[0] ** 2 + height ** 2)
    # Determine the horizontal angle to the point
    horizontal_angle = math.degrees(math.atan2(deproj[0], deproj[2]))
    # Determine the direction (left or right) based on the deprojected X coordinate
    direction = "left" if deproj[0] < 0 else "right"
    return deproj, height, calcdistfromdproj, horizontal_angle, direction


# Draws information on the color image and prints details to the console
def draw_and_print_info(className, confidence, robot_Vals, box, color_image):
    # Unpack values calculated from depth information
    depth_in, depth, deproj, calcdistfromdproj, height, horizontal_angle, direction = [val for val in robot_Vals]

    # Coordinates for the bounding box
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    print(f"<----------------------------------------------------->")
    # Print confidence and class name
    print(f"Confidence ---> {confidence * 100:.1f}%")
    print("Class name -->", className)

    # Print depth information
    print(f"Av Distance in ---> {depth_in:.3f} in", )
    print(f"Av Distance ---> {depth:.3f} mm", )

    # Print deprojected coordinates and additional calculated details
    print(f"RS Deproj  3D Coordinates: (X, Y, Z) = ({deproj[0]}, {deproj[1]}, {deproj[2]})")
    print(f"Actual Height? Calculated from Deproj = {calcdistfromdproj}")
    print(f"Calculated Depth from Deproj = {height}")
    print(f"Direction from Deproj = {direction}")
    print(f"Calculated Angle from Deproj = {horizontal_angle} Degrees to the {direction}")
    print(f"<----------------------------------------------------->\n\n")
    # Draw text on the color image for visual display
    org = [x1, y1]
    bottom = [x1, y2 + 3]
    bbottom = [x1, y2 + 35]
    cv2.putText(color_image, f"{className}: {confidence * 100:.0f}%", org, cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 2)
    cv2.putText(color_image, f"Distance: {depth_in:.3f} in", bottom, cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 2)
    cv2.putText(color_image, f"Distance: {depth:.3f} mm", bbottom, cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 2)


# Processes a mask to calculate depth and positional information
def process_mask(mask, class_name, color_image, depth_image):
    classColor = CLASS_COLORS[class_name]
    # Create a binary mask based on detected segments
    segments = mask.xy
    mask_image = np.zeros(depth_image.shape, dtype=np.uint8)
    for segment in segments:
        cv2.fillPoly(mask_image, [np.array(segment, dtype=np.int32)], 1)

    # Check if the mask area is valid
    mask_area = mask_image > 0
    y_coords, x_coords = np.where(mask_area)
    if not x_coords.size or not y_coords.size:
        raise NoMaskFound(class_name)  # Raise an exception if no valid mask area is found

    # Calculate centroid of the mask and depth information
    centerx, centery = np.mean(x_coords), np.mean(y_coords)
    depth, depth_in = calculate_depth_info_mask(depth_image, mask_area)
    # Calculate additional information based on deprojection
    deproj, height, calcdistfromdproj, horizontal_angle, direction = deproject_and_calculate(centerx, centery, depth)

    # Draw contours based on the mask and return calculated values
    contours, _ = cv2.findContours(mask_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(color_image, contours, -1, classColor, 3)
    return [depth_in, depth, deproj, calcdistfromdproj, height, horizontal_angle, direction]


# Processes a bounding box to calculate depth and positional information
def process_box(box, class_name, color_image, depth_image):
    classColor = CLASS_COLORS[class_name]
    # Extract bounding box coordinates
    x1, y1, x2, y2 = map(int, box.xyxy[0])

    # Calculate the center of the bounding box
    centerx = int((x2 + x1) / 2)
    centery = int((y2 + y1) / 2)

    # Calculate depth information for the bounding box
    depth, depth_in = calculate_depth_info_box(depth_image, box)

    # Calculate additional information based on deprojection
    deproj, height, calcdistfromdproj, horizontal_angle, direction = deproject_and_calculate(centerx, centery, depth)

    # Draw the bounding box and return calculated values
    cv2.rectangle(color_image, (x1, y1), (x2, y2), classColor, 6)
    return [depth_in, depth, deproj, calcdistfromdproj, height, horizontal_angle, direction]


if __name__ == "__main__":
    print("[INFO] Configuring Camera...")

    dc.set_Settings_from_json('camerasettings/settings1.json')  # Load camera settings
    print("[INFO] Starting video stream...")
    dc.start_Streaming()  # Start streaming from the camera

    # Main loop for processing frames
    while True:
        ret, depth_frame, color_frame, depth_colormap, depth_image = dc.get_frame()
        get_vals(depth_frame, color_frame, depth_image)  # Process the current frame

        cv2.namedWindow('Depth Color Frame', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Depth Frame', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Color Frame', cv2.WINDOW_NORMAL)
        cv2.imshow("Depth Color Frame", depth_colormap)
        cv2.imshow("Depth Frame", depth_frame)
        cv2.imshow("Color Frame", color_frame)
        key = cv2.waitKey(1)
        if key == 27:
            break
