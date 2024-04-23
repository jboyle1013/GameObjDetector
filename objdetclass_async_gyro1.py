"""
This is the File Being Used for the Detections
"""
import asyncio
import csv
import json
import math
import queue
import threading
import time
import traceback

import cv2
import numpy as np
import serial
from pynput import keyboard
from ultralytics import YOLO

from NoMaskFoundException import NoMaskFound
from detectionClass import Detection
from realsense_obj_gyro import DepthCamera

with open('config.json', 'r') as config_file:
    config = json.load(config_file)

CONFIDENCE_THRESHOLD = config["confidence_threshold"]
SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]
CAMERA_HEIGHT = config["camera_height"]
MM_TO_INCHES = config["mm_to_inches"]
MODEL_PATH = config["model_path"]
CAMERA_SETTINGS_PATH = config["camera_settings_path"]
IGNORE_JSON_PATH = config["ignore_json_path"]
CLASS_NAMES = config["class_names"]
CLASS_COLORS = {k: tuple(v) for k, v in config["class_colors"].items()}


class ObjectDetector:

    def __init__(self, model_path, camera_settings_path, ignore_json_path):
        self.gyro_datarad = None
        self.gyro_datadeg = None
        self.accel_data = None
        self.model = YOLO(model_path)
        self.dc = DepthCamera()
        self.dc.set_Settings_from_json(camera_settings_path)
        self.lock = threading.Lock()  # Lock for thread safety
        self.detections = []  # Store detections
        self.running = True
        self.time_start = None
        self.ignore_lists_dict = self.get_ignore_lists(ignore_json_path)
        self.ignore_list = []
        self.frame_queue = queue.Queue(maxsize=10)
        self.send_gyro_data = True

    def get_ignore_lists(self, ignore_json_path):
        with open(ignore_json_path, 'r') as ignore_lists:
            return json.load(ignore_lists)

    def start_keyboard_listener(self):
        """
        Starts a keyboard listener in a separate thread.
        """
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

    def start_serial_listener(self, port, baud_rate):
        """
        Starts a thread to listen for serial commands.

        :param port: The serial port to listen to (e.g., 'COM3' or '/dev/ttyUSB0').
        :param baud_rate: The baud rate for the serial communication.
        """

        def serial_listener():
            with serial.Serial(port, baud_rate) as ser:
                while self.running:
                    if ser.in_waiting:
                        line = ser.readline().decode('utf-8').strip()
                        if line == "WRITE_CSV":
                            print("Arduino Command: Writing to CSV")
                            self.write_detections_to_csv(
                                self.detections, "output.csv")
                        elif line == "QUIT":
                            print("Arduino Command: Quitting")
                            self.write_detections_to_csv(
                                self.detections, "output.csv")
                            self.dc.release()
                            break
                        elif line == "REQUEST":
                            print("Arduino Command: Sending last detections")
                            # Get the last 10 detections or however many are available
                            serialized_data = self.serialize_detections(
                                min(4, len(self.detections)))  # Mega Version
                            # serialized_data = self.serialize_detections(
                            #    min(3, len(self.detections))) # Uno Version
                            ser.write(serialized_data)
                            print("Sent detections to Arduino")
                        elif line == "GYRO":
                            if self.send_gyro_data:
                                # Fetch gyro data
                                if self.gyro_datadeg:
                                    # Format and send gyro data
                                    gyro_data_str = f"{self.gyro_datadeg['x']:.2f},{self.gyro_datadeg['y']:.2f},{self.gyro_datadeg['x']:.2f}\n"
                            ser.write(gyro_data_str.encode('utf-8'))
                        else:
                            print(line)
                            if line in self.ignore_lists_dict.keys():
                                with self.lock:
                                    self.ignore_list = self.ignore_lists_dict[line]

        listener_thread = threading.Thread(target=serial_listener)
        listener_thread.start()

    def process_detection(self, class_name, confidence, robot_Vals):

        depth_in, depth, deproj, height, horizontal_angle, direction = [
            val for val in robot_Vals]
        x, y, z = [val for val in deproj]
        checkvals = [confidence, depth, depth_in, x, y, z, horizontal_angle]

        if any(math.isnan(val) for val in checkvals):
            return

        time_now = time.process_time()
        timestamp = time_now - self.time_start
        # Create a Detection instance
        detection = Detection(class_name, confidence, depth,
                              x, y, z, horizontal_angle, direction, timestamp)

        # Add the detection to the thread-safe list
        self.add_detection(detection)

    def add_detection(self, detection_data):
        with self.lock:  # Acquire lock before modifying shared resource
            self.detections.append(detection_data)

    def get_detections(self):
        with self.lock:  # Acquire lock before accessing shared resource
            detections_copy = self.detections.copy()
            # sorted_detections = sorted(detections_copy, key=lambda d: (d.depth_mm))

        return detections_copy

    def write_detections_to_csv(self, detections, filename):
        """
        Writes all detections to a CSV file.

        :param detections: List of Detection objects.
        :param filename: Name of the file to write to.
        """
        with self.lock:  # Acquire lock before accessing shared resource
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)

                # Write the header
                writer.writerow(
                    ['Class Name', 'Confidence', 'Depth (mm)', 'X', 'Y', 'Z',
                     'Horizontal Angle', 'Direction'])

                # Write the detection data
                for detection in detections:
                    writer.writerow([detection.class_name, f"{detection.confidence:.2f}", f"{detection.depth_mm:.2f}",
                                     f"{detection.x:.2f}", f"{detection.y:.2f}",
                                     f"{detection.z:.2f}", f"{detection.horizontal_angle:.2f}", detection.direction])

    def serialize_detections(self, n):

        detections = self.get_detections()

        # Get the last n detection objects
        last_n_detections = detections[-n:]
        gyro_data_str = f",{self.gyro_datadeg['x']:.2f},{self.gyro_datadeg['y']:.2f},{self.gyro_datadeg['x']:.2f}"

        combined_data = gyro_data_str.join(
            [detection.serialize() for detection in last_n_detections])
        # Serialize each detection object and join with a delimiter
        delimiter = ';'  # Ensure this delimiter does not appear in the data
        serialized_data = delimiter.join(
            [detection for detection in combined_data])

        serialized_data = serialized_data + '\n'
        # Convert the entire string to a byte array
        return serialized_data.encode('utf-8')

    def get_vals(self, depth_image, color_image):
        """
        Processes the depth and color images to detect objects and calculate their positions.
        """
        robot_Vals = []
        results = self.model(color_image, stream=True)  # Object detection

        # coordinates
        for r in results:
            boxes = r.boxes  # Detected bounding boxes
            masks = r.masks  # Detected masks
            try:
                for mask, box in zip(masks, boxes):
                    class_name = CLASS_NAMES[int(box.cls[0])]
                    if class_name not in self.ignore_list:
                        if box.conf[0] > CONFIDENCE_THRESHOLD:
                            try:
                                robot_Vals = self.process_mask(
                                    mask, class_name, color_image, depth_image)
                            except Exception as e:
                                print("An error occurred:", e)
                                print("Traceback:", traceback.format_exc())
                                robot_Vals = self.process_box(
                                    box, class_name, color_image, depth_image)
                            finally:
                                self.process_detection(
                                    class_name, box.conf[0], robot_Vals)
                                self.draw_and_print_info(
                                    class_name, box.conf[0], robot_Vals, box, color_image)
            except Exception as e:
                print("An error occurred:", e)
                print("Traceback:", traceback.format_exc())
                for box in boxes:
                    if box.conf[0] > CONFIDENCE_THRESHOLD:
                        class_name = CLASS_NAMES[int(box.cls[0])]
                        robot_Vals = self.process_box(
                            box, class_name, color_image, depth_image)
                        self.process_detection(
                            class_name, box.conf[0], robot_Vals)
                        self.draw_and_print_info(
                            class_name, box.conf[0], robot_Vals, box, color_image)

    # Calculates average depth information within a bounding box in the depth image
    def calculate_depth_info_box(self, depth_image, bbox):
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
    def calculate_depth_info_mask(self, depth_image, mask_area):
        # Extract depth values where the mask is present
        depth_values = depth_image[mask_area]
        # Calculate average depth, excluding zero values
        depth = np.mean(depth_values[depth_values > 0])
        return depth, depth / MM_TO_INCHES

    # Deprojects pixel coordinates to 3D space and calculates additional info
    def deproject_and_calculate(self, centerx, centery, depth):
        # Deprojects pixel to point in 3D space
        deproj = self.dc.deproject([centery, centerx], depth)
        # Calculate height with respect to camera height
        height = abs(CAMERA_HEIGHT - deproj[1])
        # Calculate the direct distance from the camera to the point
        # Determine the horizontal angle to the point
        horizontal_angle = math.degrees(math.atan2(deproj[0], deproj[2]))
        # Determine the direction (left or right) based on the deprojected X coordinate
        direction = "left" if deproj[0] < 0 else "right"
        return deproj, height, horizontal_angle, direction

    # Draws information on the color image and prints details to the console
    def draw_and_print_info(self, className, confidence, robot_Vals, box, color_image):
        # Unpack values calculated from depth information
        depth_in, depth, deproj, height, horizontal_angle, direction = [
            val for val in robot_Vals]

        # Coordinates for the bounding box
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        # print(f"<----------------------------------------------------->")
        # Print class name and confidence
        # print("Class name -->", className)
        # print(f"Confidence ---> {confidence * 100:.1f}%")

        # Print depth information
        # print(f"Distance in ---> {depth_in:.3f} in", )
        # print(f"Distance ---> {depth:.3f} mm", )

        # Print deprojected coordinates and additional calculated details
        # print(
        #    f"RS Deproj  3D Coordinates: (X, Y, Z) = ({deproj[0]}, {deproj[1]}, {deproj[2]})")
        # print(f"Actual Height? Calculated from Deproj = {height}")

        # print(f"Direction from Deproj = {direction}")
        # print(
        #    f"Calculated Angle from Deproj = {horizontal_angle} Degrees to the {direction}")
        # print(f"<----------------------------------------------------->\n\n")

        # Draw text on the color image for visual display
        org = [x1, y1]
        bottom = [x1, y2 + 3]
        bbottom = [x1, y2 + 35]
        cv2.putText(color_image, f"{className}: {confidence * 100:.0f}%", org, cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0),
                    2)
        cv2.putText(color_image, f"Distance: {depth_in:.3f} in",
                    bottom, cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 2)
        cv2.putText(color_image, f"Distance: {depth:.3f} mm",
                    bbottom, cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 2)

    # Processes a mask to calculate depth and positional information
    def process_mask(self, mask, class_name, color_image, depth_image):
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
            # Raise an exception if no valid mask area is found
            raise NoMaskFound(class_name)

        # Calculate centroid of the mask and depth information
        centerx, centery = np.mean(x_coords), np.mean(y_coords)
        depth, depth_in = self.calculate_depth_info_mask(
            depth_image, mask_area)
        # Calculate additional information based on deprojection
        deproj, height, horizontal_angle, direction = self.deproject_and_calculate(
            centerx, centery, depth)

        # Draw contours based on the mask and return calculated values
        contours, _ = cv2.findContours(
            mask_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(color_image, contours, -1, classColor, 3)
        return [depth_in, depth, deproj, height, horizontal_angle, direction]

    # Processes a bounding box to calculate depth and positional information
    def process_box(self, box, class_name, color_image, depth_image):
        classColor = CLASS_COLORS[class_name]
        # Extract bounding box coordinates
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        # Calculate the center of the bounding box
        centerx = int((x2 + x1) / 2)
        centery = int((y2 + y1) / 2)

        # Calculate depth information for the bounding box
        depth, depth_in = self.calculate_depth_info_box(depth_image, box)

        # Calculate additional information based on deprojection
        deproj, height, horizontal_angle, direction = self.deproject_and_calculate(
            centerx, centery, depth)

        # Draw the bounding box and return calculated values
        cv2.rectangle(color_image, (x1, y1), (x2, y2), classColor, 6)
        return [depth_in, depth, deproj, height, horizontal_angle, direction]

    def start_detection(self):

        # Start the keyboard listener thread
        self.start_keyboard_listener()

        print("[INFO] Starting video stream...")
        self.dc.start_Streaming()
        self.dc.start_IMU()
        # Start the serial listener thread
        self.start_serial_listener(SERIAL_PORT, BAUD_RATE)
        self.time_start = time.process_time()

        # Starting the asyncio event loop
        loop = asyncio.get_event_loop()
        try:
            # Start capturing IMU data
            asyncio.ensure_future(self.capture_imu_data())
            # Start capturing frames
            asyncio.ensure_future(self.capture_frames())
            # Start processing frames
            asyncio.ensure_future(self.process_frames())
            loop.run_forever()
        except KeyboardInterrupt:
            pass
        finally:
            loop.close()

        key = cv2.waitKey(1)
        if key == 13 or (key == 119 or key == 87):
            print("Writing to CSV")
            self.write_detections_to_csv(self.detections, "output.csv")
        if key == 27:
            self.shutdown()

    async def capture_frames(self):
        while self.running:
            ret, depth_frame, color_frame = self.dc.get_frame()
            if not ret:
                continue
            if not self.frame_queue.full():
                # Storing both frames as a tuple
                self.frame_queue.put((color_frame, depth_frame))
            await asyncio.sleep(0)  # Yield control to allow other tasks to run

    async def process_frames(self):
        while self.running:
            if not self.frame_queue.empty():
                color_frame, depth_frame = self.frame_queue.get()
                # Process both color and depth frames
                self.get_vals(depth_frame, color_frame)

                # Display the frame
                cv2.namedWindow('Color Frame', cv2.WINDOW_NORMAL)
                cv2.imshow("Color Frame", color_frame)
                print(
                    f"Gyro Data Degrees:\nX: {self.gyro_datadeg['x']}\nY: {self.gyro_datadeg['y']}\nZ: {self.gyro_datadeg['z']}")
                # Check for quit key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.shutdown()

            await asyncio.sleep(0)  # Yield control to allow other tasks to run

    async def capture_imu_data(self):
        while self.running:
            self.gyro_datadeg = self.dc.get_imu_data()
            await asyncio.sleep(0)  # Adjust the sleep time as needed

    def shutdown(self):
        self.running = False
        self.write_detections_to_csv(self.detections, "output.csv")
        self.dc.release()  # Stop Camera
        self.stop_all_listeners()
        print("Shutting down...")

    def stop_all_listeners(self):
        self.running = False  # This will stop the serial listener loop
        if hasattr(self, 'listener'):
            self.listener.stop()  # Stop the keyboard listener

    def on_press(self, key):
        try:
            if key == keyboard.Key.enter or key.char in ['w', 'W']:
                print("Writing to CSV")
                self.write_detections_to_csv(self.detections, "output.csv")
            if key == keyboard.Key.esc or key.char in ['q', 'Q']:
                self.shutdown()
        except AttributeError:
            pass


# Usage of the class in the main program
if __name__ == "__main__":
    detector = ObjectDetector(
        MODEL_PATH, CAMERA_SETTINGS_PATH, IGNORE_JSON_PATH)
    detector.start_detection()
