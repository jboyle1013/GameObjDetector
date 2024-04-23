from collections import defaultdict

import cv2
import numpy as np

from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO('yolov8n.pt')

# Open the video file
video_path = "https://youtu.be/LNwODJXcvt4"
cap = cv2.VideoCapture(video_path)

# Store the track history
track_history = defaultdict(lambda: [])


# Run YOLOv8 tracking on the frame, persisting tracks between frames
results = model.track(source="https://youtu.be/LNwODJXcvt4", show=True, tracker="bytetrack.yaml")

# Get the boxes and track IDs
boxes = results[0].boxes.xywh.cpu()
track_ids = results[0].boxes.id.int().cpu().tolist()

# Visualize the results on the frame
annotated_frame = results[0].plot()

# Plot the tracks
for box, track_id in zip(boxes, track_ids):
    x, y, w, h = box
    track = track_history[track_id]
    track.append((float(x), float(y)))  # x, y center point
    if len(track) > 30:  # retain 90 tracks for 90 frames
        track.pop(0)

    # Draw the tracking lines
    points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
    cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)

# Display the annotated frame
cv2.imshow("YOLOv8 Tracking", annotated_frame)

# Break the loop if 'q' is pressed



# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()