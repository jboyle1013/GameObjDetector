import cv2
import numpy as np
from sklearn.cluster import DBSCAN

def cluster_lines(lines, eps=10, min_samples=2):
    # Flatten line coordinates for clustering
    flattened_lines = lines.reshape(lines.shape[0], -1)

    # Apply DBSCAN
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(flattened_lines)
    labels = clustering.labels_

    # Group lines by their cluster label
    clustered_lines = {}
    for label, line in zip(labels, lines):
        if label == -1:
            continue  # Ignore noise points
        if label in clustered_lines:
            clustered_lines[label].append(line)
        else:
            clustered_lines[label] = [line]

    # Merge lines in each cluster
    merged_lines = []
    for label, lines in clustered_lines.items():
        # Merge lines by averaging their endpoints
        x1 = int(np.mean([line[0][0] for line in lines]))
        y1 = int(np.mean([line[0][1] for line in lines]))
        x2 = int(np.mean([line[0][2] for line in lines]))
        y2 = int(np.mean([line[0][3] for line in lines]))
        merged_lines.append([[x1, y1, x2, y2]])

    return np.array(merged_lines)

def estimate_line_width(image, line, width_threshold):
    x1, y1, x2, y2 = line[0]
    line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # Create a mask for the line
    mask = np.zeros_like(image)
    cv2.line(mask, (x1, y1), (x2, y2), 255, 1)

    # Find the contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    # Approximate the width
    rect = cv2.minAreaRect(contours[0])
    width = min(rect[1])

    # Check if the width meets the threshold
    return width >= width_threshold



def angle_between_lines(line1, line2):
    # Calculate the angle between two lines
    x1, y1, x2, y2 = line1[0]
    x3, y3, x4, y4 = line2[0]
    v1 = (x2 - x1, y2 - y1)
    v2 = (x4 - x3, y4 - y3)
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    det = v1[0]*v2[1] - v1[1]*v2[0]
    angle = np.arctan2(det, dot)
    return np.abs(angle)

def find_intersection(line1, line2):
    # Extract points
    x1, y1, x2, y2 = line1[0]
    x3, y3, x4, y4 = line2[0]

    # Line equations
    px = (x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)
    py = (x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)
    denominator = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)

    # Check if lines are parallel
    if denominator == 0:
        return None

    # Intersection point
    x = px / denominator
    y = py / denominator
    return (x, y)

def main():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 200, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=150, maxLineGap=50)

        if lines is not None:
            # Cluster and merge lines
            merged_lines = cluster_lines(lines)

        for line in merged_lines:
            if estimate_line_width(gray, line, width_threshold=0.9):  # Set your width threshold here
                x1, y1, x2, y2 = line[0]
                cv2.arrowedLine(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow('Line Tracking', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # Release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

# def follow_line(contours, image):
#     if contours:
#         # Assuming the largest contour is the line
#         largest_contour = max(contours, key=cv2.contourArea)
#         M = cv2.moments(largest_contour)
#         if M["m00"] != 0:
#             cx = int(M["m10"] / M["m00"])
#             cy = int(M["m01"] / M["m00"])
#             # Draw centroid
#             cv2.circle(image, (cx, cy), 5, (255, 0, 0), -1)
#             cv2.arrowedLine(image, (int(M["m00"]), int(M["m00"])), (int(M["m10"]), int(M["m10"])), (0, 255, 0), 2)
#             # Here, you could add logic to steer your robot towards cx, cy
#             return cx, cy
#     return None
#
# def detect_blobs(mask):
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     return contours
#
#
# def detect_line_color(image, lower_color, upper_color):
#     hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#     mask = cv2.inRange(hsv_image, lower_color, upper_color)
#     return mask
#
#
# cap = cv2.VideoCapture(0)  # Replace with your camera setup
#
# lower_color = np.array([0, 0, 0])
# upper_color = np.array([30, 255, 255])
#
# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break
#
#     mask = detect_line_color(frame, lower_color, upper_color)
#     contours = detect_blobs(mask)
#     line_center = follow_line(contours, frame)
#
#     # Logic for steering based on line_center
#
#     cv2.imshow("Frame", frame)
#     key = cv2.waitKey(1)
#     if key == 27:  # ESC key
#         break
#
# cap.release()
# cv2.destroyAllWindows()