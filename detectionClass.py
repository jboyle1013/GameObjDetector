class Detection:
    def __init__(self, class_name=None, confidence=None, depth_mm=None, depth_in=None, x=None, y=None, z=None, horizontal_angle=None, direction=None, timestamp=None, track_id=None,
                 gyro_x=None, gyro_y=None, gyro_z=None):
        self.class_name = class_name
        self.confidence = confidence
        self.depth_mm = depth_mm
        self.depth_in = depth_in
        self.x = x
        self.y = y
        self.z = z
        self.horizontal_angle = horizontal_angle
        self.direction = direction
        self.track_id = track_id
        self.timestamp = timestamp
        self.gyro_x = gyro_x
        self.gyro_y = gyro_y
        self.gyro_z = gyro_z
        


    def serialzize(self):
        # Convert the detection data to a string or byte format for Arduino communication

        return f"{self.class_name},{self.confidence:.2f},{self.track_id},{self.depth_mm:.2f},{self.depth_in:.2f},{self.x:.2f},{self.y:.2f},{self.z:.2f},{self.direction}"
