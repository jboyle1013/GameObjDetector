class Detection:
    def __init__(self, class_name=None, confidence=None, depth_mm=None, depth_in=None, x=None, y=None, z=None, horizontal_angle=None, direction=None, timestamp=None):
        self.class_name = class_name
        self.confidence = confidence
        self.depth_mm = depth_mm
        self.depth_in = depth_in
        self.x = x
        self.y = y
        self.z = z
        self.horizontal_angle = horizontal_angle
        self.direction = direction
        self.timestamp = timestamp

    def serialize(self):
        # Convert the detection data to a string or byte format for Arduino communication

        return f"{self.class_name},{self.confidence:.2f},{self.depth_mm:.2f},{self.depth_in:.2f},{self.x:.2f},{self.y:.2f},{self.z:.2f},{self.direction}, {self.timestamp:.2f}"
