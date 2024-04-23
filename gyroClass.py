class Gyro:
    def __init__(self, gyro_x=None, gyro_y=None, gyro_z=None):
        self.X = gyro_x
        self.Y = gyro_y
        self.Z = gyro_z


    def serialize(self):
        return f"{self.X},{self.Y},{self.Z}"
