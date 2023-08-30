import struct
import numpy as np
from math import fmod

X_MAX = 200
Y_MAX = 200

class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.point = [x, y]
    
    def to_byte_list(self):
        return [x for y in self.point for x in bytearray(struct.pack('f', y))]

class RobotState(object):
    def __init__(self, position: Point, rotation: float, velocity: Point):
        self.position = position
        self.velocity = velocity
        self.rotation = rotation

    def tostring(self):
        return "({}, {}) @ r = {} deg, v = ({}, {}) ".format(self.position.x, self.position.y, fmod(360 + fmod(self.rotation + 360, 360), 360), self.velocity.x, self.velocity.y)

    def to_byte_list(self):
        values = [self.position.x, self.position.y, self.rotation, self.velocity.x, self.velocity.y]
        # print(values)
        return [x for y in values for x in bytearray(struct.pack('f', y))]
    
    def to_numpy(self):
        return np.array([self.position.x, self.position.y, self.rotation], dtype=np.float64)

    @staticmethod
    def from_list(data):
        floats = np.frombuffer(np.array(data, dtype=np.uint8).tobytes(), np.float32)
        return RobotState(position=Point(floats[0], floats[1]), rotation=floats[2], velocity=Point(floats[3], floats[4]))

class Milestone(Point): pass