from Stream import Stream
import numpy as np
from globals import *


class SensorStream(Stream):
    def __init__(self, _id):
        super().__init__(_id)
        self.id = _id
        self.current_value = None

class AccelStream(SensorStream):
    def __init__(self, _id):
        super().__init__(_id)

    def process(self, data):
        """
        ax | ay | az
        """
        data = np.array(data, dtype=np.uint8).tobytes()
        data = np.frombuffer(data, dtype=np.float32).copy()/1000
        # data[0] = np.linalg.norm(data[:2]) # set first channel to be the magnitude of ax & ay
        # data = {'x':data[0], 'y':data[1], 'z':data[2]}
        # roll = np.arctan2(data[1], data[2])
        # pitch = np.arctan2(-data[0], np.sqrt(data[1] ** 2 + data[2] ** 2) )
        # data[2] = roll * 180 / np.pi
        
        # data = 180 / np.pi * np.array([roll, pitch])

        self.current_value = data

        if DEBUG_ACCEL_STREAM:
            print(data)

        return [data]

class BatteryStream(SensorStream):
    def __init__(self, _id):
        super().__init__(_id)

    def process(self, data):
        """
        vcell | soc | crate
        """
        data = np.array(data, dtype=np.uint8).tobytes()
        data = np.frombuffer(data, dtype=np.float32)
        
        # data = {'vcell':data[0], 'soc':data[1], 'crate':data[2]}
        self.current_value = data

        if DEBUG_BATTERY_STREAM:
            print(data)

        return [data]


class GyroStream(SensorStream):
    def __init__(self, _id):
        super().__init__(_id)

    def process(self, data):
        """
        gyro_x | gyro_y | gyro_z
        """
        data = np.array(data, dtype=np.uint8).tobytes()
        data = np.frombuffer(data, dtype=np.float32) / 1000 # Output in dps

        self.current_value = data

        if DEBUG_GYRO_STREAM:
            print(data)

        return [data]
