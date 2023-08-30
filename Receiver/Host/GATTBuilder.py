from usb_packet_types import *
from helpers import *


class GATTBuilder():
    def __init__(self, robot_id):
        self.data = [BLE_COMMANDS.BLE_GATT_REQUEST, MSB_16(robot_id), LSB_16(robot_id)]

    def write(self, characteristic):
        self.data.extend([BLE_GATT_COMMANDS.BLE_WRITE_CHARACTERISTIC, MSB_16(characteristic), LSB_16(characteristic)])
        return self

    def read(self, characteristic):
        self.data.extend([BLE_GATT_COMMANDS.BLE_READ_CHARACTERISTIC, MSB_16(characteristic), LSB_16(characteristic)])
        return self
    
    def start_notify(self, characteristic):
        self.data.extend([BLE_GATT_COMMANDS.BLE_START_NOTIFICATIONS, MSB_16(characteristic), LSB_16(characteristic)])
        return self
    
    def stop_notify(self, characteristic):
        self.data.extend([BLE_GATT_COMMANDS.BLE_STOP_NOTIFICATIONS, MSB_16(characteristic), LSB_16(characteristic)])
        return self

    def add_params(self, *params):
        self.data.extend(params)
        return self

    def get(self):
        return self.data