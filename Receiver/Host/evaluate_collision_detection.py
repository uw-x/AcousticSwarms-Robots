import sys

import numpy as np
import time
import threading
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("tkAgg")
import traceback

from uuids import *
from control_values import *
from utils import *
from globals import *
from ranging import *
from helpers import *
from usb_packet_types import *
from Controller import RobotController
from StreamCollector import StreamCollector, StreamSelector
from StreamPlotter import StreamPlotter
from Milestone import Milestone, RobotState, Point
from Robot import Robot
from PathPlotter import PathPlotter
from loc_plan.High_Control import High_Status
from loc_plan.LOC_2D import extract_distance_single

import queue
import scipy.signal as signal


# OBJECTS PLACED 15cm AWAY FROM ROBOT

ExpConfig = namedtuple("ExpConfig", ["speed"])

def main():
    T = 40000
    cfgs = [ExpConfig(120),
            ExpConfig(180),
            ExpConfig(200),
            ExpConfig(250),
            ExpConfig(300)]
    
    controller = RobotController()
    
    if controller.available:
        # controller.scan(8)
        controller.scan(1)

        if len(controller.robots) > 0:
            print("All robots are ready!")

            for robot_id, robot in controller.robots.items():
                controller.monitor_battery(robot_id, enable=True)

            for cfg in cfgs:
                input("Press enter to begin next experiment...")

                controller.get_robot_by_id(0).status_stream.flush()

                controller.calibrate_sensors(0)
                controller.get_robot_by_id(0).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)
                
                print("Current speed:", cfg.speed)

                controller.enable_collision_detection(0, moving=True) # ACCEL READINGS ARE AT 2000kHz

                controller.monitor_accel(0)

                controller.update_motion(0, MOTION_CODES.MOTION_FORWARD, duration_us=1e9, speed=cfg.speed)
                controller.get_robot_by_id(0).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END, timeout=10)
                controller.disable_collision_detection(0)

                time.sleep(0.4)
                
                controller.calibrate_sensors(0)
                controller.get_robot_by_id(0).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)
                
                print("Found edge, backing off...")
                controller.update_motion(0, MOTION_CODES.MOTION_BACKWARD, duration_us=500e3, speed=150)
                controller.get_robot_by_id(0).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

                controller.monitor_accel(0, enable=False)
                accel_readings = controller.get_robot_by_id(0).accel_stream.get_remaining()
                accel_readings = np.array(accel_readings)

                print("ACCEL", accel_readings.shape)
                
                with open(f"results/collision_data/accel_readings_{cfg.speed}.npy", 'wb') as f:
                    np.save(f, accel_readings)

        else:
            print("No robots found")
    
    controller.close()
    time.sleep(2)
    for thread in threading.enumerate():
        print(thread.name)

if __name__ == "__main__":
    main()
    sys.exit()
