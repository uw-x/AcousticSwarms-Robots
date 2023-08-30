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

import scipy.signal as signal


def main():
    fname = 'results/power_consumption/streaming_battery_drain.json'
    
    if os.path.exists(fname):
        with open(fname, 'rb') as f:
            results = json.load(f)
    else:
        results = {'time':[], 'vcell':[], 'SOC':[], 'CRATE':[]}

    controller = RobotController()
    
    if controller.available:

        controller.scan(1)

        if len(controller.robots) > 0:
            print("All robots are ready!")

            for robot_id, robot in controller.robots.items():
                controller.monitor_battery(robot_id, enable=True)

            start_time = time.time()
            robot = controller.get_robot_by_id(0)

            for _id in controller.robots:
                controller.update_mic_parameters(_id, sampling_rate=FREQS.SAMPLING_FREQUENCY_50000, use_compression=True, left_gain=0x50, right_gain=0x50, channels=1, compression_bitrate=32000)
            
            controller.begin_stream(48000)
            
            while True:
                controller.stream_collector.flush()
                try:
                    battery_data = robot.battery_stream.get(timeout=1)
                    current_time = time.time() - start_time
                    
                    results['time'].append(str(current_time))
                    results['vcell'].append(str(battery_data[0]))
                    results['SOC'].append(str(battery_data[1]))
                    results['CRATE'].append(str(battery_data[2]))

                    with open(fname, 'w') as f:
                        json.dump(results, f)

                except queue.Empty:
                    if controller.get_robot_by_id(0) is None:
                        print("Disconnect found, exiting program...")
                        break

            controller.stop_stream()

        else:
            print("No robots found")
    
    controller.close()
    time.sleep(2)
    for thread in threading.enumerate():
        print(thread.name)

if __name__ == "__main__":
    main()
    sys.exit()
