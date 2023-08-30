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


def main():
    controller = RobotController()
    
    fname = 'results/power_consumption/distribution_battery_drain.json'
    
    if os.path.exists(fname):
        with open(fname, 'rb') as f:
            results = json.load(f)
    else:
        results = {'samples':[]}
    
    if controller.available:
        # controller.begin_scan()

        # controller.scan(8)
        # controller.scan(7)
        controller.scan(6)
        # controller.scan(5)
        # controller.scan(4)
        # controller.scan(3)
        # controller.scan(2)
        # controller.scan(1)

        if len(controller.robots) > 0:
            print("All robots are ready!")

            for robot_id, robot in controller.robots.items():
                controller.monitor_battery(robot_id, enable=True)            

            while True:
                input('Distribute?')
                # Distribute robots across table
                controller.robots_distribute(enable_collision_detection=False, expand_in_background=True)
                
                # # Save power consumption results
                # results['samples'].append(power_consumption)
                
                # with open(fname, 'w') as f:
                #     json.dump(results, f)
                
                print("Distributed")
                
                input('Recall?')
                controller.robots_recall()
                print("Recalled")

        else:
            print("No robots found")
    
    controller.close()
    time.sleep(2)
    for thread in threading.enumerate():
        print(thread.name)

if __name__ == "__main__":
    main()
    sys.exit()
