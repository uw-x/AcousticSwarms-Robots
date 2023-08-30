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
from Milestone import *
from loc_plan.High_Control import High_Status


# MIC_POSITIONS = np.array([
# [ PLATFORM_POSITIONS[6][0] , PLATFORM_POSITIONS[6][1] , 2 ],
# [ 16.873510695827314 , -21.52432917331634 , 2 ],
# [ 22.889434541941565 , -3.041531884443158 , 2 ],
# [ PLATFORM_POSITIONS[0][0] , PLATFORM_POSITIONS[0][1] , 2 ],
# [ 22.399299074246674 , 19.50517621291997 , 2 ],
# [ 13.870089690294838 , 34.81075948016341 , 2 ],
# [ -2.254645535511898 , 39.75890764703497 , 2 ],
# ])

MIC_POSITIONS = np.array([
[ PLATFORM_POSITIONS[6][0] , PLATFORM_POSITIONS[6][1] , 2 ],
[ 14.86304384301926 , -21.98355853034372 , 2 ],
[ 24.10288604695761 , -2.24437801351412 , 2 ],

[ 22.863926512409037 , 19.19360129586773 , 2 ],
[ 15.139489084741019 , 34.79988072880545 , 2 ],
[ -1.7880624510182503 , 39.694218413447786 , 2 ],
])

# During evaluation, robot sequence is fixed for reproducibility, but note that it is arbitrary
# So long as ordering is consistent with positions (counter-clockwise ending at base)
robot_sequence = [ 
    'A',
    'B',
    'C',
    'E',
    'F',
    'G',
    'D'
]

def cmd_input(message, expected_length=None):
    try:
        cmdlist = input(message).strip().split(' ')
        cmdlist = [x for x in cmdlist if x != '']

        if expected_length is not None:
            if expected_length != len(cmdlist):
                print("Not enough parameters, expected", expected_length, 'got', len(cmdlist))
                return cmd_input(message, expected_length)
        
        if cmdlist[0] == 'cancel':
            print("Cancelled")
            return None
    except KeyboardInterrupt:
        quit()
    except:
        cmdlist = cmd_input(message)
    
    return cmdlist

def main():

    fname = 'results/power_consumption/recall_battery_drain.json'
    
    if os.path.exists(fname):
        with open(fname, 'rb') as f:
            results = json.load(f)
    else:
        results = {'samples':[]}

    controller = RobotController()
    
    iterations = 5

    if controller.available:
        controller.scan(7)

        if len(controller.robots) > 0:
            print("All robots are ready!")

            for robot_id, robot in controller.robots.items():
                controller.monitor_battery(robot_id, enable=True)

            for i in range(iterations):
                controller.initialize_global_planner()

                # Initialize platform sequence
                controller.sequence = []
                for i, r_name in enumerate(robot_sequence):
                    robot = controller.get_robot_by_name(r_name)
                    controller.sequence.append(robot.id)
                    robot.initialize_high_controller(i, controller.global_planner)
                
                power_consumption = controller.robots_recall(probe_power_consumption = True)

                # Save power consumption results
                results['samples'].append(power_consumption)
                
                with open(fname, 'w') as f:
                    json.dump(results, f)

                input("Press any key to start next run...")

        else:
            print("No robots found")
    
    controller.close()
    time.sleep(2)
    for thread in threading.enumerate():
        print(thread.name)

if __name__ == "__main__":
    main()
    sys.exit()
