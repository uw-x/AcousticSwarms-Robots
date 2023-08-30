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
from Milestone import Point
from StreamCollector import StreamCollector, StreamSelector
from loc_plan.LOC_2D import extract_distance_single
from loc_plan.Local_Node import local_2D_by_dis_without_outlier


def estimate_position(anchor_positions, self_offset, anchor_offsets):
    achor_num = len(anchor_positions)
    dis = np.zeros((achor_num, 1))
    for i in range(0, achor_num):
        dis[i] = extract_distance_single(self_offset, anchor_offsets[i])
    
    pred_pos, _ = local_2D_by_dis_without_outlier(anchor_positions, dis, None)
    
    return pred_pos

def begin_2d_tracking(controller: RobotController, spacing: int = 200, begin_interval: int = 500):
    controller.initialize_global_planner()
    
    A = controller.get_robot_by_name('A')

    B = controller.get_robot_by_name('B')
    B.current_state.position = Point(PLATFORM_POSITIONS[0, 0], PLATFORM_POSITIONS[0, 1])

    C = controller.get_robot_by_name('C')
    C.current_state.position = Point(PLATFORM_POSITIONS[1, 0], PLATFORM_POSITIONS[1, 1])
    
    D = controller.get_robot_by_name('D')
    D.current_state.position = Point(PLATFORM_POSITIONS[2, 0], PLATFORM_POSITIONS[2, 1])

    E = controller.get_robot_by_name('E')
    E.current_state.position = Point(PLATFORM_POSITIONS[3, 0], PLATFORM_POSITIONS[3, 1])

    F = controller.get_robot_by_name('F')
    F.current_state.position = Point(PLATFORM_POSITIONS[4, 0], PLATFORM_POSITIONS[4, 1])

    G = controller.get_robot_by_name('G')
    G.current_state.position = Point(PLATFORM_POSITIONS[6, 0], PLATFORM_POSITIONS[6, 1])

    moving_robot = A
    A.moving = True

    anchors = [B, C, D, E, F, G]
    for anchor in anchors:
        anchor.moving = False

    for robot in anchors + [moving_robot]:
        mic_id = robot.id
        if mic_id == moving_robot.id:
            gain = ROBOT_SELF_GAIN
        else:
            gain = ROBOT_CROSS_GAIN

        controller.load_tof_profile(robot_id=mic_id, 
                                    sampling_rate=FREQS.SAMPLING_FREQUENCY_62500,
                                    gain=gain,
                                    is_spk=(mic_id == moving_robot.id),
                                    period=spacing//50,
                                    begin_interval=begin_interval//50)
        
        controller.stream_xcorr(mic_id, True, use_notifications=True) # Enable stream on xcorr data
        controller.get_robot_by_id(mic_id).xcorr_stream.flush() # Discard xcorr data from previous streams
        controller.get_robot_by_id(mic_id).xcorr_stream.expected_sequence_number = 0
    
    sc = StreamCollector([robot.xcorr_stream for robot in (anchors + [moving_robot])], frame_width=1)

    anchor_positions = np.array([a.current_state.position.point for a in anchors])

    return sc, moving_robot, anchor_positions

def stop_2d_tracking(controller: RobotController):
    controller.stop_tasks()

    # Disables mic notifications & resets xcorr sequence number.
    # Admittedly, this isn't the best place to do it.
    for _id in controller.robots:
        controller.stream_xcorr(_id, enable=False)
        controller.disable_mic_notifications(_id)


def main():
    controller = RobotController()
    
    if controller.available:
        controller.scan(1)

        if len(controller.robots) > 0:
            print("All robots are ready!")

            for robot_id, robot in controller.robots.items():
                controller.monitor_battery(robot_id, enable=True)
            moving_robot = controller.get_robot_by_name('A')
            input('Ready?')
            forward_time_s = 5
            
            controller.update_motion(moving_robot.id, MOTION_CODES.MOTION_FORWARD, duration_us=forward_time_s * 1e6, speed=150, controlled=False)

            input('Ready?')
            controller.calibrate_sensors(moving_robot.id)
            moving_robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)
            
            controller.update_motion(moving_robot.id, MOTION_CODES.MOTION_FORWARD, duration_us=forward_time_s * 1e6, speed=150, controlled=True)

            input('READY')


            chirp_stream_collector, moving_robot, anchor_positions = begin_2d_tracking(controller)

            ss = StreamSelector(chirp_stream_collector, moving_robot.status_stream).start()
            
            controller.calibrate_sensors(moving_robot.id)
            
            time.sleep(1)

            controller.monitor_accel(moving_robot.id)
            controller.monitor_gyro(moving_robot.id)

            time.sleep(1)

            controller.run_tasks(master = moving_robot.id)
            
            time.sleep(1)
            
            position_list = []

            moving = False

            while True:
                event_id, event_data = ss.get()

                if event_id == 0:
                    # Chirp offsets
                    xcorr_frame = event_data
                    
                    if (xcorr_frame == -1).any():
                        print("A robot has dropped an xcorr packet, ignoring ...")
                        continue

                    anchor_offsets = np.array(xcorr_frame[:-1])
                    self_offset = xcorr_frame[-1][0]
                    
                    position = estimate_position(anchor_positions, self_offset, anchor_offsets)
                    
                    print("ESTIMATED POSITION", position)
                    
                    position_list.append(position)

                    if not moving:
                        forward_time_s = 5
                        controller.update_motion(moving_robot.id, MOTION_CODES.MOTION_FORWARD, duration_us=forward_time_s * 1e6, speed=150, controlled=False)
                        moving = True

                else:
                    assert event_id == 1, "Something went wrong"
                    # Motion stopped, break
                    if event_data[0] == STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END:
                        break
            
            stop_2d_tracking(controller)
            
            # Stop monitoring IMU
            controller.monitor_accel(moving_robot.id, enable=False)
            controller.monitor_gyro(moving_robot.id, enable=False)
            
            # Close streams
            chirp_stream_collector.stop()
            ss.stop()

            # Close controller
            controller.close()

            positions = np.array(position_list)
            accel_data = moving_robot.accel_stream.get_remaining()
            gyro_data = moving_robot.gyro_stream.get_remaining()

            np.save('positions.npy', positions)
            np.save('accel_data.npy', accel_data)
            np.save('gyro_data.npy', gyro_data)

        else:
            print("No robots found")
    
    controller.close()
    time.sleep(2)
    for thread in threading.enumerate():
        print(thread.name)

if __name__ == "__main__":
    main()
    sys.exit()
