import threading
import queue

from helpers import *
from globals import *

from usb_packet_types import *
from MicStream import MicStream
from SensorStreams import *
from Milestone import Point, RobotState
from loc_plan.High_Control import High_Level_Controller
import time

from math import fmod

class XcorrStream(Stream):
    def __init__(self, _id=None):
        super().__init__(_id)
        self.expected_sequence_number = 0
        self.channels = 1
    def process(self, data):
        if DEBUG_XCORR_STREAM:
            print("[{}]".format(self.id), data)
        sequence_number = to_nbit(data[:2])
        
        res = []

        # print(sequence_number, self.expected_sequence_number)
        # print(data)
        
        if sequence_number > self.expected_sequence_number:
            while sequence_number > self.expected_sequence_number:
                res.append([-1])
                self.expected_sequence_number += 1

        res.append([to_nbit(data[2:])/self.channels])
        self.expected_sequence_number = sequence_number + 1
        
        return res

class Robot(object):
    def __init__(self, _id, addr, name):
        self.id = _id
        self.addr = addr
        self.name = name
        self.sensors = {}
        self.master = False
        self.slave = False
        self.mic_stream = MicStream(self.name, debug=False)
        self.recording_finished = False
        
        self.accel_stream = AccelStream(self.name)
        self.battery_stream = BatteryStream(self.name)
        self.mag_stream = MagStream(self.name)
        self.gyro_stream = GyroStream(self.name)

        self.xcorr_stream = XcorrStream(self.name).start()
        self.status_stream = Stream(self.name, debug= False).start()
        self.status_events = [threading.Event() for i in range(64)] # Lock, one for each kind of status

        self.current_state = RobotState(Point(0,0), 0, Point(0,0))
        self.previous_state = None
        self.current_action = 1
        self.current_milestone = RobotState(Point(0,0), 0, Point(0,0)).position

        self.moving = False

    def get_high_level_status(self):
        return self.high_controller.Current_status

    def initialize_high_controller(self, sequence_id, global_planner):
        self.high_controller = High_Level_Controller(sequence_id, global_planner) # the id need to be updated

    def begin_mic_stream(self):
        self.mic_stream.start()

    def stop_mic_stream(self):
        self.mic_stream.stop()

    def terminate(self):
        self.stop_mic_stream()
        self.accel_stream.stop()
        self.battery_stream.stop()
        self.gyro_stream.stop()
        self.xcorr_stream.stop()
        self.status_stream.stop()

    def pipe_data(self, data_type, data):
        if data_type == SENSOR_TYPES.SENSOR_MICROPHONE:
            self.mic_stream.put(data)
        elif data_type == SENSOR_TYPES.SENSOR_FG:
            self.battery_stream.put(data)

            voltage, soc, rate = np.frombuffer(bytes(data), dtype=np.float32)
            print("Robot {} is at {:.02f}% battery, ({:.02f}%/hr charge rate, {:.02f}V)".format(self.name, soc, rate, voltage))
        elif data_type == SENSOR_TYPES.SENSOR_ACCEL:
            self.accel_stream.put(data)
        elif data_type == SENSOR_TYPES.SENSOR_MAG:
            self.mag_stream.put(data)
        elif data_type == SENSOR_TYPES.SENSOR_GYRO:
            self.gyro_stream.put(data)

    def update_current_state(self, data):
        """
        data: (x, y) | r
        """
        self.current_state = RobotState.from_list(data)

    def handle_status_update(self, update_type, data):        
        if update_type == STATUS_UPDATE_TYPES.DCS_XCORR_UPDATE:
            # print(data[:4])
            self.xcorr_stream.put(data[:4])
            if self.master and self.moving:
            # if True:
                # print(data)
                # print(np.frombuffer(np.array(data[2:22], dtype=np.uint8).tobytes(), np.float32))
                self.update_current_state(data[4:24])
                self.previous_state = RobotState.from_list(data[24: 44])

                self.current_milestone = RobotState.from_list(data[44: 64]).position
                
                # print("Current state:", self.current_state.tostring())
                # print("Previous state:", self.previous_state.tostring())

                # print("Current rotation:", fmod(360 + fmod(self.current_state.rotation, 360), 360) )
                # print("Previous rotation:", fmod(360 + fmod(self.previous_state.rotation, 360), 360) ))

                # print("Current action: ", data[-4:], 'data[80:]', data[80:])
                if data[-4] != self.current_action:
                    self.status_stream.put([STATUS_UPDATE_TYPES.DCS_STATUS_NAV_TRANSITION, self.current_action, data[-4]])
                    self.current_action = data[-4]

            # start_time, end_time = np.frombuffer(bytes(data[64:80]), dtype=np.int64)
            # print([{self.name}], 'Start: ', start_time/1000, 'ms \tEnd: ', end_time/1000, 'ms \tElapsed: {}', (end_time - start_time) / 1000, 'ms')
        
        elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_MILESTONE_REACHED:
            print("*********** Milestone reached! ***********")
            self.update_current_state(data)
            self.status_stream.put([STATUS_UPDATE_TYPES.DCS_STATUS_MILESTONE_REACHED, self.current_state.to_numpy()])
        elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_MOTION_CORRECTED:
            print(" **** Motion corrected **** ")
            self.update_current_state(data)
            self.status_stream.put([STATUS_UPDATE_TYPES.DCS_STATUS_MOTION_CORRECTED, self.current_state.to_numpy()])
        elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_NEXT_MILESTONE:
            print(" **** Next milestone **** ")
            self.status_stream.put([STATUS_UPDATE_TYPES.DCS_STATUS_NEXT_MILESTONE, RobotState.from_list(data)])
        elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_NAV_TRANSITION:
            # print(" **** TRANSITION **** ")
            self.status_stream.put([STATUS_UPDATE_TYPES.DCS_STATUS_NAV_TRANSITION, data[0], data[1]])
        elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END:
            print(f"[{self.name}]  **** NAV END **** ")
            if self.moving:
                self.update_current_state(data)
            self.status_stream.put([STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END])
        elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_LOG:
            print('==' * 20)
            print()
            print("[{}] ".format(self.name), (bytes(data)).decode() )
            print()
            print('==' * 20)
            self.status_stream.put([update_type, chr(data[0]) ])
        elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_CONNECTION_INTERVAL_UPDATE:
            # print("Robot {} has updated the connection interval.".format(self.name))
            pass
        elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE:
            print("[{}] CALIBRATED".format(self.name))
            self.status_stream.put([STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE])
        elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_COLLISION_DETECTED:
            print("[{}] Collision Detected.".format(self.name))
            self.status_stream.put([STATUS_UPDATE_TYPES.DCS_STATUS_COLLISION_DETECTED])
        elif update_type == STATUS_UPDATE_TYPES.DCS_IR_STATUS_UPDATED:
            self.ir_status = data[0]
            print("[{}] IR STATUS UPDATED.".format(self.name))
            self.status_stream.put([STATUS_UPDATE_TYPES.DCS_IR_STATUS_UPDATED, data[0]])
        elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_BASE_ENTERED:
            print("[{}] BASE ENTERED.".format(self.name))
            self.status_stream.put([STATUS_UPDATE_TYPES.DCS_STATUS_BASE_ENTERED])
        elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_PING:
            print("[{}] PING REPLY".format(self.name))
            self.status_stream.put([STATUS_UPDATE_TYPES.DCS_STATUS_PING])
        else:
            print("**** UNHANDLED UPDATE OF TYPE {} *****".format(update_type))
        
        # print('Lock state', self.status_events[update_type].locked())
        # if self.status_events[update_type].locked():
        self.status_events[update_type].set()

    def wait_on_status(self, status_update_type: STATUS_UPDATE_TYPES, timeout = None):
        
        # print("waiting on status", status_update_type)
        try:
            self.status_events[status_update_type].wait(timeout=timeout)
            # print("Got it")
            return True
        except:
            # print("ERROR")
            return False        
        # t1 = time.time()
        # while True:
        #     t2 = time.time()
        #     if timeout:
        #         remaining_time = timeout - (t2 - t1)
        #         if remaining_time <= 0:
        #             break
        #     else:
        #         remaining_time = None
        #     try:
        #         update = self.status_stream.get(timeout = remaining_time)
        #         if update[0] == status_update_type:
        #             return True
        #     except:
        #         break
        
        # return False

    def get_mic_stream(self):
        return self.mic_stream.get()

    def get_stream(self):
        frames = []
        while not self.mic_stream.empty():
            frames.append(self.mic_stream.get())
        
        stream = np.concatenate(frames)

        # MASTER_OVERFLOW_TRIGGER_COUNT * TIMER_CYCLE_LENGTH * SAMPLING_FREQUENCY
        dropped_samples = int(round(self.mic_stream.extra_initial * 0.05 * self.mic_stream.sr.value))
        stream = stream[dropped_samples:]
        print("Dropping {} samples from robot {}".format(dropped_samples, self.id))
        
        print("There are {} dropped packets:".format(self.mic_stream.dropped_packets))

        return self.mic_stream.timestamp, stream

    def __del__(self):
        self.terminate()
