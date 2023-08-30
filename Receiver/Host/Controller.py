import numpy as np
import threading
import time
import glob
import json, os

from uuids import *
from control_values import *
from usb_packet_types import *
from globals import *
from helpers import *
from GATTBuilder import GATTBuilder
from usb_stream import USBStream
from Robot import Robot
from StreamCollector import StreamCollector, StreamSelector
from Milestone import Milestone, RobotState, Point

from loc_plan.High_Control import High_Status
from loc_plan.Pairwise_Node import Pairwise_Node
from loc_plan.New_Planner import Path_Planner
from loc_plan.Local_Node import estimate_angle
from math import fmod
from loc_plan.LOC_2D import extract_distance_single
import queue

from PathPlotter import PathPlotter

class PlatformSequence():
    def __init__(self, robot: int, idx: int):
        self.sequence = [robot]
        self.idx = idx
        self.tail_at_end = False

    def add(self, robot_id: int):
        self.sequence.append(robot_id)

    def head(self):
        return self.sequence[0]

    def tail(self):
        return self.sequence[-1]

    def get(self):
        return self.sequence
    
    def complete(self):
        self.tail_at_end = True
    
    def is_complete(self):
        return self.tail_at_end

    def join_sequence(self, a):
        """
        Sequence a is in front of self
        """
        for x in a.sequence:
            self.add(x)
        
        # If a is complete, then the whole sequence is now complete
        self.tail_at_end = a.tail_at_end

class RobotController():
    def __init__(self):
        self.available = False
        self.closed = True
        self.sem_closed = threading.Semaphore()
        for port in glob.glob('/dev/ttyACM*'):
            self.usb_stream = USBStream(port=port)
            time.sleep(1.5)
            if self.usb_stream.valid:
                break
            self.usb_stream.close()
            print(port, 'is invalid')

        if self.usb_stream.valid:
            self.usb_stream.flush()
            self.available = True
            self.closed = False
            print("Using device on port: ", port)

            self.robots = {}

            self.data_thread = threading.Thread(target=self.handle_usb_data)
            self.data_thread.start()

            self.robots_ready = 0

            if os.path.isfile('robots.txt'):
                with open("robots.txt", 'rb') as f:
                    self.discovered = json.load(f)
            else:
                self.discovered = {'robots':{}}

            self.global_planner = None

    def handle_usb_data(self):
        while True:
            data = self.usb_stream.get()

            if data is None:
                break

            packet_type = data[0]
            # print(packet_type)
            if packet_type == PACKET_TYPES.USB_MIC_DATA:
                params = data[2][:2]
                mic_data = data[2][2:]
                _id = to_nbit(params)
                if _id in self.robots:
                    self.robots[_id].pipe_data(SENSOR_TYPES.SENSOR_MICROPHONE, mic_data)
            elif packet_type == PACKET_TYPES.USB_SENSOR_DATA:
                params = data[2][:2]
                sensor_type = data[2][2]
                sensor_data = data[2][3:]
                _id = to_nbit(params)
                if _id in self.robots:
                    self.robots[_id].pipe_data(sensor_type, sensor_data)
            elif packet_type == PACKET_TYPES.USB_BLE_STATUS_UPDATE:
                params = data[2][:2]
                update_type = data[2][2]
                _id = to_nbit(params)

                self.robots[_id].handle_status_update(update_type, data[2][3:])

            elif packet_type == PACKET_TYPES.USB_BLE_DEVICE_CONNECTED:
                assert data[1] >= 2 # Connection handle (id) is two bytes long
                _id = to_nbit(data[2][:2])
                address = hex(to_nbit(data[2][2:]))
                if _id not in self.robots:
                    if address not in self.discovered['robots']:
                        print("Newly discovered robot, adding it to list of discovered robots")
                        self.discovered['robots'][address] = {'name':chr(ord('A') + len(self.discovered['robots']))}
                        
                        with open("robots.txt", 'w') as f:
                            json.dump(self.discovered, f)

                    name = self.discovered['robots'][address]['name']
                    print("Robot {} has connected! (addr: {})".format(name, address))
                    self.robots[_id] = Robot(_id, addr=address, name=name)

            elif packet_type == PACKET_TYPES.USB_BLE_DEVICE_DISCONNECTED:
                assert data[1] == 2 # Connection handle (id) is two bytes long
                _id = to_nbit(data[2][:2])
                del self.robots[_id]
                print("Robot {} has disconnected!".format(_id))
            elif packet_type == PACKET_TYPES.USB_BLE_INFO:
                if len(data[2]) == 1 and data[2][0] == BLE_INFO.BLE_TX_FINISHED:
                    self.usb_stream.ble_sent = True
            elif packet_type == PACKET_TYPES.USB_INFO:
                message = ''.join([chr(x) for x in data[2]])
                print("DK sent the following message: {}".format(message))
            elif packet_type == PACKET_TYPES.USB_BLE_RECORDING_FINISHED:
                _id = to_nbit(data[2][:2])
                print("recording finished {}".format(_id))
                self.robots[_id].recording_finished = True
                # self.robots[_id].recording_finished.release()
            elif packet_type == PACKET_TYPES.USB_BLE_DATABASE_DISCOVERY_COMPLETE:
                _id = to_nbit(data[2][:2])
                
                # Enable notifications on sensor characteristic
                data = GATTBuilder(_id).start_notify(SSS_SENSOR_DATA_UUID).get()
                self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)
            elif packet_type == PACKET_TYPES.USB_BLE_COMMAND:
                assert 0 # DK should not command host.
        self.sem_closed.release()

    def get_robot_by_name(self, name) -> Robot:
        """
        Returns a robot object given the robot name
        Returns null if no robot with his name exists
        """
        for _, r in self.robots.items():
            if r.name == name:
                return r
        
        return None

    def get_robot_by_id(self, id) -> Robot:
        """
        Returns a robot object given the robot id
        Returns null if no robot with his name exists
        """
        for _, r in self.robots.items():
            if r.id == id:
                return r
        
        return None

    def scan(self, num):
        self.begin_scan()
        while len(self.robots) < num:
            time.sleep(1)
        self.stop_scan()
        time.sleep(1)

    def begin_scan(self):
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, [BLE_COMMANDS.BLE_SCAN_START])
        self.scanning = True
        print("Scan started")

    def stop_scan(self):
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, [BLE_COMMANDS.BLE_SCAN_STOP])
        self.scanning = False
        print("Scan stopped")

    def make_timesync_master(self, robot_id):
        self._check_robot(robot_id)
        
        self.robots[robot_id].master = True
        self.robots[robot_id].slave = False

        data = GATTBuilder(robot_id).write(ASS_CONTROL_CHAR_UUID).add_params(ASS_CONTROL_ENABLE_MASTER).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)
        print("Robot {} is now the master".format(self.robots[robot_id].name))

    def make_timesync_slave(self, robot_id):
        self._check_robot(robot_id)
        
        self.robots[robot_id].master = False
        self.robots[robot_id].slave = True

        data = GATTBuilder(robot_id).write(ASS_CONTROL_CHAR_UUID).add_params(ASS_CONTROL_ENABLE_SLAVE).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)
        print("Robot {} is now a slave".format(self.robots[robot_id].name))
        
    def disable_timesync(self, robot_id):
        self._check_robot(robot_id)
        
        self.robots[robot_id].master = False
        self.robots[robot_id].slave = False

        data = GATTBuilder(robot_id).write(ASS_CONTROL_CHAR_UUID).add_params(ASS_CONTROL_TS_DISABLE).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)
        print("Robot {} is now a slave".format(self.robots[robot_id].name))

    def enable_status_notifications(self, robot_id):
        self._check_robot(robot_id)
        
        data = GATTBuilder(robot_id).start_notify(DCS_STATUS_CHAR_UUID).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def wait_for_robot_timestamps(self, robots):
        t = 0
        
        items = list(robots.items())
        
        for i, (_id, bot) in enumerate(items):
            # TODO: Change this to a semaphore/event
            while bot.mic_stream.timestamp is None:
                time.sleep(0.05)

            if bot.master:
                bot.mic_stream.extra_initial = 0

    def enable_mic_notifications(self, robot_id):
        self._check_robot(robot_id)
        
        self.robots[robot_id].begin_mic_stream()
        data = GATTBuilder(robot_id).start_notify(ASS_MIC_CHAR_UUID).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)
        # print("Enabled mic notifications from robot ", robot_id)
    
    def disable_mic_notifications(self, robot_id):
        self._check_robot(robot_id)
        
        data = GATTBuilder(robot_id).stop_notify(ASS_MIC_CHAR_UUID).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)
        
        # print("Stopped mic notifications from robot ", robot_id)
        self.robots[robot_id].stop_mic_stream()

    def await_recording(self, samples, robot_id):
        """
        Waits for recording finished notification & returns the recorded stream
        """
        print("Waiting for robot to send data")
        while not self.robots[robot_id].recording_finished:
            print(self.robots[robot_id].recording_finished)
            time.sleep(0.2)

        data = GATTBuilder(robot_id).write(ASS_CONTROL_CHAR_UUID).add_params(ASS_CONTROL_RECORDING_SEND).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)
        print("Requested recording")

        data = []
        while len(data) < samples:
            data.extend(self.robots[robot_id].mic_stream.get())

        self.robots[robot_id].recording_finished = False

        return data

    def enable_streaming(self, robot_id):
        self._check_robot(robot_id)
        
        self.enable_mic_notifications(robot_id)
        
        data = GATTBuilder(robot_id).write(ASS_CONTROL_CHAR_UUID).add_params(ASS_CONTROL_MIC_MODE_SET_STREAM).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)
        print("Enabled streaming from robot ", robot_id)

    def enable_recording(self, robot_id):
        self._check_robot(robot_id)
        
        # Enable recording mode
        data = GATTBuilder(robot_id).write(ASS_CONTROL_CHAR_UUID).add_params(ASS_CONTROL_MIC_MODE_SET_RECORD).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

        # Enable notifications on recording status
        data = GATTBuilder(robot_id).start_notify(ASS_STATUS_CHAR_UUID).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def stream_xcorr(self, robot_id, enable=True, use_notifications=False):
        """
        Enable streaming of chirp sample offsets
        Data can be sent using indications or notifications if there are tight latency requirements.
        """
        self._check_robot(robot_id)
        
        if enable:
            data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(DCS_COMMAND_XCORR_STREAM_ENABLE, use_notifications).get()
            self.enable_recording(robot_id) # Enable notifications on xcorr data
            self.enable_status_notifications(robot_id) # Enable notifications on xcorr data
        else:
            data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(DCS_COMMAND_XCORR_STREAM_DISABLE, use_notifications).get()
            # self.disable_recording(robot_id) # Enable notifications on xcorr data
            # self.disable_status_notifications(robot_id) # Enable notifications on xcorr data
        
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def sync_task_create(self, robot_id, task_id, interval):
        self._check_robot(robot_id)

        data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(DCS_COMMAND_SYNC_TASK_CREATE,
                                                                             task_id,
                                                                             (interval >> 24) & 0xFF,
                                                                             (interval >> 16) & 0xFF,
                                                                             (interval >> 8) & 0xFF,
                                                                             interval & 0xFF ).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def sync_task_create_recurring(self, robot_id, task_id, period, start_interval, end_interval=0):
        self._check_robot(robot_id)

        data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(DCS_COMMAND_SYNC_TASK_CREATE_RECURRING,
                                                                             task_id,
                                                                             (period >> 24) & 0xFF,
                                                                             (period >> 16) & 0xFF,
                                                                             (period >> 8) & 0xFF,
                                                                             period & 0xFF,
                                                                             (start_interval >> 24) & 0xFF,
                                                                             (start_interval >> 16) & 0xFF,
                                                                             (start_interval >> 8) & 0xFF,
                                                                             start_interval & 0xFF,
                                                                             (end_interval >> 24) & 0xFF,
                                                                             (end_interval >> 16) & 0xFF,
                                                                             (end_interval >> 8) & 0xFF,
                                                                             end_interval & 0xFF,
                                                                              ).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def chirp(self, robot_id):
        self._check_robot(robot_id)
        
        data = GATTBuilder(robot_id).write(ASS_CONTROL_CHAR_UUID).add_params(ASS_CONTROL_START_CLICK).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)
        print("Chirp at robot ", robot_id)

    def await_ble_tx_finished(self):
        self.usb_stream.ble_sent = False
        
        # Wait for usb stream to finish sending BLE data
        while not self.usb_stream.ble_sent:
            self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, [BLE_COMMANDS.BLE_TX_AWAIT_FINISHED])
            time.sleep(0.2)    

    def close(self):
        if not self.closed:
            if self.usb_stream.valid:
                for _id, robot in self.robots.items():
                    robot.terminate()
            
                self.usb_stream.processed_stream.put(None)
                self.sem_closed.acquire()
                    
                self.usb_stream.close()

            self.closed = True

    def read_battery_level(self, robot_id):
        data = GATTBuilder(robot_id).write(SSS_CONTROL_CHAR_UUID).add_params(SSS_READ_FUEL_GAUGE, (1 << SENSOR_TYPES.SENSOR_ACCEL)).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def _monitor_sensor(self, robot_id, sensor_id, enable):
        if enable:
            control = SSS_STREAM_ENABLE
        else:
            control = SSS_STREAM_DISABLE

        data = GATTBuilder(robot_id).write(SSS_CONTROL_CHAR_UUID).add_params(control, (1 << sensor_id)).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def monitor_accel(self, robot_id = None, enable=True):
        """
        Enable/Disable accelerometer monitoring (i.e. accelerometer notifications)
        """

        if robot_id is None:
            robot_id = [x[0] for x in list(self.robots.items())]
        elif not hasattr(robot_id, '__len__'):
            robot_id = [robot_id]
        
        for _id in robot_id:
            self._monitor_sensor(_id, SENSOR_TYPES.SENSOR_ACCEL, enable)
            if enable:
                self.robots[_id].accel_stream.start()
            else:
                self.robots[_id].accel_stream.stop()

    def monitor_battery(self, robot_id = None, enable=True):
        """
        Enable/Disable battery monitoring (i.e. battery notifications)        
        """

        if robot_id is None:
            robot_id = [x[0] for x in list(self.robots.items())]
        elif not hasattr(robot_id, '__len__'):
            robot_id = [robot_id]
        
        for _id in robot_id:
            self._monitor_sensor(_id, SENSOR_TYPES.SENSOR_FG, enable)
            if enable:
                self.robots[_id].battery_stream.start()
            else:
                self.robots[_id].battery_stream.stop()

    def monitor_mag(self, robot_id = None, enable=True):
        """
        Enable/Disable battery monitoring (i.e. battery notifications)        
        """

        if robot_id is None:
            robot_id = [x[0] for x in list(self.robots.items())]
        elif not hasattr(robot_id, '__len__'):
            robot_id = [robot_id]
        
        for _id in robot_id:
            self._monitor_sensor(_id, SENSOR_TYPES.SENSOR_MAG, enable)
            if enable:
                self.robots[_id].mag_stream.start()
            else:
                self.robots[_id].mag_stream.stop()

    def monitor_gyro(self, robot_id = None, enable=True):
        """
        Enable/Disable battery monitoring (i.e. battery notifications)        
        """

        if robot_id is None:
            robot_id = [x[0] for x in list(self.robots.items())]
        elif not hasattr(robot_id, '__len__'):
            robot_id = [robot_id]
        
        for _id in robot_id:
            self._monitor_sensor(_id, SENSOR_TYPES.SENSOR_GYRO, enable)
            if enable:
                self.robots[_id].gyro_stream.start()
            else:
                self.robots[_id].gyro_stream.stop()

    def enable_collision_detection(self, robot_id, moving=False):
        self._check_robot(robot_id)

        self.get_robot_by_id(robot_id).status_events[STATUS_UPDATE_TYPES.DCS_STATUS_COLLISION_DETECTED].clear()

        data = GATTBuilder(robot_id).write(SSS_CONTROL_CHAR_UUID).add_params(SSS_ENABLE_COLLISION_DETECTION, moving).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def disable_collision_detection(self, robot_id):
        self._check_robot(robot_id)
        data = GATTBuilder(robot_id).write(SSS_CONTROL_CHAR_UUID).add_params(SSS_DISABLE_COLLISION_DETECTION).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def update_motion(self, robot_id, code, duration_us: int = None, angle: float = None, speed: int = 200, controlled: bool = True):
        self._check_robot(robot_id)

        self.get_robot_by_id(robot_id).status_events[STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END].clear()
        
        params = []
        if code != MOTION_CODES.MOTION_BRAKE:
            assert (duration_us is not None) or (angle is not None), "Must provide duration or angle for the given motion code."
            if code in [MOTION_CODES.MOTION_FORWARD, MOTION_CODES.MOTION_BACKWARD, MOTION_CODES.MOTION_ROTATE_TIME]:
                assert duration_us is not None, "The given motion code only works with a duration."
                params = np.array([duration_us], dtype=np.uint64).tobytes()
            
            if code in [MOTION_CODES.MOTION_ROTATE_ANGLE, MOTION_CODES.MOTION_ROTATE_ANGLE_PRECISE]:
                assert angle is not None, "The given motion code only works with an angle."
                params = np.array([angle], dtype=np.float32).tobytes()
                
        data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(DCS_COMMAND_ROBOT_MOTION, code, *params, (speed&0xFF), ((speed >> 8) & 0xFF), controlled).get()
        
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def update_state_information(self, robot_id, 
                                 current_state: RobotState,
                                 current_milestone: Milestone = None,
                                 next_milestone: Milestone = None,
                                 motion_flags: list = []):
        self._check_robot(robot_id)

        if current_milestone is None:
            current_milestone = Milestone(-1e9, -1e9)
        
        if next_milestone is None:
            next_milestone = Milestone(-1e9, -1e9)

        flags = 0
        for flag in motion_flags:
            if flag >= 0 and flag <= 7:
                flags |= (1 << flag)
        
        params = [DCS_ROBOT_POSITION_UPDATE, *(current_state.to_byte_list() + 
                                               current_milestone.to_byte_list () + 
                                               next_milestone.to_byte_list()), flags]
        # print(params)
        # print(len(params))
        data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(*params).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def send_milestones(self, robot_id, milestones, overwrite_current = False):
        while len(milestones) > 0:
            
            if overwrite_current:
                command = DCS_COMMAND_RESET_MILESTONES
            else:
                command = DCS_COMMAND_ADD_MILESTONES
            
            params = [command, len(milestones[:30])]
            params.extend([byte for x in milestones[:30] for byte in x.to_byte_list()])
            
            data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(*params).get()
            self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

            milestones = milestones[30:]

    def enter_base(self, robot_id:int, from_base:bool, depth:np.int8, forward=True):
        """
        Orders robot to perform the base entry routine
        If from_base is false, then the robot first enters the lobby and attempts to connect to the rails
        If from_base is true, then the robot just moves forward (or backward)
        Depth is the number of markers to pass before stopping
        """
        params = [DCS_COMMAND_ENTER_BASE, from_base, np.uint8(depth), forward]
        robot = self.get_robot_by_id(robot_id)
        
        if from_base == False:
            robot.status_events[STATUS_UPDATE_TYPES.DCS_STATUS_BASE_ENTERED].clear()

            self.update_motion(robot.id, MOTION_CODES.MOTION_FORWARD, duration_us=300e3, speed=400)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            self.update_motion(robot.id, MOTION_CODES.MOTION_FORWARD, duration_us=700e3, speed=140)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            # self.update_motion(robot.id, MOTION_CODES.MOTION_BACKWARD, duration_us=150e3, speed=140)
            # robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            self.update_motion(robot.id, MOTION_CODES.MOTION_ROTATE_ANGLE, angle=90)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            self.update_motion(robot.id, MOTION_CODES.MOTION_ROTATE_ANGLE, angle=45)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            self.update_motion(robot.id, MOTION_CODES.MOTION_ROTATE_ANGLE, angle=0)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            
            self.update_motion(robot.id, MOTION_CODES.MOTION_ROTATE_ANGLE, angle=270)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            self.update_motion(robot.id, MOTION_CODES.MOTION_BACKWARD, duration_us=1e6, speed=200)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            self.update_motion(robot.id, MOTION_CODES.MOTION_ROTATE_ANGLE, angle=350)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            self.calibrate_sensors(robot.id)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)
        else:
            robot.status_events[STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END].clear()
        data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(*params).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def calibrate_sensors(self, robot_id):
        self._check_robot(robot_id)

        self.get_robot_by_id(robot_id).status_events[STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE].clear()

        print("CALIBRATION DONE", self.get_robot_by_id(robot_id).status_events[STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE].isSet())

        # data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(DCS_COMMAND_NAV_BEGIN).get()
        data = GATTBuilder(robot_id).write(SSS_CONTROL_CHAR_UUID).add_params(SSS_CALIBRATE_ALL).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def nav_begin(self, robot_id):
        self._check_robot(robot_id)

        data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(DCS_COMMAND_NAV_BEGIN).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def begin_stream(self, frame_width : float = None, channels: list = None, master: int=None, sr:int=None):
        """
        Frame width is in ms
        Channels is a list of robot ids
        If master is None, first robot is assumed to be master
        """
        if frame_width is None:
            frame_width = 50

        if channels is None:
            sorted_robots = sorted([self.robots[x] for x in self.robots], key=lambda x: x.name)
            channels = [k.id for k in sorted_robots]

        if master is None:
            master = channels[0]

        streaming_robots = [self.robots[k] for k in channels] # Sorted
        
        for _id in channels:
            self.make_timesync_slave(_id) # Needed to reset everything
            self.sync_task_create(_id, SYNC_TASKS.SYNC_TASK_MIC_START, 0)
            self.enable_streaming(_id)

        frame_widths = []

        # Compute frame width as the number of samples to cover a given time in ms for each robot sample rate
        for robot in streaming_robots:
            if sr is None:
                sr = robot.mic_stream.sr.standard_freq
            frame_widths.append(int(round(frame_width * 1e-3 * sr)))
        
        self.stream_collector = StreamCollector([robot.mic_stream for robot in streaming_robots], frame_width=frame_widths, debug=False)

        # Choose first robot as ts master
        self.run_tasks(master=master)

        self.stream_collector.start()

    def grab_frame(self, timeout=None):
        return self.stream_collector.get(timeout)

    def stop_stream(self):
        """
        Stops streaming and sets timesync master as slave.
        """
        self.stream_collector.terminate()
        dropped_packets = {}

        for stream in self.stream_collector.streams:
            robot = self.get_robot_by_name(stream.id)
            if robot is not None:
                _id = robot.id
                if hasattr(stream, "dropped_packets"):
                    print('Robot {} has {} dropped packets!'.format(stream.id, stream.dropped_packets))
                    dropped_packets[stream.id] = stream.dropped_packets
                
                self.disable_mic_notifications(_id)
                robot.mic_stream.stop()
                robot.mic_stream.flush()
                robot.mic_stream.reset()
            
        self.stop_tasks()

        return dropped_packets

    def update_mic_parameters(self, robot_id,
                              sampling_rate=None,
                              left_gain=None,
                              right_gain=None,
                              channels=None,
                              use_compression=None,
                              compression_bitrate=0):
        """
        Update mic parameters for a specific robot. ONLY ALLOWED IF ROBOT PDM IS NOT ACTIVE (i.e. mic notifications are disabled).
        """
        self._check_robot(robot_id)

        if sampling_rate is None:
            clk_freq = 0xFF
            decimation = 0xFF
            ratio = 0xFF
        else:
            clk_freq = freq_list[sampling_rate].div32
            decimation = freq_list[sampling_rate].decimation_factor
            ratio = freq_list[sampling_rate].ratio
            self.robots[robot_id].mic_stream.set_sampling_rate(sampling_rate)

        if left_gain is None:
            left_gain = 0xFF
        
        if right_gain is None:
            right_gain = 0xFF
        
        if channels is None:
            channels = 0xFF
        else:
            assert channels in [1, 2]
            self.get_robot_by_id(robot_id).mic_stream.channels = channels
            self.get_robot_by_id(robot_id).xcorr_stream.channels = channels
            channels = 2 - channels # mode 0 -> Stereo, mode 1 -> Mono

        if use_compression is None:
            use_compression = 0xFF
            compression_bitrate = 0
        else:
            print('use compression', use_compression)
            self.robots[robot_id].mic_stream.use_compression = use_compression
            self.robots[robot_id].mic_stream.compression_bitrate = compression_bitrate
            if use_compression:
                self.robots[robot_id].mic_stream.create_decoder(channels=1)

        data = GATTBuilder(robot_id).write(ASS_CONTROL_CHAR_UUID).add_params(ASS_CONTROL_MIC_UPDATE_PARAMS,
                                                                     ((clk_freq>>24)& 0xFF),
                                                                     ((clk_freq>>16) & 0xFF),
                                                                     ((clk_freq>>8) & 0xFF),
                                                                     (clk_freq) & 0xFF,
                                                                     decimation,
                                                                     left_gain,
                                                                     right_gain,
                                                                     channels,
                                                                     ratio,
                                                                     use_compression,
                                                                     (compression_bitrate >> 24) & 0xFF,
                                                                     (compression_bitrate >> 16) & 0xFF,
                                                                     (compression_bitrate >> 8) & 0xFF,
                                                                     compression_bitrate & 0xFF).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def flush_mic_stream(self, robot_id):
        self._check_robot(robot_id)
        
        self.robots[robot_id].mic_stream.flush()

    def _check_robot(self, _id):
        if _id not in self.robots:
            print("Robot {} has not yet connected".format(self.robots[_id].name))
            assert 0 # Change to exception?

    def __del__(self):
        self.close()

    def ping(self, robot_id):
        """
        Ping robot and expects a reply. This happens on the event Queue so by the time a reply is
        sent and received by host, all previously sent commands should have been processed.
        """
        self._check_robot(robot_id)
        
        self.get_robot_by_id(robot_id).status_events[STATUS_UPDATE_TYPES.DCS_STATUS_PING].clear()

        data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(DCS_COMMAND_PING).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def run_tasks(self, master=None):
        if master is None:
            master = list(self.robots.items())[0][0]

        self.await_ble_tx_finished()

        for _id in self.robots:
            print("ID", _id)
            robot = self.get_robot_by_id(_id)
            if robot.slave or robot.master:
                robot.status_stream.start()
                print('Pinging', robot.name)
                self.ping(_id)
        
        print("Awaiting")
        self.await_ble_tx_finished()
        for _id in self.robots:
            robot = self.get_robot_by_id(_id)
            if robot.slave or robot.master:
                print('Checking for ping', robot.name)
                robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_PING)
        
        # Choose first robot as ts master
        self.make_timesync_master(master)
        
        self.await_ble_tx_finished()
        self.ts_master = master

    def stop_tasks(self):
        if self.get_robot_by_id(self.ts_master) is not None:
            self.make_timesync_slave(self.ts_master)
            self.ping(self.ts_master)
            self.get_robot_by_id(self.ts_master).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_PING)

        for _id in self.robots.keys():
            self.disable_timesync(_id)

    def enable_ir_status_stream(self, robot_id, val: bool = True):
        self._check_robot(robot_id)
        if val:
            val = SSS_ENABLE_IR_STATUS_STREAM
        else:
            val = SSS_DISABLE_IR_STATUS_STREAM
        
        data = GATTBuilder(robot_id).write(SSS_CONTROL_CHAR_UUID).add_params(val).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def probe_battery_information(self):
        """
        Get battery information for all robots.
        Returns a dictionary, keys are the robot names, values correspond to battery information.
        """
        
        print("Probing battery information...")
        
        streams = []
        
        for _id in self.robots:
            robot = self.get_robot_by_id(_id)
            robot.battery_stream.flush()
            self.read_battery_level(_id)
            streams.append(robot.battery_stream)
        
        collector = StreamCollector(streams=streams, frame_width=3).start()
        
        battery_info = collector.get()

        result = {}

        for i, _id in enumerate(self.robots):
            robot = self.get_robot_by_id(_id)
            result[robot.name] = {}
            result[robot.name]['vcell'] = str(battery_info[i, 0])
            result[robot.name]['SOC'] = str(battery_info[i, 1])
            result[robot.name]['CRATE'] = str(battery_info[i, 2])

        collector.stop()

        return result

    def disable_ir_status_stream(self, robot_id):
        return self.enable_ir_status_stream(robot_id, False)

    def read_ir_status(self, robot_id, blocking=True):
        """
        Reads IR sensor status from robot_id
        """
        self._check_robot(robot_id)
        
        data = GATTBuilder(robot_id).write(SSS_CONTROL_CHAR_UUID).add_params(SSS_IR_STATUS_READ).get()
        
        self.get_robot_by_id(robot_id).status_events[STATUS_UPDATE_TYPES.DCS_IR_STATUS_UPDATED].clear()
        self.enable_ir_status_stream(robot_id, True)
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

        if blocking:
            self.get_robot_by_id(robot_id).wait_on_status(STATUS_UPDATE_TYPES.DCS_IR_STATUS_UPDATED)
            self.disable_ir_status_stream(robot_id)

    def _get_mean_pos(self, robot: Robot, sc: StreamCollector, N: int, plan_node):
        """
        Blocks until N separate xcorr frames have been received
        Computes the average of the localization estimate over all N samples
        @param robot: Robot to localize
        @param sc: StreamCollector object of xcorr streams
        @param N: Number of xcorr samples to consider
        @param plan_node: Planner node to compute position from xcorr samples
        """

        time.sleep(0.3) # Ignore chirps from the last 0.3 seconds because they might have been affected by noise/motion
        sc.flush()
        samples = []
        
        i = 0
        while i < N:
            # Get xcorr frame from stream collector
            xcorr_frame = sc.get()
            
            # Make sure xcorr frames are synchronized
            if (xcorr_frame == -1).any():
                print("A robot has dropped an xcorr packet, ignoring ...")
                continue

            # Separate anchor offsets from self offsets
            anchor_offsets = np.array(xcorr_frame[:-1])
            self_offset = xcorr_frame[-1][0]

            print('Anchor offsets', anchor_offsets.tolist())
            print('Self offsets', self_offset)

            # Compute position estimate
            mean, velocity, (estimated_angle, confidence) = plan_node.update_offset(sample_offsets=anchor_offsets, 
                                                                                    self_offset=self_offset,
                                                                                    old_pos=robot.previous_state.to_numpy(),
                                                                                    new_pos=robot.current_state.to_numpy())
            mean = mean.reshape(3, 1)
            
            # Add position estimate to list of estimates
            samples.append(np.array([mean[0][0], mean[1][0]]))

            i += 1
        
        print('Samples', samples)
        # Take the mean of the estimates
        pos = np.array(samples).mean(axis=0)
        return pos

    def navigate(self, milestones: list, robot: Robot, anchors: list, offset: int=200, spacing: int=200, accurate_end_pos: bool = False):
        """
        Moves a robot from its current along a set of given milestones.
        Blocks until robot has reached the last milestone.
        @param milestones The milestones that the robot needs to move alone
        @param: robot The robot to move along the milestones
        @param: anchors The robots to act as anchors for the robot to use to localize
        """
        # If there are no milestones, don't do anything
        if len(milestones) == 0:
            return

        robot.moving = True
        for a in anchors:
            a.moving = False
        
        anchor_ids = [r.id for r in anchors]
        robot_id = robot.id
        
        print("[NAVIGATE]")
        print('Anchors', [self.robots[x].name for x in anchor_ids])
        print('Robot', self.robots[robot_id].name)

        anchor_positions = np.array([anchor.current_state.to_numpy() for anchor in anchors])

        print("ANCHOR POSITIONS", anchor_positions)

        sr_data = FREQS.SAMPLING_FREQUENCY_62500
        sampling_rate = freq_list[sr_data].value
        
        self.calibrate_sensors(robot_id)
        self.get_robot_by_id(robot_id).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE, timeout=1)

        spk_id = robot_id

        # Start microphone at each robot & enable status notifications (to get xcorr data)
        for mic_id in anchor_ids + [robot_id]:
            # self.make_timesync_slave(mic_id) # Needed to reset everything
            # if mic_id != spk_id:
            #     self.update_mic_parameters(mic_id, sampling_rate=sr_data, left_gain=ROBOT_CROSS_GAIN, right_gain=ROBOT_CROSS_GAIN, use_compression=False, channels=2)
            # else:
            #     self.update_mic_parameters(mic_id, sampling_rate=sr_data, left_gain=ROBOT_SELF_GAIN, right_gain=ROBOT_SELF_GAIN, use_compression=False, channels=2)
            # self.sync_task_create(mic_id, SYNC_TASKS.SYNC_TASK_MIC_START, 0) # Start microphone immediately
            
            if mic_id != spk_id:
                gain = ROBOT_CROSS_GAIN
            else:
                gain = ROBOT_SELF_GAIN

            self.load_tof_profile(robot_id=mic_id, 
                                  sampling_rate=sr_data,
                                  gain=gain,
                                  is_spk=(mic_id == spk_id),
                                  period=spacing//50,
                                  begin_interval=offset//50)
            
            self.stream_xcorr(mic_id, True, use_notifications=True) # Enable stream on xcorr data
            self.get_robot_by_id(mic_id).xcorr_stream.flush() # Discard xcorr data from previous streams
            self.get_robot_by_id(mic_id).xcorr_stream.expected_sequence_number = 0
        
        # # Arm speaker task at robot to track.
        # self.sync_task_create_recurring(spk_id, SYNC_TASKS.SYNC_TASK_SPK_START, spacing//50, offset//50)

        # # Arm mic tasks 50 ms before at all other robots
        # for mic_id in anchor_ids + [robot_id]:
        #     self.sync_task_create_recurring(mic_id, SYNC_TASKS.SYNC_TASK_MIC_RECORD, period=spacing//50, start_interval=(offset)//50)

        plan_node = robot.high_controller
        
        sc = StreamCollector([self.robots[_id].xcorr_stream for _id in (anchor_ids + [robot_id])], frame_width=1)
        status_stream = self.robots[robot_id].status_stream

        # # DEBUGGING ONLY
        # for mic_id in anchor_ids + [robot_id]:
        #     self.enable_recording(mic_id)
        #     # self.enable_mic_notifications(mic_id)
        #     self.robots[mic_id].mic_stream.flush()
        
        # audio = StreamCollector([anchor.mic_stream for anchor in anchors] + [robot.mic_stream], frame_width=int(round(0.05 * sampling_rate)), debug=False).start()

        event_stream = StreamSelector(sc, status_stream)
        
        self.send_milestones(robot_id, [*milestones])
        
        # for milestone in milestones:
        #     print(milestone.point)

        # Wait for calibration signals
        # print("CALIBRATION")
        
        pp = PathPlotter(anchors + [robot])

        print("Triggering")
        
        self.run_tasks(master=spk_id) # Trigger
        # time.sleep(0.5)
        self.nav_begin(robot_id)
        print("TRIGGERED")

        sc.start()
        event_stream.start()
        pp.start()

        tracking = True
        chirps_done = 0
        first_update_received = False
        robot_initial_pos = np.array(robot.current_state.position.point)
        
        while tracking:
            try:
                event = event_stream.get()
                # print(event)

                # XCorr update
                if event[0] == 0:
                    t1 = time.time()
                    
                    xcorr_frame = event[1]
                    
                    if (xcorr_frame == -1).any():
                        print("A robot has dropped an xcorr packet, ignoring ...")
                        continue

                    anchor_offsets = np.array(xcorr_frame[:-1])
                    self_offset = xcorr_frame[-1][0]

                    print('Anchor offsets', anchor_offsets.tolist())
                    print('Self offsets', self_offset)

                    current_milestone = self.robots[robot_id].current_milestone
                    current_action = self.robots[robot_id].current_action

                    vel = np.array([self.robots[robot_id].previous_state.velocity.x, self.robots[robot_id].previous_state.velocity.y])

                    ts = time.time()

                    #### saving the debug info ####
                    plan_node.save_debug_data("old_velocity", vel)
                    plan_node.save_debug_data("current_milestone", current_milestone)
                    plan_node.save_debug_data("current_action", current_action)
                    #### saving the debug info ####

                    te = time.time()
                    print('Time taken to save debug:', te - ts)

                    ts = time.time()
                    
                    mean, velocity, (estimated_angle, confidence) = plan_node.update_offset(sample_offsets=anchor_offsets, 
                                                                        self_offset=self_offset,
                                                                        old_pos=self.robots[robot_id].previous_state.to_numpy(),
                                                                        new_pos=self.robots[robot_id].current_state.to_numpy())

                    te = time.time()
                    print('Time taken to save update offset:', te - ts)

                    ts = time.time()

                    print("Estimated state:", self.robots[robot_id].current_state.tostring())
                    rotation = self.robots[robot_id].current_state.rotation
                    mean = mean.reshape(3, 1)
                    velocity = velocity.reshape(2, 1)

                    flags = []
                    if confidence > 0.8:
                        flags = [MOTION_FLAGS.UPDATE_ANGLE]
                        print("************************* ANGLE CORRECTED *************************")
                    else:
                        estimated_angle = self.robots[robot_id].current_state.rotation

                    estimated_angle = np.rad2deg(estimated_angle)
                    print('Estimate angle:', estimated_angle, ' ({} confidence)'.format(confidence))
                    self.robots[robot_id].current_state = RobotState(position=Point(mean[0][0], mean[1][0]), 
                                                                     rotation=fmod(360 + estimated_angle, 360),
                                                                     velocity=Point(velocity[0][0], velocity[1][0]))
                    # print("Previous state:", self.robots[robot_id].previous_state.tostring())
                    # print("Current state:", self.robots[robot_id].current_state.tostring())
                    
                    # if not first_update_received:
                    #     curr_pos = np.array(robot.current_state.position.point)
                    #     if np.linalg.norm(curr_pos - robot_initial_pos) > 5:
                    #         milestones, back_angle = robot.high_controller.process_action(High_Status.BACK_NAV, # CAREFUL!! THIS MUST BE SET TO BACK OR OUT DEPENDING ON STAGE
                    #                                                                                  landmarks=anchor_positions,
                    #                                                                                  init_pos=robot.current_state.to_numpy())
                    #         self.send_milestones(robot_id, [*milestones], overwrite_current=True)
                    #         print("OVERWRITING MILESTONES")
                        
                    #     first_update_received = True

                    chirps_done += 1

                    print('Current Milesone: ({}, {})'.format(str(current_milestone.x), str(current_milestone.y)))
                    # print('Current Action:', current_action)
                    # print('CURRENT MASTER', self.ts_master)
                    
                    self.update_state_information(robot_id=robot_id,
                                                  current_state=self.robots[robot_id].current_state,
                                                  current_milestone=current_milestone,
                                                  motion_flags=flags)

                    te = time.time()
                    print('Time taken to save update state information:', te - ts)

                    ts = time.time()

                    distances = []
                    for i in range(len(anchors)):
                        distance = extract_distance_single(self_offset, anchor_offsets[i][0])
                        distances.append(distance)
                    
                    pp.update(anchors + [robot], distances + [0])

                    te = time.time()
                    print('Time taken to save update plot:', te - ts)

                    t2 = time.time()
                    
                    print('Time taken this iteration: {}ms'.format(1000*(t2- t1)))
                
                # Status update
                elif event[0] == 1:
                    val = event[1]
                    update_type = val[0]
                    if update_type == STATUS_UPDATE_TYPES.DCS_STATUS_MOTION_CORRECTED:
                        print("Current state:", self.robots[robot_id].current_state.tostring())
                        plan_node.update_middlepoint(val[1])

                    elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_NEXT_MILESTONE:
                        milestone = val[1].position
                        current_milestone = milestone
                        print('[NEXT MILESTONE]: ({}, {})'.format(current_milestone.x, current_milestone.y))
                        plan_node.save_debug_data("now_pos", self.robots[robot_id].current_state.to_numpy())
                        plan_node.save_debug_data("now_milestone", [str(milestone.x), str(milestone.y)])
                    
                    elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_NAV_TRANSITION:
                        a_from = val[1]
                        a_to = val[2]
                        plan_node.update_status(a_to)
                        print("Robot changing action from {} to {}".format(a_from, a_to))
                    elif update_type == STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END:
                        self.update_motion(robot.id, MOTION_CODES.MOTION_BRAKE)
                        tracking = False

                # elif event[0] == 2:
                #     frame = event[1]
                #     write_audio_file(f'recordings/track_recording{i}_{sampling_rate}.wav', frame.astype(np.int16), sr=sampling_rate)
                #     i += 1
                #     print("Saved")
                # ???
                else:
                    assert 1, "Code should not go here"
            except KeyboardInterrupt:
                robot.high_controller.loc_node.save_debug_data()
                break
        
        # Stop stream selector so as not to consume status codes
        event_stream.stop()
        
        if accurate_end_pos:
            self.update_motion(robot.id, code=MOTION_CODES.MOTION_ROTATE_ANGLE, angle=270)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            
            N = 5
            pos1 = self._get_mean_pos(robot, sc, N, plan_node)
            print("Measured pos 1", pos1)

            # input("Waiting for keypress...")
            FINE_TUNE_SPEED = 120
            FINE_TUNE_DUR = 1.5

            self.calibrate_sensors(robot.id)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE, timeout=1)

            # Move forward for 1 second
            self.update_motion(robot.id, code=MOTION_CODES.MOTION_FORWARD, duration_us=FINE_TUNE_DUR * 1e6, speed=FINE_TUNE_SPEED)
            
            samples = [pos1]
            
            sc.flush()
            for i in range(int(FINE_TUNE_DUR/(spacing*1e-3)) - 1):
                # XCorr update
                xcorr_frame = sc.get()
                
                if (xcorr_frame == -1).any():
                    print("A robot has dropped an xcorr packet, ignoring ...")
                    continue

                anchor_offsets = np.array(xcorr_frame[:-1])
                self_offset = xcorr_frame[-1][0]

                mean, velocity, (estimated_angle, confidence) = plan_node.update_offset(sample_offsets=anchor_offsets, 
                                                                                        self_offset=self_offset,
                                                                                        old_pos=self.robots[robot_id].previous_state.to_numpy(),
                                                                                        new_pos=self.robots[robot_id].current_state.to_numpy())
                mean = mean.reshape(3, 1)
        
                # Add position estimate to list of estimates
                samples.append(np.array([mean[0][0], mean[1][0]]))
            
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END, timeout=FINE_TUNE_DUR)
            
            pos2 = self._get_mean_pos(robot, sc, N, plan_node)
            print("Measured pos 2", pos2)

            samples.append(pos2)

            samples = np.array(samples).T
            angle, confidence = estimate_angle(samples[0].reshape(-1, 1), samples[1].reshape(-1, 1))

            print(samples)

            dir = pos2 - pos1

            if confidence > 0.9:
                true_angle = np.rad2deg(angle)
                print("ANGLE", true_angle, "\tCONFIDENCE", confidence)
            else:
                print("******" * 10)
                print("NOT CONFIDENT ENOUGH" * 10)
                print("******" * 10)
                true_angle = np.rad2deg(np.arctan2(dir[1], dir[0]))
                print("ANGLE", np.rad2deg(angle), "\tCONFIDENCE", confidence)

            # input("Waiting for keypress...")

            self.calibrate_sensors(robot.id)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE, timeout=1)

            print("TRUE ANGLE", true_angle)
            
            robot.current_state = RobotState(position=Point(pos2[0], pos2[1]), rotation=true_angle)
            self.update_state_information(robot_id=robot_id,
                                        current_state=RobotState(position=Point(pos2[0], pos2[1]), rotation=true_angle),
                                        motion_flags=[MOTION_FLAGS.FORCE_UPDATE])
            self.await_ble_tx_finished()

            target_pos = np.array([-2, -2])
            approx_speed = np.linalg.norm(dir) / FINE_TUNE_DUR # cm/s

            new_pos = pos2
            tries = 0
            
            while ((new_pos[0] > -1 or new_pos[0] < -3 or new_pos[1] < -4) and tries < 3) or \
                   ((new_pos[0] > 0 or new_pos[0] < -4 or new_pos[1] < -4) and tries > 3):
                dir2 = (target_pos - new_pos)
                
                new_angle = fmod(np.rad2deg(np.arctan2(dir2[1], dir2[0])) + 360, 360)
                duration_us = max([(np.linalg.norm(dir2) / approx_speed), 0.1]) * 1e6

                self.update_motion(robot.id, code=MOTION_CODES.MOTION_ROTATE_ANGLE, angle=new_angle)
                robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
                
                self.update_motion(robot.id, code=MOTION_CODES.MOTION_FORWARD, duration_us=duration_us, speed=FINE_TUNE_SPEED)
                robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

                new_pos = self._get_mean_pos(robot, sc, N, plan_node)
                print("NEW POS", new_pos)
                
                tries += 1

            self.calibrate_sensors(robot.id)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE, timeout=1)

            self.update_state_information(robot_id=robot_id,
                            current_state=RobotState(position=Point(new_pos[0], new_pos[1]), rotation=robot.current_state.rotation),
                            motion_flags=[MOTION_FLAGS.FORCE_UPDATE])
            self.await_ble_tx_finished()

            target_pos = np.array([-3.5, 8])
            dir2 = (target_pos - new_pos)

            # Rotate so that you face backwards to (-1.5, 10)
            # target_angle = fmod(np.rad2deg(np.arctan2(dir2[1], dir2[0])) + 180, 360)
            target_angle = fmod(np.rad2deg(np.arctan2(dir2[1], dir2[0])), 360)
            
            self.update_motion(robot.id, code=MOTION_CODES.MOTION_ROTATE_ANGLE_PRECISE, angle=target_angle)
            robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            
            print("CURRENT ANGLE", robot.current_state.rotation)
            print("TRUE ANGLE", true_angle)
            print("NEW ANGLE", new_angle)
            print("TARGET ANGLE", target_angle)
            print("Approx speed", approx_speed)
            
            print("FINAL POS:", new_pos, "FINAL ROT:", fmod(target_angle, 360))
            pp.update(anchors + [robot])

        sc.stop()

        self.stop_tasks()

        pp.stop()

        # Disables mic notifications & resets xcorr sequence number.
        # Admittedly, this isn't the best place to do it.
        for _id in self.robots:
            print(self.robots[_id].name)
            self.stream_xcorr(_id, enable=False)
            self.disable_mic_notifications(_id)

    def get_platform_sequence(self):
        """
        Orders the robots to localize themselves within the base station and
        returns a list of robot ids, the order in which they are positioned in the platform
        such that the i-th robot is located at the i-th marker.        
        """
        TIMEOUT = 2
        SPEED = 160 # Speed to move within platform
        sequence = np.arange(len(self.robots)).tolist()
        
        print("CALIBRATING SENSORS & ENABLING COLLISIONS")
            
        for _id in self.robots:
            self.update_motion(_id, MOTION_CODES.MOTION_BACKWARD, duration_us=150e3, speed=SPEED)
        
        for _id in self.robots:
            self.get_robot_by_id(_id).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
        
        for _id in self.robots:
            self.calibrate_sensors(_id)
        
        for _id in self.robots:
            self.get_robot_by_id(_id).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)

        print("DONE")

        sequences = [PlatformSequence(_id, i) for i, _id in enumerate(self.robots.keys())]

        L = list(self.robots.keys()) # Robots that should be moved
        
        # moving_robot = self.get_robot_by_id(remaining[0]) # Get random robot

        def choose_random_incomplete_sequence(seqs: list):
            incomplete_sequences = [seq for seq in seqs if seq.is_complete() == False]
            
            if len(incomplete_sequences) == 0:
                raise Exception # All sequence are incomplete
            
            idx = np.random.randint(len(incomplete_sequences))
            return incomplete_sequences[idx]

        # current_sequence = [x for x in sequences if self.get_robot_by_name('B').id in x.get()][0]
        current_sequence = choose_random_incomplete_sequence(sequences)

        ir_stop = []
        
        IR_STATUS_NONE = 0
        direction = MOTION_CODES.MOTION_FORWARD_PULSE

        while len(sequences) > 1:
            
            print("Current sequences:")
            for seq in sequences:
                for x in seq.sequence:
                    print(self.get_robot_by_id(x).name, end=' ')
                if seq.is_complete():
                    print('[COMPLETED]', end='')
                print()

            if current_sequence.is_complete():
                try:
                    current_sequence = choose_random_incomplete_sequence(sequences)
                except:
                    # This should only happen when the only two remaining sequences
                    # are a lone robot at the last marker, and all other robots
                    # forming their own sequence 
                    assert len(sequences) == 2
                    sequences = sorted(sequences, key = lambda x: len(x.sequence))
                    assert len(sequences[0].sequence) == 1 and len(sequences[1].sequence) == len(self.robots) - 1
                    sequences[0].join_sequence(sequences[1])
                    sequences = [sequences[0]]
                    break
                    
            moving_robot = self.get_robot_by_id(current_sequence.tail())

            print("Current moving robot:", moving_robot.name)

            # input('Waiting for keypress ...')

            self.read_ir_status(moving_robot.id)
            if moving_robot.ir_status == IR_STATUS_NONE:
                print("ROBOT IS AT END")

                current_sequence.complete()
                
                continue
            
            collision_enabled_ids = [seq.head() for seq in sequences if seq.head() not in current_sequence.sequence]
            for _id in collision_enabled_ids:
                self.enable_collision_detection(_id)
            
            print("COLLISION ROBOTS", [self.get_robot_by_id(x).name for x in collision_enabled_ids])
            
            # Disable collision detection for moving robot
            self.disable_collision_detection(moving_robot.id)

            for _id in self.robots:
                self.robots[_id].status_stream.flush()

            # moving_robot.status_stream.flush()

            selector = StreamSelector(moving_robot.status_stream, *[self.robots[x].status_stream for x in collision_enabled_ids])
            selector.flush()
            selector.start()
            
            self.enable_ir_status_stream(moving_robot.id)
            
            print("MOVING")
            self.update_motion(moving_robot.id, direction, 1e18, speed=SPEED, controlled=False)

            last_robot = None
            
            # Wait until we get a collision detected update from one of the robots
            while True:
                try:
                    event = selector.get(timeout=TIMEOUT)
                except queue.Empty:
                    self.update_motion(moving_robot.id, MOTION_CODES.MOTION_BRAKE)
                    event = None
                    found = False
                    while not found:
                        if direction == MOTION_CODES.MOTION_FORWARD_PULSE:
                            self.update_motion(moving_robot.id, MOTION_CODES.MOTION_BACKWARD, duration_us=100e3, speed=170)
                            moving_robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
                            time.sleep(0.2)
                            self.update_motion(moving_robot.id, MOTION_CODES.MOTION_FORWARD, duration_us=300e3, speed=500)
                        else:
                            self.update_motion(moving_robot.id, MOTION_CODES.MOTION_FORWARD, duration_us=100e3, speed=170)
                            moving_robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
                            time.sleep(0.2)
                            self.update_motion(moving_robot.id, MOTION_CODES.MOTION_BACKWARD, duration_us=300e3, speed=500)
                        moving_robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
                        time.sleep(0.4)
                        
                        while not selector.empty():
                            event = selector.get()
                            print('EEE', event)
                            if event[1][0] == STATUS_UPDATE_TYPES.DCS_STATUS_COLLISION_DETECTED or\
                              (event[0] == 0 and event[1][0] == STATUS_UPDATE_TYPES.DCS_IR_STATUS_UPDATED and event[1][1] == IR_STATUS_NONE):
                                print("FOUND EXIT CONDITION")
                                found = True
                                break
                
                print('Event', event)
                if event[1][0] == STATUS_UPDATE_TYPES.DCS_STATUS_COLLISION_DETECTED:
                    # Brake the currently moving robot
                    self.update_motion(moving_robot.id, MOTION_CODES.MOTION_BRAKE)

                    # Get robot that has felt the collision
                    collided_id = collision_enabled_ids[event[0]-1]

                    print("FOUND COLLISION BETWEEN {} and {}", moving_robot.name, self.get_robot_by_id(collided_id).name)

                    collided_sequence_idx = -1
                    for idx, sequence in enumerate(sequences):
                        if collided_id == sequence.head():
                            collided_sequence_idx = idx
                            break
                    
                    print(current_sequence.idx)
                    print(sequences[collided_sequence_idx].idx)
                    
                    # Collided sequence is in front of current sequence
                    current_sequence.join_sequence(sequences[collided_sequence_idx])

                    # Remove the collided sequence
                    sequences = sequences[:collided_sequence_idx] + sequences[collided_sequence_idx + 1:]

                    # # Add an edge between moving robot and collided robot
                    # # Edge (a, b) exists if a is in front of b
                    # edges.append([collided_id, moving_robot.id])
                    
                    # # Set the moving robot to be the one that felt the collision
                    # self.disable_collision_detection(collided_id)

                    # if collided_id in remaining:
                    #     moving_robot = self.get_robot_by_id(collided_id)
                    # elif len(remaining) > 0:
                    #     moving_robot = self.get_robot_by_id(remaining[0])
                    # # time.sleep(1)
                    break
                
                elif event[0] == 0 and event[1][0] == STATUS_UPDATE_TYPES.DCS_IR_STATUS_UPDATED:
                    print(event[1][1])
                    if event[1][1] == IR_STATUS_NONE:
                        # Stop immediately
                        self.update_motion(moving_robot.id, MOTION_CODES.MOTION_BRAKE)
                        
                        # Move back if robot passed marker
                        self.read_ir_status(moving_robot.id)
                        print(moving_robot.ir_status)
                        if moving_robot.ir_status != IR_STATUS_NONE:
                            self.update_motion(moving_robot.id, MOTION_CODES.MOTION_BACKWARD, duration_us=500e3)
                            moving_robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
                            self.enter_base(moving_robot.id, from_base=True, depth=-1)
                            moving_robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
                        
                        current_sequence.complete()

                        ir_stop.append(moving_robot.id)

                        # ends.append(moving_robot.id)

                        # assert len(ends) in [1, 2] # Can only discover one or both ends

                        # if len(remaining) > 0:
                        #     moving_robot = self.get_robot_by_id(remaining[0])
                        # else:
                        #     last_robot = moving_robot.id
                        
                        break

            self.disable_ir_status_stream(moving_robot.id)
            for _id in collision_enabled_ids:
                self.disable_collision_detection(_id)
            selector.stop()
        
        sequence = sequences[0].get()[::-1]

        print("SEQUENCE FOUND", [self.robots[x].name for x in sequence])
        if len(sequence) < len(self.robots):
            print("Sequence search failed, retrying ...")
            return self.get_platform_sequence()

        print("*** DONE")
        # self.robots[sequence[0]].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

        for x in self.robots:
            self.robots[x].status_stream.flush()
            self.disable_collision_detection(x)
            self.disable_ir_status_stream(x)

        self.read_ir_status(sequence[0])
        print(self.get_robot_by_id(sequence[0]).name, self.get_robot_by_id(sequence[0]).ir_status)
        self.read_ir_status(sequence[-1])

        print(self.get_robot_by_id(sequence[-1]).name, self.get_robot_by_id(sequence[-1]).ir_status)
        
        if sequence[0] not in ir_stop and self.robots[sequence[0]].ir_status != IR_STATUS_NONE:
            print("Moving robot {} to front".format(self.robots[sequence[0]].name))
            self.enter_base(sequence[0], from_base=True, depth=100)
        
        if sequence[-1] not in ir_stop and self.robots[sequence[-1]].ir_status != IR_STATUS_NONE:
            print("Moving robot {} to back".format(self.robots[sequence[-1]].name))
            self.enter_base(sequence[-1], from_base=True, depth=-100)
        
        if sequence[0] not in ir_stop and self.robots[sequence[0]].ir_status != IR_STATUS_NONE:
            self.robots[sequence[0]].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
        
        if sequence[-1] not in ir_stop and self.robots[sequence[-1]].ir_status != IR_STATUS_NONE:
            self.robots[sequence[-1]].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

        print("NEXT")

        # Move all robots except the end to first
        for i in range(1, len(sequence) - 1):
            print("Moving robot {}".format(self.robots[sequence[i]].name))
            self.robots[sequence[i-1]].status_stream.flush()
            self.enable_collision_detection(sequence[i-1])
            self.update_motion(sequence[i], MOTION_CODES.MOTION_FORWARD_PULSE, 1e18, speed=SPEED, controlled=False)
            self.get_robot_by_id(sequence[i-1]).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_COLLISION_DETECTED, timeout=2 * TIMEOUT)
            self.update_motion(sequence[i], MOTION_CODES.MOTION_BRAKE)
            self.disable_collision_detection(sequence[i-1])
            # time.sleep(1)

        # Move all robots to their correct positions
        for i in range(len(sequence) - 2, 0, -1):
            print("Moving robot {}".format(self.robots[sequence[i]].name))
            self.robots[sequence[i+1]].status_stream.flush()
            self.enable_collision_detection(sequence[i+1])
            self.update_motion(sequence[i], MOTION_CODES.MOTION_BACKWARD_PULSE, 1e18, speed=SPEED, controlled=False)
            self.get_robot_by_id(sequence[i+1]).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_COLLISION_DETECTED, timeout=2 * TIMEOUT)
            self.update_motion(sequence[i], MOTION_CODES.MOTION_BRAKE)
            self.disable_collision_detection(sequence[i+1])

            time.sleep(0.2)
            self.read_ir_status(sequence[i])
            print(self.get_robot_by_id(sequence[i]).name, 'IR STATUS', self.get_robot_by_id(sequence[i]).ir_status)
            if self.get_robot_by_id(sequence[i]).ir_status not in [1,2]:
                self.enter_base(sequence[i], from_base=True, depth=1)
                self.robots[sequence[i]].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            
            # time.sleep(0.5)
            time.sleep(1)
        
        for x in self.robots:
            self.robots[x].status_stream.flush()

        self.robots[sequence[0]].state = RobotState(Point(PLATFORM_POSITIONS[0][0], PLATFORM_POSITIONS[0][1]), rotation=0, velocity=Point(0,0))

        for i in range(len(sequence)-1):
            robot_idx = len(sequence) - 1 - i
            platform_idx = len(PLATFORM_POSITIONS) - 1 - i
            pos = PLATFORM_POSITIONS[platform_idx]
            self.robots[sequence[robot_idx]].current_state = RobotState(Point(pos[0], pos[1]), rotation=0, velocity=Point(0,0))

        for x in self.robots:
            self.robots[x].status_stream.flush()

        # Go over one more time to fine-tune positions
        print("Fine tuning ...")
        for i in range(len(sequence)):
            self.read_ir_status(sequence[i])
        
        moving = []
        if self.robots[sequence[0]].ir_status != 0:
            # self.update_motion(sequence[0], MOTION_CODES.MOTION_BACKWARD, 150e3)
            # self.get_robot_by_id(sequence[0]).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            self.enter_base(sequence[0], from_base=True, depth=-1)
            moving.append(sequence[0])
        
        for i in range(1, len(sequence) - 1):
            if self.robots[sequence[i]].ir_status not in [1, 2]: # IR is on white
                self.update_motion(sequence[i], MOTION_CODES.MOTION_BACKWARD, 150e3)
                self.get_robot_by_id(sequence[i]).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
                self.enter_base(sequence[i], from_base=True, depth=1)
                moving.append(sequence[i])

        if self.robots[sequence[-1]].ir_status != 0:
            self.update_motion(sequence[-1], MOTION_CODES.MOTION_BACKWARD, 100e3)
            self.get_robot_by_id(sequence[-1]).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            self.enter_base(sequence[-1], from_base=True, depth=1)
            moving.append(sequence[-1])

        print('Robots undergoing fine tuning', moving)

        for m in moving:
            self.robots[m].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

        print("Position fine tune complete.")

        for i in range(len(sequence)):
            print("Robot {} is at position {}".format(self.robots[sequence[i]].name, self.robots[sequence[i]].current_state.to_numpy()))

        # Proceed

        # # For debugging purposes
        # sequence = [self.get_robot_by_name('D').id, self.get_robot_by_name('A').id, self.get_robot_by_name('B').id, self.get_robot_by_name('C').id]
        
        for seq_id, robot_id in enumerate(sequence):
            # Set the sequence id of the robot
            # self.robots[robot_id].status_stream.start()
            self.get_robot_by_id(robot_id).initialize_high_controller(seq_id, self.global_planner)
        
        return sequence

    def move_out(self, robot, anchors, chirp_spacing, chirp_offset):
        """
        Moves robot outside the platform using anchors to localize
        @param robot: robot to move out
        @param anchors: robots to use as anchors for localization
        """
        
        robot_id = robot.id
        print("Now moving robot: {}".format(self.robots[robot_id].name))
        self.robots[robot_id].status_stream.start()

        # Tweak
        self.update_motion(robot_id, MOTION_CODES.MOTION_BACKWARD, 200e3)
        self.robots[robot_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

        # If robot is not at the front, move it to the front
        self.read_ir_status(robot.id)

        # ans = 'y'
        # while ans == 'y':
        #     print("IR_STATUS", robot.ir_status)
        #     ans='x'
        #     while ans not in ['y', 'n']:
        #         ans = input('Again?(y/n)')
            
            # self.read_ir_status(robot.id)

        if robot.ir_status != 0:
            self.enter_base(robot.id, from_base=True, depth=100)
            self.robots[robot_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

        print("STATE BEFORE:", self.robots[robot_id].current_state.tostring())
        self.robots[robot_id].current_state = RobotState(position=Point(PLATFORM_POSITIONS[0][0], PLATFORM_POSITIONS[0][1] + 5), rotation=90, velocity=Point(0,0))
        print("STATE AFTER:", self.robots[robot_id].current_state.tostring())

        # Calibrate sensors on platform
        self.calibrate_sensors(robot_id)
        self.robots[robot_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE, timeout=1)
        print("CALIBRATE:", self.robots[robot_id].current_state.tostring())

        # Move forward for 500 ms to just about qqleave the base
        self.update_motion(robot_id, MOTION_CODES.MOTION_FORWARD, 300e3, controlled=False, speed=400)
        self.robots[robot_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
        self.update_motion(robot_id, MOTION_CODES.MOTION_FORWARD, 300e3)
        self.robots[robot_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
        self.update_motion(robot_id, MOTION_CODES.MOTION_BRAKE)
        time.sleep(0.3) # Wait for robot to come to a stop

        # Calibrate sensors outside platform
        self.calibrate_sensors(robot_id)

        if len(anchors) == 1:
            # Tweak
            self.update_motion(anchors[0].id, MOTION_CODES.MOTION_FORWARD, 200e3)
            self.robots[anchors[0].id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

        self.enter_base(anchors[0].id, from_base=True, depth=100)
        self.robots[anchors[0].id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
        self.robots[anchors[0].id].current_state = RobotState(position=Point(PLATFORM_POSITIONS[0][0], PLATFORM_POSITIONS[0][1]), rotation=90, velocity=Point(0,0))
        
        # if len(anchors) > 1:
        #     self.robots[anchors[0].id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
        #     self.robots[anchors[0].id].current_state = RobotState(position=Point(PLATFORM_POSITIONS[0][0], PLATFORM_POSITIONS[0][1]), rotation=90, velocity=Point(0,0))

        if len(anchors) == 3:
            self.enter_base(anchors[1].id, from_base=True, depth=3)
            self.robots[anchors[1].id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            self.robots[anchors[1].id].current_state = RobotState(position=Point(PLATFORM_POSITIONS[1][0], PLATFORM_POSITIONS[1][1]), rotation=90, velocity=Point(0,0))

        if len(anchors) == 4:
            self.enter_base(anchors[1].id, from_base=True, depth=2)
            self.robots[anchors[1].id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            self.robots[anchors[1].id].current_state = RobotState(position=Point(PLATFORM_POSITIONS[2][0], PLATFORM_POSITIONS[2][1]), rotation=90, velocity=Point(0,0))

            self.enter_base(anchors[2].id, from_base=True, depth=1)
            self.robots[anchors[2].id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            self.robots[anchors[2].id].current_state = RobotState(position=Point(PLATFORM_POSITIONS[4][0], PLATFORM_POSITIONS[4][1]), rotation=90, velocity=Point(0,0))
        
        if len(anchors) == 5:
            self.enter_base(anchors[1].id, from_base=True, depth=1)
            self.robots[anchors[1].id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            self.robots[anchors[1].id].current_state = RobotState(position=Point(PLATFORM_POSITIONS[2][0], PLATFORM_POSITIONS[2][1]), rotation=90, velocity=Point(0,0))

        anchor_positions = np.array([anchor.current_state.to_numpy() for anchor in anchors])
        anchors = sorted(anchors, key = lambda x : x.name)

        # # Calibrate sensors outside platform
        # self.calibrate_sensors(robot_id)
        self.robots[robot_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)
        
        self.update_state_information(robot_id=robot_id,
                                        current_state=self.robots[robot_id].current_state,
                                        motion_flags=[MOTION_FLAGS.FORCE_UPDATE])
        print("INITIAL STATE:", self.robots[robot_id].current_state.tostring())
        
        anchor_positions = np.array([anchor.current_state.to_numpy() for anchor in anchors])
        print('ANCHOR POSITIONS', anchor_positions)

        anchor_positions = anchor_positions[:, :2]

        # Obtain milestones to go to starting position
        milestones, angle = self.robots[robot_id].high_controller.process_action(High_Status.OUT_NAV, landmarks = anchor_positions)
        print(len(milestones), self.robots[robot_id].high_controller.Current_status)
        self.navigate(milestones[1:], self.robots[robot_id], anchors, spacing = chirp_spacing, offset = chirp_offset) # Move to position

        # print("(BEFORE ROTATION) CURRENT STATE", robot.current_state.tostring())
        # input("BEFORE ROTATION. Press any key ...")
        
        # Rotate to correct angle
        self.update_motion(robot_id, MOTION_CODES.MOTION_ROTATE_ANGLE, angle=angle)
        self.robots[robot_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

        self.update_motion(robot_id, MOTION_CODES.MOTION_ROTATE_ANGLE_PRECISE, angle=angle)
        self.robots[robot_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

        print("(AFTER ROTATION) CURRENT STATE", robot.current_state.tostring())

        self.get_robot_by_id(robot_id).current_state.rotation = angle

        return angle

    def enable_edge_detection(self, robot_id):
        self._check_robot(robot_id)

        data = GATTBuilder(robot_id).write(SSS_CONTROL_CHAR_UUID).add_params(SSS_IR_ENABLE_EDGE_DETECTION).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def disable_edge_detection(self, robot_id):
        self._check_robot(robot_id)
        
        data = GATTBuilder(robot_id).write(SSS_CONTROL_CHAR_UUID).add_params(SSS_IR_DISABLE_EDGE_DETECTION).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)

    def initialize_global_planner(self):
        self.global_planner = Path_Planner(len(self.robots) - 1)

    def robots_distribute(self, chirp_spacing=200, chirp_offset=500, enable_collision_detection=False, probe_power_consumption=False, expand_in_background=False):
        """
        Distributes robots on the platform outside the platform to form the distribute microphone array.
        """

        if probe_power_consumption:
            power_start = self.probe_battery_information()

        self.initialize_global_planner()
        
        self.sequence = self.get_platform_sequence()

        if probe_power_consumption:
            power_sequencing_done = self.probe_battery_information()

        # input("Sequence ready. Press any key to start:")

        power_stages = []
        
        expansion_threads = []
        
        # Go over all robots except last (leave one in the platform)
        for idx, _id in enumerate(self.sequence[:-1]):
            # Move robot to correct starting position & rotation

            # Choose anchors to be the robots that come after in platform sequence
            anchors = [self.robots[x] for x in self.sequence[idx+1:]]
            move_out_angle = self.move_out(self.robots[_id], anchors=anchors, chirp_spacing = chirp_spacing, chirp_offset = chirp_offset)
            # input('Reached destination. Press any key to continue...')

            def expand(_robot_id, _angle):
                # Move robot for 3 seconds to expand array
                self.enable_edge_detection(_robot_id)

                self.calibrate_sensors(_robot_id)
                self.get_robot_by_id(_robot_id).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)

                self.update_motion(_robot_id, MOTION_CODES.MOTION_FORWARD, 10e6, speed=200)

                if enable_collision_detection:
                    time.sleep(0.5)
                    self.enable_collision_detection(_robot_id, moving=True)

                # Wait for robot nav end signal
                self.robots[_robot_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
                self.disable_edge_detection(_robot_id)
                
                if enable_collision_detection:
                    self.disable_collision_detection(_robot_id)

                # Rotate by 180 degrees (in order to point inwards)
                _angle = _angle + 180
                self.update_motion(_robot_id, MOTION_CODES.MOTION_ROTATE_ANGLE, angle=_angle)
                self.robots[_robot_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            if expand_in_background:
                t = threading.Thread(target=expand, args=(_id, move_out_angle))
                expansion_threads.append(t)
                t.start()
            else:
                expand(_id, move_out_angle)

            if probe_power_consumption:
                power_stage = self.probe_battery_information()
                power_stages.append(power_stage)

        if expand_in_background:
            for thread in expansion_threads:
                thread.join()

        # Give last robot a little nudge so it's not right on the last anchor
        self.update_motion(self.sequence[-1], MOTION_CODES.MOTION_BACKWARD, 200e3)
        self.get_robot_by_id(self.sequence[-1]).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

        print("Done")

        if probe_power_consumption:
            power_end = self.probe_battery_information()

            power_consumption_info = dict(power_start=power_start,
                                          power_sequencing_done=power_sequencing_done,
                                          power_stages=power_stages,
                                          power_end=power_end)

            return power_consumption_info

    def robots_contract(self, robot_ids: list, center_robot_id: int, thresh: int =50, spacing: int = 200, offset: int = 500):
        """
        Moves robots in a straight line towards a given target robot with id center_robot_id 
        Blocks until all robots are within threshold cm from the center robot
        Also assigns approximate positions based on distance from center robot and current angle
        """
        anchor_ids = robot_ids
        robot_id = center_robot_id
        spk_id = robot_id

        self.robots[robot_id].moving = False
        for _id in anchor_ids:
            self.robots[_id].moving = True

        sr_data = FREQS.SAMPLING_FREQUENCY_62500
        sampling_rate = freq_list[sr_data].value

        # spk_id = robot_id

        # # Start microphone at each robot & enable status notifications (to get xcorr data)
        # for mic_id in anchor_ids + [robot_id]:
        #     self.make_timesync_slave(mic_id) # Needed to reset everything
        #     if mic_id != spk_id:
        #         self.update_mic_parameters(mic_id, sampling_rate=sr_data, left_gain=0x40, right_gain=0x40, use_compression=False)
        #     else:
        #         self.update_mic_parameters(mic_id, sampling_rate=sr_data, left_gain=0x14, right_gain=0x14, use_compression=False)
        #     self.sync_task_create(mic_id, SYNC_TASKS.SYNC_TASK_MIC_START, 0) # Start microphone immediately
        #     self.stream_xcorr(mic_id, True) # Enable stream on xcorr data
        #     self.robots[mic_id].xcorr_stream.flush() # Discard xcorr data from previous streams        
        #     self.get_robot_by_id(mic_id).xcorr_stream.expected_sequence_number = 0
        
        # # Arm speaker task at robot to track.
        # self.sync_task_create_recurring(spk_id, SYNC_TASKS.SYNC_TASK_SPK_START, spacing//50, offset//50)

        print('Robots', [self.robots[x].name for x in robot_ids])
        print('Center', self.robots[center_robot_id].name)
        
        for mic_id in anchor_ids:
            self.get_robot_by_id(mic_id).status_stream.flush()
            self.calibrate_sensors(mic_id)

        # Start microphone at each robot & enable status notifications (to get xcorr data)
        for mic_id in self.robots.keys():
            # self.make_timesync_slave(mic_id) # Needed to reset everything
            # self.update_mic_parameters(mic_id, sampling_rate=sr_data, use_compression=False)
            # if mic_id != spk_id:
            #     self.update_mic_parameters(mic_id, sampling_rate=sr_data, left_gain=ROBOT_CROSS_GAIN, right_gain=ROBOT_CROSS_GAIN, use_compression=False, channels=2)
            # else:
            #     self.update_mic_parameters(mic_id, sampling_rate=sr_data, left_gain=ROBOT_SELF_GAIN, right_gain=ROBOT_SELF_GAIN, use_compression=False, channels=2)
            # self.sync_task_create(mic_id, SYNC_TASKS.SYNC_TASK_MIC_START, 0) # Start microphone immediately
            
            if mic_id != spk_id:
                gain = ROBOT_CROSS_GAIN
            else:
                gain = ROBOT_SELF_GAIN

            self.load_tof_profile(robot_id=mic_id, 
                                  sampling_rate=sr_data,
                                  gain=gain,
                                  is_spk=(mic_id == spk_id),
                                  period=spacing//50,
                                  begin_interval=offset//50)

            self.stream_xcorr(mic_id, True) # Enable stream on xcorr data
            self.robots[mic_id].xcorr_stream.flush() # Discard xcorr data from previous streams
            self.get_robot_by_id(mic_id).xcorr_stream.expected_sequence_number = 0
        
        for mic_id in anchor_ids:
            self.robots[mic_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)

        print("Microphone requests sent")
        
        # # Arm speaker task at robot to track.
        # self.sync_task_create_recurring(spk_id, SYNC_TASKS.SYNC_TASK_SPK_START, spacing//50, offset//50)

        # # Arm mic tasks 50 ms before at all other robots
        # for mic_id in anchor_ids + [robot_id]:
        #     self.sync_task_create_recurring(mic_id, SYNC_TASKS.SYNC_TASK_MIC_RECORD, period=spacing//50, start_interval=(offset)//50)

        sc = StreamCollector([self.robots[_id].xcorr_stream for _id in (robot_ids + [center_robot_id])], frame_width=1)

        FINISH_FLAG = False
        FLAG_LIST = [False for _id in robot_ids]
        print("Begin to robots_contract process ................")

        for idx, _id in enumerate(robot_ids):
            self.robots[_id].high_controller.process_action(High_Status.BACK_FORWARD)

        self.run_tasks(master=spk_id) # Trigger
        print("TRIGGERED")

        sc.start()

        print("READY")
        i = 0
        
        while not FINISH_FLAG:
            try:
                xcorr_frame = sc.get()

                # if i < 5:contract(
                #     i += 1
                #     continue
                
                if (xcorr_frame == -1).any():
                    print("A robot has dropped an xcorr packet, ignoring ...")
                    continue

                anchor_offsets = np.array(xcorr_frame[:-1])
                self_offset = xcorr_frame[-1][0]

                print('Anchor offsets', anchor_offsets.tolist())
                print('Self offsets', self_offset)

                braked = []
                
                for idx, _id in enumerate(robot_ids):
                    ARRIVE_FLAG = self.robots[_id].high_controller.monitor_back_range(anchor_offsets[idx, 0], self_offset)
                    # print(self.get_robot_by_id(_id).name, 'Arrived?', ARRIVE_FLAG)
                    if i > 0:
                        if ARRIVE_FLAG:
                            print("Braking robot", self.get_robot_by_id(_id).name)
                            self.update_motion(_id, MOTION_CODES.MOTION_BRAKE)
                            braked.append(_id)
                            FLAG_LIST[idx] = True
                    if i == 0:
                        if not ARRIVE_FLAG:
                            self.update_motion(_id, MOTION_CODES.MOTION_FORWARD, duration_us=100e6, speed=300)
                        else:
                            FLAG_LIST[idx] = True

                for _id in braked:
                    self.get_robot_by_id(_id).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

                FINISH_FLAG = True
                for idx, _id in enumerate(robot_ids):
                    if FLAG_LIST[idx] == False:
                        FINISH_FLAG = False
                        break

                i += 1
            except KeyboardInterrupt:
                break                
        
        sc.stop()
        self.stop_tasks()

        # Disables mic notifications & resets xcorr sequence number.
        # Admittedly, this isn't the best place to do it.
        for _id in self.robots.keys():
            print(self.robots[_id].name)
            self.stream_xcorr(_id, enable=False)
            self.disable_mic_notifications(_id)
        
        for _id in self.robots.keys():
            self.update_motion(_id, MOTION_CODES.MOTION_BRAKE)
            self.get_robot_by_id(_id).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

    def load_tof_profile(self, robot_id: int, sampling_rate: FREQS, gain: int, is_spk: bool, period: int, begin_interval: int, end_interval: int=0):
        clk_freq = freq_list[sampling_rate].div32
        decimation = freq_list[sampling_rate].decimation_factor
        ratio = freq_list[sampling_rate].ratio

        self.robots[robot_id].master = True
        self.robots[robot_id].slave = False
        
        self.robots[robot_id].mic_stream.set_sampling_rate(sampling_rate)
        self.robots[robot_id].mic_stream.set_sampling_rate(sampling_rate)
        self.get_robot_by_id(robot_id).mic_stream.channels = 2
        self.get_robot_by_id(robot_id).xcorr_stream.channels = 2

        data = [DCS_COMMAND_LOAD_PROFILE,
                TASK_PROFILE.TASK_PROFILE_LOCALIZATION,
                ((clk_freq>>24)& 0xFF),
                ((clk_freq>>16) & 0xFF),
                ((clk_freq>>8) & 0xFF),
                (clk_freq) & 0xFF,
                decimation,
                ratio,
                gain,
                is_spk,
                ((period>>8) & 0xFF),
                (period) & 0xFF,
                ((begin_interval>>24) & 0xFF),
                ((begin_interval>>16) & 0xFF),
                ((begin_interval>>8) & 0xFF),
                (begin_interval) & 0xFF,
                ((end_interval>>24) & 0xFF),
                ((end_interval>>16) & 0xFF),
                ((end_interval>>8) & 0xFF),
                (end_interval) & 0xFF]

        data = GATTBuilder(robot_id).write(DCS_COMMAND_CHAR_UUID).add_params(*data).get()
        self.usb_stream.send(PACKET_TYPES.USB_BLE_COMMAND, data)        

    def record_chirps(self, spk_robot, mic_robots, iterations, spacing=150, offset=500, cross_gain=ROBOT_CROSS_GAIN):
        for _id in self.robots:
            self.robots[_id].moving = False

        sr_data = FREQS.SAMPLING_FREQUENCY_62500
        sampling_rate = freq_list[sr_data].value

        spk_id = spk_robot.id
        mic_ids = [x.id for x in mic_robots]

        print('Robot {} is now chirping!'.format(self.robots[spk_id].name))

        t1 = time.time()
        # Start microphone at each robot & enable status notifications (to get xcorr data)
        for mic_id in mic_ids:
            # self.make_timesync_slave(mic_id) # Needed to reset everything
            
            # if mic_id != spk_id:
            #     self.update_mic_parameters(mic_id, sampling_rate=sr_data, left_gain=cross_gain, right_gain=cross_gain, use_compression=False, channels=2)
            # else:
            #     self.update_mic_parameters(mic_id, sampling_rate=sr_data, left_gain=ROBOT_SELF_GAIN, right_gain=ROBOT_SELF_GAIN, use_compression=False, channels=2)
            
            # # Start microphone immediately
            # self.sync_task_create(mic_id, SYNC_TASKS.SYNC_TASK_MIC_START, 0)
            
            # # Recorde every spacing intervals
            # self.sync_task_create_recurring(mic_id, SYNC_TASKS.SYNC_TASK_MIC_RECORD, period=spacing//50, start_interval=offset//50, end_interval=offset//50 + (spacing//50) * iterations )
            
            # self.stream_xcorr(mic_id, True) # Enable stream on xcorr data
            
            if mic_id != spk_id:
                gain = cross_gain
            else:
                gain = ROBOT_SELF_GAIN 
            
            self.load_tof_profile(robot_id=mic_id, sampling_rate=sr_data, gain=gain, is_spk=(mic_id == spk_id), period=spacing//50, begin_interval=offset//50, end_interval=(offset + iterations * spacing - 1)//50 + 1)
            self.stream_xcorr(mic_id, True)
            self.robots[mic_id].xcorr_stream.flush() # Discard xcorr data from previous streams

        # # Arm task at central robot
        # self.sync_task_create_recurring(spk_id, SYNC_TASKS.SYNC_TASK_SPK_START, period = spacing//50, start_interval=offset//50, end_interval=(offset + iterations * spacing - 1)//50 + 1)

        print('Put speak tasks')

        spk_robot_order = 0
        for i in range(len(mic_ids)):
            if spk_id == mic_robots[i].id:
                spk_robot_order = i

        sc = StreamCollector([self.robots[_id].xcorr_stream for _id in mic_ids], frame_width=1)
        sample_offsets = np.zeros((iterations, len(mic_ids)))

        print('Triggering')
        self.run_tasks(master=spk_id) # Trigger
        sc.start()

        t2 = time.time()

        print('time taken to send all mics', t2 - t1)

        print('Triggered')
        
        for i in range(iterations):
            try:
                sample_offsets[i] = sc.get()[:, 0]
                
                # TODO: REMOVE
                # sample_offsets[i][0] = 375
                sample_offsets[i][spk_robot_order] = 375

                print("Received frame {} of {}.".format(i+1, iterations))
            except KeyboardInterrupt:
                break
        self.stop_tasks()
        sc.stop()
        
        # Disables mic notifications & resets xcorr sequence number.
        # Admittedly, this isn't the best place to do it.
        for _id in self.robots.keys():
            self.stream_xcorr(_id, enable=False)
            self.disable_mic_notifications(_id)

        return sample_offsets

    def localize_all(self, center_id, iterations=10, cross_gain=ROBOT_CROSS_GAIN):
        """
        Localizes all robots given a moving robot in the station        
        """

        positions = PLATFORM_POSITIONS

        # positions = np.array([[-3.6, 15.3],
        #                       [-3.6, 11.2],])

        loc_node = Pairwise_Node(len(self.robots), positions.shape[0], True)  # Robot_num, anchor_num
        
        # FOR DEBUGGING ONLY
        # self.sequence = np.arange(len(self.robots)).tolist()
        
        # Read IR status of center_id
        self.read_ir_status(center_id)

        # If robot is not on last marker, move it to the last marker
        if self.robots[center_id].ir_status != 0:
            self.update_motion(center_id, MOTION_CODES.MOTION_BACKWARD, duration_us=200e3)
            self.robots[center_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            # Wait for robot to move to final position
            self.enter_base(center_id, from_base=True, depth = 100)
            self.robots[center_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

        data = {'experiments':[], 'anchors':positions.tolist()}

        # Sort robots by name
        sorted_robots = sorted([self.robots[i] for i in self.robots], key=lambda x: x.name)
        center_robot = self.robots[center_id]

        for i in range(positions.shape[0]):
            results = self.record_chirps(center_robot, sorted_robots, iterations, cross_gain=cross_gain)

            exp = {'speaker':center_robot.name, 'sample_offsets':results.tolist()}
            data['experiments'].append(exp)
            
            # Update 1
            loc_node.update_anchor_chirp(positions[i], results, sorted_robots.index(center_robot))
            
            if i < positions.shape[0] - 1:
                self.update_motion(center_id, MOTION_CODES.MOTION_BACKWARD, duration_us=200e3)
                self.robots[center_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
                # Move to next position
                self.enter_base(center_id, from_base=True, depth = -1)
                self.robots[center_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
        
        # Go over all robots
        for i in range(len(sorted_robots)):
            if sorted_robots[i].name == center_robot.name:
                continue
            
            results = self.record_chirps(sorted_robots[i], sorted_robots, iterations, cross_gain=cross_gain)
            
            exp = {'speaker':sorted_robots[i].name, 'sample_offsets':results.tolist()}
            data['experiments'].append(exp)
            
            # Update 2
            pos_robot = sorted_robots[i].current_state.position.point
            pos_robot = np.array(pos_robot)
            print('Res', results.shape)
            print('Pos', pos_robot.shape)
            print('Robot {} position: ', sorted_robots[i].name, pos_robot)
            print(i)
            loc_node.update_out_robot_chirp(pos_robot, results, i)

        positions, _ = loc_node.do_localize()
        
        import matplotlib.pyplot as plt
        plt.close('all')

        with open('DEBUG/pairwise_localization_debug.json', 'w') as f:
            json.dump(data, f)

        for i in range(len(sorted_robots)):
            sorted_robots[i].current_state.position = Point(positions[i][0], positions[i][1])   
            print("Robot {} is at position {}".format(sorted_robots[i].name, sorted_robots[i].current_state.position.point))

    def move_robot_in_base(self, robot_id, old_pos, new_pos):
        robot = self.get_robot_by_id(robot_id)
        robot.status_stream.flush()

        self.update_motion(robot_id, MOTION_CODES.MOTION_FORWARD, 200e3)
        robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
        
        self.enter_base(robot_id, from_base=True, depth=old_pos - new_pos)
        robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
        robot.current_state = RobotState(position=Point(PLATFORM_POSITIONS[new_pos][0], PLATFORM_POSITIONS[new_pos][1]), rotation=0, velocity=Point(0,0))

    def robots_recall(self, probe_power_consumption = False):
        """
        Orders the robots to go back to base
        """
        if self.global_planner is None:
            self.initialize_global_planner()

            # FOR DEBUGGING PURPOSES
            self.get_robot_by_name("B").current_state = RobotState(position=Point(0, -50),
                                                                rotation=90)
            self.get_robot_by_name("E").current_state = RobotState(position=Point(50, 0),
                                                                rotation=180)
            self.get_robot_by_name("A").current_state = RobotState(position=Point(0, 50),
                                                                rotation=180)
            self.get_robot_by_name("F").current_state = RobotState(position=Point(0, 50),
                                                                rotation=270)
            self.get_robot_by_name("G").current_state = RobotState(position=Point(0, 50),
                                                                rotation=270)

            self.sequence = [
                            self.get_robot_by_name("B").id, 
                            self.get_robot_by_name("E").id, 
                            self.get_robot_by_name("A").id,
                            self.get_robot_by_name("F").id,
                            self.get_robot_by_name("G").id,
                            self.get_robot_by_name("D").id
                            ]
            
            for _id in self.sequence[:-1]:
                self.update_state_information(robot_id=_id,
                                            current_state=self.robots[_id].current_state,
                                            motion_flags=[MOTION_FLAGS.FORCE_UPDATE])
            
            # print([self.robots[x].name for x in self.sequence])
            for seq_id, robot_id in enumerate(self.sequence):
                # assert self.global_planner is not None
                self.robots[robot_id].initialize_high_controller(seq_id, self.global_planner)

        if probe_power_consumption:
            power_start = self.probe_battery_information()

        # rotation_list = [
        #         90,
        #         120,
        #         150,
        #         210,
        #         240,
        #         270,
        #         180,
        #     ]

        anchors = [self.robots[x] for x in self.robots.keys()]
        anchors = sorted(anchors, key = lambda x : x.name)
        anchor_positions = np.array([anchor.current_state.to_numpy() for anchor in anchors])
        print('ANCHOR POSITIONS', anchor_positions)

        outside_safe_zone = self.sequence[:-1]

        # Bring robots into safe zone, assuming last robot is at center
        self.robots_contract(outside_safe_zone, center_robot_id=self.sequence[-1])

        if probe_power_consumption:
            power_contract_finished = self.probe_battery_information()
        
        print("Contracted")
        anchors = [self.robots[x] for x in self.robots.keys()]
        anchors = sorted(anchors, key = lambda x : x.name)
        anchor_positions = np.array([anchor.current_state.to_numpy() for anchor in anchors])
        print('ANCHOR POSITIONS', anchor_positions)

        ### update coarse position
        for _id in self.sequence[:-1]:
            coarse_pos, _ = self.robots[_id].high_controller.process_action(High_Status.PAIRWISE_LOC, landmarks = None, init_pos = self.robots[_id].current_state.to_numpy())
            self.robots[_id].current_state.position = Point(coarse_pos[0], coarse_pos[1])

        # Localize robots in safe zone, last robot in sequence in the center
        self.localize_all(self.sequence[-1], 5)
        print("Localized all")

        if probe_power_consumption:
            power_localization_finished = self.probe_battery_information()

        power_stages = []
        # Navigate robots into platform
        for idx, _id in enumerate(self.sequence[:-1]):
            #Obtain milestones to return to platform
            # landmarks = None # first just use the three anchors in platform
            # milestones, _ = self.robots[_id].high_controller.process_action(High_Status.BACK_NAV, landmarks, self.robots[_id].current_state.to_numpy())
            print("ID", _id, "IDX", idx)
            robot = self.get_robot_by_id(_id)
            anchors = [self.robots[x] for x in self.robots if x != _id]
            anchors = sorted(anchors, key = lambda x : x.name)
            anchor_positions = np.array([anchor.current_state.to_numpy() for anchor in anchors])
            print('ANCHOR POSITIONS', anchor_positions)
            print('MOVING ROBOT', robot.name)
            milestones, back_angle = robot.high_controller.process_action(High_Status.BACK_NAV, landmarks=anchor_positions, init_pos=self.robots[_id].current_state.to_numpy())
            
            # robot.current_state.rotation = rotation_list[idx]
            print("ROBOT STATE", robot.current_state.tostring())
            # input("DEBUG")

            ### update the backangle as the initial angle here for each robot
            # for _id in self.sequence[:-1]:
            #     self.update_state_information(robot_id=_id,
            #                                 current_state=self.robots[_id].current_state,
            #                                 motion_flags=[MOTION_FLAGS.FORCE_UPDATE])

            robot = self.robots[_id]
            robot.current_state.position = milestones[0]
            print("MILESTONE 0", milestones[0].point)
            self.update_state_information(robot_id=_id,
                                          current_state=self.robots[_id].current_state,
                                          motion_flags=[MOTION_FLAGS.FORCE_UPDATE])
            self.navigate(milestones, robot, anchors, accurate_end_pos=True)

            # Move all robots except the last one into the base to make room for new robot
            if idx > 4:
                self.move_robot_in_base(self.sequence[idx - 1], len(PLATFORM_POSITIONS) - 1, idx)
                # self.enter_base(previous, from_base=True, depth=len(PLATFORM_POSITIONS) - 1 - idx)
                # self.robots[previous].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
                # self.robots[previous].current_state = RobotState(position=Point(PLATFORM_POSITIONS[-idx - 1][0], PLATFORM_POSITIONS[-idx - 1][1]), rotation=0, velocity=Point(0,0))
            elif idx == 0:
                # Move first robot to end
                self.get_robot_by_id(self.sequence[-1]).status_stream.flush()
                self.enter_base(self.sequence[-1], from_base=True, depth=100)
                self.move_robot_in_base(self.sequence[-1], len(PLATFORM_POSITIONS) - 1, 0)
                # self.robots[self.sequence[-1]].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
                # self.robots[self.sequence[-1]].current_state = RobotState(position=Point(PLATFORM_POSITIONS[0][0], PLATFORM_POSITIONS[0][1]), rotation=0, velocity=Point(0,0))
            elif idx == 1: # Place second robot on position 4 to minimize ambiguity
                self.move_robot_in_base(self.sequence[0], len(PLATFORM_POSITIONS) - 1, 4)
            elif idx == 2: # Place second robot on position 2 & third robot on position 4 to minimize ambiguity
                self.move_robot_in_base(self.sequence[0], 4, 2)
                self.move_robot_in_base(self.sequence[1], len(PLATFORM_POSITIONS) - 1, 4)
            elif idx == 3: # Place second robot on position 1, third robot on position 4 to minimize ambiguity
                self.move_robot_in_base(self.sequence[0], 2, 1)
                self.move_robot_in_base(self.sequence[1], 4, 2)
                self.move_robot_in_base(self.sequence[2], len(PLATFORM_POSITIONS) - 1, 4)
            elif idx == 4: # Place second robot on position 1, third robot on position 4 to minimize ambiguity
                self.move_robot_in_base(self.sequence[2], 4, 3)
                self.move_robot_in_base(self.sequence[3], len(PLATFORM_POSITIONS) - 1, 4)

            self.robots[_id].status_stream.flush()

            # self.update_motion(_id, code=MOTION_CODES.MOTION_ROTATE_ANGLE, angle=270)
            # self.robots[_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            # Move robot into the platform
            self.enter_base(_id, from_base=False, depth=1)
            self.robots[_id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_BASE_ENTERED)
            
            self.robots[_id].current_state = RobotState(position=Point(PLATFORM_POSITIONS[-1][0], PLATFORM_POSITIONS[-1][1]), rotation=0, velocity=Point(0,0))

            # Save debug data
            self.robots[_id].high_controller.process_action(High_Status.IN_PLATFORM)

            if probe_power_consumption:
                power_stage = self.probe_battery_information()
                power_stages.append(power_stage)
        
        # Update new sequence because robots that were already on the platform
        # should now appear first
        self.sequence = self.sequence[-1:] + self.sequence[:-1]

        print("Sequence is now:", self.sequence)

        if probe_power_consumption:
            power_end = self.probe_battery_information()

            power_consumption_info = dict(power_start=power_start,
                                          power_contract_finished=power_contract_finished,
                                          power_localization_finished=power_localization_finished,
                                          power_stages=power_stages,
                                          power_end=power_end)

            return power_consumption_info        

    def entry_maneuver(self, robot: Robot):
        idx = self.sequence.index(robot.id)
        
        if idx > 4:
            self.move_robot_in_base(self.sequence[idx - 1], len(PLATFORM_POSITIONS) - 1, idx)
        elif idx == 0:
            # Move first robot to end
            self.get_robot_by_id(self.sequence[-1]).status_stream.flush()
            self.enter_base(self.sequence[-1], from_base=True, depth=100)
            self.move_robot_in_base(self.sequence[-1], len(PLATFORM_POSITIONS) - 1, 0)
        elif idx == 1: # Place second robot on position 4 to minimize ambiguity
            self.move_robot_in_base(self.sequence[0], len(PLATFORM_POSITIONS) - 1, 4)
        elif idx == 2: # Place second robot on position 2 & third robot on position 4 to minimize ambiguity
            self.move_robot_in_base(self.sequence[0], 4, 2)
            self.move_robot_in_base(self.sequence[1], len(PLATFORM_POSITIONS) - 1, 4)
        elif idx == 3: # Place second robot on position 1, third robot on position 4 to minimize ambiguity
            self.move_robot_in_base(self.sequence[0], 2, 1)
            self.move_robot_in_base(self.sequence[1], 4, 2)
            self.move_robot_in_base(self.sequence[2], len(PLATFORM_POSITIONS) - 1, 4)
        elif idx == 4: # Place second robot on position 1, third robot on position 4 to minimize ambiguity
            self.move_robot_in_base(self.sequence[2], 4, 3)
            self.move_robot_in_base(self.sequence[3], len(PLATFORM_POSITIONS) - 1, 4)

        robot.status_stream.flush()

        # Move robot into the platform
        self.enter_base(robot.id, from_base=False, depth=1)
        robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_BASE_ENTERED)
        
        robot.current_state = RobotState(position=Point(PLATFORM_POSITIONS[-1][0], PLATFORM_POSITIONS[-1][1]), rotation=0, velocity=Point(0,0))
        robot.high_controller.process_action(High_Status.IN_PLATFORM)
