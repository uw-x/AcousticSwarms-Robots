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


def record_and_send(controller:RobotController, num_chirps, spacing, offset, spk_id, prefix="", channels=1):
    assert channels in [1, 2], "Robot can have either 1 or 2 channel audio input"
    
    N = num_chirps
    # speaker_id = len(controller.robots) - 1 # Last robot to connet is the master & speaker
    speaker_id = spk_id

    sr_data = FREQS.SAMPLING_FREQUENCY_62500
    sampling_rate = freq_list[sr_data].value
        
    for _id in controller.robots:
        controller.get_robot_by_id(_id).mic_stream.flush()
        controller.get_robot_by_id(_id).status_stream.flush()
        controller.get_robot_by_id(_id).status_stream.start() # Start status stream to await TS update
    
    anchors = [r for _id, r in controller.robots.items()]
    anchors = sorted(anchors, key=lambda x: x.name)

    sc = StreamCollector([r.mic_stream for r in anchors], frame_width=1 + int(round(0.05 * sampling_rate))).start()

    order = [speaker_id]
    for r in anchors:
        if r.id != speaker_id:
            order.append(r.id)

    xcorr_sc = StreamCollector([controller.robots[_id].xcorr_stream for _id in order], frame_width=1).start()
    ss = StreamSelector(sc, xcorr_sc).start()
    
    for _id, robot in controller.robots.items():
        # Enabling notifications must be LAST!
        # controller.make_timesync_slave(_id)
        # if _id != speaker_id:
        #     controller.update_mic_parameters(_id, sampling_rate=sr_data, left_gain=0x30, right_gain=0x30, use_compression=False, channels=channels)
        # else:
        #     controller.update_mic_parameters(_id, sampling_rate=sr_data, left_gain=ROBOT_SELF_GAIN, right_gain=ROBOT_SELF_GAIN, use_compression=False, channels=channels)
        
        # controller.sync_task_create(_id, SYNC_TASKS.SYNC_TASK_MIC_START, 0) # Start microphone immediately
        
        
        if _id != speaker_id:
            gain = 0x30
            is_spk = False
        else:
            gain = ROBOT_SELF_GAIN
            is_spk = True

        controller.load_tof_profile(_id, sampling_rate=sr_data, gain=gain, is_spk=is_spk, period=spacing//50, begin_interval=offset//50, end_interval=(offset + num_chirps * spacing - 1)//50 + 1)
        
        controller.enable_recording(_id)
        controller.stream_xcorr(_id, use_notifications=False)
        controller.enable_mic_notifications(_id)
    
    # # Arm speaker task at robot to track.
    # controller.sync_task_create_recurring(spk_id, SYNC_TASKS.SYNC_TASK_SPK_START, period=spacing//50, start_interval=offset//50, end_interval=(offset + N * spacing - 1)//50 + 1)

    # # Arm mic tasks 50 ms before at all other robots
    # for mic_id in controller.robots:
    #     if mic_id != speaker_id:
    #         controller.sync_task_create_recurring(mic_id, SYNC_TASKS.SYNC_TASK_MIC_RECORD, period=spacing//50, start_interval=(offset)//50, end_interval=(offset + N * spacing - 1)//50 + 1)
    #     else:    #         controller.sync_task_create_recurring(mic_id, SYNC_TASKS.SYNC_TASK_MIC_RECORD, period=spacing//50, start_interval=(offset)//50, end_interval=(offset + N * spacing - 1)//50 + 1)

    frames = []
    # current_frame = {'xcorrs':[], 'ts_adjustment+':0, 'ts_adjustment-':0}
    current_frame = {'xcorrs':[]}

    # controller.robots[0].xcorr_stream.start()
    controller.run_tasks(master=speaker_id)
    t1 = time.time()
    
    record_id = 0
    chirp_num = 0
    dist = []
    while True:
        try:
            ts = time.time()
            t, val = ss.get()
            te = time.time()
            print("Time between consecutive chirp results", te - ts)
            
            if t == 0:
                frame = val
                print('FRAME SHAPE', frame.shape)
                if channels == 2:
                    frame = frame.transpose(0, 2, 1)
                    frame = frame.reshape(frame.shape[0] * frame.shape[1], -1)
                
                record_id += 1
                write_audio_file(f'recordings/{prefix}recording{record_id}_{sampling_rate}.wav', frame.astype(np.int16), sr=sampling_rate)
                print("Saved")

                if record_id == N:
                    break
                # elif record_id > N:
                #     quit()

                for _id in controller.robots:
                    controller.robots[_id].mic_stream.next_sequence_number = 1
            elif t == 1:
                chirp_num += 1
                
                # val[0,0] = 375
                xcorrs = val
                assert all(xcorrs != 0)
                current_frame['xcorrs'] = xcorrs.tolist()
                frames.append(current_frame)

                if any(xcorrs[:, 0] < xcorrs[0, 0]):
                    print(xcorrs)
                    print("ERRORRRRR" * 100)
                    # quit()
                    # record_id = N
                
                # current_frame = {'xcorrs':[], 'ts_adjustment+':0, 'ts_adjustment-':0}
                print('RECEIVED XCORR:',xcorrs)
                current_frame = {'xcorrs':[]}
                for i in range(xcorrs.shape[0]):
                    distance = extract_distance_single(xcorrs[0][0], xcorrs[i][0])
                    print(f'[{controller.robots[order[i]].name}], Distances', distance)
                    dist.append(distance)
                
                # if chirp_num == N:
                #     break
            elif t == 2:
                pass
                # if val[0] == STATUS_UPDATE_TYPES.DCS_STATUS_LOG:
                #     if val[1] == '+':
                #         current_frame['ts_adjustment+']+=1
                #     elif val[1] == '-':
                #         current_frame['ts_adjustment-']+=1
            else:
                print("************** WARNING CODE SHOULD NOT GO HERE **************")
        except KeyboardInterrupt:
            traceback.print_exc()
            break
    t2 = time.time()
    print("Chirp latency: ", (t2 - t1) - offset * 1e-3 - (N-1) * spacing * 1e-3, 's')

    d = {'chirps':[]}
    for frame in frames:
        d["chirps"].append(frame)
    with open(f"recordings/{prefix}_xcorr_samples.json", 'w') as f:
        json.dump(d, f)

    dist = np.mean(np.array(dist), axis=0)

    controller.stop_tasks()
    sc.terminate()
    xcorr_sc.terminate()
    ss.terminate()

    for robot_id, robot in controller.robots.items():
        controller.disable_mic_notifications(robot_id)
        controller.stream_xcorr(_id, enable=False)
        print("There are {} dropped packets from robot {}.".format(robot.mic_stream.dropped_packets, robot_id))

    # for thread in threading.enumerate(): 
    #     print(thread.name)

    return dist

def record(controller: RobotController,
           duration: float,
           sampling_rate: FREQS,
           use_compression: bool = True,
           channels: int=1,
           left_gain:int=0x50,
           right_gain:int=0x50,
           compression_bitrate:int = 32000) -> np.ndarray:
    """
    Records from all robots a requested amount of seconds
    @param controller: robot controller to use
    @param duration: amount of time to record (in milliseconds)
    @param sampling_rate: recording sampling rate
    @param use_compression: whether or not to enable compression on microphones
    """

    for _id in controller.robots:
        controller.update_mic_parameters(_id, sampling_rate=sampling_rate, use_compression=use_compression, left_gain=left_gain, right_gain=right_gain, channels=channels, compression_bitrate=compression_bitrate)

    controller.begin_stream(frame_width=duration)
        
    frame = controller.grab_frame()

    controller.stop_stream()

    return frame


def stream(controller: RobotController,
           chunklength: float,
           sampling_rate: FREQS,
           use_compression: bool = True,
           callback = None,
           channels: int=1,
           left_gain:int=0x50,
           right_gain:int=0x50):
    """
    Streams from all robots and calls a callback every time chunklength samples are received
    @param controller: robot controller to use
    @param chunklength: time of each chunk (in milliseconds)
    @param sampling_rate: recording sampling rate
    @param use_compression: whether or not to enable compression on microphones
    @param callback: callback function, must be callback(controller, array)
    """

    if callback is None:
        callback = lambda x: x

    for _id in controller.robots:
        controller.update_mic_parameters(_id,
                                         sampling_rate=sampling_rate,
                                         use_compression=use_compression,
                                         right_gain=right_gain,
                                         left_gain=left_gain,
                                         channels=channels)

    controller.begin_stream(frame_width=chunklength)
        
    while True:
        try:
            frame = controller.grab_frame()

            if channels == 2:
                frame = frame.transpose(0, 2, 1)
                frame = frame.reshape(frame.shape[0] * frame.shape[1], -1)

            callback(controller, frame.T)
        except KeyboardInterrupt:
            break

    controller.stop_stream()

    return frame

import queue
import sounddevice as sd

streamq = queue.Queue()


def localize_callback(controller, frame):
    # TODO: Push audio recording & microphone positions to cloud

    # TODO: Await results (possibly in another thread)
    pass

def stream_callback(controller, frame):
    # print(f'[main] [{int(time.time() * 1e6)}] Num elements in q: {streamq.qsize()}')
    streamq.put(frame[:, :2]) # Put first two channels in the stream queue

def sd_callback(indata, outdata, frames, time, status):
    try:
        outdata[:] = streamq.get_nowait()
    except queue.Empty:
        outdata[:] = np.zeros(outdata.shape, dtype=np.int16)

# def sd_callback(indata, outdata, frames, time, status):
#     outdata[:] = indata

def track2d(controller: RobotController, spacing = 200, offset = 500):
    controller.initialize_global_planner()

    # F = controller.get_robot_by_name('F')
    # F.current_state.position = Point(PLATFORM_POSITIONS[0, 0], PLATFORM_POSITIONS[0, 1])
    
    B = controller.get_robot_by_name('F')
    B.current_state.position = Point(PLATFORM_POSITIONS[0, 0], PLATFORM_POSITIONS[0, 1])

    # C = controller.get_robot_by_name('C')
    # C.current_state.position = Point(PLATFORM_POSITIONS[3, 0], PLATFORM_POSITIONS[3, 1])
    
    A = controller.get_robot_by_name('B')
    # A.current_state.position = Point(PLATFORM_POSITIONS[2, 0], PLATFORM_POSITIONS[2, 1])
    A.current_state.position = Point(PLATFORM_POSITIONS[4, 0], PLATFORM_POSITIONS[4, 1])
    
    D = controller.get_robot_by_name('A')
    # D.current_state.position = Point(PLATFORM_POSITIONS[4, 0], PLATFORM_POSITIONS[4, 1])
    D.current_state.position = Point(PLATFORM_POSITIONS[6, 0], PLATFORM_POSITIONS[6, 1])

    E = controller.get_robot_by_name('C')
    # E.current_state.position = Point(PLATFORM_POSITIONS[6, 0], PLATFORM_POSITIONS[6, 1])
    E.current_state.position = Point(-4, 44)

    G = controller.get_robot_by_name('E')
    G.current_state.position = Point(0, 40)

    robot = G
    robot.moving = True
    anchors = [B, A, D, E]

    anchors = sorted(anchors, key = lambda x : x.name)
    for a in anchors:
        a.moving = False
    
    anchor_ids = [r.id for r in anchors]
    robot_id = robot.id

    print("[NAVIGATE]")
    print('Anchors', [controller.robots[x].name for x in anchor_ids])
    print('Robot', controller.robots[robot_id].name)

    sr_data = FREQS.SAMPLING_FREQUENCY_62500
    sampling_rate = freq_list[sr_data].value

    spk_id = robot_id

    # Start microphone at each robot & enable status notifications (to get xcorr data)
    for mic_id in anchor_ids + [robot_id]:
        controller.make_timesync_slave(mic_id) # Needed to reset everything
        if mic_id != spk_id:
            controller.update_mic_parameters(mic_id, sampling_rate=sr_data, left_gain=ROBOT_CROSS_GAIN, right_gain=ROBOT_CROSS_GAIN, use_compression=False, channels=2)
        else:
            controller.update_mic_parameters(mic_id, sampling_rate=sr_data, left_gain=ROBOT_SELF_GAIN, right_gain=ROBOT_SELF_GAIN, use_compression=False, channels=2)
        controller.sync_task_create(mic_id, SYNC_TASKS.SYNC_TASK_MIC_START, 0) # Start microphone immediately
        controller.stream_xcorr(mic_id, True, use_notifications=True) # Enable stream on xcorr data
        controller.robots[mic_id].xcorr_stream.flush() # Discard xcorr data from previous streams        
        controller.get_robot_by_id(mic_id).xcorr_stream.expected_sequence_number = 0
    
    # Arm speaker task at robot to track.
    controller.sync_task_create_recurring(spk_id, SYNC_TASKS.SYNC_TASK_SPK_START, spacing//50, offset//50)

    # Arm mic tasks 50 ms before at all other robots
    for mic_id in anchor_ids + [robot_id]:
        controller.sync_task_create_recurring(mic_id, SYNC_TASKS.SYNC_TASK_MIC_RECORD, period=spacing//50, start_interval=(offset)//50)

    robot.initialize_high_controller(0, controller.global_planner)
    plan_node = robot.high_controller
    anchor_pos = np.array([r.current_state.position.point for r in anchors])
    plan_node.process_action(High_Status.BACK_NAV, landmarks = anchor_pos, init_pos = np.array(robot.current_state.position.point))
    
    sc = StreamCollector([controller.robots[_id].xcorr_stream for _id in (anchor_ids + [robot_id])], frame_width=1)
    status_stream = controller.robots[robot_id].status_stream

    # # DEBUGGING ONLY
    for mic_id in anchor_ids + [robot_id]:
        controller.enable_recording(mic_id)
        # controller.enable_mic_notifications(mic_id)
        # controller.robots[mic_id].mic_stream.flush()
    
    audio = StreamCollector([anchor.mic_stream for anchor in anchors] + [robot.mic_stream], frame_width=1 + int(round(0.05 * sampling_rate)), debug=False).start()

    event_stream = StreamSelector(sc, status_stream, audio)
    # for milestone in milestones:
    #     print(milestone.point)

    # Wait for calibration signals
    # print("CALIBRATION")
    
    pp = PathPlotter(anchors + [robot])

    print("Triggering")
    
    controller.run_tasks(master=spk_id) # Trigger
    print("TRIGGERED")
    t1 = time.time()
    num_chirps = 0


    sc.start()
    event_stream.start()
    pp.start()
    
    tracking = True
    record_id = 0
    while tracking:
        try:
            event = event_stream.get()
            # print(event)

            # XCorr update
            if event[0] == 0:                
                xcorr_frame = event[1]
                
                if (xcorr_frame == -1).any():
                    print("A robot has dropped an xcorr packet, ignoring ...")
                    continue

                anchor_offsets = np.array(xcorr_frame[:-1])
                self_offset = xcorr_frame[-1][0]

                print('Anchor offsets', anchor_offsets.tolist())
                print('Self offsets', self_offset)

                distances = []
                for i in range(len(anchors)):
                    distance = extract_distance_single(self_offset, anchor_offsets[i][0])
                    distances.append(distance)

                vel = np.array([controller.robots[robot_id].previous_state.velocity.x, controller.robots[robot_id].previous_state.velocity.y])

                mean, velocity, (estimated_angle, confidence) = plan_node.update_offset(sample_offsets=anchor_offsets, 
                                                                    self_offset=self_offset,
                                                                    old_pos=controller.robots[robot_id].previous_state.to_numpy(),
                                                                    new_pos=controller.robots[robot_id].current_state.to_numpy())

                print("Estimated state:", controller.robots[robot_id].current_state.tostring())
                rotation = controller.robots[robot_id].current_state.rotation
                mean = mean.reshape(3, 1)
                velocity = velocity.reshape(2, 1)

                flags = []
                if confidence > 0.8:
                    flags = [MOTION_FLAGS.UPDATE_ANGLE]
                else:
                    estimated_angle = controller.robots[robot_id].current_state.rotation

                estimated_angle = np.rad2deg(estimated_angle)
                print('Estimate angle:', estimated_angle, ' ({} confidence)'.format(confidence))
                controller.robots[robot_id].current_state = RobotState(position=Point(mean[0][0], mean[1][0]), 
                                                                        rotation=estimated_angle,
                                                                        velocity=Point(velocity[0][0], velocity[1][0]))

                pp.update(anchors + [robot], distances + [0])
                print('Current pos', controller.get_robot_by_id(robot.id).current_state.position.point)
                t2 = time.time()
                print('Latency:', t2 - t1 - offset * 1e-3 - spacing * num_chirps * 1e-3, 's')
                
                num_chirps += 1

            elif event[0] == 2:
                frame = event[1]
                record_id += 1
                write_audio_file(f'recordings/recording{record_id}_{sampling_rate}.wav', frame.astype(np.int16), sr=sampling_rate)
                print("Saved")

                for _id in controller.robots:
                    controller.robots[_id].mic_stream.next_sequence_number = 1  
            else:
                assert 1, "Code should not go here"
        except KeyboardInterrupt:
            break
    sc.stop()
    event_stream.stop()
    pp.stop()

    controller.stop_tasks()

    # Disables mic notifications & resets xcorr sequence number.
    # Admittedly, this isn't the best place to do it.
    for _id in controller.robots:
        print(controller.robots[_id].name)
        controller.stream_xcorr(_id, enable=False)
        controller.disable_mic_notifications(_id)

def loopback():
    ostream = sd.Stream(samplerate=16000,
                        dtype=np.int16,
                        channels=2, 
                        callback=sd_callback,
                        latency='low',
                        blocksize=256)
    ostream.start()
    time.sleep(200)


def do_random_walk(controller: RobotController):
    A = controller.get_robot_by_name('A')

    for i in range(40):
        try:
            forward_time = np.random.uniform(0.5, 2)
            random_speed = np.random.randint(120, 200)
            controller.update_motion(A.id, MOTION_CODES.MOTION_FORWARD, duration_us=forward_time * 1e6, speed=random_speed)
            A.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            
            rotation_time = np.random.uniform(0.25, 0.75)
            random_speed = np.random.randint(120, 170)
            controller.update_motion(A.id, MOTION_CODES.MOTION_ROTATE_TIME, duration_us=rotation_time * 1e6, speed=random_speed)
            A.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
        except KeyboardInterrupt:
            break

def check(x):
    if not x:
        print("TIMED OUT")
    else:
        print("Received")

def main():
    controller = RobotController()
    
    # loopback()
    # return

    # controller.localize_all_debug()
    
    if controller.available:
        # controller.begin_scan()

        # controller.scan(8)
        controller.scan(7)
        # controller.scan(6)
        # controller.scan(5)
        # controller.scan(4)
        # controller.scan(3)
        # controller.scan(2)
        # controller.scan(1)

        if len(controller.robots) > 0:
            print("All robots are ready!")

            for robot_id, robot in controller.robots.items():
                controller.monitor_battery(robot_id, enable=True)
            
            # controller.get_platform_sequence()
            # controller.enter_base(0, from_base=True, depth=-1)
            
            # controller.monitor_gyro(0)
            # time.sleep(2)
            # for i in range(5):
            #     print(controller.probe_battery_information())
            #     time.sleep(5)

            # controller.calibrate_sensors(0)
            # controller.get_robot_by_id(0).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)

            # do_random_walk(controller)

            # time.sleep(100)

            # controller.test()

            # controller.get_robot_by_id(0).current_state = RobotState(position=Point(-2, -2), rotation=90)
            # controller.update_state_information(0, current_state=controller.get_robot_by_id(0).current_state, motion_flags=[MOTION_FLAGS.FORCE_UPDATE])
            # controller.enter_base(0, False, 1)
            # time.sleep(1000)

            # for i in range(20):
            #     controller.calibrate_sensors(0)
            #     controller.get_robot_by_id(0).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)

            # controller.update_motion(0, MOTION_CODES.MOTION_FORWARD, duration_us=1e6, speed=200)
            # controller.get_robot_by_id(0).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            # seq = controller.get_platform_sequence()
            # for x in seq:
            #     print(controller.get_robot_by_id(x).name)

            # for i in range(1):
            #     sr_idx = FREQS.SAMPLING_FREQUENCY_62500
            #     # recording = record(controller, 2000, sampling_rate = sr_idx, use_compression=False)
            #     # write_audio_file(f"recordings/recording{i}.wav", recording, sr=freq_list[sr_idx].value)
            #     name = np.random.choice(['A'])
            #     d = record_and_send(controller, 2, 2000, offset=500, spk_id=controller.get_robot_by_name(name).id, prefix=str(i), channels=2)
            #     input("Waiting for keypress...")

            # for i in range(10):
            #     controller.update_motion(0, MOTION_CODES.MOTION_ROTATE_ANGLE, angle=(180 * (i+2)) % 360 )
            #     controller.get_robot_by_id(0).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            #     time.sleep(2)
            
            # input('Press key to start recording...')

            # sr_idx = FREQS.SAMPLING_FREQUENCY_16000
            # sr_idx = FREQS.SAMPLING_FREQUENCY_50000
            # recording = record(controller, 10000, sr_idx, use_compression=True, channels=1, left_gain=0x50, right_gain=0x50)
            # write_audio_file("recordings/recording1.wav", recording, sr=freq_list[sr_idx].standard_freq)

            # sr_idx = FREQS.SAMPLING_FREQUENCY_50000
            # for i in range(40):
            #     recording = record(controller, 1000, sr_idx, use_compression=True, channels=1, left_gain=0x50, right_gain=0x50)
                # write_audio_file("recordings/recording2.wav", recording, sr=freq_list[sr_idx].standard_freq)
            
            # robot = controller.get_robot_by_id(0)
            # controller.calibrate_sensors(robot.id)
            # robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)

            # controller.update_state_information(robot.id, current_state= RobotState(position=Point(-1, -2), rotation=90), motion_flags=[MOTION_FLAGS.FORCE_UPDATE])
            # controller.enter_base(robot.id, from_base=False, depth=1)
            # robot.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_BASE_ENTERED)

            # for i in range(10):
            #     track2d(controller)

            input('Go:')
            recording = record(controller, 10000, sampling_rate = FREQS.SAMPLING_FREQUENCY_50000, use_compression=True, left_gain=0x25, right_gain=0x25)
            write_audio_file('rir.wav', recording, 48000)

            # # Distribute robots across table
            # controller.robots_distribute(expand_in_background=True)
            # print("Distributed")

            # time.sleep(5)

            # controller.localize_all(controller.get_robot_by_name('D').id, iterations=5)
            # print("MIC_POSITIONS = np.array([")
            # for x in ['A', 'B', 'C', 'D', 'E', 'F', 'G']:
            #     pos = controller.get_robot_by_name(x).current_state.position
            #     print('[', pos.x, ',', pos.y,  ',', 2, '],')
            # print('])')
            # controller.move_robot_in_base(controller.get_robot_by_name('D').id, 6, 0)
            
            # # # Record for 5 seconds @ 16kHz with compression
            # sr_idx = FREQS.SAMPLING_FREQUENCY_50000
            # # sr_idx = FREQS.SAMPLING_FREQUENCY_16000
            # print("Recording ...")
            # recording = record(controller, 5000, sampling_rate = sr_idx, use_compression=False)
            # write_audio_file(f"recordings/recording.wav", recording, sr=freq_list[sr_idx].value)
            # print("Recording finished")
            
            # # for i in range(100):
            # #     d = record_and_send(controller, 1, 4000, 500, spk_id=controller.get_robot_by_name('D').id, prefix='', channels=2)

            # #     if previous_d != None:
            # #         if (np.absolute(previous_d - d) > 3).any():
            # #             print(d)
            # #             print(previous_d)
            # #             print("ERROR"*100)
            # #             quit()
                
            # #     previous_d = d
            # # recording = record(controller, 2000, sampling_rate = FREQS.SAMPLING_FREQUENCY_50000, use_compression=True)
            # # write_audio_file(f"recordings/recording1.wav", recording, sr=freq_list[sr_idx].value)
            # # recording = record(controller, 2000, sampling_rate = sr_idx, use_compression=True)
            # # write_audio_file(f"recordings/recording2.wav", recording, sr=freq_list[sr_idx].value)

            # # Bring robots back to base
            # controller.robots_recall()
            # print("Recalled")

            # robot_sequence = [ 
            #     'F',
            #     'B',
            #     'C',
            #     'E',
            #     'A',
            #     'G',
            #     'D'
            # ]

            # controller.initialize_global_planner()

            # # Initialize platform sequence
            # controller.sequence = []
            # for i, r_name in enumerate(robot_sequence):
            #     robot = controller.get_robot_by_name(r_name)
            #     controller.sequence.append(robot.id)
            #     robot.initialize_high_controller(i, controller.global_planner)
            
            # controller.robots_recall()
            # print("Recalled")
            

            
            # for i in range(6):
            #     input("Awaiting keypress")
            #     controller.localize_all(controller.get_robot_by_name('D').id, iterations=5)
            #     print("MIC_POSITIONS = np.array([")
            #     for x in ['A', 'B', 'C', 'D', 'E', 'F', 'G']:
            #         pos = controller.get_robot_by_name(x).current_state.position
            #         print('[', pos.x, ',', pos.y,  ',', 2, '],')
            #     print('])')
            #     controller.move_robot_in_base(controller.get_robot_by_name('D').id, 6, 0)

            # controller.enter_base(0, False, 1)
            # controller.get_robot_by_id(0).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_BASE_ENTERED)

            # time.sleep(100)

            # controller.enable_ir_status_stream(0, True)
            # controller.enter_base(0, True, 2)
            # controller.robots[0].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            # sequence = ['C', 'E', 'A', 'B']

            # in_base = []
            # for r in sequence:
            #     for b in in_base:
            #         controller.enter_base(b.id, True, 1)
            #         b.wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            #     r = controller.get_robot_by_name(r)
            #     controller.enter_base(r.id, from_base=False, depth=1)
            #     controller.robots[r.id].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_BASE_ENTERED)
            #     in_base.append(r)

            # new_sequence = controller.get_platform_sequence()
            # print([controller.robots[x].name for x in new_sequence])app: Disconnected. conn_handle: 0x0, reason: 0x22

            # controller.nav_begin(0)

            # for i in controller.robots.keys():
            #     controller.calibrate_sensors(i)
            
            # for i in controller.robots.keys():
            #     controller.robots[i].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)
            
            # time.sleep(1000)
            # for i in controller.robots.keys():
            #     controller.update_motion(i, MOTION_CODES.MOTION_FORWARD, duration_us = 1e9, controlled = False, speed=150)
            
            # for i in controller.robots.keys():
            #     controller.robots[i].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            # controller.update_motion(0, MOTION_CODES.MOTION_FORWARD, duration_us = 1e9, controlled = False, speed=150)
            # controller.robots[0].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)

            # controller.update_motion(0, MOTION_CODES.MOTION_ROTATE_TIME, duration_us = 100e6, controlled = False, speed=200)
            # controller.robots[0].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            
            # controller.monitor_accel(0)

            # # sp = StreamPlotter(controller.get_robot_by_name('D').accel_stream, 3, 100, 5, 1, y_bounds=[-4000, 4000])
            # sp = StreamPlotter(controller.get_robot_by_name('D').accel_stream, 3, 100, 50, 10, y_bounds=[-2000, 2000])
            # sp.show()
            
            # controller.robots[0].wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_NAV_END)
            
            # for i in range(10):
            #     N = 30
            #     record_and_send(controller, N, 2000, spk_id=controller.get_robot_by_name('C').id, prefix=str(i))
            #     input('Waiting for keypress ...')

            # sr_data = FREQS.SAMPLING_FREQUENCY_62500

            # # Start microphone at each robot & enable status notifications (to get xcorr data)
            # for mic_id in controller.robots:
            #     controller.update_mic_parameters(mic_id, sampling_rate=sr_data)
            #     controller.sync_task_create(mic_id, SYNC_TASKS.SYNC_TASK_MIC_START, 0) # Start microphone immediately
            #     controller.stream_xcorr(mic_id, True) # Enable stream on xcorr data

            # spk_id = 0
            # spacing = 500
            # offset = 1000
            # # Arm speaker task at robot to track.
            # controller.sync_task_create_recurring(spk_id, SYNC_TASKS.SYNC_TASK_SPK_START, spacing//50, offset//50)

            # # Arm mic tasks 50 ms before at all other robots
            # for mic_id in controller.robots:
            #     controller.sync_task_create_recurring(mic_id, SYNC_TASKS.SYNC_TASK_MIC_RECORD, period=spacing//50, start_interval=(offset)//50)

            # controller.calibrate_sensors(spk_id)
            # controller.await_ble_tx_finished()
            # controller.run_tasks(master=spk_id)

            # speed = 150
            # bw = 4.55
            # decimation = 2
            # IDR = 1000
            # angle = 45
            
            # tries = 0
            # again = True
            # while again:
            #     moving = controller.get_robot_by_name('E')
            #     sta = controller.get_robot_by_name('A')
                
            #     controller.calibrate_sensors(sta.id)
            #     controller.calibrate_sensors(moving.id)
            #     time.sleep(1)
            #     controller.get_robot_by_id(sta.id).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)
            #     controller.get_robot_by_id(moving.id).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)

            #     controller.get_robot_by_id(sta.id).accel_stream.flush()
            #     controller.monitor_accel(sta.id)

                
            #     controller.update_motion(moving.id, code=MOTION_CODES.MOTION_FORWARD_PULSE, duration_us=1e9, speed=speed, controlled=False)
                
            #     time.sleep(5)
            #     controller.update_motion(moving.id, code=MOTION_CODES.MOTION_BRAKE, duration_us=1e9, speed=speed)
            #     controller.monitor_accel(sta.id, enable=False)

            #     fname = f'CollisionData/STA_Collision_{IDR}Hz_Dec{decimation}_BW{bw}_Speed{speed}_Angle{angle}_Rep{tries}.dat'
            #     rem = np.array(sta.accel_stream.get_remaining()).T
                
            #     np.save(fname, rem)

            #     again = not (input('Again?(y/n)') == 'n')
            #     tries += 1

            # tries = 0
            # again = True
            # while again:
            #     controller.calibrate_sensors(0)
            #     time.sleep(1)
            #     controller.get_robot_by_id(0).wait_on_status(STATUS_UPDATE_TYPES.DCS_STATUS_CALIBRATION_DONE)

            #     controller.get_robot_by_id(0).accel_stream.flush()
            #     controller.monitor_accel(0)
            #     controller.update_motion(0, code=MOTION_CODES.MOTION_FORWARD, duration_us=1e9, speed=speed)
                
            #     # sp = StreamPlotter(controller.robots[0].accel_stream,
            #     #                 nchannels = 3,
            #     #                 total_samples = 50,
            #     #                 samples_per_update = 10,
            #     #                 decimation=1,
            #     #                 y_bounds=[-1, 1])
                
            #     # print('B', sp.total_data.shape)
            #     # sp.show()
            #     time.sleep(3)
            #     controller.update_motion(0, code=MOTION_CODES.MOTION_BRAKE, duration_us=1e9, speed=speed)
            #     controller.monitor_accel(0, enable=False)

            #     fname = f'CollisionData/Collision_{IDR}Hz_Dec{decimation}_BW{bw}_Speed{speed}_Angle{angle}_Rep{tries}.dat'
            #     rem = np.array(controller.get_robot_by_id(0).accel_stream.get_remaining()).T
                
            #     np.save(fname, rem)

            #     # sp.total_data = np.concatenate([sp.total_data,rem], axis=1)
            #     # print(sp.total_data.shape)
            #     # np.save(fname, sp.total_data)

            #     again = not (input('Again?(y/n)') == 'n')
            #     tries += 1            

        else:
            print("No robots found")
    
    controller.close()
    time.sleep(2)
    for thread in threading.enumerate():
        print(thread.name)

# mic_positions = np.array([
#     [  3.29250426 ,-39.0098524 , 2],
#     [ 41.79681634 ,-33.74729947, 2],
#     [ 45.50735476 , -5.8325838 , 2],
#     [  3.8        ,  6.1       , 2],
#     [ 45.03634808 , 26.23467655, 2],
#     [ 34.80468528 , 53.47656921, 2],
#     [  3.7256411 ,  54.24744386, 2]]
# )
# mic_positions /= 100
# d = {"mic":[]}
# for i in range(mic_positions.shape[0]):
#     d["mic"].append(dict(pos=mic_positions[i].tolist()))
# print(d)

if __name__ == "__main__":
    main()
    sys.exit()
