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
    
    try:
        frame = controller.grab_frame(timeout = (duration)/1000)
    except queue.Empty:
        frame = np.zeros((len(controller.robots), int(round(duration / 1e3 * freq_list[sampling_rate].standard_freq))))

    dropped_packets = controller.stop_stream()

    return frame, dropped_packets


ExpConfig = namedtuple("ExpConfig", ["sr", "use_compression", "compression_bitrate"])

def main():
    T = 40000
    cfgs = [ExpConfig(FREQS.SAMPLING_FREQUENCY_50000, 0, 0),
            ExpConfig(FREQS.SAMPLING_FREQUENCY_50000, 1, 256000),
            ExpConfig(FREQS.SAMPLING_FREQUENCY_50000, 1, 128000),
            ExpConfig(FREQS.SAMPLING_FREQUENCY_50000, 1, 64000),
            ExpConfig(FREQS.SAMPLING_FREQUENCY_50000, 1, 32000),
            ExpConfig(FREQS.SAMPLING_FREQUENCY_50000, 1, 16000),]

    cfgs = [ExpConfig(FREQS.SAMPLING_FREQUENCY_50000, 1, 64000)]
    
    controller = RobotController()
    
    if controller.available:
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

            for cfg in cfgs:
                input("Press enter to begin next experiment...")

                sr = cfg.sr
                use_compression = cfg.use_compression
                bitrate = cfg.compression_bitrate

                for _id in controller.robots:
                    controller.get_robot_by_id(_id).mic_stream.enable_count_packet_drop_only()

                y, dropped_packets = record(controller=controller, duration=T, sampling_rate=sr, use_compression=use_compression, channels=1, compression_bitrate=bitrate)
                # y, dropped_packets = record(controller=controller, duration=T, sampling_rate=sr, use_compression=use_compression, channels=1)
                write_audio_file('recordings/output.wav', y, freq_list[sr].standard_freq)
                
                if not use_compression:
                    bitrate = freq_list[sr].value * 16 # bitrate = number of samples / s * (number of bits / sample)
                
                experiment_name = f'{bitrate}kbps'
                
                write_dir = os.path.join('results', 'compression')
                os.makedirs(write_dir, exist_ok=True)
                write_file = os.path.join(write_dir, f'{experiment_name}.json')
                with open(write_file, 'w') as f:
                    json.dump(dropped_packets, f)
                
                print("Experiment done, data saved to", write_file)

        else:
            print("No robots found")
    
    controller.close()
    time.sleep(2)
    for thread in threading.enumerate():
        print(thread.name)

if __name__ == "__main__":
    main()
    sys.exit()
