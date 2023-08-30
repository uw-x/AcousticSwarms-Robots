DEBUG = False
DEBUG_USB_STREAM = False

DEBUG_AUDIO_STREAM = False
# DEBUG_AUDIO_STREAM = True

# DEBUG_XCORR_STREAM = True
DEBUG_XCORR_STREAM = False

DEBUG_BATTERY_STREAM = False
DEBUG_ACCEL_STREAM = False
DEBUG_MAG_STREAM = False
DEBUG_GYRO_STREAM = False

DEBUG_FRAMES = False
# DEBUG_FRAMES = True

USE_COMPRESSION = True
# USE_COMPRESSION = False

ROBOT_SELF_GAIN = 0x03
ROBOT_CROSS_GAIN = 0x30

DEV_NAME = 'shio'

class QueueEnd: pass


# DIV32: 0x08000000 -> CLK: 1.000 MHz -> SR: 15625 Hz
# DIV31: 0x08400000 -> CLK: 1.032 MHz -> SR: 16125 Hz
# DIV30: 0x08800000 -> CLK: 1.067 MHz -> SR: 16667 Hz
# DIV25: 0x0A000000 -> CLK: 1.280 MHz -> SR: 20000 Hz
# DIV16: 0x10000000 -> CLK: 2.000 MHz -> SR: 31250 Hz
# DIV12: 0x15000000 -> CLK: 2.667 MHz -> SR: 41667 Hz
# DIV10: 0x19000000 -> CLK: 3.200 MHz -> SR: 50000 Hz
# DIV08: 0x20000000 -> CLK: 4.000 MHz -> SR: 62500 Hz

from collections import namedtuple
from enum import IntEnum

import numpy as np
# PLATFORM_POSITIONS = np.array([[-3.7, 15.3],
#                               [-3.7, 11.2],
#                               [-0.2, 9.5],
#                               [0, 13.5],
#                               [3.2, 15.2],
#                               [3.5, 10.3],
#                               [3.5, 6.2]])

PLATFORM_POSITIONS = np.array([[-3.6, 15.3],
                              [-3.6, 11],
                              [-0.2, 9.4],
                              [0, 13.6],
                              [3.6, 14.7],
                              [3.8, 10.1],
                              [3.8, 6.1]])

class MOTION_CODES(IntEnum):
    MOTION_FORWARD = 0
    MOTION_BACKWARD = 1
    MOTION_ROTATE_ANGLE = 2
    MOTION_ROTATE_TIME = 3
    MOTION_BRAKE = 4
    MOTION_FORWARD_PULSE = 5
    MOTION_BACKWARD_PULSE = 6
    MOTION_ROTATE_ANGLE_PRECISE = 7
    

class FREQS(IntEnum):
    SAMPLING_FREQUENCY_8000 = 0
    SAMPLING_FREQUENCY_10000 = 1
    SAMPLING_FREQUENCY_16129 = 2
    SAMPLING_FREQUENCY_20000 = 3
    SAMPLING_FREQUENCY_23809 = 4
    SAMPLING_FREQUENCY_31250 = 5
    SAMPLING_FREQUENCY_41667 = 6
    SAMPLING_FREQUENCY_50000 = 7
    SAMPLING_FREQUENCY_62500 = 8
    SAMPLING_FREQUENCY_16000 = 9
    SAMPLING_FREQUENCY_48000 = 10

SAMPLING_FREQUENCY_DEFAULT = FREQS.SAMPLING_FREQUENCY_16129

FreqDescriptor = namedtuple("FreqDescriptor", ["value", "standard_freq", "div32", "decimation_factor", "ratio"])

freq_list = [FreqDescriptor(8000, 8000, 0x05100000, 1, 1),
             FreqDescriptor(10000, 8000, 0x0A000000, 2, 0),
             FreqDescriptor(16129, 16000, 0x08400000, 1, 0),
             FreqDescriptor(20000, 16000, 0x0A000000, 1, 0),
             FreqDescriptor(23809, 24000, 0x0C000000, 1, 0),
             FreqDescriptor(31250, 32000, 0x10000000, 1, 0),
             FreqDescriptor(41667, 44100, 0x15000000, 1, 0),
             FreqDescriptor(50000, 48000, 0x19000000, 1, 0),
             FreqDescriptor(62500, 62500, 0x1F000000, 1, 0),
             FreqDescriptor(16000, 16000, 0x0A000000, 1, 1),]
# Might be possible to go to 71.428kHz

def get_sampling_frequency(freq_idx):
    return freq_list[freq_idx]

MASTER_DROPPED_SAMPLES = 0

# MASTER_DROPPED_SAMPLES = int(round(3 * 0.05 * FS))
# MASTER_DROPPED_SAMPLES = int(round(2 * 0.05 * FS))