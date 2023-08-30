import soundfile as sf
import os
import numpy as np
import threading


def to_nbit(x):
    # Takes a length n byte array and returns an unsigned n-byte integer
    n = len(x)
    res = 0
    for i in range(n):
        res += x[i] << (((n-1)-i)*8)
    return res

def LSB_16(x):
    return (x & 0xFF)

def MSB_16(x):
    return (x >> 8) & 0xFF


def list_active_threads():
    for thread in threading.enumerate(): 
        if thread.is_alive:
            print(thread.name)

# PLEASE ENFORCE: USE 16 bit integers only when outputting audio
# Use 32 bit floats EVERYWHERE ELSE (For computations)

def signal_float2int(x):
    return (32767 * x).astype(np.int16)

def signal_int2float(x):
    return (x / 32767.0).astype(np.float32)

def cos(f, T, sr, reps=1, channels=1):
    """
    Creates a cosine at f over T seconds, repeated reps amount of times, used to warm up speaker before chirp
    """
    t = np.arange(0, T, 1/sr)
    y = np.cos(2 * np.pi * f * t)
    y = np.tile(y, (channels, reps))
    
    return y.T

def up_chirp(f0, fmax, T, sr, reps=1, channels=1):
    """
    Creates a chirp from f0 to fmax over T seconds, repeated reps amount of times
    """
    t = np.arange(0, T, 1/sr)
    A = (fmax - f0) / T
    y = np.cos(2 * np.pi * (f0 + 0.5 * A * t) * t)
    y = np.tile(y, (channels, reps))

    return y.T

def down_chirp(f0, fmax, T, sr, reps=1, channels=1):
    """
    Creates a chirp from fmax to f0 over T seconds, repeated reps amount of times
    """
    t = np.arange(0, T, 1/sr)
    A = -(fmax - f0) / T
    y = np.cos(2 * np.pi * (fmax + 0.5 * A * t) * t)
    y = np.tile(y, (channels, reps))

    return y.T

def write_audio_file(file_path, data, sr):
    """
    Writes audio file to system memory.
    @param file_path: Path of the file to write to
    @param data: Audio signal to write (n_channels x n_samples)
    @param sr: Sampling rate
    """
    sf.write(file_path, data.T, sr)
