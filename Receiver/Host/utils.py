import librosa
import soundfile as sf
import os
import json
import numpy as np
import random


def seed_all(seed):
    random.seed(seed)
    np.random.seed(seed)

def read_input_dir(dir, sr):
    metadata_path = os.path.join(dir, 'metadata.json')
    with open(metadata_path, 'rb') as f:
        metadata = json.load(f)

    audio_path = os.path.join(dir, 'received.wav')
    audio = read_audio_file(audio_path, sr)

    return metadata, audio

def read_audio_file(file_path, sr) -> np.ndarray:
    """
    Reads audio file to system memory.
    """
    return librosa.core.load(file_path, mono=False, sr=sr)[0]

def write_audio_file(file_path, data, sr):
    """
    Writes audio file to system memory.
    @param file_path: Path of the file to write to
    @param data: Audio signal to write (n_channels x n_samples)
    @param sr: Sampling rate
    """
    sf.write(file_path, data.T, sr)

def write_metadata(file_path, metadata):
    with open(file_path, 'w') as f:
        json.dump(metadata, f)