from helpers import *
from globals import *
from Stream import Stream
import time


if USE_COMPRESSION:
    import opuslib

def get_packet_mode(data):
    if data[0]&0x80:
        mode = 'MODE_CELT_ONLY'
    elif (data[0]&0x60) == 0x60:
        mode = 'MODE_HYBRID'
    else:
        mode = 'MODE_SILK_ONLY'
    
    print(mode)

class CompressedStreamFragment():
    def __init__(self):
        self.residue = []
        self.reset()
    
    def feed(self, data = []):
        self.residue += data
        bytestream = self.residue
        
        if not self.length_read:
            if len(bytestream) < 2:
                return False
            
            if DEBUG_AUDIO_STREAM:
                print("Length Read")
            
            self.length_read = True
            self.length = (bytestream[0] << 8) + bytestream[1]
            
            bytestream = bytestream[2:]
            self.residue = bytestream

            if DEBUG_AUDIO_STREAM:
                print('Length:', self.length)

        if self.length_read and (not self.data_read):
            if len(bytestream) < self.length:
                # self.data.extend(bytestream)
                return False

            if DEBUG_AUDIO_STREAM:
                print("Data Read")
            
            self.data_read = True
            split_index = self.length - len(self.data)
            self.data.extend(bytestream[: split_index])
            bytestream = bytestream[split_index :]
            self.residue = bytestream

            if DEBUG_AUDIO_STREAM:
                print("Data:", self.data[:30])

        return True

    def get(self):
        if self.length_read and self.data_read:
            result = self.data
            self.reset()
            return result
        
        return None

    def reset(self):
        self.length_read = False
        self.data_read = False

        self.length = []
        self.data = []
        
class MicStream(Stream):
    def __init__(self, stream_id = None, debug=False):
        super().__init__(stream_id, debug=debug)
        self.use_compression = False
        self.sr = get_sampling_frequency(SAMPLING_FREQUENCY_DEFAULT)
        self.channels = 1
        self._count_packet_drop_only = False
        self.reset()
    
    def enable_count_packet_drop_only(self):
        self._count_packet_drop_only = True
    
    def disable_count_packet_drop_only(self):
        self._count_packet_drop_only = False

    def update_sampling_frequency(self, sr_idx):
        self.sr = sr_idx
        if self.use_compression:
            self.create_decoder()

    def create_decoder(self, channels=1):
        self.decoder = opuslib.api.decoder.create_state(self.sr.standard_freq, channels)

    def reset(self):
        self.next_sequence_number = 0
        self.dropped_packets = 0
        self.timestamp = None
        self.extra_initial = 0
        self.compressed_stream_fragment = CompressedStreamFragment()
        super().reset()

    def set_sampling_rate(self, sr):
        """
        Takes in a FREQS enum
        """
        self.sr = get_sampling_frequency(sr)

    def get_sampling_rate(self):
        """
        Returns a sampling rate descriptor
        """
        return self.sr

    def process(self, data):
        sequence_number = to_nbit(data[:2]) # First 2 bytes are sequence_number
        # print("[Mic Stream {}] Data length".format(self.id), len(data))
        # if self.tmp and sequence_number > 0:
        #     self.tmp = False
        #     print(data)
        # if len(data) < 200:
        #     print(data)
        data = data[2:]
        
        res = []
        if DEBUG_AUDIO_STREAM:
            print("[Mic Stream {}] Sequence number".format(self.id), sequence_number)
            print("[Mic Stream {}] Next sequence number".format(self.id), self.next_sequence_number)
        
        if self.timestamp is None and sequence_number != 0:
            return res
        elif sequence_number == 0:
            assert self.next_sequence_number == 0
            # print(f'Stream {self.id}:', data[:8])
            # self.mic_timestamp = to_nbit(data[:8]) # Next 8 bytes are metadata
            self.timestamp = to_nbit(data[:8]) & 0xFFFFFFFF
            print(f"[{self.id}] Stream started at interval {to_nbit(data[:2])} with timestamp {self.timestamp}")
        else:
            if not self._count_packet_drop_only:
                # if sequence_number % 100 == 1:
                #     print('Length:', len(data), 'Sequence number:', sequence_number)
                #     print(data)
                data = np.array(data, dtype=np.uint8).tobytes() # Remaining are actual audio samples
                
                # print('Received:', data)
                # print(len(data))
                if self.use_compression:
                    cmp = b''
                    
                    while self.compressed_stream_fragment.feed(data):
                        data = self.compressed_stream_fragment.get()
                        # print(bytes(data))
                        data = opuslib.api.decoder.decode(self.decoder, bytes(data), len(data), 1500, False, 1)
                        # print(len(data))
                        cmp += data
                        assert len(data) == 2 * int(round(0.02 * self.sr.standard_freq))
                        
                        data = []
                    
                    data = cmp
            
                data = np.frombuffer(data, dtype=np.int16)
            else:
                data = np.array([0])

            if self.channels == 2:
                ch1 = data[::2]
                ch2 = data[1::2]
                # print(ch1.shape)
                data = np.column_stack([ch1, ch2])
                # print(data.shape)
                # print(data)
            
            # print('Stream {}:'.format(self.id), sequence_number)
            # print(len(data))
            # print('Decoded:', data)
            if self.next_sequence_number < sequence_number:
                print("[{}] There are dropped packets!".format(self.id))
                print('Expected: {}, Actual: {}'.format(self.next_sequence_number, sequence_number))

            while self.next_sequence_number < sequence_number:
                res.append(np.zeros_like(data))
                self.next_sequence_number += 1
                self.dropped_packets += 1
            
            res.append(data)
        
        self.next_sequence_number += 1

        return res