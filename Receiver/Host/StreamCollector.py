import threading
import queue

from globals import *
import numpy as np
from Stream import Stream
import time


class StreamDataSegmenter(Stream):
    def __init__(self, incoming_queue, sem, fw, debug=False, idx=0):
        super().__init__()
        self.current_data = []
        self._leading_frames_dropped = False
        self.incoming = incoming_queue
        self.frame_width = fw
        self.sem = sem
        self.debug = debug
        self.idx = idx
    
    # DO NOT FLUSH I/O STREAM BECAUSE INCOMING MIC DATA MAY BE LOST
    def start(self):
        self.started = True
        
        self.handler_thread.start()

    def process(self, data):
        res = []

        if self.debug:
            print("RECEIVED DATA")

        self.current_data.extend(data)
        
        while len(self.current_data) >= self.frame_width:
            res.append(self.current_data[:self.frame_width])
            self.current_data = self.current_data[self.frame_width:]

        return res

    def on_batch_processed(self, processed_batch):
        # If at least 1 element was added to the queue, release the semaphore
        if len(processed_batch) > 0:
            self.sem.release()

class StreamCollector(Stream):
    def __init__(self, streams, frame_width, debug=False):
        """
        Collects individual robot streams and places them as numpy arrays of size (n_channel x interval) in a queue
        Frame width can either be a single integer, the frame width accross all channels, or an array, the frame
        width across each channel
        """
        super().__init__(self)
        self.channels = [[] for i in range(len(streams))]
        self.sem = threading.Semaphore(0)
        self.done = False
        self.streams = streams
        self.debug = debug

        self.frame_width = frame_width
        if not hasattr(self.frame_width, "__len__"):
            self.frame_width = [self.frame_width] * len(self.channels)

        self.segmentation_streams = []
        for i in range(len(streams)):
            self.segmentation_streams.append(StreamDataSegmenter(streams[i].processed, self.sem, self.frame_width[i], self.debug, idx=i))
            
            if not streams[i].started:
                streams[i].start()

        self.processed = queue.Queue()
        
    def start(self):
        super().start()
        self.frame_packing_thread = threading.Thread(target=self.pack_frames)
        for stream in self.segmentation_streams:
            stream.start()
        
        self.frame_packing_thread.start()
        return self

    def pack_frames(self):
        while True:
            self.sem.acquire()

            if self.done:
                break
            
            if self.debug:
                print([stream.outsize() for stream in self.segmentation_streams])
            
            ready = all([stream.outsize() > 0 for stream in self.segmentation_streams])
            while ready:
                if DEBUG_FRAMES:
                    print("Packing a new frame!")
                frame = [] #(len(self.robots), self.frame_width)
                for i in range(len(self.channels)):
                    data = self.segmentation_streams[i].get_nowait() # This must be true!
                    frame.append(data)
                
                # If all frame widths are the same, return frame as a numpy array
                if max(self.frame_width) == min(self.frame_width):
                    frame = np.array(frame)
                
                self.processed.put(frame)
                
                ready = all([stream.outsize() > 0 for stream in self.segmentation_streams])
        
    def get(self, timeout=None):
        x =  self.processed.get(timeout=timeout)
        return x

    def stop(self):
        super().stop()
        for stream in self.segmentation_streams:
            stream.stop()

        self.done = True
        self.sem.release()

        if self.frame_packing_thread.is_alive:
            self.frame_packing_thread.join()

    def terminate(self):
        self.stop()

    def __del__(self):
        self.terminate()

class SelectorStream(Stream):
    def __init__(self, _id, incoming_queue, sem, debug=False):
        super().__init__(_id, debug)
        self.incoming = incoming_queue
        self.sem = sem

    def process(self, data):
        return [data]

    def on_batch_processed(self, processed_batch):
        if len(processed_batch) > 0:
            self.sem.release()

class StreamSelector(Stream):
    """
    Merges streams into one. Returned elements are tuples in form: (idx, val)
    where idx corresponds to the index of the stream that returned value "val"
    """
    def __init__(self, *args):
        super().__init__(self)
        self.streams = args
        self.sem = threading.Semaphore()
        self.done = False

        self.selector_streams = []
        for idx, stream in enumerate(self.streams):
            ss = SelectorStream(stream.id, stream.processed, self.sem, debug=False)
            self.selector_streams.append(ss)

    def start(self):
        super().start()
        self.frame_packing_thread = threading.Thread(target=self.pack_frames)
        
        for stream in self.selector_streams:
            stream.start()

        for i in range(len(self.streams)):
            if not self.streams[i].started:
                self.streams[i].start()

        self.frame_packing_thread.start()
        return self

    def pack_frames(self):
        while True:
            self.sem.acquire()

            if self.done:
                break
            
            for i, stream in enumerate(self.selector_streams):
                if stream.processed.qsize() > 0:
                    y = stream.get()
                    # print(y)
                    self.processed.put((i, y))
        
    def get(self, timeout=None):
        return self.processed.get(timeout=timeout)

    def stop(self):
        super().stop()
        for stream in self.selector_streams:
            stream.stop()

        self.done = True
        self.sem.release()

        if self.frame_packing_thread.is_alive:
            self.frame_packing_thread.join()

    def terminate(self):
        self.stop()

    def __del__(self):
        self.terminate()