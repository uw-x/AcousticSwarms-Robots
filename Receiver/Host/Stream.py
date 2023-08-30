import queue
import threading

from globals import *


class Stream():
    def __init__(self, _id = None, debug=False):
        self.started = False
        self.id = _id
        self.debug = debug
        self.incoming = queue.Queue()
        self.processed = queue.Queue()
        self.reset()

    def reset(self):
        self.handler_thread = threading.Thread(target=self.process_incoming_data)

    def start(self):
        if not self.started:
            self.started = True
        
            # Flush any elements from previous stream
            self.flush()
            
            self.handler_thread.start()

            if self.debug:
                 print(type(self), 'started thread', self.handler_thread.name)
        return self

    def stop(self):
        if self.started:
            self.incoming.put(QueueEnd())
            
            if self.debug:
                print(self.incoming.qsize())
            
            self.handler_thread.join()
            self.reset()

            if self.debug:
                print(type(self), 'stopped thread', self.handler_thread.name)
        self.started = False

    def put(self, x):
        self.incoming.put(x)

    def put_nowait(self, x):
        self.incoming.put_nowait(x)
    
    def get(self, timeout = None):
        return self.processed.get(timeout=timeout)

    def get_nowait(self):
        return self.processed.get_nowait()

    def empty(self):
        return self.processed.qsize() == 0

    def outsize(self):
        return self.processed.qsize()

    def insize(self):
        return self.incoming.qsize()

    def flush(self):
        while self.incoming.qsize() > 0:
            self.incoming.get()

        while self.processed.qsize() > 0:
            self.processed.get()

    def process_incoming_data(self):
        while True:
            data = self.incoming.get()

            if self.debug:
                print("Received data:", data)

            if type(data) == QueueEnd:
                break
            
            processed = self.process(data)
            
            for element in processed:
                self.processed.put(element)
                self.on_element_processed(element)

            self.on_batch_processed(processed)

            if self.debug:
                print("Processed queue now has {} elements".format(self.processed.qsize()))

    def on_batch_processed(self, processed_batch):
        """
        Function called after putting a batch of elements in the processed queue
        """
        pass
    
    def on_element_processed(self, element):
        """
        Function called after putting a single element in the processed queue
        """
        pass

    def process(self, data):
        """
        Returns a list of elements to place in the processed queue
        """
        return [data]

    def get_remaining(self):
        res = []
        while not self.empty():
            res.append(self.get())
        
        if self.debug:
            print("Data after processing", res)

        return res