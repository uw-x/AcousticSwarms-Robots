import serial
import time
import queue, threading
from usb_packet_types import PACKET_TYPES
import numpy as np
import sys
import time
from globals import *
import traceback
import array


class StreamChunkFragment(object):
    def __init__(self):
        self.residue = []
        self.reset()
    
    def feed(self, bytestream = []):
        self.residue.extend(bytestream)
        bytestream = self.residue
        
        if not self.metadata_read:
            if len(bytestream) < 1:
                return False
            
            if DEBUG_USB_STREAM:
                print("Metadata Read")
            
            self.metadata_read = True
            self.metadata = bytestream[0]
            bytestream = bytestream[1:]
            self.residue = bytestream

            if DEBUG_USB_STREAM:
                print('Metadata:', self.metadata)
        
        if self.metadata_read and (not self.length_read):
            if len(bytestream) < 2:
                return False
            
            if DEBUG_USB_STREAM:
                print("Length Read")
            self.length_read = True
            self.length = (bytestream[0] << 8) + bytestream[1]
            bytestream = bytestream[2:]
            self.residue = bytestream

            if DEBUG_USB_STREAM:
                print('Length:', self.length)

        if self.metadata_read and self.length_read and (not self.data_read):
            if len(bytestream) < self.length:
                # self.data.extend(bytestream)
                return False

            if DEBUG_USB_STREAM:
                print("Data Read")
            self.data_read = True
            split_index =self.length - len(self.data)
            self.data.extend(bytestream[: split_index])
            bytestream = bytestream[split_index :]
            self.residue = bytestream

            if DEBUG_USB_STREAM:
                print("Data:", self.data[:30])

        return True

    def get(self):
        if self.length_read and self.metadata_read and self.data_read:
            result = (self.metadata, self.length, self.data)
            self.reset()
            return result
        
        return None

    def reset(self):
        self.metadata_read = False
        self.length_read = False
        self.data_read = False
        
        self.metadata = []
        self.length = []
        self.data = []
        

class USBStream(object):
    def __init__(self, port='/dev/ttyACM1'):
        self.dev = serial.Serial(timeout=1)
        self.dev.setDTR(True)
        self.dev.setPort(port)
        
        self.valid = False
        self.could_open = False

        try:
            self.dev.open()
        except:
            print("Port not open")
            return

        self.could_open = True
        time.sleep(1)
        # self.dev.write('0'.encode())

        # if not self.dev.is_open:
        #     print("Port not open")
        #     return
        # else:
        #     print("Connected")

        self.closed = False

        self.bytes_received = 0
        self.current_stream_fragment = StreamChunkFragment()
        
        self.incoming_bytestream = queue.Queue()
        self.processed_stream = queue.Queue()
        self.stream_parser_thread = threading.Thread(target=self.process_incoming_data)
        self.stream_parser_thread.start()
        
        self.stream_reader_thread = threading.Thread(target=self.monitor_serial, daemon=True)
        self.stream_reader_thread.start()

    def get(self):
        return self.processed_stream.get()

    def get_nowait(self):
        return self.processed_stream.get_nowait()

    def empty(self):
        return self.processed_stream.qsize() == 0

    def stop(self):
        self.incoming_bytestream.put(None)

    def process_incoming_data(self):
        while True:
            data = self.incoming_bytestream.get()
            #print("process data---", data)
            if data is None:
                break
            
            self.valid = True
            if DEBUG_USB_STREAM:
                print("Now processing {} bytes of data".format(len(data)))

            while self.current_stream_fragment.feed(data):
                processed = self.current_stream_fragment.get()
                
                if DEBUG_USB_STREAM:
                    print("Succesfully processed 1 chunk of data, data length:", processed[1])
                self.processed_stream.put_nowait(processed)
                data = []
    
    def monitor_serial(self):
        while True:
            # Blocks until 1 bytes is available
            try:
                bs = self.dev.read(1)
                
                if len(bs) == 0:
                    if self.closed:
                        break
                    else:
                        continue

                data = list(bs)
                '''
                if len(data) == 0:
                    print("ddddd1")
                    continue
                print("read---", data)
                '''

                # If 1 byte was read, then it's likely there are many more bytes
                
                if self.dev.in_waiting:
                    wait_byte = self.dev.read(self.dev.in_waiting)
                    #print(wait_byte)
                    data.extend(list(wait_byte))

                # bs = self.dev.readline()
                # print('Numpy', np.frombuffer(bs, dtype=np.uint8)[1:-1])
                # print(str(bs)[1:-1].replace('Ö', '0'))
                # assert bs[-1] == ord('\n')
                # data = list(bs)[:-1]
                # print(data)

                if DEBUG_USB_STREAM:
                    print("Received {} bytes of data".format(len(data)))
                self.incoming_bytestream.put_nowait(data)
            except:
                print("USB Stream received exception!")
                traceback.print_exc()
                break
        
        self.dev.close()
        
        print("USB DEVICE CLOSED")

    def send(self, type: PACKET_TYPES, data):
        """
        Sends a command to the received
        type: Type of packet to send
        data: Data stream (as a list of bytes)
        """
        length = len(data)
        data = [type, (length >> 8) & 0xFF, length & 0xFF, *data]
        # print(data)
        data = bytes(data)
        # print(data)
        self.dev.write(data)
        self.dev.flushOutput()
        
        # THIS IS TO AVOID BLUETOOTH ISSUES
        # MAKE SURE THAT FOR AUDIO OUTPUT STREAMING TO FIX THIS
        time.sleep(0.01)

    def close(self):
        self.closed = True
        self.dev.flushOutput()
        self.incoming_bytestream.put(None)
        self.processed_stream.put(None)
        self.stream_parser_thread.join()

    def flush(self):
        self.dev.flushInput()
        
        while self.incoming_bytestream.qsize() > 0:
            self.incoming_bytestream.get()

        while self.processed_stream.qsize() > 0:
            self.processed_stream.get()

if __name__ == "__main__":
    port = '/dev/ttyACM1'
    stream = USBStream(port=port)

    while True:
        try:
            data = stream.get()
            # handle(data)
            message = ''.join([chr(x) for x in data[2]])
            print('Metadata: {}, Length: {}'.format(data[0], data[1]))
            # print('Data: {}'.format(data[2]))
            print('Data: {}'.format(message))
        except KeyboardInterrupt:
            stream.close()
            sys.exit()
