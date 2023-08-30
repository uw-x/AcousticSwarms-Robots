import threading

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("tkAgg")

from Stream import Stream

class StreamPlotter():
    def __init__(self, stream, nchannels, total_samples, samples_per_update, decimation=1, y_bounds=[-10,10]):
        """
        total samples: Total number of samples shown in a single plot
        samples per update: Number of samples to wait for before updating plot (Before downsampling!)
        decimation: only include every decimation'th sample (useful for data with very high sample rates like audio)
        """
        self.stream = stream
        self.downsample_rate = decimation
        self.length = total_samples
        self.fw = samples_per_update
        self.num_channels = nchannels
        self.y_bounds = y_bounds
        
        self.plotdata = np.zeros((nchannels, self.length))
        self.lines = [[] for i in range(nchannels)]

        self.current_data = np.zeros((nchannels, 1))
        self.total_data = np.zeros((nchannels, 1))

        self.fig, axes = plt.subplots(nchannels, 1, squeeze=False)
        
        for i in range(nchannels):
            axes[i][0].set_ylim(y_bounds)
            self.lines[i] = axes[i][0].plot(self.plotdata[i])
            axes[i][0].grid()

        plt.ion()
        plt.show()

    def show(self):
        # self.update_thread = threading.Thread(target=self.plot_stream)
        # self.update_thread.start()
        self.plot_stream()
    
    def plot_stream(self):
        while True:
            try:
                data = self.stream.get()
                if len(self.current_data.shape) == len(data.shape) + 1:
                    data = np.expand_dims(data, axis=len(data.shape))

                self.total_data = np.concatenate([self.total_data, data], axis=1)
                self.current_data = np.concatenate([self.current_data, data], axis=1)

                if self.current_data.shape[-1] >= self.fw:
                    frame = self.current_data[:, :self.fw]
                    frame = frame[:, ::self.downsample_rate]
                    
                    self.current_data = self.current_data[:, self.fw:]

                    self.plotdata = np.roll(self.plotdata, -frame.shape[1], axis=1)
                    self.plotdata[:, -frame.shape[1]:] = frame

                    for i in range(frame.shape[0]):
                        # print("Setting")
                        self.lines[i][0].set_ydata(self.plotdata[i])

                    # print("Drawing")
                    self.fig.canvas.draw()
                    # print("Flusghin")
                    self.fig.canvas.flush_events()

                    plt.pause(0.001)

            except KeyboardInterrupt:
                break

        plt.close(self.fig)