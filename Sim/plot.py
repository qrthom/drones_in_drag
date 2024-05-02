import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class DroneAnimation:
    def __init__(self, data, figsize=(10, 6), interval=50):  # interval in milliseconds
        self.data = data
        self.interval = interval
        self.fig, self.ax = plt.subplots(figsize=figsize)
        self.ax.set_xlim(-6.5, 6.5)  # Adjust based on your data range
        self.ax.set_ylim(-6.5, 6.5)  # Adjust based on your data range
        
        # Initialize lines for each drone to plot the paths
        self.paths = [self.ax.plot(data[i, 0, 0], data[i, 1, 0], alpha=0.3)[0] for i in range(8)]
        # Initialize dots for each drone to show current positions
        self.dots = [self.ax.plot(data[i, 0, 0], data[i, 1, 0], 'o')[0] for i in range(8)]

    def update(self, frame):
        # Update the path and the position of the dot for each drone
        for i, (path, dot) in enumerate(zip(self.paths, self.dots)):
            path.set_data(self.data[i, 0, :frame], self.data[i, 1, :frame])
            dot.set_data(self.data[i, 0, frame], self.data[i, 1, frame])
        return self.paths + self.dots

    def animate(self):
        # Create the animation with control over the frame update interval
        ani = FuncAnimation(self.fig, self.update, frames=self.data.shape[2], interval=self.interval, blit=True)
        plt.show()