import matplotlib.pyplot as plt
import numpy as np

import matplotlib.animation as animation
from matplotlib.lines import Line2D


class ControllerMonitor:
    def __init__(self, ax, maxt=2, dt=0.02,
                 y_axis_limits: tuple = (-0.5, 2),
                 title: str = None, x_label: str = None, y_label: str = None):

        # Save the axis being used for this controller
        self.ax = ax

        # Set the appearance of the subplot
        if title is not None:
            self.ax.set_title(title)
        if x_label is not None:
            self.ax.set_xlabel(x_label)
        if y_label is not None:
            self.ax.set_ylabel(y_label)

        self.dt = dt
        self.maxt = maxt

        # Define variables to store data to be plotted
        self.time_data = [0]
        self.set_point_data = [0]
        self.actual_value_data = [0]

        # Define lines that will be plotted
        self.set_point_line = Line2D(self.time_data, self.set_point_data, color='r')
        self.actual_value_line = Line2D(self.time_data, self.actual_value_data, color='g')

        # Add the lines to the plot
        self.ax.add_line(self.set_point_line)
        self.ax.add_line(self.actual_value_line)

        # Define the forward and backwards limits of the time axis
        self.prior_time_shown = 5  # seconds
        self.future_time_shown = 5  # seconds

        self.max_age_of_data = self.prior_time_shown + self.future_time_shown - 1  # seconds

        self.x_limits = (-self.prior_time_shown, self.future_time_shown)

        # Set the limits of the plot
        self.ax.set_xlim(self.x_limits)
        self.ax.set_ylim(-y_axis_limits[0], y_axis_limits[1])

    def update(self, data):
        y = data[0]
        z = data[1]

        # This slightly more complex calculation avoids floating-point issues
        # from just repeatedly adding `self.dt` to the previous value.
        t = self.time_data[0] + len(self.time_data) * self.dt

        """lastt = self.time_data[-1]
        if lastt >= self.time_data[0] + self.maxt:  # reset the arrays
            self.time_data = [self.time_data[-1]]
            self.set_point_data = [self.set_point_data[-1]]
            self.actual_value_data = [self.actual_value_data[-1]]"""
        self.time_data.append(t)
        self.set_point_data.append(y)
        self.actual_value_data.append(z)

        oldest_allowed_time = self.time_data[-1] - self.max_age_of_data

        removal_index = 0

        for time in self.time_data:
            if time >= oldest_allowed_time:
                break
            removal_index = removal_index + 1

        self.time_data = self.time_data[removal_index::]
        self.set_point_data = self.set_point_data[removal_index::]
        self.actual_value_data = self.actual_value_data[removal_index::]
        if self.time_data[-1] >= self.x_limits[1] - 1:
            self.x_limits = (self.time_data[-1] - self.prior_time_shown, self.time_data[-1] + self.future_time_shown)
            self.ax.set_xlim(self.x_limits)
            self.ax.figure.canvas.draw()

        self.set_point_line.set_data(self.time_data, self.set_point_data)
        self.actual_value_line.set_data(self.time_data, self.actual_value_data)
        return self.set_point_line, self.actual_value_line


def emitter(p=0.1):
    """Return a random value in [0, 1) with probability p, else 0."""
    while True:
        v = np.random.rand()
        if v > p:
            yield (0., 1.)
        else:
            yield (np.random.rand(), 1.)


# Fixing random state for reproducibility
np.random.seed(19680801 // 10)


fig, ax = plt.subplots(1, 2)
scope = ControllerMonitor(ax[0], title="Scope", x_label="seconds", y_label="meters")
scope_two = ControllerMonitor(ax[1], title="Scope Two")

# pass a generator in "emitter" to produce data for the update func
ani = animation.FuncAnimation(fig, scope.update, emitter, interval=50,
                              blit=True, save_count=100)

ani = animation.FuncAnimation(fig, scope_two.update, emitter, interval=50,
                              blit=True, save_count=100)

plt.show()
