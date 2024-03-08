#!/usr/bin/env python3

"""
File containing ControllerTuningGraphsNode class.
"""

# Import standard ROS packages
from geometry_msgs.msg import WrenchStamped, TwistStamped
from armer_msgs.msg import ManipulatorState

# Import standard python packages
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D
from numpy import floor, ceil

# Import custom ROS packages
from thyroid_ultrasound_messages.msg import Float64Stamped

# Import custom python packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_support.TopicNames import *
from thyroid_ultrasound_robot_control_support.Helpers.calc_rpy import calc_rpy
from thyroid_ultrasound_robot_control_support.Helpers.convert_pose_to_transform_matrix import \
    convert_pose_to_transform_matrix


# TODO - Dream - Build in proper logging through BasicNode class
# TODO - Dream - Add proper exception handling for each part
# TODO - Dream - Fix the updating of the chart limits

class ControllerTuningGraphsNode(BasicNode):

    def __init__(self):
        # Call the init function of the super class
        super().__init__()

        # Create the node object
        init_node(CONTROLLER_TUNING_GRAPHS)

        # Define custom node shutdown behavior
        on_shutdown(self.shutdown_node)

        # Get the current time to use to offset all other times
        temp_time = Time.now()

        # Calculate the time stamp as a single float number
        self.start_time = temp_time.secs + (temp_time.nsecs / 10 ** 9)

        # Define variables to store information for each graph
        self.latest_pos_x_lin_time = 0.0
        self.latest_pos_x_lin_data = 0.0
        self.pos_x_lin_set_point = 0.0

        self.latest_img_y_lin_time = 0.0
        self.latest_img_y_lin_data = 0.0
        self.img_y_lin_set_point = 0.0

        self.latest_force_z_lin_time = 0.0
        self.latest_force_z_lin_data = 0.0
        self.force_z_lin_set_point = 0.0

        self.latest_img_x_ang_time = 0.0
        self.latest_img_x_ang_data = 0.0
        self.img_x_ang_set_point = 0.0

        self.latest_pos_y_ang_time = 0.0
        self.latest_pos_y_ang_data = 0.0
        self.pos_y_ang_set_point = 0.0

        self.latest_pos_z_ang_time = 0.0
        self.latest_pos_z_ang_data = 0.0
        self.pos_z_ang_set_point = 0.0

        # Define a flag for when to capture angular pose data
        self.capture_ang_pose_data = False

        # Define a flag for when to capture the set-points
        self.show_constant_ang_pose_set_points = False

        # Define subscribers for each value that needs to be monitored
        Subscriber(ROBOT_DERIVED_FORCE, WrenchStamped, self.robot_force_callback)
        Subscriber(RC_FORCE_SET_POINT, Float64, self.robot_force_set_point_callback)
        Subscriber(RC_POSITION_ERROR, TwistStamped, self.position_error_callback)
        Subscriber(RC_IMAGE_ERROR, TwistStamped, self.image_error_callback)
        Subscriber(RC_PATIENT_CONTACT_ERROR, Float64Stamped, self.patient_contact_callback)
        # Subscriber(ARMER_STATE, ManipulatorState, self.robot_pose_callback)

        # Define subscribers to listen for when to capture angular pose data
        # Subscriber(CREATE_TRAJECTORY, Float64, self.create_trajectory_command_callback)
        # Subscriber(CLEAR_TRAJECTORY, Bool, self.clear_trajectory_command_callback)

        # Create the figure used for showing the plots
        self.fig, ax = plt.subplots(3, 2)

        # Add a title to the window
        self.fig.canvas.manager.set_window_title("PID Controller Graphs")

        # Create individual graph objects for monitoring each controller
        self.pos_x_lin_monitor = ControllerMonitor(ax[0][0], y_axis_limits=(-0.025, 0.025),
                                                   y_axis_limit_multiples=(0.005, 0.005),
                                                   title="Position - X Linear", y_label="Position Error (m)",
                                                   x_label="Time (s)")

        self.img_y_lin_monitor = ControllerMonitor(ax[0][1], y_axis_limits=(-0.025, 0.025),
                                                   y_axis_limit_multiples=(0.005, 0.005),
                                                   title="Image - Y Linear", y_label="Position Error (m)",
                                                   x_label="Time (s)")

        self.force_z_lin_monitor = ControllerMonitor(ax[1][0], y_axis_limits=(-0.5, 10),
                                                     y_axis_limit_multiples=(2.5, 2.5),
                                                     title="Force - Z Linear",
                                                     y_label="Force (N)", x_label="Time (s)")

        self.img_x_ang_monitor = ControllerMonitor(ax[1][1], y_axis_limits=(-1., 1),
                                                   y_axis_limit_multiples=(0.25, 0.25),
                                                   title="Img - X Angular",
                                                   y_label="Slope", x_label="Time (s)")

        self.pos_y_ang_monitor = ControllerMonitor(ax[2][0], y_axis_limits=(-10, 10),
                                                   y_axis_limit_multiples=(5, 5),
                                                   title="Pose - Pitch",
                                                   y_label="Position (deg)", x_label="Time (s)")

        self.pos_z_ang_monitor = ControllerMonitor(ax[2][1], y_axis_limits=(-10, 10),
                                                   y_axis_limit_multiples=(5, 5),
                                                   title="Pose - Yaw",
                                                   y_label="Position (deg)", x_label="Time (s)")

        # Create animations for each graph object
        animation.FuncAnimation(self.fig, self.pos_x_lin_monitor.update, self.get_latest_pos_x_lin_values,
                                interval=50,
                                blit=True, save_count=100)

        animation.FuncAnimation(self.fig, self.img_y_lin_monitor.update, self.get_latest_img_y_lin_values,
                                interval=50,
                                blit=True, save_count=100)

        animation.FuncAnimation(self.fig, self.force_z_lin_monitor.update, self.get_latest_force_z_lin_values,
                                interval=50,
                                blit=True, save_count=100)

        animation.FuncAnimation(self.fig, self.img_x_ang_monitor.update, self.get_latest_img_x_ang_values,
                                interval=50,
                                blit=True, save_count=100)

        animation.FuncAnimation(self.fig, self.pos_y_ang_monitor.update, self.get_latest_pos_y_ang_values,
                                interval=50,
                                blit=True, save_count=100)

        animation.FuncAnimation(self.fig, self.pos_z_ang_monitor.update, self.get_latest_pos_z_ang_values,
                                interval=50,
                                blit=True, save_count=100)

        # Show the plot
        plt.show()

    ##############################
    # Define ROS message callbacks
    # region

    def position_error_callback(self, msg: TwistStamped):
        self.latest_pos_x_lin_time = (msg.header.stamp.secs + (msg.header.stamp.nsecs / 10 ** 9)) - self.start_time
        self.latest_pos_x_lin_data = msg.twist.linear.x
        self.latest_pos_y_ang_time = self.latest_pos_x_lin_time
        self.latest_pos_y_ang_data = msg.twist.angular.y
        self.latest_pos_z_ang_time = self.latest_pos_x_lin_time
        self.latest_pos_z_ang_data = msg.twist.angular.z

    def image_error_callback(self, data: TwistStamped):
        self.latest_img_y_lin_time = (data.header.stamp.secs + (data.header.stamp.nsecs / 10 ** 9)) - self.start_time
        self.latest_img_y_lin_data = data.twist.linear.x

    def robot_force_callback(self, data: WrenchStamped):
        self.latest_force_z_lin_time = (data.header.stamp.secs + (data.header.stamp.nsecs / 10 ** 9)) - self.start_time
        self.latest_force_z_lin_data = data.wrench.force.z

    def patient_contact_callback(self, msg: Float64Stamped):
        self.latest_img_x_ang_time = (msg.header.stamp.secs + (msg.header.stamp.nsecs / 10 ** 9)) - self.start_time
        self.latest_img_x_ang_data = msg.data.data

    def robot_force_set_point_callback(self, data: Float64):
        self.force_z_lin_set_point = data.data

    """def robot_pose_callback(self, message: ManipulatorState):
        
        Adds the robot pose to the queue of data to be recorded as x ,y, z, roll, pitch, yaw data points.

        Parameters
        ----------
        message
            A Float64MultiArrayStamped message containing the pose of the robot as a 4x4 homogeneous
            transformation matrix.
        

        # if self.capture_ang_pose_data:

        # Pull out the timestamp
        self.latest_pos_y_ang_time = (message.ee_pose.header.stamp.secs +
                                      (message.ee_pose.header.stamp.nsecs / 10 ** 9)) - self.start_time
        self.latest_pos_z_ang_time = self.latest_pos_y_ang_time

        # Pull the transformation out of the message
        pose = convert_pose_to_transform_matrix(message.ee_pose.pose)

        # Calculate the roll, pitch, and yaw of the pose
        pose_roll, pose_pitch, pose_yaw = calc_rpy(pose[0:3, 0:3])

        # Set the latest data points
        self.latest_pos_y_ang_data = pose_pitch
        self.latest_pos_z_ang_data = pose_yaw

        if not self.show_constant_ang_pose_set_points:
            self.pos_y_ang_set_point = self.latest_pos_y_ang_data
            self.pos_z_ang_set_point = self.latest_pos_z_ang_data

    def create_trajectory_command_callback(self, _):
        self.show_constant_ang_pose_set_points = True

    def clear_trajectory_command_callback(self, _):
        self.show_constant_ang_pose_set_points = False"""

    # endregion

    ############################
    # Define animation callbacks
    # region
    def get_latest_pos_x_lin_values(self):
        yield self.latest_pos_x_lin_time, self.latest_pos_x_lin_data, self.pos_x_lin_set_point

    def get_latest_img_y_lin_values(self):
        yield self.latest_img_y_lin_time, self.latest_img_y_lin_data, self.img_y_lin_set_point

    def get_latest_force_z_lin_values(self):
        yield self.latest_force_z_lin_time, self.latest_force_z_lin_data, self.force_z_lin_set_point

    def get_latest_img_x_ang_values(self):
        yield self.latest_img_x_ang_time, self.latest_img_x_ang_data, self.img_x_ang_set_point

    def get_latest_pos_y_ang_values(self):
        yield self.latest_pos_y_ang_time, self.latest_pos_y_ang_data, self.pos_y_ang_set_point

    def get_latest_pos_z_ang_values(self):
        yield self.latest_pos_z_ang_time, self.latest_pos_z_ang_data, self.pos_z_ang_set_point

    # endregion

    def shutdown_node(self):
        """
        Define the custom shutdown behavior of the node to close the window.
        """
        print("Shutdown signal received.")
        plt.close(self.fig)


class ControllerMonitor:
    def __init__(self, ax, y_axis_limits: tuple = (0, 2), y_axis_limit_multiples: tuple = (0.5, 0.5),
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

        # Calculate the max age of the data allowed to be saved
        self.max_age_of_data = self.prior_time_shown + self.future_time_shown - 1  # seconds

        # Save the x and y limits of the graph
        self.x_limits = (-self.prior_time_shown, self.future_time_shown)
        self.y_limits = (y_axis_limits[0], y_axis_limits[1])
        self.y_limit_lower_multiple = y_axis_limit_multiples[0]
        self.y_limit_upper_multiple = y_axis_limit_multiples[1]

        # Set the limits of the plot
        self.ax.set_xlim(self.x_limits)
        self.ax.set_ylim(self.y_limits[0], self.y_limits[1])

    def update(self, data):
        """
        The function called to update the graph at each data point.
        """

        # Rename the data passed in for clarity
        new_time = data[0]
        new_actual_value = data[1]
        new_set_point_value = data[2]

        # Add the new data to each data set
        self.time_data.append(new_time)
        self.set_point_data.append(new_set_point_value)
        self.actual_value_data.append(new_actual_value)

        # Calculate the oldest any data is allowed to be within the saved data set
        oldest_allowed_data_age = self.time_data[-1] - self.max_age_of_data

        # Note the index at which the data stops being too old
        removal_index = 0

        # Go through the time data set to determine where data stops being too old
        for time in self.time_data:
            if time >= oldest_allowed_data_age:
                break
            removal_index = removal_index + 1

        # Remove any data which is too old
        self.time_data = self.time_data[removal_index::]
        self.set_point_data = self.set_point_data[removal_index::]
        self.actual_value_data = self.actual_value_data[removal_index::]

        # Create a variable to save a new upper limit if it is needed
        new_upper_limit = (0.5 + ceil(max([max(self.actual_value_data), max(self.set_point_data)]) /
                                      self.y_limit_upper_multiple)) * self.y_limit_upper_multiple

        """# Find the largest data value in the data set
        actual_data_upper_limit = max(self.actual_value_data)

        # If the largest data value is bigger than the current upper y limit
        if abs(actual_data_upper_limit - self.y_limits[1]) > self.y_limit_upper_multiple:
            # Save the new upper limit
            new_upper_limit = actual_data_upper_limit

        # Find the largest data value in the set point data set
        set_point_upper_limit = max(self.set_point_data)

        # If the largest set point data is bigger than the current upper y limit
        if abs(set_point_upper_limit - self.y_limits[1]) > self.y_limit_upper_multiple:

            # If the set point upper limit is also bigger than the largest actual data value
            if set_point_upper_limit > actual_data_upper_limit:
                # Save the new upper limit
                new_upper_limit = set_point_upper_limit"""

        # Create a variable to save a new lower limit if it is needed
        new_lower_limit = (-0.5 + floor(min([min(self.actual_value_data), min(self.set_point_data)]) /
                                        self.y_limit_lower_multiple)) * self.y_limit_lower_multiple

        """# Find the lowest data value in the data set
        actual_data_lower_limit = min(self.actual_value_data)

        # If the lowest data value is lower than the current lower y limit
        if abs(actual_data_lower_limit - self.y_limits[0]) > self.y_limit_lower_multiple:
            # Save the new lower limit
            new_lower_limit = actual_data_lower_limit

        # Find the lowest data value in the set point data set
        set_point_lower_limit = min(self.set_point_data)

        # If the lowest set point data is lower than the current lower y limit
        if abs(set_point_lower_limit - self.y_limits[0]) > self.y_limit_lower_multiple:

            # If the set point lower limit is also lower than the lowest actual data value
            if set_point_lower_limit < actual_data_lower_limit:
                # Save the new lower limit
                new_lower_limit = set_point_lower_limit"""

        # Define a flag to see if either y limit has been changed
        y_limit_changed = False

        # If a new upper limit has been calculated
        if not new_upper_limit == self.y_limits[1]:
            # Calculate and save the new upper y limit
            self.y_limits = (self.y_limits[0], new_upper_limit)

            # Note that the y limits have changed
            y_limit_changed = True

        # If a new lower limit has been calculated
        if not new_lower_limit == self.y_limits[0]:
            # Calculate and save the new lower y limit
            self.y_limits = (new_lower_limit, self.y_limits[1])

            # Note that the y limits have changed
            y_limit_changed = True

        # If the y limits have changed
        if y_limit_changed:
            # Change the y limits on the graph
            self.ax.set_ylim(self.y_limits[0], self.y_limits[1])

        # Check to see if the time axis of the graph needs to be updated
        if self.time_data[-1] >= self.x_limits[1] - 1:
            # Calculate and save the new time axis limits
            self.x_limits = (self.time_data[-1] - self.prior_time_shown, self.time_data[-1] + self.future_time_shown)

            # Set the x limits on the graph
            self.ax.set_xlim(self.x_limits)

            # Redraw the graph
            self.ax.figure.canvas.draw()

        # Add the new data to the set point and actual data lines
        self.set_point_line.set_data(self.time_data, self.set_point_data)
        self.actual_value_line.set_data(self.time_data, self.actual_value_data)

        # Return each line
        return self.set_point_line, self.actual_value_line


if __name__ == "__main__":
    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    node = ControllerTuningGraphsNode()
