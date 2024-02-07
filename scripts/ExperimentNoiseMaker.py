#!/usr/bin/env python3

"""
File containing ExperimentNoiseMaker class.
"""

# Import standard python packages
from numpy import sin, pi

# Import standard ROS packages
from geometry_msgs.msg import TwistStamped

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *


class ExperimentNoiseMaker(BasicNode):

    def __init__(self, pub_rate: int = 50, time_to_complete_full_noise_cycle: float = 2.0):
        """
        Creates a node for sending velocity command noise.

        Parameters
        ----------
        pub_rate
            The rate at which new velocity messages will be published
        time_to_complete_full_noise_cycle
            The time, in seconds, to complete one full cycle of noise
        """
        # Call the constructor of the super class
        super().__init__()

        # Create the node
        init_node(EXPERIMENT_NOISE_MAKER)

        # Create a flag to when to create noise
        self.actively_creating_noise = False

        # Define a variable for calculating the noise
        self.noise_progress_counter = int(0)

        # Define the maximum value for the progress counter
        self.progress_counter_max = int(pub_rate * time_to_complete_full_noise_cycle)

        # Create a rate object to control how long the node sleeps
        self.rate = Rate(pub_rate)

        # Create a publisher for publishing velocity noise
        self.velocity_commands_publisher = Publisher(EXP_NOISE_VELOCITIES, TwistStamped, queue_size=1)

        # Define a subscriber for listening to noise commands
        Subscriber(EXP_CREATE_NOISE_COMMAND, Bool, self.create_noise_command_callback)

    def create_noise_command_callback(self, message: Bool):
        """
        Save the control command sent to the node.
        """
        self.actively_creating_noise = message.data

    def main_loop(self):
        # Create a new message and stamp it
        new_message = TwistStamped()
        new_message.header.stamp = Time.now()

        # If current creating noise
        if self.actively_creating_noise:
            # Add the noise in the Y axis
            new_message.twist.linear.y = round(
                .01 * sin(2 * pi * (self.noise_progress_counter / self.progress_counter_max)), 5)

            # Increment the counter for the next step
            self.noise_progress_counter = int((self.noise_progress_counter + 1) % self.progress_counter_max)

        # Publish the new message
        self.velocity_commands_publisher.publish(new_message)

        # Wait before continuing
        self.rate.sleep()


if __name__ == '__main__':

    # Create the node
    node = ExperimentNoiseMaker()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    while not is_shutdown():
        node.main_loop()

    print("Node terminated.")
