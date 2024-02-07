#!/usr/bin/env python3

"""
File containing the ExperimentDataRecorder class.
"""

# TODO - Dream - Add proper logging through BasicNode class
# TODO - Dream - Add proper error catching with exceptions

# Import standard packages
from os import mkdir
from os.path import isdir

from cv2 import cvtColor, COLOR_GRAY2BGR
from matplotlib.image import imsave
from numpy import array
from csv import DictWriter

# Import standard ROS packages
from geometry_msgs.msg import WrenchStamped, TwistStamped
from thyroid_ultrasound_imaging_support.ImageData.convert_image_message_to_array import convert_image_message_to_array
from thyroid_ultrasound_robot_control_support.Helpers.calc_rpy import calc_rpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_messages.msg import SaveExperimentDataCommand, Float64MultiArrayStamped, image_data_message
from thyroid_ultrasound_imaging_support.Validation.date_stamp_str import date_stamp_str
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData

# Define constants to use when writing data to CSV files
MESSAGE_ID: str = 'Message ID Number'
STAMP_SECS: str = 'Timestamp (s)'
STAMP_NSECS: str = 'Timestamp (ns)'
POSE_X: str = 'Robot Pose X (m)'
POSE_Y: str = 'Robot Pose Y (m)'
POSE_Z: str = 'Robot Pose Z (m)'
POSE_ROLL: str = 'Robot Pose Roll (m)'
POSE_PITCH: str = 'Robot Pose Pitch (m)'
POSE_YAW: str = 'Robot Pose Yaw (m)'
FORCE: str = 'Force (N)'
RAW_IMAGE_NAME: str = 'Raw Image Name'
RAW_IMAGE_ARRAY: str = 'Raw Image Array'
IMAGE_CENTROID: str = 'Centroid Error in X (m)'


class ExperimentDataRecorder(BasicNode):

    def __init__(self, path: str):
        """
        Creates a node for recording data about experiments.

        Parameters
        ----------
        path
            A valid path to a folder where the recorded data should be saved
        """

        # Call the init of the super class
        super().__init__()

        # Ensure the file path is valid
        if isdir(path):
            # Save the location in which to save data
            self.root_save_data_location = path
        else:
            raise Exception(path + " is not a valid path.")

        # Define variable to store the current folder in which data is being saved
        self.current_folder_path = None

        # Define a flag to note when data is being saved
        self.actively_recording_data = False

        # Define flags to note which types of data are being saved
        self.record_pose = False
        self.record_force = False
        self.record_raw_image = False
        self.record_image_data_objects = False
        self.record_image_centroid = False

        # Define variables to save where each kind of data is saved
        self.pose_file = None
        self.pose_file_writer = None
        self.force_file = None
        self.force_file_writer = None
        self.raw_image_folder = None
        self.image_data_objects_folder = None
        self.image_centroid_file = None
        self.image_centroid_file_writer = None

        # Define variables to store the data that has not been recorded yet
        self.pose_queue = []
        self.force_queue = []
        self.raw_image_queue = []
        self.image_data_object_queue = []
        self.image_centroid_queue = []

        # Create the node
        init_node(EXPERIMENT_DATA_RECORDER)

        # Define custom behaviour for the node when it shuts down
        on_shutdown(self.close_open_documents)

        # Define the command subscribers
        Subscriber(EXP_SAVE_DATA_COMMAND, SaveExperimentDataCommand, self.save_data_command_callback)

        # Create a subscriber for the robot pose
        Subscriber(ROBOT_DERIVED_POSE, Float64MultiArrayStamped, self.robot_pose_callback)

        # Create a subscriber for the robot force
        Subscriber(ROBOT_DERIVED_FORCE, WrenchStamped, self.robot_force_callback)

        # Create subscriber for the raw images
        Subscriber(IMAGE_SOURCE, Image, self.raw_image_callback)

        # Create a subscriber for the image data objects
        Subscriber(IMAGE_FILTERED, image_data_message, self.image_data_object_callback)

        # Create a subscriber for the image centroid error
        Subscriber(RC_IMAGE_ERROR, TwistStamped, self.image_centroid_callback)

    def save_data_command_callback(self, data: SaveExperimentDataCommand):
        """
        Selects which data should be saved based on the fields of the message.

        Parameters
        ----------
        data
            A SaveExperimentDataCommand message showing which data should be saved.
        """

        # If any the command has been sent to save any data,
        if data.save_pose or data.save_force or data.save_raw_image or data.save_image_data or data.save_image_centroid:

            # Generate a new data stamp
            date_stamp = date_stamp_str(suffix="")

            # Create a new folder structure for this set of data
            self.current_folder_path = self.root_save_data_location + '/' + date_stamp + '_experiment' + '/'
            mkdir(self.current_folder_path)

            # If saving the pose data
            if data.save_pose:
                # Create a file to save the data in
                self.pose_file = open(self.current_folder_path + "Pose_" + date_stamp + ".csv", mode='w')
                self.pose_file_writer = DictWriter(self.pose_file, delimiter=',',
                                                   fieldnames=[MESSAGE_ID, STAMP_SECS, STAMP_NSECS,
                                                               POSE_X, POSE_Y, POSE_Z,
                                                               POSE_ROLL, POSE_PITCH, POSE_YAW])
                self.pose_file_writer.writeheader()

                # Update the flag that the data needs to be saved
                self.record_pose = data.save_pose

            # If saving the force data
            if data.save_force:
                # Create a file to save the data in
                self.force_file = open(self.current_folder_path + 'Force_' + date_stamp + '.csv', mode='w')
                self.force_file_writer = DictWriter(self.force_file, delimiter=',',
                                                    fieldnames=[MESSAGE_ID, STAMP_SECS, STAMP_NSECS, FORCE])
                self.force_file_writer.writeheader()

                # Update the flag that the data needs to be stored
                self.record_force = data.save_force

            # If saving the raw images
            if data.save_raw_image:
                # Create a new folder to save the images in and save the path to that folder
                self.raw_image_folder = self.current_folder_path + 'RawImages_' + date_stamp + '/'
                mkdir(self.raw_image_folder)

                # Update the flag that the data needs to be stored
                self.record_raw_image = data.save_raw_image

            # If saving the image data objects
            if data.save_image_data:
                # Create a new folder to save the objects in and save the path to that folder
                self.image_data_objects_folder = self.current_folder_path + 'ImageData_' + date_stamp + '/'
                mkdir(self.image_data_objects_folder)

                # Update the flag that the data needs to be stored
                self.record_image_data_objects = data.save_image_data

            if data.save_image_centroid:
                # Create a file to save the data in
                self.image_centroid_file = open(self.current_folder_path + 'Centroid_' + date_stamp + '.csv', mode='w')
                self.image_centroid_file_writer = DictWriter(self.image_centroid_file, delimiter=',',
                                                             fieldnames=[MESSAGE_ID, STAMP_SECS, STAMP_NSECS,
                                                                         IMAGE_CENTROID])
                self.image_centroid_file_writer.writeheader()

                # Update the flag that data needs to be stored
                self.record_image_centroid = data.save_image_centroid

            # Update that new data is now being recorded
            self.actively_recording_data = True

        # otherwise
        else:

            # Note that new data is not being recorded anymore
            self.actively_recording_data = False

            # Close all open documents
            self.close_open_documents()

    def robot_pose_callback(self, message: Float64MultiArrayStamped):
        """
        Adds the robot pose to the queue of data to be recorded as x ,y, z, roll, pitch, yaw data points.

        Parameters
        ----------
        message
            A Float64MultiArrayStamped message containing the pose of the robot as a 4x4 homogeneous
            transformation matrix.
        """

        # If this data is being recorded
        if self.record_pose:
            # Pull the transformation out of the message
            pose = array(message.data.data).reshape((4, 4))

            # Calculate the roll, pitch, and yaw of the pose
            pose_roll, pose_pitch, pose_yaw = calc_rpy(pose[0:3, 0:3])

            # Append the new data to the queue
            self.pose_queue.append({MESSAGE_ID: message.header.seq,
                                    STAMP_SECS: message.header.stamp.secs,
                                    STAMP_NSECS: message.header.stamp.nsecs,
                                    POSE_X: pose[0, 3],
                                    POSE_Y: pose[1, 3],
                                    POSE_Z: pose[2, 3],
                                    POSE_ROLL: pose_roll,
                                    POSE_PITCH: pose_pitch,
                                    POSE_YAW: pose_yaw})

    def robot_force_callback(self, message: WrenchStamped):
        """
        Adds the measured force of the robot in the Z axis to the appropriate queue.

        Parameters
        ----------
        message
            A WrenchStamped message containing the force measured at the end effector in the Z axis.
        """
        # If force data is being recorded
        if self.record_force:
            # Append the new data to the queue
            self.force_queue.append({MESSAGE_ID: message.header.seq,
                                     STAMP_SECS: message.header.stamp.secs,
                                     STAMP_NSECS: message.header.stamp.nsecs,
                                     FORCE: message.wrench.force.z})

    def raw_image_callback(self, message: Image):
        """
        Adds the image captured by the ultrasound probe to the appropriate queue.

        Parameters
        ----------
        message
            A Image message containing the ultrasound image.
        """
        # If the raw images are being saved
        if self.record_raw_image:
            # Append the new data to the queue
            self.raw_image_queue.append({RAW_IMAGE_NAME: self.raw_image_folder + 'RawImage_' + str(
                message.header.seq) + str(message.header.stamp.secs) + '_' + str(message.header.stamp.nsecs) + '.png',
                                         RAW_IMAGE_ARRAY: cvtColor(convert_image_message_to_array(message),
                                                                   COLOR_GRAY2BGR)})

    def image_data_object_callback(self, message: image_data_message):
        """
        Adds the filtered image data objects to the appropriate queue.

        Parameters
        ----------
        message
            An image_data_message object containing the filtered ultrasound image
        """
        # If the image data objects are being saved
        if self.record_image_data_objects:
            # Append the new data to the queue
            self.image_data_object_queue.append(ImageData(image_data_msg=message))

    def image_centroid_callback(self, message: TwistStamped):
        """
        Adds the error of the image centroid to the appropriate queue as distance in the X axis.

        Parameters
        ----------
        message
            A TwistStamped message containing the error of the image_centroid from the center of the image in meters.
        """
        # If image centroid data is being recorded
        if self.record_image_centroid:
            # Append the new data to the queue
            self.image_centroid_queue.append({MESSAGE_ID: message.header.seq,
                                              STAMP_SECS: message.header.stamp.secs,
                                              STAMP_NSECS: message.header.stamp.nsecs,
                                              IMAGE_CENTROID: message.twist.linear.x})

    def close_open_documents(self):
        """
        Closes any open files.
        """
        # Close all open documents
        if self.pose_file is not None:
            self.pose_file.close()
        if self.force_file is not None:
            self.force_file.close()
        if self.image_centroid_file is not None:
            self.image_centroid_file.close()

    def main_loop(self):
        """
        Performs the main function of the node, saving data while it is available when it has been commanded to.
        """

        # if there is data that needs to be saved
        if self.actively_recording_data:
            # If pose data is being recorded and there is new data to record
            if self.record_pose and len(self.pose_queue) > 0:
                # Write the new data to the file
                self.pose_file_writer.writerow(self.pose_queue.pop(-1))
            # If force data is being recorded and there is new data to record
            if self.record_force and len(self.force_queue) > 0:
                # Write the new data to the file
                self.force_file_writer.writerow(self.force_queue.pop(-1))
            # If raw image data is being recorded and there is new data to record
            if self.record_raw_image and len(self.raw_image_queue) > 0:
                # Save the new image to the folder
                temp_entry = self.raw_image_queue.pop(-1)
                imsave(temp_entry[RAW_IMAGE_NAME], temp_entry[RAW_IMAGE_ARRAY])
            # If image data objects are being recorded and there is new data to record
            if self.record_image_data_objects and len(self.image_data_object_queue) > 0:
                # Save the image data object to the folder
                temp_image_data_object: ImageData = self.image_data_object_queue.pop(-1)
                temp_image_data_object.save_object_to_file(self.image_data_objects_folder)
            # If the image centroid is being recorded and there is new data to record
            if self.record_image_centroid and len(self.image_centroid_queue) > 0:
                # Write the image centroid to the file
                self.image_centroid_file_writer.writerow(self.image_centroid_queue.pop(-1))


if __name__ == '__main__':

    # Create the node object
    node = ExperimentDataRecorder(path='/home/ben/thyroid_ultrasound_data/experimentation/')

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    while not is_shutdown():
        node.main_loop()

    print("Node terminated.")
