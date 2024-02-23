#!/usr/bin/env python3

"""
File containing UserInterface class.
"""

# TODO - Dream - Add robot controls to allow the user to move the robot around and override the image segmentation
# TODO - Dream - Add robot controls to allow the user to manually scan but keep the image segmentation, force control,
#  and balance control active
# TODO - Dream - Add a command to stop all motion and return all states back to the robot not moving
# TODO - Medium - Add trajectory pause button and activation and deactivation of robot controls

# Import standard packages
from tkinter import *
from tkinter.scrolledtext import ScrolledText
from tkinter.filedialog import askdirectory
import tkinter.ttk as ttk
from argparse import ArgumentParser

# Import ROS packages
from geometry_msgs.msg import WrenchStamped, TwistStamped, Twist
from std_msgs.msg import String, UInt8

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_support.TopicNames import *
from thyroid_ultrasound_messages.msg import SaveExperimentDataCommand, RegisteredDataMsg

# Define constants used for logging purposes
VERBOSE: int = int(0)
# NORMAL: int = int(1)

# Define constant to use as incremental increase and decrease value for force set-point
INCREMENTAL_FORCE_CHANGE: float = 0.1
NUM_DIGITS_OF_FORCE_TO_DISPLAY: int = int(2)

# Define constants for GUI elements
START_IMAGE_STREAMING: str = "Start\nImage Streaming"
STOP_IMAGE_STREAMING: str = "Stop\nImage Streaming"
START_IMAGE_CONTROL: str = "Start\nImage Control"
STOP_IMAGE_CONTROL: str = "Stop\nImage Control"
START_FORCE_CONTROL: str = "Start\nForce Control"
STOP_FORCE_CONTROL: str = "Stop\nForce Control"
START_BALANCING_CONTROL: str = "Start\nBalancing Control"
STOP_BALANCING_CONTROL: str = "Stop\nBalancing Control"
START_POSE_CONTROL: str = "Start\nPose Control"
STOP_POSE_CONTROL: str = "Stop\nPose Control"
TEST_FORCE_CONTROL: str = "Test Force\n Profile"
STOP_TEST_FORCE_CONTROL: str = "Stop Testing\n Force Control"
START_SAVING_IMAGES: str = "Start Saving Images"
STOP_SAVING_IMAGES: str = "Stop Saving Images"
START_SENDING_OVERRIDE_VALUE: str = "Start sending\noverride value"
STOP_SENDING_OVERRIDE_VALUE: str = "Stop sending\noverride value"
START_SENDING_REGISTERED_DATA_OVERRIDE_VALUE: str = "Start sending registered data override value"
STOP_SENDING_REGISTERED_DATA_OVERRIDE_VALUE: str = "Stop sending registered data override value"
START_SAVING_EXPERIMENT_DATA: str = "Start saving\nexperiment data"
STOP_SAVING_EXPERIMENT_DATA: str = "Stop saving\nexperiment data"
TOGGLE_SAVING_EXPERIMENT_DATA: str = "Toggle saving\nexperiment data"
START_CREATING_VELOCITY_NOISE: str = "Start creating\nvelocity noise"
STOP_CREATING_VELOCITY_NOISE: str = "Stop creating\nvelocity noise"

# Define constants for parameters of widgets
WIDGET_TEXT: str = 'text'
WIDGET_STATE: str = 'state'
BUTTON_PRESS: str = '<ButtonPress-1>'
BUTTON_RELEASE: str = '<ButtonRelease-1>'

# Define grid geometry constants
LEFT_COLUMN: int = int(0)
LL_MIDDLE_COLUMN: int = int(1)
L_MIDDLE_COLUMN: int = int(2)
LR_MIDDLE_COLUMN: int = int(3)
MIDDLE_COLUMN: int = int(4)
RL_MIDDLE_COLUMN: int = int(5)
R_MIDDLE_COLUMN: int = int(6)
RR_MIDDLE_COLUMN: int = int(7)
RIGHT_COLUMN: int = int(8)

SINGLE_COLUMN: int = int(1)
TWO_COLUMN: int = int(2)
THREE_COLUMN: int = int(3)
FOUR_COLUMN: int = int(4)
FULL_WIDTH: int = int(9)

SINGLE_ROW: int = int(1)
DOUBLE_ROW: int = int(2)
TRIPLE_ROT: int = int(3)

GRAPHICS_WINDOW: int = int(3)

# Define empty status string
EMPTY_STATUS: str = "STATUS: "

# Define function mode constants
TESTING: int = int(0)
RUNNING: int = int(1)

# Define constants for yes and no radio buttons
YES_BUTTON: int = int(1)
NO_BUTTON: int = int(0)

# Define constants for robot control
POSITIVE_X: str = '+X'
POSITIVE_Y: str = '+Y'
POSITIVE_PITCH: str = '+Pitch'
POSITIVE_YAW: str = '+Yaw'
NEGATIVE_X: str = '-X'
NEGATIVE_Y: str = '-Y'
NEGATIVE_PITCH: str = '-Pitch'
NEGATIVE_YAW: str = '-Yaw'
NO_MOVEMENT: str = 'STOP'


class UserInterface(BasicNode):
    def __init__(self, parent=None):

        # Add a call to the parent class
        super().__init__()

        # Added to ensure TKinter works in the ROS framework
        self.parent = parent

        # Allow an argument to be passed to the code that determines which mode is being used
        parser = ArgumentParser()
        parser.add_argument("--testing_mode", "--tm", dest="testing_mode", action="store_true", default=False,
                            help="Functionality of the GUI")
        parser.add_argument("--experimentation_mode", "--em", dest="experimentation_mode", action="store_true",
                            default=False, help="Functionality of the GUI")
        parser.add_argument("__name", default="")
        parser.add_argument("__log", default="")

        # Parse the arguments passed to the code
        passed_arguments = parser.parse_args()

        # ---------------------------------------
        # Create ROS components of the User Interface
        # region

        # Startup the node
        init_node('UserInterface')

        # Define custom shutdown behavior
        on_shutdown(self.shutdown_node)

        # Create subscribers to listen to the PID values from the robot control node
        self.pid_controller_publisher = Publisher(CONTROLLER_SELECTOR, UInt8, queue_size=1)
        Subscriber(P_GAIN_CURRENT, Float64, self.p_gain_message_callback)
        Subscriber(I_GAIN_CURRENT, Float64, self.i_gain_message_callback)
        Subscriber(D_GAIN_CURRENT, Float64, self.d_gain_message_callback)
        self.p_gain_publisher = Publisher(P_GAIN_SETTING, Float64, queue_size=1)
        self.i_gain_publisher = Publisher(I_GAIN_SETTING, Float64, queue_size=1)
        self.d_gain_publisher = Publisher(D_GAIN_SETTING, Float64, queue_size=1)

        # Create a publisher to publish the overall speed factor for the robot
        self.speed_selector_publisher = Publisher(RC_OVERALL_ROBOT_SPEED, Float64, queue_size=1)

        # Create a publisher to publish the command to use image feedback
        self.use_image_feedback_command_publisher = Publisher(USE_IMAGE_FEEDBACK, Bool, queue_size=1)

        # Create a publisher to publish the command to use force feedback
        self.use_force_feedback_command_publisher = Publisher(USE_FORCE_FEEDBACK, Bool,
                                                              queue_size=1)

        # Create a publisher to publish the command to use balancing feedback
        self.use_balancing_feedback_command_publisher = Publisher(USE_BALANCING_FEEDBACK, Bool,
                                                                  queue_size=1)

        # Create a publisher to publish the command to start and stop the robot motion
        self.stop_robot_motion_command_publisher = Publisher(STOP_ALL_MOTION, Bool, queue_size=1)

        # Create a publisher to publish the command to generate a new image cropping
        self.generate_new_image_cropping_command_publisher = Publisher(CROP_IMAGE_FROM_POINTS, Bool,
                                                                       queue_size=1)

        # Create a publisher to publish the command to identify the thyroid with points
        self.identify_thyroid_from_points_command_publisher = Publisher(IDENTIFY_THYROID_FROM_POINTS, Bool,
                                                                        queue_size=1)

        # Create a publisher to publish the command to use a template to identify the thyroid with a template
        self.identify_thyroid_from_template_command_publisher = Publisher(IDENTIFY_THYROID_FROM_TEMPLATE,
                                                                          Bool, queue_size=1)

        # Create a publisher to publish the command to have the user generate the threshold parameters of the
        # thresholding filter
        self.generate_threshold_filter_parameters_command_publisher = Publisher(
            GENERATE_THRESHOLD_PARAMETERS, Bool, queue_size=1)

        # Create a publisher to publish the desired force for the robot to exert
        self.force_set_point_publisher = Publisher(RC_FORCE_SET_POINT, Float64, queue_size=1)

        # Create a publisher to publish the command to start and stop streaming images
        self.image_streaming_command_publisher = Publisher(IMAGE_STREAMING_CONTROL, Bool, queue_size=1)

        # Create a publisher to publish the command to start and stop streaming images
        self.restart_image_streaming_command_publisher = Publisher(IMAGE_STREAMING_RESTART, Bool,
                                                                   queue_size=1)

        # Create a publisher to publish the command to test the force profile
        self.test_force_profile_publisher = Publisher(TEST_FORCE_PROFILE, Bool, queue_size=1)

        # Create a publisher to publish the command to load image crop coordinates from a file
        self.load_existing_image_cropping_command_publisher = Publisher(CROP_IMAGE_FROM_TEMPLATE,
                                                                        Bool, queue_size=1)

        # Create a publisher to publish the command to scan upwards
        self.scan_command_publisher = Publisher(CREATE_TRAJECTORY, Float64, queue_size=1)

        # Create a publisher to publish the command to clear the current trajectory
        self.clear_trajectory_command_publisher = Publisher(CLEAR_TRAJECTORY, Bool, queue_size=1)

        # Create a publisher to publish the command to complete a full scan
        self.complete_full_scan_command_publisher = Publisher(COMPLETE_FULL_SCAN, Bool, queue_size=1)

        # Create a publisher to publish the command to generate a volume
        self.generate_volume_command_publisher = Publisher(GENERATE_VOLUME, Bool, queue_size=1)

        # Create a publisher to publish the command to display the generated volume
        self.display_volume_command_publisher = Publisher(DISPLAY_VOLUME, Bool, queue_size=1)

        # Create a publisher to publish the imaging depth of the US probe
        self.imaging_depth_publisher = Publisher(IMAGE_DEPTH, Float64, queue_size=1)

        # Create a publisher to publish the command to use pose feedback
        self.use_pose_feedback_command_publisher = Publisher(USE_POSE_FEEDBACK, Bool, queue_size=1)

        self.send_image_saving_command_publisher = Publisher(SAVE_IMAGES, Bool, queue_size=1)

        self.send_folder_destination_publisher = Publisher(SAVED_IMAGES_DESTINATION, String, queue_size=1)

        # Create a publisher to publish an override signal if the patient is in the image
        self.patient_contact_publisher = Publisher(IMAGE_PATIENT_CONTACT, Bool, queue_size=1)

        # Create a publisher to publish an override signal for the registered data
        self.registered_data_publisher = Publisher(REGISTERED_DATA_REAL_TIME, RegisteredDataMsg, queue_size=1)

        # Define command publishers for saving experiment data
        self.save_experiment_data_command_publisher = Publisher(EXP_SAVE_DATA_COMMAND, SaveExperimentDataCommand,
                                                                queue_size=1)

        # Define a command to start publishing random velocity noise
        self.create_noise_command_publisher = Publisher(EXP_CREATE_NOISE_COMMAND, Bool, queue_size=1)

        # Define a publisher for publishing the location in which to save registered data
        self.registered_data_save_location_publisher = Publisher(REGISTERED_DATA_SAVE_LOCATION, String, queue_size=1)

        # Define a publisher for publishing the location from which to load registered data
        self.registered_data_load_location_publisher = Publisher(REGISTERED_DATA_LOAD_LOCATION, String, queue_size=1)

        # Define a publisher for publishing the location in which to save volume data
        self.volume_data_save_location_publisher = Publisher(VOLUME_DATA_SAVE_LOCATION, String, queue_size=1)

        # Define a publisher for publishing robot control velocity commands
        self.manual_robot_control_publisher = Publisher(RC_MANUAL_CONTROL_INPUT, TwistStamped, queue_size=1)

        # Create a subscriber to listen to the external force felt by the robot
        Subscriber(ROBOT_DERIVED_FORCE, WrenchStamped, self.robot_sensed_force_callback)

        # Create a subscriber to hear debug messages
        Subscriber(LOGGING, log_message, self.debug_status_messages_callback)

        # Create a subscriber to listen for when the trajectory has been completed
        Subscriber(RC_TRAJECTORY_COMPLETE, Bool, self.trajectory_complete_callback)

        # endregion
        # ---------------------------------------

        # -------------------------------------------
        # Create GUI components of the User Interface
        # region

        # Define parameters used in the logic of the GUI
        self.currently_using_image_control = False
        self.currently_using_force_feedback = False
        self.currently_using_pose_feedback = False
        self.currently_using_balancing_feedback = False

        # Define a variable to hold the velocity to publish for the manual control
        self.manual_control_velocity_message = None

        # Set the title of the window
        self.parent.title("User Interface")

        # Define the frame in which all objects will be created
        main_content_frame = Frame(parent)

        # Define a frame to hold the always visible control buttons
        button_controls_frame = Frame(main_content_frame)

        # Define a frame to hold the robot control buttons
        robot_controls_frame = Frame(main_content_frame)

        # Define a notebook to manage the tabs within the GUI
        tab_controller = ttk.Notebook(main_content_frame)

        # Define the frames that will be used in the tabs
        exam_setup_frame = ttk.Frame(tab_controller)
        thyroid_exam_frame = ttk.Frame(tab_controller)
        nodule_exam_frame = ttk.Frame(tab_controller)
        status_logging_frame = ttk.Frame(tab_controller)
        developer_frame = ttk.Frame(tab_controller)
        experimentation_frame = ttk.Frame(tab_controller)

        # Add the frames to the tab controller
        tab_controller.add(exam_setup_frame, text="Exam Setup")
        tab_controller.add(thyroid_exam_frame, text="Thyroid Exam")
        tab_controller.add(nodule_exam_frame, text="Nodule Exam")
        tab_controller.add(status_logging_frame, text="Status Logger")
        if passed_arguments.experimentation_mode:
            tab_controller.add(developer_frame, text="Developer")
        if passed_arguments.experimentation_mode:
            tab_controller.add(experimentation_frame, text="Experimentation")

        # Define the widgets used in the button_controls_frame
        # region
        # Define the buttons used for image streaming in testing
        self.image_streaming_button = ttk.Button(button_controls_frame, text=START_IMAGE_STREAMING,
                                                 command=self.image_streaming_button_callback)
        self.restart_image_streaming_button = ttk.Button(button_controls_frame, text="Restart\nImage Streaming",
                                                         command=self.restart_image_streaming_button_callback,
                                                         )

        # List the widgets used for testing in the always visible frame at the bottom
        button_controls_testing_widgets = [
            WidgetCreationObject(self.image_streaming_button,
                                 col_num=LEFT_COLUMN, col_span=SINGLE_COLUMN,
                                 row_num=0, row_span=SINGLE_ROW,
                                 padx=5, pady=5),
            WidgetCreationObject(self.restart_image_streaming_button,
                                 col_num=L_MIDDLE_COLUMN, col_span=SINGLE_COLUMN,
                                 row_num=0, row_span=SINGLE_ROW,
                                 padx=5, pady=5),
        ]

        # Define the buttons used for general control
        self.image_control_button = ttk.Button(button_controls_frame, text=START_IMAGE_CONTROL,
                                               command=self.image_control_button_callback)
        self.force_control_button = ttk.Button(button_controls_frame, text=START_FORCE_CONTROL,
                                               command=self.force_control_button_callback)
        self.balancing_control_button = ttk.Button(button_controls_frame, text=START_BALANCING_CONTROL,
                                                   command=self.balancing_control_button_callback)
        self.pose_control_button = ttk.Button(button_controls_frame, text=STOP_POSE_CONTROL,
                                              command=self.pose_control_button_callback, state=DISABLED)

        # List the widgets used for general control in the always visible frame at the bottom
        button_controls_streaming_widgets = [
            WidgetCreationObject(self.force_control_button, col_num=MIDDLE_COLUMN, row_num=0),
            WidgetCreationObject(self.balancing_control_button, col_num=R_MIDDLE_COLUMN, row_num=0),
            WidgetCreationObject(self.image_control_button, col_num=RIGHT_COLUMN, row_num=0, ),
            WidgetCreationObject(self.pose_control_button, col_num=RIGHT_COLUMN + 1, row_num=0)
        ]

        # Select which widgets to display in the button_controls_frame based on which function mode the GUI is in
        self.button_controls_frame_widgets = button_controls_streaming_widgets
        if passed_arguments.testing_mode:
            self.button_controls_frame_widgets = button_controls_testing_widgets + self.button_controls_frame_widgets

        # endregion

        # Define the widgets used for the robot_controls_frame
        # region
        self.positive_x_movement_button = ttk.Button(robot_controls_frame, text=POSITIVE_X, width=5)
        self.positive_x_movement_button.bind(BUTTON_PRESS,
                                             lambda event, axis_and_direction=POSITIVE_X:
                                             self.robot_control_button_callback(axis_and_direction=axis_and_direction))
        self.positive_x_movement_button.bind(BUTTON_RELEASE,
                                             lambda event, axis_and_direction=NO_MOVEMENT:
                                             self.robot_control_button_callback(axis_and_direction=axis_and_direction))

        self.negative_x_movement_button = ttk.Button(robot_controls_frame, text=NEGATIVE_X, width=5)
        self.negative_x_movement_button.bind(BUTTON_PRESS,
                                             lambda event, axis_and_direction=NEGATIVE_X:
                                             self.robot_control_button_callback(axis_and_direction=axis_and_direction))
        self.negative_x_movement_button.bind(BUTTON_RELEASE,
                                             lambda event, axis_and_direction=NO_MOVEMENT:
                                             self.robot_control_button_callback(axis_and_direction=axis_and_direction))

        self.positive_y_movement_button = ttk.Button(robot_controls_frame, text=POSITIVE_Y, width=5)
        self.positive_y_movement_button.bind(BUTTON_PRESS,
                                             lambda event, axis_and_direction=POSITIVE_Y:
                                             self.robot_control_button_callback(axis_and_direction=axis_and_direction))
        self.positive_y_movement_button.bind(BUTTON_RELEASE,
                                             lambda event, axis_and_direction=NO_MOVEMENT:
                                             self.robot_control_button_callback(axis_and_direction=axis_and_direction))

        self.negative_y_movement_button = ttk.Button(robot_controls_frame, text=NEGATIVE_Y, width=5)
        self.negative_y_movement_button.bind(BUTTON_PRESS,
                                             lambda event, axis_and_direction=NEGATIVE_Y:
                                             self.robot_control_button_callback(axis_and_direction=axis_and_direction))
        self.negative_y_movement_button.bind(BUTTON_RELEASE,
                                             lambda event, axis_and_direction=NO_MOVEMENT:
                                             self.robot_control_button_callback(axis_and_direction=axis_and_direction))

        self.positive_pitch_movement_button = ttk.Button(robot_controls_frame, text=POSITIVE_PITCH, width=5)
        self.positive_pitch_movement_button.bind(BUTTON_PRESS,
                                                 lambda event, axis_and_direction=POSITIVE_PITCH:
                                                 self.robot_control_button_callback(
                                                     axis_and_direction=axis_and_direction))
        self.positive_pitch_movement_button.bind(BUTTON_RELEASE,
                                                 lambda event, axis_and_direction=NO_MOVEMENT:
                                                 self.robot_control_button_callback(
                                                     axis_and_direction=axis_and_direction))

        self.negative_pitch_movement_button = ttk.Button(robot_controls_frame, text=NEGATIVE_PITCH, width=5)
        self.negative_pitch_movement_button.bind(BUTTON_PRESS,
                                                 lambda event, axis_and_direction=NEGATIVE_PITCH:
                                                 self.robot_control_button_callback(
                                                     axis_and_direction=axis_and_direction))
        self.negative_pitch_movement_button.bind(BUTTON_RELEASE,
                                                 lambda event, axis_and_direction=NO_MOVEMENT:
                                                 self.robot_control_button_callback(
                                                     axis_and_direction=axis_and_direction))

        self.positive_yaw_movement_button = ttk.Button(robot_controls_frame, text=POSITIVE_YAW, width=5)
        self.positive_yaw_movement_button.bind(BUTTON_PRESS,
                                               lambda event, axis_and_direction=POSITIVE_YAW:
                                               self.robot_control_button_callback(
                                                   axis_and_direction=axis_and_direction))
        self.positive_yaw_movement_button.bind(BUTTON_RELEASE,
                                               lambda event, axis_and_direction=NO_MOVEMENT:
                                               self.robot_control_button_callback(
                                                   axis_and_direction=axis_and_direction))

        self.negative_yaw_movement_button = ttk.Button(robot_controls_frame, text=NEGATIVE_YAW, width=5)
        self.negative_yaw_movement_button.bind(BUTTON_PRESS,
                                               lambda event, axis_and_direction=NEGATIVE_YAW:
                                               self.robot_control_button_callback(
                                                   axis_and_direction=axis_and_direction))
        self.negative_yaw_movement_button.bind(BUTTON_RELEASE,
                                               lambda event, axis_and_direction=NO_MOVEMENT:
                                               self.robot_control_button_callback(
                                                   axis_and_direction=axis_and_direction))

        robot_controls_widgets = [
            WidgetCreationObject(self.positive_x_movement_button, col_num=MIDDLE_COLUMN, row_num=1),
            WidgetCreationObject(self.negative_x_movement_button, col_num=MIDDLE_COLUMN, row_num=3),
            WidgetCreationObject(self.positive_y_movement_button, col_num=RL_MIDDLE_COLUMN, row_num=2),
            WidgetCreationObject(self.negative_y_movement_button, col_num=LR_MIDDLE_COLUMN, row_num=2),
            WidgetCreationObject(self.negative_pitch_movement_button, col_num=RR_MIDDLE_COLUMN, row_num=1),
            WidgetCreationObject(self.positive_pitch_movement_button, col_num=RR_MIDDLE_COLUMN, row_num=3),
            WidgetCreationObject(self.positive_yaw_movement_button, col_num=RL_MIDDLE_COLUMN, row_num=0),
            WidgetCreationObject(self.negative_yaw_movement_button, col_num=LR_MIDDLE_COLUMN, row_num=0),
        ]
        # endregion

        # Define the widgets used in the exam_setup_frame
        # region
        validation_command = self.parent.register(self.entry_widget_float_validation)
        self.force_set_point_entry = ttk.Entry(exam_setup_frame, validate=ALL,
                                               validatecommand=(validation_command, '%P'),
                                               width=5, justify=CENTER)
        self.force_set_point_entry.insert(0, '0.0')
        force_set_point_increase_button = ttk.Button(exam_setup_frame, text="+",
                                                     command=lambda: self.force_set_point_change_incremental(
                                                         INCREMENTAL_FORCE_CHANGE))
        force_set_point_decrease_button = ttk.Button(exam_setup_frame, text="-",
                                                     command=lambda: self.force_set_point_change_incremental(
                                                         -INCREMENTAL_FORCE_CHANGE
                                                     ))
        self.current_force_string_var = StringVar(exam_setup_frame, "0.0")
        self.test_force_profile_button = ttk.Button(exam_setup_frame, text=TEST_FORCE_CONTROL,
                                                    command=self.test_force_profile_callback)
        self.current_force_label = ttk.Label(exam_setup_frame, textvariable=self.current_force_string_var,
                                             anchor=CENTER, justify=CENTER)
        self.select_image_crop_variable = IntVar()
        self.generate_new_image_cropping_button = ttk.Button(exam_setup_frame, text="Generate a New\nImage Cropping",
                                                             command=self.generate_new_image_cropping_button_callback,
                                                             state=DISABLED)
        self.load_existing_image_cropping_button = ttk.Button(exam_setup_frame,
                                                              text="Load Existing\nImage Cropping",
                                                              command=self.load_existing_image_cropping_button_callback,
                                                              state=DISABLED)
        self.imaging_depth_entry = ttk.Entry(exam_setup_frame, validate=ALL,
                                             validatecommand=(validation_command, '%P'), width=5, justify=CENTER)
        self.imaging_depth_entry.insert(0, '5.0')
        self.imaging_depth_submit_callback()
        self.identify_thyroid_from_points_button = ttk.Button(exam_setup_frame,
                                                              text="Identify Region of Interest from Points",
                                                              command=self.identify_thyroid_from_points_button_callback,
                                                              )

        # List the widgets used to populate the exam_setup_frame
        self.exam_setup_widgets = [
            WidgetCreationObject(ttk.Label(exam_setup_frame, text="Desired Force", anchor=CENTER, justify=CENTER),
                                 col_num=LEFT_COLUMN, row_num=0),
            WidgetCreationObject(self.force_set_point_entry, col_num=L_MIDDLE_COLUMN, row_num=0, pady=20),
            WidgetCreationObject(ttk.Label(exam_setup_frame, text="N"), col_num=MIDDLE_COLUMN, row_num=0),
            WidgetCreationObject(force_set_point_increase_button, col_num=R_MIDDLE_COLUMN, row_num=0),
            WidgetCreationObject(force_set_point_decrease_button, col_num=R_MIDDLE_COLUMN, row_num=1),
            WidgetCreationObject(
                ttk.Button(exam_setup_frame, text="Send New\nSet-point", command=self.force_set_point_submit_callback),
                col_num=RIGHT_COLUMN, row_num=0, row_span=DOUBLE_ROW),
            WidgetCreationObject(ttk.Label(exam_setup_frame, text="Current Force", anchor=CENTER, justify=CENTER),
                                 col_num=LEFT_COLUMN, row_num=1, pady=20),
            WidgetCreationObject(self.current_force_label, col_num=L_MIDDLE_COLUMN, row_num=1),
            WidgetCreationObject(ttk.Label(exam_setup_frame, text="N"),
                                 col_num=MIDDLE_COLUMN, row_num=1, ipadx=2, padx=0),
            WidgetCreationObject(ttk.Separator(exam_setup_frame),
                                 col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=3, padx=0),
            WidgetCreationObject(ttk.Label(exam_setup_frame, text="Crop the raw image?", anchor=CENTER, justify=CENTER),
                                 col_num=LEFT_COLUMN, row_num=4, row_span=DOUBLE_ROW),
            WidgetCreationObject(Radiobutton(exam_setup_frame, text="Yes", variable=self.select_image_crop_variable,
                                             value=YES_BUTTON, command=self.select_image_crop_callback,
                                             anchor=CENTER, justify=CENTER), col_num=L_MIDDLE_COLUMN, row_num=4),
            WidgetCreationObject(Radiobutton(exam_setup_frame, text="No", variable=self.select_image_crop_variable,
                                             value=NO_BUTTON, command=self.select_image_crop_callback,
                                             anchor=CENTER, justify=CENTER), col_num=MIDDLE_COLUMN, row_num=4),
            WidgetCreationObject(self.generate_new_image_cropping_button, col_num=R_MIDDLE_COLUMN, row_num=4),
            WidgetCreationObject(self.load_existing_image_cropping_button, col_num=RIGHT_COLUMN, row_num=4),
            WidgetCreationObject(ttk.Separator(exam_setup_frame), col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                 row_num=5, padx=0),
            WidgetCreationObject(ttk.Label(exam_setup_frame, text="Set the imaging depth\nof the US scanner:",
                                           anchor=CENTER, justify=CENTER), col_num=LEFT_COLUMN, row_num=6),
            WidgetCreationObject(self.imaging_depth_entry, col_num=L_MIDDLE_COLUMN, row_num=6, pady=20),
            WidgetCreationObject(ttk.Label(exam_setup_frame, text="cm"), col_num=MIDDLE_COLUMN, row_num=6),
            WidgetCreationObject(ttk.Button(exam_setup_frame, text="Send", command=self.imaging_depth_submit_callback),
                                 col_num=R_MIDDLE_COLUMN, col_span=TWO_COLUMN, row_num=6),
            WidgetCreationObject(ttk.Separator(exam_setup_frame), col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                 row_num=7, padx=0),
            WidgetCreationObject(self.identify_thyroid_from_points_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                 row_num=8),
        ]

        # endregion

        # Define the widgets used to populate the thyroid exam frame
        # region
        self.identify_thyroid_from_template_button = \
            ttk.Button(thyroid_exam_frame,
                       text="Identify Thyroid from Template",
                       command=self.identify_thyroid_from_template_button_callback,
                       )
        self.scan_positive_button = ttk.Button(thyroid_exam_frame,
                                               text="Scan Positive",
                                               command=self.scan_positive_button_callback,
                                               )
        self.scan_negative_button = ttk.Button(thyroid_exam_frame,
                                               text="Scan Negative",
                                               command=self.scan_negative_button_callback,
                                               )
        self.scan_distance_entry = Entry(thyroid_exam_frame, validate=ALL, validatecommand=(validation_command, '%P'),
                                         justify=CENTER, width=5)
        self.scan_distance_entry.insert(0, '6.0')
        self.registered_data_save_location_button = ttk.Button(thyroid_exam_frame,
                                                               text='Select Location to Save Exam Data',
                                                               command=self.registered_data_save_location_button_callback)
        self.registered_data_save_location_str_var = StringVar(thyroid_exam_frame,
                                                               "/home/ben/thyroid_ultrasound_data/testing_and_validation/registered_data")
        self.registered_data_load_location_button = ttk.Button(thyroid_exam_frame,
                                                               text='Select Data to Generate Volume',
                                                               command=self.registered_data_load_location_button_callback)
        self.registered_data_load_location_str_var = StringVar(thyroid_exam_frame,
                                                               self.registered_data_save_location_str_var.get())
        self.volume_data_save_location_button = ttk.Button(thyroid_exam_frame,
                                                           text='Select Location to Save Volume Data',
                                                           command=self.volume_data_save_location_button_callback)
        self.volume_data_save_location_str_var = StringVar(thyroid_exam_frame,
                                                           "/home/ben/thyroid_ultrasound_data/testing_and_validation/volume_data")

        self.complete_full_scan_button = ttk.Button(thyroid_exam_frame,
                                                    text="Complete\n Full Scan",
                                                    command=self.complete_full_scan_button_callback,
                                                    )
        self.generate_volume_selector_variable = IntVar()
        self.display_volume_selector_variable = IntVar()

        # List the widgets to used to populate the thyroid exam frame
        self.thyroid_exam_widgets = [
            # WidgetCreationObject(self.identify_thyroid_from_template_button,
            #                      col_num=R_MIDDLE_COLUMN, col_span=TWO_COLUMN, row_num=0),
            WidgetCreationObject(ttk.Label(thyroid_exam_frame, text="Scanning Distance", anchor=CENTER, justify=CENTER),
                                 col_num=LEFT_COLUMN, col_span=TWO_COLUMN, row_num=0),
            WidgetCreationObject(self.scan_distance_entry, col_num=L_MIDDLE_COLUMN, row_num=0),
            WidgetCreationObject(ttk.Label(thyroid_exam_frame, text="cm"),
                                 col_num=MIDDLE_COLUMN, row_num=0, ipadx=2, padx=0),
            WidgetCreationObject(self.scan_positive_button,
                                 col_num=R_MIDDLE_COLUMN, row_num=0),
            WidgetCreationObject(self.scan_negative_button,
                                 col_num=RIGHT_COLUMN, row_num=0),
            WidgetCreationObject(ttk.Separator(thyroid_exam_frame),
                                 col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=1),
            WidgetCreationObject(self.registered_data_save_location_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                 row_num=2),
            WidgetCreationObject(ttk.Label(thyroid_exam_frame, text="Selected Location:"), col_num=LEFT_COLUMN,
                                 col_span=TWO_COLUMN, row_num=3),
            WidgetCreationObject(ttk.Label(thyroid_exam_frame, textvariable=self.registered_data_save_location_str_var),
                                 col_num=L_MIDDLE_COLUMN, col_span=FULL_WIDTH - TWO_COLUMN, row_num=3),
            WidgetCreationObject(ttk.Separator(thyroid_exam_frame), col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                 row_num=4),
            WidgetCreationObject(self.registered_data_load_location_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                 row_num=5),
            WidgetCreationObject(ttk.Label(thyroid_exam_frame, text='Selected Location:'), col_num=LEFT_COLUMN,
                                 col_span=TWO_COLUMN, row_num=6),
            WidgetCreationObject(ttk.Label(thyroid_exam_frame, textvariable=self.registered_data_load_location_str_var),
                                 col_num=L_MIDDLE_COLUMN, col_span=FULL_WIDTH - TWO_COLUMN, row_num=6),
            WidgetCreationObject(ttk.Separator(thyroid_exam_frame), col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                 row_num=7),
            WidgetCreationObject(self.volume_data_save_location_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                 row_num=8),
            WidgetCreationObject(ttk.Label(thyroid_exam_frame, text="Selected Location:"), col_num=LEFT_COLUMN,
                                 col_span=TWO_COLUMN, row_num=9),
            WidgetCreationObject(ttk.Label(thyroid_exam_frame, textvariable=self.volume_data_save_location_str_var),
                                 col_num=L_MIDDLE_COLUMN, col_span=FULL_WIDTH - TWO_COLUMN, row_num=9),
            WidgetCreationObject(ttk.Button(thyroid_exam_frame, text='Generate Volume',
                                            command=self.generate_volume_button_callback), col_num=LEFT_COLUMN,
                                 col_span=FULL_WIDTH, row_num=10)
            # WidgetCreationObject(self.complete_full_scan_button,
            #                      col_num=RIGHT_COLUMN, col_span=SINGLE_COLUMN,
            #                      row_num=1, row_span=DOUBLE_ROW),
            # WidgetCreationObject(ttk.Label(thyroid_exam_frame, text="Would you like to generate a volume?"),
            #                      col_num=LEFT_COLUMN, col_span=THREE_COLUMN, row_num=3, row_span=SINGLE_ROW),
            # WidgetCreationObject(
            #     Radiobutton(thyroid_exam_frame, text="Yes", variable=self.generate_volume_selector_variable,
            #                 value=1, command=self.generate_volume_button_callback),
            #     col_num=R_MIDDLE_COLUMN, col_span=SINGLE_COLUMN,
            #     row_num=3, row_span=SINGLE_ROW),
            # WidgetCreationObject(
            #     Radiobutton(thyroid_exam_frame, text="No", variable=self.generate_volume_selector_variable,
            #                 value=0, command=self.generate_volume_button_callback),
            #     col_num=RIGHT_COLUMN, col_span=SINGLE_COLUMN,
            #     row_num=3, row_span=SINGLE_ROW),
            # WidgetCreationObject(ttk.Label(thyroid_exam_frame, text="Would you like to display the volume?"),
            #                      col_num=LEFT_COLUMN, col_span=THREE_COLUMN,
            #                      row_num=4, row_span=SINGLE_ROW),
            # WidgetCreationObject(
            #     Radiobutton(thyroid_exam_frame, text="Yes", variable=self.display_volume_selector_variable,
            #                 value=1, command=self.display_volume_button_callback),
            #     col_num=R_MIDDLE_COLUMN, col_span=SINGLE_COLUMN,
            #     row_num=4, row_span=SINGLE_ROW),
            # WidgetCreationObject(
            #     Radiobutton(thyroid_exam_frame, text="No", variable=self.display_volume_selector_variable,
            #                 value=0, command=self.display_volume_button_callback),
            #     col_num=RIGHT_COLUMN, col_span=SINGLE_COLUMN,
            #     row_num=4, row_span=SINGLE_ROW),
        ]

        # endregion

        # List the widgets used to populate the nodule exam frame
        self.nodule_exam_widgets = []

        # Define the widgets used to populate the status_logging_frame
        # region
        self.status_log_string = "Application started."
        self.status_label = ScrolledText(status_logging_frame, wrap=WORD)
        self.status_label.insert(INSERT, self.status_log_string)
        self.status_label.configure(state=DISABLED)

        # Define the widgets used for setting up the status logging
        status_logging_widgets = [
            WidgetCreationObject(self.status_label, col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                 row_num=0, row_span=SINGLE_ROW),
        ]

        # endregion

        # List the widgets used to populate the developer window
        # region
        self.pid_selector = IntVar()
        self.p_gain_entry = ttk.Entry(developer_frame, validate=ALL, validatecommand=(validation_command, '%P'),
                                      justify=CENTER, width=5)
        self.p_gain_entry.insert(0, "0.000")
        self.p_gain_var = StringVar(developer_frame, "0.000")
        self.i_gain_entry = ttk.Entry(developer_frame, validate=ALL, validatecommand=(validation_command, '%P'),
                                      justify=CENTER, width=5)
        self.i_gain_entry.insert(0, "0.000")
        self.i_gain_var = StringVar(developer_frame, "0.000")
        self.d_gain_entry = ttk.Entry(developer_frame, validate=ALL, validatecommand=(validation_command, '%P'),
                                      justify=CENTER, width=5)
        self.d_gain_entry.insert(0, "0.000")
        self.d_gain_var = StringVar(developer_frame, "0.000")
        self.save_images_button = ttk.Button(developer_frame, text=START_SAVING_IMAGES,
                                             command=self.send_image_saving_command_callback)
        self.select_image_destination_directory = ttk.Button(developer_frame,
                                                             text='Select Location of Saved Images',
                                                             command=self.send_save_images_destination)
        self.speed_selector = IntVar(value=10)

        developer_widgets = [
            WidgetCreationObject(ttk.Label(developer_frame, text="Select\nController"),
                                 col_num=LEFT_COLUMN, row_num=0, row_span=DOUBLE_ROW),
            WidgetCreationObject(Radiobutton(developer_frame, text="x-lin-trj", variable=self.pid_selector,
                                             value=0, command=self.pid_controller_selection_callback),
                                 col_num=L_MIDDLE_COLUMN, row_num=0),
            WidgetCreationObject(Radiobutton(developer_frame, text="y-lin-img", variable=self.pid_selector,
                                             value=1, command=self.pid_controller_selection_callback),
                                 col_num=MIDDLE_COLUMN, row_num=0),
            WidgetCreationObject(Radiobutton(developer_frame, text="z-lin-force", variable=self.pid_selector,
                                             value=2, command=self.pid_controller_selection_callback),
                                 col_num=R_MIDDLE_COLUMN, row_num=0),
            WidgetCreationObject(Radiobutton(developer_frame, text="x-ang-N/A", variable=self.pid_selector,
                                             value=3, command=self.pid_controller_selection_callback),
                                 col_num=L_MIDDLE_COLUMN, row_num=1),
            WidgetCreationObject(Radiobutton(developer_frame, text="y-ang-N/A", variable=self.pid_selector,
                                             value=4, command=self.pid_controller_selection_callback),
                                 col_num=MIDDLE_COLUMN, row_num=1),
            WidgetCreationObject(Radiobutton(developer_frame, text="z-ang-N/A", variable=self.pid_selector,
                                             value=5, command=self.pid_controller_selection_callback),
                                 col_num=R_MIDDLE_COLUMN, row_num=1),
            WidgetCreationObject(ttk.Separator(developer_frame),
                                 col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=2, padx=0),
            WidgetCreationObject(ttk.Label(developer_frame, text="P", anchor=CENTER, justify=CENTER),
                                 col_num=L_MIDDLE_COLUMN, row_num=3),
            WidgetCreationObject(ttk.Label(developer_frame, text="I", anchor=CENTER, justify=CENTER),
                                 col_num=MIDDLE_COLUMN, row_num=3),
            WidgetCreationObject(ttk.Label(developer_frame, text="D", anchor=CENTER, justify=CENTER),
                                 col_num=R_MIDDLE_COLUMN, row_num=3),
            WidgetCreationObject(ttk.Label(developer_frame, text="Current Values:"),
                                 col_num=LEFT_COLUMN, row_num=4),
            WidgetCreationObject(ttk.Label(developer_frame, textvariable=self.p_gain_var,
                                           anchor=CENTER, justify=CENTER),
                                 col_num=L_MIDDLE_COLUMN, row_num=4),
            WidgetCreationObject(ttk.Label(developer_frame, textvariable=self.i_gain_var,
                                           anchor=CENTER, justify=CENTER),
                                 col_num=MIDDLE_COLUMN, row_num=4),
            WidgetCreationObject(ttk.Label(developer_frame, textvariable=self.d_gain_var,
                                           anchor=CENTER, justify=CENTER),
                                 col_num=R_MIDDLE_COLUMN, row_num=4),
            WidgetCreationObject(ttk.Label(developer_frame, text="Set to:"), col_num=LEFT_COLUMN, row_num=5),
            WidgetCreationObject(self.p_gain_entry, col_num=L_MIDDLE_COLUMN, row_num=5),
            WidgetCreationObject(self.i_gain_entry, col_num=MIDDLE_COLUMN, row_num=5),
            WidgetCreationObject(self.d_gain_entry, col_num=R_MIDDLE_COLUMN, row_num=5),
            WidgetCreationObject(
                ttk.Button(developer_frame, text="Set Values", command=self.pid_value_setting_callback),
                col_num=RIGHT_COLUMN, row_num=5),
            WidgetCreationObject(ttk.Separator(developer_frame),
                                 col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=6, padx=0),
            WidgetCreationObject(self.select_image_destination_directory, col_num=LEFT_COLUMN,
                                 col_span=FOUR_COLUMN + 1, row_num=7),
            WidgetCreationObject(self.save_images_button, col_num=RL_MIDDLE_COLUMN, col_span=FOUR_COLUMN, row_num=7),
            WidgetCreationObject(ttk.Label(developer_frame, text="Select the overall speed\nfactor for the robot."),
                                 col_num=LEFT_COLUMN, col_span=TWO_COLUMN, row_num=8, row_span=DOUBLE_ROW),
            WidgetCreationObject(Radiobutton(developer_frame, text="10%", variable=self.speed_selector,
                                             value=10, command=self.speed_selector_callback),
                                 col_num=L_MIDDLE_COLUMN, row_num=8),
            WidgetCreationObject(Radiobutton(developer_frame, text="25%", variable=self.speed_selector,
                                             value=25, command=self.speed_selector_callback),
                                 col_num=MIDDLE_COLUMN, row_num=8),
            WidgetCreationObject(Radiobutton(developer_frame, text="50%", variable=self.speed_selector,
                                             value=50, command=self.speed_selector_callback),
                                 col_num=R_MIDDLE_COLUMN, row_num=8),
            WidgetCreationObject(Radiobutton(developer_frame, text="75%", variable=self.speed_selector,
                                             value=75, command=self.speed_selector_callback),
                                 col_num=L_MIDDLE_COLUMN, row_num=9),
            WidgetCreationObject(Radiobutton(developer_frame, text="100%", variable=self.speed_selector,
                                             value=100, command=self.speed_selector_callback),
                                 col_num=MIDDLE_COLUMN, row_num=9),
            WidgetCreationObject(Radiobutton(developer_frame, text="125%", variable=self.speed_selector,
                                             value=125, command=self.speed_selector_callback),
                                 col_num=R_MIDDLE_COLUMN, row_num=9),
        ]

        # endregion

        # Define the widgets used to populate the experimentation window
        # region
        self.select_patient_contact_override_variable = IntVar()
        self.is_patient_contact_override_active = False
        self.in_contact_radio_button = ttk.Radiobutton(experimentation_frame, text="In-contact", value=YES_BUTTON,
                                                       variable=self.select_patient_contact_override_variable)
        self.not_in_contact_radio_button = ttk.Radiobutton(experimentation_frame, text="Not In-contact",
                                                           value=NO_BUTTON,
                                                           variable=self.select_patient_contact_override_variable)
        self.send_patient_contact_override_button = ttk.Button(experimentation_frame,
                                                               text=START_SENDING_OVERRIDE_VALUE,
                                                               command=self.send_patient_contact_override_button_callback)
        self.is_registered_data_override_active = False
        self.send_registered_data_override_button = ttk.Button(experimentation_frame,
                                                               text=START_SENDING_REGISTERED_DATA_OVERRIDE_VALUE,
                                                               command=self.send_registered_data_override_button_callback)
        self.save_robot_pose_variable = IntVar()
        self.save_robot_pose_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes", value=YES_BUTTON,
                                                          variable=self.save_robot_pose_variable)
        self.save_robot_pose_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                         variable=self.save_robot_pose_variable)
        self.save_robot_force_variable = IntVar()
        self.save_robot_force_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes", value=YES_BUTTON,
                                                           variable=self.save_robot_force_variable)
        self.save_robot_force_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                          variable=self.save_robot_force_variable)
        self.save_raw_image_variable = IntVar()
        self.save_raw_image_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes", value=YES_BUTTON,
                                                         variable=self.save_raw_image_variable)
        self.save_raw_image_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                        variable=self.save_raw_image_variable)
        self.save_image_data_objects_variable = IntVar()
        self.save_image_data_objects_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes", value=YES_BUTTON,
                                                                  variable=self.save_image_data_objects_variable)
        self.save_image_data_objects_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                                 variable=self.save_image_data_objects_variable)
        self.save_image_centroid_variable = IntVar()
        self.save_image_centroid_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes", value=YES_BUTTON,
                                                              variable=self.save_image_centroid_variable)
        self.save_image_centroid_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                             variable=self.save_image_centroid_variable)
        self.save_skin_error_variable = IntVar()
        self.save_skin_error_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes", value=YES_BUTTON,
                                                          variable=self.save_skin_error_variable)
        self.save_skin_error_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                         variable=self.save_skin_error_variable)
        self.is_experiment_data_saving_active = False
        self.save_experiment_data_button = ttk.Button(experimentation_frame,
                                                      text=START_SAVING_EXPERIMENT_DATA,
                                                      command=self.save_experiment_data_button_callback)
        self.create_noise_button = ttk.Button(experimentation_frame, text=START_CREATING_VELOCITY_NOISE,
                                              command=self.create_noise_button_callback)

        experimentation_widgets = [
            WidgetCreationObject(ttk.Label(experimentation_frame, text="Select the value to send to override\n"
                                                                       "the calculated patient contact value",
                                           anchor=CENTER, justify=CENTER),
                                 col_num=LEFT_COLUMN, col_span=FOUR_COLUMN, row_num=0, row_span=DOUBLE_ROW),
            WidgetCreationObject(self.in_contact_radio_button, col_num=MIDDLE_COLUMN, col_span=TWO_COLUMN,
                                 row_num=0, sticky=''),
            WidgetCreationObject(self.not_in_contact_radio_button, col_num=MIDDLE_COLUMN, col_span=TWO_COLUMN,
                                 row_num=1, sticky=''),
            WidgetCreationObject(self.send_patient_contact_override_button, col_num=R_MIDDLE_COLUMN,
                                 col_span=TWO_COLUMN, row_num=0, row_span=DOUBLE_ROW),
            WidgetCreationObject(ttk.Separator(experimentation_frame),
                                 col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=2, padx=0),
            WidgetCreationObject(self.send_registered_data_override_button, col_num=LEFT_COLUMN, row_num=3,
                                 col_span=FULL_WIDTH),
            WidgetCreationObject(ttk.Separator(experimentation_frame),
                                 col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=7, padx=0),
            WidgetCreationObject(ttk.Label(experimentation_frame,
                                           text="Select the data\nto save for\nthis experiment", anchor=CENTER),
                                 col_num=LEFT_COLUMN, col_span=TWO_COLUMN, row_num=8, row_span=3),
            WidgetCreationObject(ttk.Label(experimentation_frame, text="Save\nrobot pose",
                                           anchor=CENTER, justify=CENTER), col_num=L_MIDDLE_COLUMN, row_num=8),
            WidgetCreationObject(self.save_robot_pose_yes_button, col_num=L_MIDDLE_COLUMN, row_num=9, sticky=""),
            WidgetCreationObject(self.save_robot_pose_no_button, col_num=L_MIDDLE_COLUMN, row_num=10, sticky=""),
            WidgetCreationObject(ttk.Label(experimentation_frame, text="Save\nrobot force",
                                           anchor=CENTER, justify=CENTER), col_num=LR_MIDDLE_COLUMN, row_num=8),
            WidgetCreationObject(self.save_robot_force_yes_button, col_num=LR_MIDDLE_COLUMN, row_num=9, sticky=""),
            WidgetCreationObject(self.save_robot_force_no_button, col_num=LR_MIDDLE_COLUMN, row_num=10, sticky=""),
            WidgetCreationObject(ttk.Label(experimentation_frame, text="Save\nraw images",
                                           anchor=CENTER, justify=CENTER), col_num=MIDDLE_COLUMN, row_num=8),
            WidgetCreationObject(self.save_raw_image_yes_button, col_num=MIDDLE_COLUMN, row_num=9, sticky=""),
            WidgetCreationObject(self.save_raw_image_no_button, col_num=MIDDLE_COLUMN, row_num=10, sticky=""),
            WidgetCreationObject(ttk.Label(experimentation_frame, text="Save\nimage data",
                                           anchor=CENTER, justify=CENTER), col_num=RL_MIDDLE_COLUMN, row_num=8),
            WidgetCreationObject(self.save_image_data_objects_yes_button, col_num=RL_MIDDLE_COLUMN, row_num=9,
                                 sticky=""),
            WidgetCreationObject(self.save_image_data_objects_no_button, col_num=RL_MIDDLE_COLUMN, row_num=10,
                                 sticky=""),
            WidgetCreationObject(ttk.Label(experimentation_frame, text="Save image\ncentroid",
                                           anchor=CENTER, justify=CENTER), col_num=R_MIDDLE_COLUMN, row_num=8),
            WidgetCreationObject(self.save_image_centroid_yes_button, col_num=R_MIDDLE_COLUMN, row_num=9, sticky=""),
            WidgetCreationObject(self.save_image_centroid_no_button, col_num=R_MIDDLE_COLUMN, row_num=10, sticky=""),
            WidgetCreationObject(ttk.Label(experimentation_frame, text="Save skin\nerror",
                                           anchor=CENTER, justify=CENTER), col_num=RR_MIDDLE_COLUMN, row_num=8),
            WidgetCreationObject(self.save_skin_error_yes_button, col_num=RR_MIDDLE_COLUMN, row_num=9, sticky=""),
            WidgetCreationObject(self.save_skin_error_no_button, col_num=RR_MIDDLE_COLUMN, row_num=10, sticky=""),
            WidgetCreationObject(self.save_experiment_data_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                 row_num=11),
            WidgetCreationObject(ttk.Separator(experimentation_frame),
                                 col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=12, padx=0),
            WidgetCreationObject(self.create_noise_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=13)
        ]

        # endregion

        # Add the widgets to each frame
        list_of_list_of_widgets = [
            self.button_controls_frame_widgets,
            robot_controls_widgets,
            self.exam_setup_widgets,
            self.thyroid_exam_widgets,
            self.nodule_exam_widgets,
            status_logging_widgets,
            developer_widgets,
            experimentation_widgets
        ]
        for list_of_widgets in list_of_list_of_widgets:
            for widget in list_of_widgets:
                widget.add_widget()

        # Add the parent frame as the only grid object in the window
        main_content_frame.grid(column=0, row=0)

        # Add the two frames to the main frame
        tab_controller.grid(column=LEFT_COLUMN, columnspan=SINGLE_COLUMN, row=0)
        button_controls_frame.grid(column=LEFT_COLUMN, columnspan=SINGLE_COLUMN, row=1)
        robot_controls_frame.grid(column=RIGHT_COLUMN, columnspan=SINGLE_COLUMN, row=0, rowspan=DOUBLE_ROW)

        # Allow the window to be resized
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)
        main_content_frame.columnconfigure(0, weight=1)
        main_content_frame.rowconfigure(0, weight=1)

        # endregion
        # -------------------------------------------

    ############################################################################
    # Define GUI button callbacks
    # region

    def robot_control_button_callback(self, axis_and_direction: str):

        # Get the velocity to give the message
        linear_movement_speed = 0.01
        angular_movement_speed = 0.1

        # Get the speed factor
        speed_factor = 1.0

        # Create the common message
        self.manual_control_velocity_message = Twist()

        if axis_and_direction == NO_MOVEMENT:
            self.manual_control_velocity_message = None
        elif axis_and_direction == POSITIVE_X:
            self.manual_control_velocity_message.linear.x = linear_movement_speed * speed_factor
        elif axis_and_direction == NEGATIVE_X:
            self.manual_control_velocity_message.linear.x = -linear_movement_speed * speed_factor
        elif axis_and_direction == POSITIVE_Y:
            self.manual_control_velocity_message.linear.y = linear_movement_speed * speed_factor
        elif axis_and_direction == NEGATIVE_Y:
            self.manual_control_velocity_message.linear.y = -linear_movement_speed * speed_factor
        elif axis_and_direction == POSITIVE_PITCH:
            self.manual_control_velocity_message.angular.y = angular_movement_speed * speed_factor
        elif axis_and_direction == NEGATIVE_PITCH:
            self.manual_control_velocity_message.angular.y = -angular_movement_speed * speed_factor
        elif axis_and_direction == POSITIVE_YAW:
            self.manual_control_velocity_message.angular.z = angular_movement_speed * speed_factor
        elif axis_and_direction == NEGATIVE_YAW:
            self.manual_control_velocity_message.angular.z = -angular_movement_speed * speed_factor
        else:
            raise Exception(axis_and_direction + " is not a recognized movement direction.")

    @staticmethod
    def entry_widget_float_validation(new_entry: str) -> bool:
        """
        Validates any value entered into the force set-point entry field.

        Parameters
        ----------
        new_entry:
            A string representing the new value placed in the entry field.
        """

        # Check that the new entry is a float or an integer
        if str.isnumeric(new_entry.replace('.', '')) or new_entry == "":

            # Send confirmation that the entry is allowed
            return True
        else:

            # Do not confirm the entry
            return False

    def force_set_point_submit_callback(self) -> None:
        """
        Publishes the set-point entered into the entry field when the Enter key is pressed.
        """

        # Send force set point value
        self.force_set_point_publisher.publish(
            Float64(round(float(self.force_set_point_entry.get()), NUM_DIGITS_OF_FORCE_TO_DISPLAY))
        )

    def force_set_point_change_incremental(self, change: float) -> None:
        """
        Incrementally change the force set-point based on the given change.

        :param: change: The amount by which to change the set-point as a float.
        :return: None
        """

        # Try to get the value from the entry field
        try:
            entry_value = round(float(self.force_set_point_entry.get()), NUM_DIGITS_OF_FORCE_TO_DISPLAY)
        except ValueError:
            entry_value = 0.00

        # Calculate the new set point value based on the current value and the incremental change
        new_set_point_value = round(entry_value + change, NUM_DIGITS_OF_FORCE_TO_DISPLAY)

        # Ensure that the force set point cannot go below zero
        if new_set_point_value < 0:
            new_set_point_value = 0

        # Set the new value of the box
        self.force_set_point_entry.delete(0, END)
        self.force_set_point_entry.insert(0, str(new_set_point_value))

        # Send the new value out of the UI
        self.force_set_point_publisher.publish(
            Float64(round(float(self.force_set_point_entry.get()), NUM_DIGITS_OF_FORCE_TO_DISPLAY))
        )

    def test_force_profile_callback(self) -> None:
        """
        Sends the command to start or stop testing the force control profile based on the current button text.

        :return: None
        """

        # Get the current text of the button
        current_button_state = self.test_force_profile_button.cget(WIDGET_TEXT)

        # Send the correct message based on the current state of the button
        if current_button_state == TEST_FORCE_CONTROL:

            # Set the new text of the button
            new_text = STOP_TEST_FORCE_CONTROL

            # Send a command to conduct the test
            self.test_force_profile_publisher.publish(Bool(True))

        else:

            # If the current text of the button is not the only other option
            if current_button_state != STOP_TEST_FORCE_CONTROL:
                # Raise an exception
                raise Exception("Button state was not recognized.")

            # Set the new text of the button
            new_text = TEST_FORCE_CONTROL

            # Send a command to stop conducting the test
            self.test_force_profile_publisher.publish(Bool(False))

        self.test_force_profile_button.configure(text=new_text)

    def generate_new_image_cropping_button_callback(self) -> None:
        """
        Publishes the command to generate new image crop coordinates.
        :return: None
        """
        self.generate_new_image_cropping_command_publisher.publish(Bool(True))

    def load_existing_image_cropping_button_callback(self) -> None:
        """
        Publishes the command to load an existing image cropping.
        """
        self.load_existing_image_cropping_command_publisher.publish(Bool(True))

    def select_image_crop_callback(self) -> None:
        """
        Change the state of the image crop buttons based on radio button selection.
        """
        if bool(self.select_image_crop_variable.get()):
            self.generate_new_image_cropping_button.configure(state=NORMAL)
            self.load_existing_image_cropping_button.configure(state=NORMAL)
        else:
            self.generate_new_image_cropping_button.configure(state=DISABLED)
            self.load_existing_image_cropping_button.configure(state=DISABLED)

    def imaging_depth_submit_callback(self) -> None:
        """
        Publishes the set-point entered into the entry field when the Enter key is pressed.
        """

        # Send force set point value
        self.imaging_depth_publisher.publish(
            Float64(round(float(self.imaging_depth_entry.get()), NUM_DIGITS_OF_FORCE_TO_DISPLAY))
        )

    def identify_thyroid_from_points_button_callback(self) -> None:
        """
        Publish the command to generate the grabcut filter mask
        """
        self.identify_thyroid_from_points_command_publisher.publish(Bool(True))

    def identify_thyroid_from_template_button_callback(self) -> None:
        """
        Publish the command to identify the thyroid in the image using a template.
        """
        self.identify_thyroid_from_template_command_publisher.publish(Bool(True))

    def scan_positive_button_callback(self) -> None:
        """
        Publish the command to scan upwards.
        """
        self.scan_command_publisher.publish(
            Float64(float(self.scan_distance_entry.get()) / 100)  # convert cm to m
        )

        # Update the pose control button
        self.update_pose_control_button()

        # Update the data saving button
        self.save_experiment_data_button_callback(action=START_SAVING_EXPERIMENT_DATA)

    def scan_negative_button_callback(self) -> None:
        """
        Publish the command to scan downwards.
        """
        self.scan_command_publisher.publish(
            Float64(-float(self.scan_distance_entry.get()) / 100)  # convert cm to m
        )

        # Update the pose control button
        self.update_pose_control_button()

        # Update the data saving button
        self.save_experiment_data_button_callback(action=START_SAVING_EXPERIMENT_DATA)

    def registered_data_save_location_button_callback(self) -> None:
        """
        Select the directory in which to save the registered data, publishes it, and displays it.
        """
        selected_directory = askdirectory(initialdir=self.registered_data_save_location_str_var.get(),
                                          title="Select the destination for the exam data.")
        if len(selected_directory) > 3:
            self.registered_data_save_location_publisher.publish(String(selected_directory))
            self.registered_data_save_location_str_var.set(selected_directory)

    def registered_data_load_location_button_callback(self) -> None:
        """
        Select the directory from which to load the data used to generate the volume. Then publish it and display it.
        """
        selected_directory = askdirectory(initialdir=self.registered_data_load_location_str_var.get(),
                                          title="Select the data to load to generate the volume.")
        if len(selected_directory) > 3:
            self.registered_data_load_location_publisher.publish(String(selected_directory))
            self.registered_data_load_location_str_var.set(selected_directory)

    def volume_data_save_location_button_callback(self) -> None:
        """
        Select the directory in which to save the volume data, publish it, and display it.
        """
        selected_directory = askdirectory(initialdir=self.volume_data_save_location_str_var.get(),
                                          title="Select the destination for the volume data.")
        if len(selected_directory) > 3:
            self.volume_data_save_location_publisher.publish(String(selected_directory))
            self.volume_data_save_location_str_var.set(selected_directory)

    def update_pose_control_button(self) -> None:

        # Set that pose feedback is now being used
        self.currently_using_pose_feedback = True

        # Publish that pose feedback is now being used
        self.use_pose_feedback_command_publisher.publish(Bool(self.currently_using_pose_feedback))

        # Update the pose control button
        self.pose_control_button[WIDGET_STATE] = NORMAL

    def trajectory_complete_callback(self, msg: Bool) -> None:
        """
        Resets the pose control and experiment data saving buttons.
        """

        # Set the state to be that the robot is not currently using force_feedback
        self.currently_using_pose_feedback = False

        # Publish the command to stop using force feedback
        self.use_pose_feedback_command_publisher.publish(Bool(self.currently_using_pose_feedback))

        # Set the state of the button
        self.pose_control_button[WIDGET_STATE] = DISABLED

        # Reset the save experiment data buttons
        self.save_experiment_data_button_callback(action=STOP_SAVING_EXPERIMENT_DATA)

    def complete_full_scan_button_callback(self) -> None:
        """
        Publishes the command to complete a full scan.
        """
        self.complete_full_scan_command_publisher.publish(Bool(True))

    def generate_volume_button_callback(self) -> None:
        """
        Publish the command to generate a volume from the ultrasound images.
        """
        self.generate_volume_command_publisher.publish(Bool(True))

    def display_volume_button_callback(self) -> None:
        """
        Publish the command to display the generated volume.
        """
        if bool(self.generate_volume_selector_variable.get()):
            self.display_volume_command_publisher.publish(Bool(True))
        else:
            self.display_volume_command_publisher.publish(Bool(False))

    def image_streaming_button_callback(self):
        """
        Toggles if the ultrasound images will be streamed, based on the user input.
        """
        # Get the current text of the button
        button_text = self.image_streaming_button[WIDGET_TEXT]

        # If the button currently says "Start"
        if button_text == START_IMAGE_STREAMING:

            # Publish the command to start filtering images
            self.image_streaming_command_publisher.publish(Bool(True))

            # Set it to say "Stop"
            new_button_text = STOP_IMAGE_STREAMING

        # If the button currently says "Stop"
        else:

            # Publish the command to stop filtering images
            self.image_streaming_command_publisher.publish(Bool(False))

            # Set the button to say "Stop"
            new_button_text = START_IMAGE_STREAMING

        # Set the new text of the button
        self.image_streaming_button[WIDGET_TEXT] = new_button_text

    def restart_image_streaming_button_callback(self):
        self.restart_image_streaming_command_publisher.publish(Bool(True))

    def image_control_button_callback(self):
        """
        Toggles if the robot will use image feedback, based on the user input.
        """
        # Get the current text of the button
        button_text = self.image_control_button[WIDGET_TEXT]

        # If the button currently says "Start"
        if button_text == START_IMAGE_CONTROL:

            # Publish the command to start using image control
            self.use_image_feedback_command_publisher.publish(Bool(True))

            # Set it to say "Stop"
            new_button_text = STOP_IMAGE_CONTROL

            # Set the state to be that image control is currently being used
            self.currently_using_image_control = True

        # If the button currently says "Stop"
        else:

            # Publish the command to stop filtering images
            self.use_image_feedback_command_publisher.publish(Bool(False))

            # Set the button to say "Stop"
            new_button_text = START_IMAGE_CONTROL

            # Set the state to be that image control is not currently being used
            self.currently_using_image_control = False

        # Set the new text of the button
        self.image_control_button[WIDGET_TEXT] = new_button_text

    def force_control_button_callback(self):
        """
        Toggles if the robot will use force feedback, based on the user input.
        """

        # Get the current text of the button
        button_text = self.force_control_button[WIDGET_TEXT]

        # If the button currently says "Start"
        if button_text == START_FORCE_CONTROL:

            # Set it to say "Stop"
            new_button_text = STOP_FORCE_CONTROL

            # Set the state to be that the robot is currently using force feedback
            self.currently_using_force_feedback = True

        # If the button currently says "Stop"
        else:

            # Set it to say "Start"
            new_button_text = START_FORCE_CONTROL

            # Set the state to be that the robot is not currently using force_feedback
            self.currently_using_force_feedback = False

        # Publish the command to stop using force feedback
        self.use_force_feedback_command_publisher.publish(Bool(self.currently_using_force_feedback))

        # Set the new text of the button
        self.force_control_button[WIDGET_TEXT] = new_button_text

    def balancing_control_button_callback(self):
        """
        Toggles if the robot will use balancing feedback, based on the user input.
        """

        # Get the current text of the button
        button_text = self.balancing_control_button[WIDGET_TEXT]

        # If the button currently says "Start"
        if button_text == START_BALANCING_CONTROL:

            # Set it to say "Stop"
            new_button_text = STOP_BALANCING_CONTROL

            # Set the state to be that the robot is currently using force feedback
            self.currently_using_balancing_feedback = True

        # If the button currently says "Stop"
        else:

            # Set it to say "Start"
            new_button_text = START_BALANCING_CONTROL

            # Set the state to be that the robot is not currently using force_feedback
            self.currently_using_balancing_feedback = False

        # Publish the command to stop using force feedback
        self.use_balancing_feedback_command_publisher.publish(Bool(self.currently_using_balancing_feedback))

        # Set the new text of the button
        self.balancing_control_button[WIDGET_TEXT] = new_button_text

    def pose_control_button_callback(self):
        """
        Toggles if the robot will use pose feedback, based on the user input.
        """

        # Set the state to be that the robot is not currently using force_feedback
        self.currently_using_pose_feedback = False

        # Publish the command to stop using force feedback
        self.use_pose_feedback_command_publisher.publish(Bool(self.currently_using_pose_feedback))

        # Publish the command to clear the trajectory
        self.clear_trajectory_command_publisher.publish(Bool(True))

        # Set the state of the button
        self.pose_control_button[WIDGET_STATE] = DISABLED

        # Reset the save experiment data buttons
        self.save_experiment_data_button_callback(action=STOP_SAVING_EXPERIMENT_DATA)

    def pid_controller_selection_callback(self) -> None:
        """
        Publish the pid controller selected on the interface.
        """
        self.pid_controller_publisher.publish(UInt8(self.pid_selector.get()))

    def speed_selector_callback(self) -> None:
        """
        Publish the speed selected on the interface.
        """
        self.speed_selector_publisher.publish(Float64(self.speed_selector.get() / 100))

    def pid_value_setting_callback(self) -> None:
        """
        Publish the values selected in the interface
        """
        self.p_gain_publisher.publish(Float64(float(self.p_gain_entry.get())))
        self.i_gain_publisher.publish(Float64(float(self.i_gain_entry.get())))
        self.d_gain_publisher.publish(Float64(float(self.d_gain_entry.get())))

    def send_image_saving_command_callback(self) -> None:

        if self.save_images_button.cget(WIDGET_TEXT) == START_SAVING_IMAGES:
            self.send_image_saving_command_publisher.publish(Bool(True))
            self.save_images_button.configure(text=STOP_SAVING_IMAGES)
        elif self.save_images_button.cget(WIDGET_TEXT) == STOP_SAVING_IMAGES:
            self.send_image_saving_command_publisher.publish(Bool(False))
            self.save_images_button.configure(text=START_SAVING_IMAGES)
        else:
            raise Exception("Something very strange has happened here.")

    def send_save_images_destination(self) -> None:
        self.send_folder_destination_publisher.publish(askdirectory(initialdir='/home/ben',
                                                                    title="Select destination for saved images."))

    def send_patient_contact_override_button_callback(self) -> None:
        if self.send_patient_contact_override_button[WIDGET_TEXT] == START_SENDING_OVERRIDE_VALUE:
            self.is_patient_contact_override_active = True
            self.in_contact_radio_button[WIDGET_STATE] = DISABLED
            self.not_in_contact_radio_button[WIDGET_STATE] = DISABLED
            self.send_patient_contact_override_button[WIDGET_TEXT] = STOP_SENDING_OVERRIDE_VALUE
        elif self.send_patient_contact_override_button[WIDGET_TEXT] == STOP_SENDING_OVERRIDE_VALUE:
            self.is_patient_contact_override_active = False
            self.in_contact_radio_button[WIDGET_STATE] = NORMAL
            self.not_in_contact_radio_button[WIDGET_STATE] = NORMAL
            self.send_patient_contact_override_button[WIDGET_TEXT] = START_SENDING_OVERRIDE_VALUE
        else:
            raise Exception("Button text was not recognized")

    def send_registered_data_override_button_callback(self) -> None:
        if self.send_registered_data_override_button[WIDGET_TEXT] == START_SENDING_REGISTERED_DATA_OVERRIDE_VALUE:
            self.is_registered_data_override_active = True
            self.send_registered_data_override_button[WIDGET_TEXT] = STOP_SENDING_REGISTERED_DATA_OVERRIDE_VALUE
        elif self.send_registered_data_override_button[WIDGET_TEXT] == STOP_SENDING_REGISTERED_DATA_OVERRIDE_VALUE:
            self.is_registered_data_override_active = False
            self.send_registered_data_override_button[WIDGET_TEXT] = START_SENDING_REGISTERED_DATA_OVERRIDE_VALUE
        else:
            raise Exception("Button text was not recognized")

    def save_experiment_data_button_callback(self, action: str = TOGGLE_SAVING_EXPERIMENT_DATA) -> None:

        new_command_message = SaveExperimentDataCommand()

        if (action == TOGGLE_SAVING_EXPERIMENT_DATA and
            self.save_experiment_data_button[WIDGET_TEXT] == START_SAVING_EXPERIMENT_DATA) or \
                action == START_SAVING_EXPERIMENT_DATA:
            self.save_experiment_data_button[WIDGET_TEXT] = STOP_SAVING_EXPERIMENT_DATA
            if bool(self.save_robot_pose_variable.get()):
                new_command_message.save_pose = True
            if bool(self.save_robot_force_variable.get()):
                new_command_message.save_force = True
            if bool(self.save_raw_image_variable.get()):
                new_command_message.save_raw_image = True
            if bool(self.save_image_data_objects_variable.get()):
                new_command_message.save_image_data = True
            if bool(self.save_image_centroid_variable.get()):
                new_command_message.save_image_centroid = True
            if bool(self.save_skin_error_variable.get()):
                new_command_message.save_skin_error = True
            self.save_robot_pose_yes_button[WIDGET_STATE] = DISABLED
            self.save_robot_pose_no_button[WIDGET_STATE] = DISABLED
            self.save_robot_force_yes_button[WIDGET_STATE] = DISABLED
            self.save_robot_force_no_button[WIDGET_STATE] = DISABLED
            self.save_raw_image_yes_button[WIDGET_STATE] = DISABLED
            self.save_raw_image_no_button[WIDGET_STATE] = DISABLED
            self.save_image_data_objects_yes_button[WIDGET_STATE] = DISABLED
            self.save_image_data_objects_no_button[WIDGET_STATE] = DISABLED
            self.save_image_centroid_yes_button[WIDGET_STATE] = DISABLED
            self.save_image_centroid_no_button[WIDGET_STATE] = DISABLED
            self.save_skin_error_yes_button[WIDGET_STATE] = DISABLED
            self.save_skin_error_no_button[WIDGET_STATE] = DISABLED

        elif (action == TOGGLE_SAVING_EXPERIMENT_DATA and
              self.save_experiment_data_button[WIDGET_TEXT] == STOP_SAVING_EXPERIMENT_DATA) or \
                action == STOP_SAVING_EXPERIMENT_DATA:
            self.save_experiment_data_button[WIDGET_TEXT] = START_SAVING_EXPERIMENT_DATA
            self.save_robot_pose_yes_button[WIDGET_STATE] = NORMAL
            self.save_robot_pose_no_button[WIDGET_STATE] = NORMAL
            self.save_robot_force_yes_button[WIDGET_STATE] = NORMAL
            self.save_robot_force_no_button[WIDGET_STATE] = NORMAL
            self.save_raw_image_yes_button[WIDGET_STATE] = NORMAL
            self.save_raw_image_no_button[WIDGET_STATE] = NORMAL
            self.save_image_data_objects_yes_button[WIDGET_STATE] = NORMAL
            self.save_image_data_objects_no_button[WIDGET_STATE] = NORMAL
            self.save_image_centroid_yes_button[WIDGET_STATE] = NORMAL
            self.save_image_centroid_no_button[WIDGET_STATE] = NORMAL
            self.save_skin_error_yes_button[WIDGET_STATE] = NORMAL
            self.save_skin_error_no_button[WIDGET_STATE] = NORMAL
        else:
            raise Exception("Button text was not recognized.")

        self.save_experiment_data_command_publisher.publish(new_command_message)

    def create_noise_button_callback(self) -> None:

        if self.create_noise_button[WIDGET_TEXT] == START_CREATING_VELOCITY_NOISE:
            self.create_noise_command_publisher.publish(Bool(True))
            self.create_noise_button[WIDGET_TEXT] = STOP_CREATING_VELOCITY_NOISE
        elif self.create_noise_button[WIDGET_TEXT] == STOP_CREATING_VELOCITY_NOISE:
            self.create_noise_command_publisher.publish(Bool(False))
            self.create_noise_button[WIDGET_TEXT] = START_CREATING_VELOCITY_NOISE
        else:
            raise Exception("Button text was not recognized.")

    # endregion
    ############################################################################

    #############################################################################
    # Define ROS callbacks
    # region
    def robot_sensed_force_callback(self, data: WrenchStamped):
        """
        A callback function to update the GUI with the current Z force felt by the robot.

        Parameters
        ----------
        data
            The WrenchStamped message sent by the robot containing the current force experienced by the robot.
        """
        self.current_force_string_var.set(str(round(data.wrench.force.z, 2)))

    def debug_status_messages_callback(self, data: log_message):
        self.update_status(data)

    def p_gain_message_callback(self, data: Float64) -> None:
        """
        Set the value of the p gain sent to the node.
        """
        self.p_gain_var.set(str(data.data))

    def i_gain_message_callback(self, data: Float64) -> None:
        """
        Set the value of the i gain sent to the node.
        """
        self.i_gain_var.set(str(data.data))

    def d_gain_message_callback(self, data: Float64) -> None:
        """
        Set the value of the d gain sent to the node.
        """
        self.d_gain_var.set(str(data.data))

    def shutdown_node(self):
        """
        Define the custom shutdown behavior of the node to close the window.
        """
        print("\nShutdown signal received.")
        self.parent.destroy()

    # endregion
    #############################################################################

    #############################################################################
    # Define Helpers
    # region
    # noinspection PyTypeChecker
    def update_status(self, message: log_message = None) -> None:
        """
        Defines a function to update the status label at the bottom of the window with a given message.

        Parameters
        ----------
        message
            The message to be displayed. A period will be added to the end of this message automatically.
            If no message is given, the empty status string will be shown.
        """
        if message is not None:
            pass
            """self.status_log_string = self.status_log_string + '\n' + message.message + "."
            self.status_label.configure(state=NORMAL)
            self.status_label.delete('1.0', END)
            self.status_label.insert(INSERT, self.status_log_string)
            self.status_label.configure(state=DISABLED)"""

        # TODO - Low - Turn this into a better error logging device. Show message, time sent, and node sending
        # TODO - Low - Create an option to see minimal, standard, or verbose logging data.

    @staticmethod
    def add_widgets(list_of_widgets: list) -> None:
        """
        Adds a list of widgets to a grid based on a given position and size.

        Parameters
        ----------
        list_of_widgets
            A list of widgets where each entry in the list is formatted as:
             (widget object, column position, column span, row position, row span)
        """
        for widget in list_of_widgets:
            widget[0].grid(column=widget[1], columnspan=widget[2], row=widget[3], rowspan=widget[4], sticky='nsew')

    def disable_enable_pages(self, always_visible_page: bool = None,
                             exam_setup_page: bool = None,
                             thyroid_exam_page: bool = None,
                             nodule_exam_page: bool = None,
                             ) -> None:
        """
        Disables or enables all the widgets on a given frame based on the given status.
        """

        # For each page status, enable/disable each widget accordingly
        for widget_list, page_status in zip([self.button_controls_frame_widgets,
                                             self.exam_setup_widgets,
                                             self.thyroid_exam_widgets,
                                             self.nodule_exam_widgets],
                                            [always_visible_page,
                                             exam_setup_page,
                                             thyroid_exam_page,
                                             nodule_exam_page]
                                            ):
            self.disable_enable_page(widget_list, page_status)

    @staticmethod
    def disable_enable_page(list_of_widgets, new_state: Bool = None) -> None:
        """
        Disables or enables every widget in a list of widgets based on the given state.
        """

        # If a state has been given, select the corresponding status
        if new_state is not None:
            if new_state:
                new_status = NORMAL
            else:
                new_status = DISABLED

            # Apply that status to each widget
            for widget in list_of_widgets:
                widget.configure(state=new_status)

    # endregion
    #############################################################################

    def first_time_publishing_loop(self):
        """
        Publishes messages a single time on startup.
        """
        self.registered_data_save_location_publisher.publish(String(self.registered_data_save_location_str_var.get()))
        self.registered_data_load_location_publisher.publish(String(self.registered_data_load_location_str_var.get()))
        self.volume_data_save_location_publisher.publish(String(self.volume_data_save_location_str_var.get()))
        self.speed_selector_publisher.publish(Float64(1.0))

    def publishing_loop(self):
        """
        Publishes the different commands at regular intervals.
        """
        self.use_image_feedback_command_publisher.publish(Bool(self.currently_using_image_control))
        self.use_force_feedback_command_publisher.publish(Bool(self.currently_using_force_feedback))
        self.use_pose_feedback_command_publisher.publish(Bool(self.currently_using_pose_feedback))
        self.use_balancing_feedback_command_publisher.publish(Bool(self.currently_using_balancing_feedback))
        if self.is_patient_contact_override_active:
            self.patient_contact_publisher.publish(Bool(bool(self.select_patient_contact_override_variable.get())))
        if self.is_registered_data_override_active:
            self.registered_data_publisher.publish(RegisteredDataMsg())
        new_msg = TwistStamped()
        new_msg.header.stamp = Time.now()
        if self.manual_control_velocity_message is not None:
            new_msg.twist = self.manual_control_velocity_message
        self.manual_robot_control_publisher.publish(new_msg)
        root.after(25, self.publishing_loop)


# (ttk.Label(exam_setup_frame, text="Current Set-point (N):"), LEFT_COLUMN, SINGLE_COLUMN, 0, DOUBLE_ROW)


class WidgetCreationObject:

    def __init__(self, widget_object,
                 col_num: int, row_num: int,
                 col_span: int = SINGLE_ROW, row_span: int = SINGLE_COLUMN,
                 ipadx: int = 0, ipady: int = 0,
                 padx: int = 5, pady: int = 5,
                 sticky: str = 'nsew'):
        self.widget_object = widget_object
        self.col_num = col_num
        self.col_span = col_span
        self.row_num = row_num
        self.row_span = row_span
        self.ipadx = ipadx
        self.ipady = ipady
        self.padx = padx
        self.pady = pady
        self.sticky = sticky

    def add_widget(self):
        self.widget_object.grid(column=self.col_num,
                                columnspan=self.col_span,
                                row=self.row_num,
                                rowspan=self.row_span,
                                ipadx=self.ipadx,
                                ipady=self.ipady,
                                padx=self.padx,
                                pady=self.pady,
                                sticky=self.sticky
                                )


if __name__ == "__main__":
    # Create a root window
    root = Tk()

    # Create the control application within the root window
    node = UserInterface(root)

    # Run the first loop publishers
    node.first_time_publishing_loop()

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Define the publishing rate
    publishing_rate = 10  # hz

    # Run the publishing loop in the background alongside the TKinter main loop
    root.after(round(1000 / publishing_rate), node.publishing_loop)

    # Run until the window is closed.
    root.mainloop()
