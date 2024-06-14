#!/usr/bin/env python3

"""
File containing UserInterface class.
"""
# TODO - Dream - Turn this into a better error logging device. Show message, time sent, and node sending
# TODO - Dream - Create an option to see minimal, standard, or verbose logging data.
# TODO - Dream - Add a command to stop all motion and return all states back to the robot not moving

# Import standard packages
from tkinter import *
from tkinter.scrolledtext import ScrolledText
from tkinter.filedialog import askdirectory
import tkinter.ttk as ttk
from argparse import ArgumentParser
from os.path import isdir
from copy import copy
from datetime import datetime

# Import ROS packages
from geometry_msgs.msg import WrenchStamped, TwistStamped, Twist
from std_msgs.msg import Int8

# Import custom python packages
from thyroid_ultrasound_imaging_support.Visualization.VisualizationConstants import *
from thyroid_ultrasound_support.Constants.SharedConstants import REST_PHASE, GROWTH_PHASE

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_messages.msg import SaveExperimentDataCommand

# Define constants used for logging purposes
VERBOSE: int = int(0)

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
RESUME_TRAJECTORY: str = 'Resume\nTrajectory Following'
PAUSE_TRAJECTORY: str = 'Pause\nTrajectory Following'
START_POSE_CONTROL: str = "Start\nPose Control"
STOP_POSE_CONTROL: str = "Stop\nPose Control"
TEST_FORCE_CONTROL: str = "Test Force\n Profile"
STOP_TEST_FORCE_CONTROL: str = "Stop Testing\n Force Control"
START_SAVING_IMAGES: str = "Start Saving Images"
STOP_SAVING_IMAGES: str = "Stop Saving Images"
START_SENDING_OVERRIDE_VALUE: str = "Start sending\noverride value"
STOP_SENDING_OVERRIDE_VALUE: str = "Stop sending\noverride value"
START_SENDING_PATIENT_CONTACT_OVERRIDE_VALUE: str = "Start overriding patient contact value"
STOP_SENDING_PATIENT_CONTACT_OVERRIDE_VALUE: str = "Stop overriding patient contact value"
START_SENDING_FORCE_CONTROL_OVERRIDE_VALUE: str = "Start overriding force control value"
STOP_SENDING_FORCE_CONTROL_OVERRIDE_VALUE: str = "Stop overriding force control value"
START_SENDING_REGISTERED_DATA_OVERRIDE_VALUE: str = "Start overriding registered data value"
STOP_SENDING_REGISTERED_DATA_OVERRIDE_VALUE: str = "Stop overriding registered data value"
START_SENDING_IMAGE_CENTERED_OVERRIDE_VALUE: str = "Start overriding image centered value"
STOP_SENDING_IMAGE_CENTERED_OVERRIDE_VALUE: str = "Stop overriding image centered value"
START_SENDING_IMAGE_BALANCED_OVERRIDE_VALUE: str = "Start overriding image balanced value"
STOP_SENDING_IMAGE_BALANCED_OVERRIDE_VALUE: str = "Stop overriding image balanced value"
START_SAVING_EXPERIMENT_DATA: str = "Start saving\nexperiment data"
STOP_SAVING_EXPERIMENT_DATA: str = "Stop saving\nexperiment data"
TOGGLE_SAVING_EXPERIMENT_DATA: str = "Toggle saving\nexperiment data"
START_CREATING_VELOCITY_NOISE: str = "Start creating\nvelocity noise"
STOP_CREATING_VELOCITY_NOISE: str = "Stop creating\nvelocity noise"
PLAYBACK_STREAM_IN_CHRONOLOGICAL_ORDER: str = "Playback images in chronological order."
PLAYBACK_STREAM_IN_REVERSE_CHRONOLOGICAL_ORDER: str = "Playback images in reverse chronological order."
START_PUBLISHING_CONTROLLER_STATUS_VALUES: str = "Start publishing controller status values."
STOP_PUBLISHING_CONTROLLER_STATUS_VALUES: str = "Stop publishing controller status values."
ACTIVATE_MANUAL_CONTROLS: str = "Activate\nManual Controls"
DEACTIVATE_MANUAL_CONTROLS: str = "Deactivate\nManual Controls"
SET_TO_REST_PHASE: str = 'Set to Rest Phase'
SET_TO_GROWTH_PHASE: str = 'Set to Growth Phase'

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

# Define the maximum number of status messages to save
MAXIMUM_STATUS_MESSAGES: int = int(50)

# Define function mode constants
TESTING: int = int(0)
RUNNING: int = int(1)

# Define constants for yes and no radio buttons
YES_BUTTON: int = int(1)
NO_BUTTON: int = int(0)

# Define constants for the left, right, and middle radio buttons
LEFT_BUTTON: int = int(-1)
MIDDLE_BUTTON: int = int(0)
RIGHT_BUTTON: int = int(1)

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

        # Startup the node
        init_node(USER_INTERFACE)

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

        # --------------------------------------------
        # Define the known list of nodes in the system
        # region

        # Create a variable for all the expected node names
        self.node_names_all = list(CORE_NAMES)

        # Add the correct nodes based on the arguments passed
        if passed_arguments.testing_mode:
            self.node_names_all.insert(0, CLARIUS_US_SPOOF)
        else:
            self.node_names_all.insert(0, CLARIUS_US_PUBLISHER)
        if passed_arguments.testing_mode or passed_arguments.experimentation_mode:
            self.node_names_all.append(CONTROLLER_TUNING_GRAPHS)
        if passed_arguments.experimentation_mode:
            self.node_names_all.append(EXPERIMENT_DATA_RECORDER)

        # endregion
        # --------------------------------------------

        # -------------------------------------
        # Define the basic structure of the GUI
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
        # nodule_exam_frame = ttk.Frame(tab_controller)
        status_logging_frame = ttk.Frame(tab_controller)
        developer_frame = ttk.Frame(tab_controller)
        experimentation_frame = ttk.Frame(tab_controller)

        # Add the frames to the tab controller
        tab_controller.add(exam_setup_frame, text="Exam Setup")
        tab_controller.add(thyroid_exam_frame, text="Thyroid Exam")
        # tab_controller.add(nodule_exam_frame, text="Nodule Exam")
        tab_controller.add(status_logging_frame, text="Status Logger")
        if passed_arguments.experimentation_mode:
            tab_controller.add(developer_frame, text="Developer")
        if passed_arguments.experimentation_mode:
            tab_controller.add(experimentation_frame, text="Experimentation")

        # endregion
        # -------------------------------------

        # ----------------------------------------
        # Define the logging components of the GUI
        # region

        # Define row counter
        ee = 0

        # Define a dictionary in which to store the current status message variables
        self.current_status_messages = {}

        # Define a dictionary in which to store the time of the most recent message
        self.current_status_times = {}

        # Define a dictionary in which to store the status message history for each node
        self.status_log_dict = {}

        # Define a variable for storing which node status to show
        self.selected_node = StringVar()

        # For each known node,
        for node_name in self.node_names_all:
            # Add the node to the status dictionaries
            self.current_status_messages.update({node_name: StringVar()})
            self.current_status_times.update({node_name: StringVar()})
            self.status_log_dict.update({node_name: []})

            # Create a label for the node
            ee = create_widget_object(ttk.Label(status_logging_frame, text=node_name + ':'),
                                      col_num=MIDDLE_COLUMN, row_num=ee)

            # Create a current status label for the node
            ee = create_widget_object(ttk.Label(status_logging_frame,
                                                textvariable=self.current_status_times[node_name]),
                                      col_num=R_MIDDLE_COLUMN, row_num=ee)

            # Create a current status label for the node
            ee = create_widget_object(ttk.Label(status_logging_frame,
                                                textvariable=self.current_status_messages[node_name]),
                                      col_num=RR_MIDDLE_COLUMN, row_num=ee)

            # Create a radio button to select the status
            ee = create_widget_object(
                ttk.Radiobutton(status_logging_frame, text="", variable=self.selected_node, value=node_name,
                                command=self.select_node),
                col_num=RIGHT_COLUMN, row_num=ee, increment_row=True)

        # Define logging window
        self.status_label = ScrolledText(status_logging_frame, wrap=WORD, width=35)
        self.status_label.configure(state=DISABLED)
        ee = create_widget_object(self.status_label, col_num=LEFT_COLUMN, col_span=SINGLE_COLUMN,
                                  row_num=0, row_span=len(self.node_names_all))

        # Add to the log
        self.ui_update_log('Application started')

        temp_msg = log_message(source=USER_INTERFACE, message='ACTIVE')
        temp_msg.header.stamp = Time.now()
        self.update_status(temp_msg)

        # Select the user interface node to display
        self.selected_node.set(USER_INTERFACE)
        self.select_node()

        # endregion
        # ----------------------------------------

        # ---------------------------------------
        # Create ROS components of the User Interface
        # region

        # Define custom shutdown behavior
        on_shutdown(self.shutdown_node)

        # Wait for the services to be built
        # wait_for_service(IPR_REGISTERED_DATA_SAVE_LOCATION)
        # wait_for_service(NRTS_REGISTERED_DATA_LOAD_LOCATION)
        # wait_for_service(VG_REGISTERED_DATA_LOAD_LOCATION)
        # wait_for_service(VG_VOLUME_DATA_SAVE_LOCATION)

        try:
            # Define the trajectory management node client proxies
            self.tm_override_patient_contact_service = ServiceProxy(TM_OVERRIDE_PATIENT_CONTACT, BoolRequest)
            self.tm_override_force_control_service = ServiceProxy(TM_OVERRIDE_FORCE_CONTROL, BoolRequest)
            self.tm_override_image_balanced_service = ServiceProxy(TM_OVERRIDE_IMAGE_BALANCED, BoolRequest)
            self.tm_override_image_centered_service = ServiceProxy(TM_OVERRIDE_IMAGE_CENTERED, BoolRequest)
            self.tm_override_data_registered_service = ServiceProxy(TM_OVERRIDE_DATA_REGISTERED, BoolRequest)
            self.create_trajectory_service = ServiceProxy(TM_CREATE_TRAJECTORY, Float64Request)
            self.image_spacing_selection_service = ServiceProxy(TM_SET_TRAJECTORY_SPACING, Float64Request)
            self.clear_trajectory_service = ServiceProxy(TM_CLEAR_TRAJECTORY, BoolRequest)
            self.registered_data_save_location_service = ServiceProxy(IPR_REGISTERED_DATA_SAVE_LOCATION, StringRequest)
        except ServiceException:
            pass

        try:
            # Define the robot control node client proxies
            self.rc_override_patient_contact_service = ServiceProxy(RC_OVERRIDE_PATIENT_CONTACT, BoolRequest)
            self.view_controller_gains_service = ServiceProxy(RC_VIEW_CONTROLLER_GAINS, ViewControllerGains)
            self.set_controller_gains_service = ServiceProxy(RC_SET_CONTROLLER_GAINS, SetControllerGains)
            self.speed_selector_service = ServiceProxy(RC_OVERALL_ROBOT_SPEED, Float64Request)
            self.use_pose_feedback_command_service = ServiceProxy(RC_USE_POSE_CONTROL, BoolRequest)
            self.use_force_feedback_command_service = ServiceProxy(RC_USE_FORCE_CONTROL, BoolRequest)
            self.use_image_feedback_command_service = ServiceProxy(RC_USE_IMAGE_CENTERING, BoolRequest)
            self.use_balancing_feedback_command_service = ServiceProxy(RC_USE_IMAGE_BALANCING, BoolRequest)
            self.publish_controller_statuses_service = ServiceProxy(RC_PUBLISH_CONTROLLER_STATUSES, BoolRequest)
        except ServiceException:
            pass

        # Define the clarius control node client proxies
        self.send_image_saving_command_service = ServiceProxy(CC_SAVE_IMAGES, BoolRequest)
        self.send_folder_destination_service = ServiceProxy(CC_SAVED_IMAGES_DESTINATION, StringRequest)

        # Define the clarius spoof node client proxies
        if passed_arguments.testing_mode:
            self.image_streaming_location_service = ServiceProxy(CS_IMAGE_LOCATION, StringRequest)
            self.image_streaming_command_service = ServiceProxy(CS_IMAGE_STREAMING_CONTROL, BoolRequest)
            self.restart_image_streaming_command_service = ServiceProxy(CS_IMAGE_STREAMING_RESTART, BoolRequest)
            self.reverse_stream_order_service = ServiceProxy(CS_IMAGE_STREAMING_REVERSE_PLAYBACK_DIRECTION, BoolRequest)
            self.image_streaming_frequency_service = ServiceProxy(CS_IMAGE_STREAMING_SET_FREQUENCY, Float64Request)

        # Define the image based user input node service proxies
        self.generate_new_image_cropping_command_service = ServiceProxy(IB_UI_CROP_IMAGE_FROM_POINTS, BoolRequest)
        self.load_existing_image_cropping_command_service = ServiceProxy(IB_UI_CROP_IMAGE_FROM_TEMPLATE,
                                                                         BoolRequest)
        self.identify_thyroid_from_points_command_service = ServiceProxy(IB_UI_IDENTIFY_THYROID_FROM_POINTS,
                                                                         BoolRequest)

        # Define the real-time segmentation service proxies
        self.set_segmentation_phase_service = ServiceProxy(RTS_SET_SEGMENTATION_PHASE, StringRequest)

        # Define the image position registration service proxies
        self.register_new_data_service = ServiceProxy(IPR_REGISTER_NEW_DATA, BoolRequest)

        # Define the non-real-time segmentation node service proxies
        self.nrts_registered_data_load_location_service = ServiceProxy(NRTS_REGISTERED_DATA_LOAD_LOCATION,
                                                                       StringRequest)
        self.nrts_generate_volume_command_service = ServiceProxy(NRTS_GENERATE_VOLUME, BoolRequest)

        # Define the volume generation node service proxies
        self.vg_registered_data_load_location_service = ServiceProxy(VG_REGISTERED_DATA_LOAD_LOCATION,
                                                                     StringRequest)
        self.vg_generate_volume_command_service = ServiceProxy(VG_GENERATE_VOLUME, BoolRequest)
        self.display_loaded_volume_command_service = ServiceProxy(VG_DISPLAY_VOLUME, BoolRequest)
        self.volume_data_save_location_service = ServiceProxy(VG_VOLUME_DATA_SAVE_LOCATION, StringRequest)
        self.volume_data_load_location_service = ServiceProxy(VG_VOLUME_DATA_LOAD_LOCATION, StringRequest)

        # Define the visualization node service proxies
        self.show_visualization_services = {}
        for visualization_name, service_name in zip([SHOW_ORIGINAL, SHOW_CROPPED, SHOW_RECOLOR, SHOW_BLUR, SHOW_MASK,
                                                     SHOW_POST_PROCESSED_MASK, SHOW_SURE_FOREGROUND,
                                                     SHOW_SURE_BACKGROUND, SHOW_PROBABLE_FOREGROUND,
                                                     SHOW_INITIALIZED_MASK, SHOW_CENTROIDS_ONLY,
                                                     SHOW_CENTROIDS_CROSS_ONLY, SHOW_MASK_CENTROIDS_CROSS_OVERLAY,
                                                     SHOW_FOREGROUND, SHOW_SKIN_APPROXIMATION,
                                                     SHOW_GRABCUT_USER_INITIALIZATION_0],
                                                    [VIS_STATUS_SHOW_ORIGINAL, VIS_STATUS_SHOW_CROPPED,
                                                     VIS_STATUS_SHOW_RECOLOR, VIS_STATUS_SHOW_BLUR,
                                                     VIS_STATUS_SHOW_RESULT_MASK, VIS_STATUS_SHOW_POST_PROCESSED_MASK,
                                                     VIS_STATUS_SHOW_SURE_FOREGROUND, VIS_STATUS_SHOW_SURE_BACKGROUND,
                                                     VIS_STATUS_SHOW_PROBABLE_FOREGROUND,
                                                     VIS_STATUS_SHOW_INITIALIZATION_MASK,
                                                     VIS_STATUS_SHOW_CENTROIDS_ONLY,
                                                     VIS_STATUS_SHOW_CENTROIDS_CROSS_ONLY,
                                                     VIS_STATUS_SHOW_MASK_CENTROIDS_CROSS_OVERLAY,
                                                     VIS_STATUS_SHOW_FOREGROUND,
                                                     VIS_STATUS_SHOW_SKIN_APPROXIMATION,
                                                     VIS_STATUS_SHOW_GRABCUT_USER_INITIALIZATION_0]):
            self.show_visualization_services.update({visualization_name: ServiceProxy(service_name,
                                                                                      StatusVisualization)})

        # Create a publisher to publish the command to start and stop the robot motion
        # self.stop_robot_motion_command_publisher = Publisher(STOP_ALL_MOTION, Bool, queue_size=1)

        # Create a publisher to publish the desired force for the robot to exert
        self.force_set_point_publisher = Publisher(RC_FORCE_SET_POINT, Float64, queue_size=1)

        # Create a publisher to publish the imaging depth of the US probe
        self.imaging_depth_publisher = Publisher(IMAGE_DEPTH, Float64, queue_size=1)

        # Define command publishers for saving experiment data
        self.save_experiment_data_command_publisher = Publisher(EXP_SAVE_DATA_COMMAND, SaveExperimentDataCommand,
                                                                queue_size=1)

        # Define a command to start publishing random velocity noise
        self.create_noise_command_publisher = Publisher(EXP_CREATE_NOISE_COMMAND, Bool, queue_size=1)

        # Define a publisher for publishing robot control velocity commands
        self.manual_robot_control_publisher = Publisher(RC_MANUAL_CONTROL_INPUT, TwistStamped, queue_size=1)

        # Define a publisher for publishing image centering commands
        self.image_centering_side_publisher = Publisher(RC_IMAGE_CENTERING_SIDE, Int8, queue_size=1)

        # Create a subscriber to listen to the external force felt by the robot
        Subscriber(ROBOT_DERIVED_FORCE, WrenchStamped, self.robot_sensed_force_callback)

        # Create a subscriber to hear node logging messages
        Subscriber(LOGGING, log_message, self.log_messages_callback)

        # Create a subscriber to hear node statuses
        Subscriber(STATUS, log_message, self.node_statuses_callback)

        # Create a subscriber to listen for when the trajectory has been completed
        Service(UI_TRAJECTORY_COMPLETE, BoolRequest, self.trajectory_complete_handler)

        # Create subscribers to listen for the progress of the volume generation node
        Subscriber(EXP_ALL_DATA_SAVED, Bool, self.all_data_saved_callback)
        Subscriber(EXP_DATA_REMAINING_TO_SAVE, String, self.remaining_data_callback)
        Subscriber(VOLUME_DATA, Float64, self.volume_data_result_callback)

        # endregion
        # ---------------------------------------

        # -------------------------------------------
        # Create GUI components of the User Interface
        # region

        # Define the widgets used in the button_controls_frame
        # region

        # Define the row incrementer
        aa = 0

        # If in testing mode, add the streaming buttons
        if passed_arguments.testing_mode:
            self.image_streaming_button = ttk.Button(button_controls_frame, text=START_IMAGE_STREAMING,
                                                     command=self.image_streaming_button_callback,
                                                     state=DISABLED)
            aa = create_widget_object(self.image_streaming_button,
                                      col_num=LEFT_COLUMN, col_span=SINGLE_COLUMN,
                                      row_num=aa, row_span=SINGLE_ROW)
            self.restart_image_streaming_button = ttk.Button(button_controls_frame, text="Restart\nImage Streaming",
                                                             command=self.restart_image_streaming_button_callback,
                                                             state=DISABLED)
            aa = create_widget_object(self.restart_image_streaming_button,
                                      col_num=LL_MIDDLE_COLUMN, col_span=SINGLE_COLUMN,
                                      row_num=aa, row_span=SINGLE_ROW)

        # Define the force control widget
        self.force_control_button = ttk.Button(button_controls_frame, text=START_FORCE_CONTROL,
                                               command=self.force_control_button_callback)
        aa = create_widget_object(self.force_control_button, col_num=L_MIDDLE_COLUMN, row_num=aa)

        # Define the image balancing widget
        self.balancing_control_button = ttk.Button(button_controls_frame, text=START_BALANCING_CONTROL,
                                                   command=self.balancing_control_button_callback)
        aa = create_widget_object(self.balancing_control_button, col_num=LR_MIDDLE_COLUMN, row_num=aa)

        # Define the image centering widget
        self.image_control_button = ttk.Button(button_controls_frame, text=START_IMAGE_CONTROL,
                                               command=self.image_control_button_callback)
        aa = create_widget_object(self.image_control_button, col_num=MIDDLE_COLUMN, row_num=aa)

        # Define the trajectory following widget
        self.trajectory_following_button = ttk.Button(button_controls_frame, text=PAUSE_TRAJECTORY,
                                                      command=self.trajectory_following_button_callback,
                                                      state=DISABLED)
        aa = create_widget_object(self.trajectory_following_button, col_num=RL_MIDDLE_COLUMN, row_num=aa)

        # Define the pose control widget
        self.pose_control_button = ttk.Button(button_controls_frame, text=STOP_POSE_CONTROL,
                                              command=self.pose_control_button_callback, state=DISABLED)
        aa = create_widget_object(self.pose_control_button, col_num=RR_MIDDLE_COLUMN, row_num=aa)

        # Update the status log
        self.ui_update_log('Button controls added')

        # endregion

        # Define the widgets used for the robot_controls_frame
        # region
        bb = 0

        # Define the label for the section
        bb = create_widget_object(ttk.Label(robot_controls_frame, text="Manual Robot Controls",
                                            anchor=CENTER, justify=CENTER),
                                  col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=bb, increment_row=True)

        bb = create_horizontal_separator(robot_controls_frame, bb)

        # Define the activation widget
        self.manual_controls_status_button = ttk.Button(robot_controls_frame, text=ACTIVATE_MANUAL_CONTROLS,
                                                        command=self.manual_controls_status_button_callback)
        bb = create_widget_object(self.manual_controls_status_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=bb, increment_row=True)

        bb = create_horizontal_separator(robot_controls_frame, bb)

        # Define the negative x widget
        self.negative_x_movement_button = ttk.Button(robot_controls_frame, text=NEGATIVE_X, width=5, state=DISABLED)
        self.negative_x_movement_button.bind(BUTTON_PRESS,
                                             lambda event, axis_and_direction=NEGATIVE_X:
                                             self.robot_control_button_callback(
                                                 axis_and_direction=axis_and_direction))
        self.negative_x_movement_button.bind(BUTTON_RELEASE,
                                             lambda event, axis_and_direction=NO_MOVEMENT:
                                             self.robot_control_button_callback(
                                                 axis_and_direction=axis_and_direction))
        bb = create_widget_object(self.negative_x_movement_button, col_num=LEFT_COLUMN, row_num=bb)

        # Define the positive x widget
        self.positive_x_movement_button = ttk.Button(robot_controls_frame, text=POSITIVE_X, width=5, state=DISABLED)
        self.positive_x_movement_button.bind(BUTTON_PRESS,
                                             lambda event, axis_and_direction=POSITIVE_X:
                                             self.robot_control_button_callback(
                                                 axis_and_direction=axis_and_direction))
        self.positive_x_movement_button.bind(BUTTON_RELEASE,
                                             lambda event, axis_and_direction=NO_MOVEMENT:
                                             self.robot_control_button_callback(
                                                 axis_and_direction=axis_and_direction))
        bb = create_widget_object(self.positive_x_movement_button, col_num=RIGHT_COLUMN, row_num=bb,
                                  increment_row=True)

        bb = create_horizontal_separator(robot_controls_frame, bb)

        # Define the negative y widget
        self.negative_y_movement_button = ttk.Button(robot_controls_frame, text=NEGATIVE_Y, width=5, state=DISABLED)
        self.negative_y_movement_button.bind(BUTTON_PRESS,
                                             lambda event, axis_and_direction=NEGATIVE_Y:
                                             self.robot_control_button_callback(
                                                 axis_and_direction=axis_and_direction))
        self.negative_y_movement_button.bind(BUTTON_RELEASE,
                                             lambda event, axis_and_direction=NO_MOVEMENT:
                                             self.robot_control_button_callback(
                                                 axis_and_direction=axis_and_direction))
        bb = create_widget_object(self.negative_y_movement_button, col_num=LEFT_COLUMN, row_num=bb)

        # Define the positive y widget
        self.positive_y_movement_button = ttk.Button(robot_controls_frame, text=POSITIVE_Y, width=5, state=DISABLED)
        self.positive_y_movement_button.bind(BUTTON_PRESS,
                                             lambda event, axis_and_direction=POSITIVE_Y:
                                             self.robot_control_button_callback(
                                                 axis_and_direction=axis_and_direction))
        self.positive_y_movement_button.bind(BUTTON_RELEASE,
                                             lambda event, axis_and_direction=NO_MOVEMENT:
                                             self.robot_control_button_callback(
                                                 axis_and_direction=axis_and_direction))
        bb = create_widget_object(self.positive_y_movement_button, col_num=RIGHT_COLUMN, row_num=bb,
                                  increment_row=True)

        bb = create_horizontal_separator(robot_controls_frame, bb)

        # Define the negative pitch widget
        self.negative_pitch_movement_button = ttk.Button(robot_controls_frame, text=NEGATIVE_PITCH, width=5,
                                                         state=DISABLED)
        self.negative_pitch_movement_button.bind(BUTTON_PRESS,
                                                 lambda event, axis_and_direction=NEGATIVE_PITCH:
                                                 self.robot_control_button_callback(
                                                     axis_and_direction=axis_and_direction))
        self.negative_pitch_movement_button.bind(BUTTON_RELEASE,
                                                 lambda event, axis_and_direction=NO_MOVEMENT:
                                                 self.robot_control_button_callback(
                                                     axis_and_direction=axis_and_direction))
        bb = create_widget_object(self.negative_pitch_movement_button, col_num=LEFT_COLUMN, row_num=bb)

        # Define the positive pitch button
        self.positive_pitch_movement_button = ttk.Button(robot_controls_frame, text=POSITIVE_PITCH, width=5,
                                                         state=DISABLED)
        self.positive_pitch_movement_button.bind(BUTTON_PRESS,
                                                 lambda event, axis_and_direction=POSITIVE_PITCH:
                                                 self.robot_control_button_callback(
                                                     axis_and_direction=axis_and_direction))
        self.positive_pitch_movement_button.bind(BUTTON_RELEASE,
                                                 lambda event, axis_and_direction=NO_MOVEMENT:
                                                 self.robot_control_button_callback(
                                                     axis_and_direction=axis_and_direction))
        bb = create_widget_object(self.positive_pitch_movement_button, col_num=RIGHT_COLUMN, row_num=bb,
                                  increment_row=True)

        bb = create_horizontal_separator(robot_controls_frame, bb)

        # Define the negative yaw widget
        self.negative_yaw_movement_button = ttk.Button(robot_controls_frame, text=NEGATIVE_YAW, width=5,
                                                       state=DISABLED)
        self.negative_yaw_movement_button.bind(BUTTON_PRESS,
                                               lambda event, axis_and_direction=NEGATIVE_YAW:
                                               self.robot_control_button_callback(
                                                   axis_and_direction=axis_and_direction))
        self.negative_yaw_movement_button.bind(BUTTON_RELEASE,
                                               lambda event, axis_and_direction=NO_MOVEMENT:
                                               self.robot_control_button_callback(
                                                   axis_and_direction=axis_and_direction))
        bb = create_widget_object(self.negative_yaw_movement_button, col_num=LEFT_COLUMN, row_num=bb)

        # Define the positive yaw widget
        self.positive_yaw_movement_button = ttk.Button(robot_controls_frame, text=POSITIVE_YAW, width=5,
                                                       state=DISABLED)
        self.positive_yaw_movement_button.bind(BUTTON_PRESS,
                                               lambda event, axis_and_direction=POSITIVE_YAW:
                                               self.robot_control_button_callback(
                                                   axis_and_direction=axis_and_direction))
        self.positive_yaw_movement_button.bind(BUTTON_RELEASE,
                                               lambda event, axis_and_direction=NO_MOVEMENT:
                                               self.robot_control_button_callback(
                                                   axis_and_direction=axis_and_direction))
        bb = create_widget_object(self.positive_yaw_movement_button, col_num=RIGHT_COLUMN, row_num=bb,
                                  increment_row=True)

        bb = create_horizontal_separator(robot_controls_frame, bb)

        # Define speed selector label
        bb = create_widget_object(ttk.Label(robot_controls_frame, text="Select the overall\nspeed of the robot.",
                                            anchor=CENTER, justify=CENTER),
                                  col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=bb, increment_row=True)

        # Define speed selector variable
        self.speed_selector_variable = IntVar(value=100)

        # Define speed options
        bb = create_widget_object(
            Radiobutton(robot_controls_frame, text="Slow", variable=self.speed_selector_variable,
                        value=50, command=self.speed_selector_callback),
            col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=bb, increment_row=True)
        bb = create_widget_object(
            Radiobutton(robot_controls_frame, text="Normal", variable=self.speed_selector_variable,
                        value=100, command=self.speed_selector_callback),
            col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=bb, increment_row=True)
        bb = create_widget_object(
            Radiobutton(robot_controls_frame, text="Fast", variable=self.speed_selector_variable,
                        value=125, command=self.speed_selector_callback),
            col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=bb, increment_row=True)

        # Update the status log
        self.ui_update_log('Robot controls added')

        # endregion

        # Define the widgets used in the exam_setup_frame
        # region

        # Define the row counter
        cc = 0

        # Define the force entry label
        cc = create_widget_object(ttk.Label(exam_setup_frame, text="Desired Force", anchor=CENTER, justify=CENTER),
                                  col_num=LEFT_COLUMN, row_num=cc)

        # Define the force set point entry widget
        validation_command = self.parent.register(self.entry_widget_float_validation)
        self.force_set_point_entry = ttk.Entry(exam_setup_frame, validate=ALL,
                                               validatecommand=(validation_command, '%P'),
                                               width=5, justify=CENTER)
        self.force_set_point_entry.insert(0, '0.0')
        cc = create_widget_object(self.force_set_point_entry, col_num=L_MIDDLE_COLUMN, row_num=cc, pady=20)

        # Define the newton label
        cc = create_widget_object(ttk.Label(exam_setup_frame, text="N"), col_num=MIDDLE_COLUMN, row_num=cc)

        # Define the force increase button
        cc = create_widget_object(ttk.Button(exam_setup_frame, text="+",
                                             command=lambda: self.force_set_point_change_incremental(
                                                 INCREMENTAL_FORCE_CHANGE)), col_num=R_MIDDLE_COLUMN, row_num=cc)

        # Define the force send button
        cc = create_widget_object(ttk.Button(exam_setup_frame, text="Send New\nSet-point",
                                             command=self.force_set_point_submit_callback),
                                  col_num=RIGHT_COLUMN, row_num=cc, row_span=DOUBLE_ROW, increment_row=True)

        # Define the current force label
        cc = create_widget_object(ttk.Label(exam_setup_frame, text="Current Force", anchor=CENTER, justify=CENTER),
                                  col_num=LEFT_COLUMN, row_num=cc, pady=20)

        # Define the current force updating label
        self.current_force_string_var = StringVar(exam_setup_frame, "0.0")
        self.current_force_label = ttk.Label(exam_setup_frame, textvariable=self.current_force_string_var,
                                             anchor=CENTER, justify=CENTER)
        cc = create_widget_object(self.current_force_label, col_num=L_MIDDLE_COLUMN, row_num=cc)

        # Define the newton label
        cc = create_widget_object(ttk.Label(exam_setup_frame, text="N"), col_num=MIDDLE_COLUMN, row_num=cc,
                                  ipadx=2, padx=0)

        # Define the force decrease button
        cc = create_widget_object(ttk.Button(exam_setup_frame, text="-",
                                             command=lambda: self.force_set_point_change_incremental(
                                                 -INCREMENTAL_FORCE_CHANGE)),
                                  col_num=R_MIDDLE_COLUMN, row_num=cc, increment_row=True)

        # Define a separator
        cc = create_horizontal_separator(exam_setup_frame, cc)

        # Define crop question label
        cc = create_widget_object(ttk.Label(exam_setup_frame, text="Crop the raw image?",
                                            anchor=CENTER, justify=CENTER),
                                  col_num=LEFT_COLUMN, row_num=cc, row_span=DOUBLE_ROW)

        # Define the crop selection variable
        self.select_image_crop_variable = IntVar(value=NO_BUTTON)

        # Define the crop selection yes option
        cc = create_widget_object(
            Radiobutton(exam_setup_frame, text="Yes", variable=self.select_image_crop_variable,
                        value=YES_BUTTON, command=self.select_image_crop_callback,
                        anchor=CENTER, justify=CENTER), col_num=L_MIDDLE_COLUMN, row_num=cc)

        # Define the crop selection no option
        cc = create_widget_object(Radiobutton(exam_setup_frame, text="No", variable=self.select_image_crop_variable,
                                              value=NO_BUTTON, command=self.select_image_crop_callback,
                                              anchor=CENTER, justify=CENTER), col_num=MIDDLE_COLUMN, row_num=cc)

        # Define generate new image cropping button
        self.generate_new_image_cropping_button = ttk.Button(exam_setup_frame,
                                                             text="Generate a New\nImage Cropping",
                                                             command=self.generate_new_image_cropping_button_callback,
                                                             state=DISABLED)
        cc = create_widget_object(self.generate_new_image_cropping_button, col_num=R_MIDDLE_COLUMN, row_num=cc)

        # Define the load existing image cropping button
        self.load_existing_image_cropping_button = ttk.Button(exam_setup_frame,
                                                              text="Load Existing\nImage Cropping",
                                                              command=self.load_existing_image_cropping_button_callback,
                                                              state=DISABLED)
        cc = create_widget_object(self.load_existing_image_cropping_button, col_num=RIGHT_COLUMN, row_num=cc,
                                  increment_row=True)

        # Define a separator
        cc = create_horizontal_separator(exam_setup_frame, cc)

        # Define the depth entry label
        cc = create_widget_object(ttk.Label(exam_setup_frame, text="Set the imaging depth\nof the US scanner:",
                                            anchor=CENTER, justify=CENTER), col_num=LEFT_COLUMN, row_num=cc)

        # Define the depth entry field
        self.imaging_depth_entry = ttk.Entry(exam_setup_frame, validate=ALL,
                                             validatecommand=(validation_command, '%P'), width=5, justify=CENTER)
        self.imaging_depth_entry.insert(0, '5.0')
        self.imaging_depth_submit_callback()
        cc = create_widget_object(self.imaging_depth_entry, col_num=L_MIDDLE_COLUMN, row_num=cc, pady=20)

        # Define the centimeter label
        cc = create_widget_object(ttk.Label(exam_setup_frame, text="cm"), col_num=MIDDLE_COLUMN, row_num=cc)

        # Define the send button
        cc = create_widget_object(
            ttk.Button(exam_setup_frame, text="Send", command=self.imaging_depth_submit_callback),
            col_num=R_MIDDLE_COLUMN, col_span=TWO_COLUMN, row_num=cc, increment_row=True)

        # Define a horizontal separator
        cc = create_horizontal_separator(exam_setup_frame, cc)

        self.identify_thyroid_from_points_button = ttk.Button(exam_setup_frame,
                                                              text="Identify Region of Interest from Points",
                                                              command=self.identify_thyroid_from_points_button_callback,
                                                              )
        cc = create_widget_object(self.identify_thyroid_from_points_button, col_num=LEFT_COLUMN,
                                  col_span=FULL_WIDTH,
                                  row_num=cc, increment_row=True)

        # Define a separator
        cc = create_horizontal_separator(exam_setup_frame, cc)

        # Define select thyroid side label
        cc = create_widget_object(
            ttk.Label(exam_setup_frame, text='Select where to center\n the Region of Interest'),
            col_num=LEFT_COLUMN, col_span=TWO_COLUMN, row_num=cc)

        # Define the centering side variable
        self.image_centering_side_variable = IntVar(value=MIDDLE_BUTTON)
        self.previous_image_centering_side_variable_value = self.image_centering_side_variable.get()

        # Define the image centering left option
        cc = create_widget_object(Radiobutton(exam_setup_frame, text="Left",
                                              variable=self.image_centering_side_variable,
                                              value=LEFT_BUTTON, anchor=CENTER, justify=CENTER),
                                  col_num=L_MIDDLE_COLUMN, row_num=cc)

        # Define the image centering right option
        cc = create_widget_object(Radiobutton(exam_setup_frame, text="Right",
                                              variable=self.image_centering_side_variable,
                                              value=RIGHT_BUTTON, anchor=CENTER, justify=CENTER),
                                  col_num=RIGHT_COLUMN, row_num=cc)

        # Define the image centering middle option
        cc = create_widget_object(Radiobutton(exam_setup_frame, text="Middle",
                                              variable=self.image_centering_side_variable,
                                              value=MIDDLE_BUTTON, anchor=CENTER, justify=CENTER),
                                  col_num=R_MIDDLE_COLUMN, row_num=cc, increment_row=True)

        cc = create_horizontal_separator(exam_setup_frame, cc)

        # Define the visualization selection label
        cc = create_widget_object(ttk.Label(exam_setup_frame, text="Select the images\n to display:"),
                                  col_num=LEFT_COLUMN, col_span=TWO_COLUMN, row_num=cc)

        # Define the variables to store the selections
        self.show_visualization_variables = {SHOW_ORIGINAL: IntVar(value=0),
                                             SHOW_CROPPED: IntVar(value=0),
                                             SHOW_FOREGROUND: IntVar(value=0)}

        # Define the widgets
        cc = create_widget_object(ttk.Checkbutton(exam_setup_frame, text='Original',
                                                  variable=self.show_visualization_variables[SHOW_ORIGINAL],
                                                  command=lambda visualization=SHOW_ORIGINAL:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=L_MIDDLE_COLUMN, row_num=cc)
        cc = create_widget_object(ttk.Checkbutton(exam_setup_frame, text='Cropped',
                                                  variable=self.show_visualization_variables[SHOW_CROPPED],
                                                  command=lambda visualization=SHOW_CROPPED:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=R_MIDDLE_COLUMN, row_num=cc)
        cc = create_widget_object(ttk.Checkbutton(exam_setup_frame, text='Region of\nInterest',
                                                  variable=self.show_visualization_variables[SHOW_FOREGROUND],
                                                  command=lambda visualization=SHOW_FOREGROUND:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=RIGHT_COLUMN, row_num=cc, increment_row=True)

        # Update the status log
        self.ui_update_log('Exam setup page added')

        # endregion

        # Define the widgets used to populate the thyroid exam frame
        # region

        # Define the row counter
        dd = 0

        # Define the registered data save location button
        self.registered_data_save_location_button = \
            ttk.Button(thyroid_exam_frame, text='Select Location to Save Exam Data',
                       command=self.registered_data_save_location_button_callback)
        dd = create_widget_object(self.registered_data_save_location_button, col_num=LEFT_COLUMN,
                                  col_span=FULL_WIDTH,
                                  row_num=dd, increment_row=True)

        # Define a separator
        dd = create_horizontal_separator(thyroid_exam_frame, dd)

        # Define the selected location label
        dd = create_widget_object(ttk.Label(thyroid_exam_frame, text="Selected Location:"), col_num=LEFT_COLUMN,
                                  col_span=TWO_COLUMN, row_num=dd)

        # Define the current location variable label
        self.registered_data_save_location_str_var = StringVar(thyroid_exam_frame, '')
        dd = create_widget_object(
            ttk.Label(thyroid_exam_frame, textvariable=self.registered_data_save_location_str_var),
            col_num=L_MIDDLE_COLUMN, col_span=FULL_WIDTH - TWO_COLUMN,
            row_num=dd, increment_row=True)

        # Define a separator
        dd = create_horizontal_separator(thyroid_exam_frame, dd)

        # Define the image spacing selector
        self.image_spacing_variable = IntVar(value=10)
        dd = create_widget_object(ttk.Label(thyroid_exam_frame, text="Image Spacing:"), col_num=LEFT_COLUMN,
                                  col_span=TWO_COLUMN, row_num=dd)
        dd = create_widget_object(Radiobutton(thyroid_exam_frame, text="2.0mm",
                                              variable=self.image_spacing_variable,
                                              value=20, anchor=CENTER, justify=CENTER,
                                              command=self.image_spacing_selection_callback),
                                  col_num=L_MIDDLE_COLUMN, row_num=dd)
        dd = create_widget_object(Radiobutton(thyroid_exam_frame, text="1.0mm",
                                              variable=self.image_spacing_variable,
                                              value=10, anchor=CENTER, justify=CENTER,
                                              command=self.image_spacing_selection_callback),
                                  col_num=LR_MIDDLE_COLUMN, col_span=TWO_COLUMN, row_num=dd)
        dd = create_widget_object(Radiobutton(thyroid_exam_frame, text="0.5mm",
                                              variable=self.image_spacing_variable,
                                              value=5, anchor=CENTER, justify=CENTER,
                                              command=self.image_spacing_selection_callback),
                                  col_num=R_MIDDLE_COLUMN, row_num=dd, increment_row=True)

        # Define the scan label
        dd = create_widget_object(
            ttk.Label(thyroid_exam_frame, text="Scanning Distance"),
            col_num=LEFT_COLUMN, col_span=TWO_COLUMN, row_num=dd)

        # Define the scan distance entry field
        self.scan_distance_entry = Entry(thyroid_exam_frame, validate=ALL,
                                         validatecommand=(validation_command, '%P'),
                                         justify=CENTER, width=5)
        self.scan_distance_entry.insert(0, '6.0')
        dd = create_widget_object(self.scan_distance_entry, col_num=L_MIDDLE_COLUMN, row_num=dd)

        # Define the centimeter label
        dd = create_widget_object(ttk.Label(thyroid_exam_frame, text="cm"),
                                  col_num=MIDDLE_COLUMN, row_num=dd, ipadx=2, padx=0)

        # Define the scan positive button
        self.scan_positive_button = ttk.Button(thyroid_exam_frame, text="Scan Positive",
                                               command=self.scan_positive_button_callback,
                                               state=DISABLED)
        dd = create_widget_object(self.scan_positive_button, col_num=R_MIDDLE_COLUMN, row_num=dd)

        # Define the scan negative button
        self.scan_negative_button = ttk.Button(thyroid_exam_frame, text="Scan Negative",
                                               command=self.scan_negative_button_callback,
                                               state=DISABLED)
        dd = create_widget_object(self.scan_negative_button, col_num=RIGHT_COLUMN, row_num=dd, increment_row=True)

        # Create a horizontal separator
        dd = create_horizontal_separator(thyroid_exam_frame, dd)

        # Create a horizontal separator
        dd = create_horizontal_separator(thyroid_exam_frame, dd)

        # Define the registered data load button
        self.registered_data_load_location_button = \
            ttk.Button(thyroid_exam_frame,
                       text='Select Exam Data from which to Generate Volume',
                       command=self.registered_data_load_location_button_callback)
        dd = create_widget_object(self.registered_data_load_location_button,
                                  col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=dd, increment_row=True)

        # Define the selected location label
        dd = create_widget_object(ttk.Label(thyroid_exam_frame, text='Selected Location:'), col_num=LEFT_COLUMN,
                                  col_span=TWO_COLUMN, row_num=dd)

        # Define the registered data load location variable
        self.registered_data_load_location_str_var = StringVar(thyroid_exam_frame, '')
        dd = create_widget_object(ttk.Label(thyroid_exam_frame,
                                            textvariable=self.registered_data_load_location_str_var),
                                  col_num=L_MIDDLE_COLUMN, col_span=FULL_WIDTH - TWO_COLUMN,
                                  row_num=dd, increment_row=True)

        # Define the volume data save location button
        self.volume_data_save_location_button = ttk.Button(thyroid_exam_frame,
                                                           text='Select Location to Save Generated Volume Data',
                                                           command=self.volume_data_save_location_button_callback)
        dd = create_widget_object(self.volume_data_save_location_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=dd, increment_row=True)

        # Define the selected location label
        dd = create_widget_object(ttk.Label(thyroid_exam_frame, text="Selected Location:"), col_num=LEFT_COLUMN,
                                  col_span=TWO_COLUMN, row_num=dd)

        # Define volume data save location label
        self.volume_data_save_location_str_var = StringVar(thyroid_exam_frame, '')
        dd = create_widget_object(
            ttk.Label(thyroid_exam_frame, textvariable=self.volume_data_save_location_str_var),
            col_num=L_MIDDLE_COLUMN, col_span=FULL_WIDTH - TWO_COLUMN,
            row_num=dd, increment_row=True)

        # Define generate volume button
        self.generate_new_volume_button = ttk.Button(thyroid_exam_frame, text='Generate New Volume',
                                                     command=self.generate_volume_button_callback,
                                                     state=DISABLED)
        dd = create_widget_object(self.generate_new_volume_button, col_num=LEFT_COLUMN,
                                  col_span=FULL_WIDTH, row_num=dd, increment_row=True)

        # Create a horizontal separator
        dd = create_horizontal_separator(thyroid_exam_frame, dd)

        # Create a horizontal separator
        dd = create_horizontal_separator(thyroid_exam_frame, dd)

        # Define the volume data loading button
        self.volume_data_load_location_button = ttk.Button(thyroid_exam_frame,
                                                           text='Select Volume Data to Load',
                                                           command=self.volume_data_load_location_button_callback)
        dd = create_widget_object(self.volume_data_load_location_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=dd, increment_row=True)

        # Define the selected location label
        dd = create_widget_object(ttk.Label(thyroid_exam_frame, text="Selected Location:"), col_num=LEFT_COLUMN,
                                  col_span=TWO_COLUMN, row_num=dd)

        # Define volume data load location label
        self.volume_data_load_location_str_var = StringVar(thyroid_exam_frame, '')
        dd = create_widget_object(
            ttk.Label(thyroid_exam_frame, textvariable=self.volume_data_load_location_str_var),
            col_num=L_MIDDLE_COLUMN, col_span=FULL_WIDTH - TWO_COLUMN,
            row_num=dd, increment_row=True)

        # Define display volume button
        self.display_loaded_volume_button = ttk.Button(thyroid_exam_frame, text='Display Loaded Volume',
                                                       command=self.display_loaded_volume_button_callback,
                                                       state=DISABLED)
        dd = create_widget_object(self.display_loaded_volume_button, col_num=LEFT_COLUMN,
                                  col_span=FULL_WIDTH, row_num=dd, increment_row=True)

        dd = create_horizontal_separator(thyroid_exam_frame, dd)
        dd = create_horizontal_separator(thyroid_exam_frame, dd)

        # Create calculated volume label
        dd = create_widget_object(ttk.Label(thyroid_exam_frame, text='Calculated Volume:'),
                                  col_num=LEFT_COLUMN, row_num=dd)

        # Create label for displaying the volume
        self.volume_data_result_str_var = StringVar()
        self.volume_data_result_str_var.set('0.00')
        dd = create_widget_object(ttk.Label(thyroid_exam_frame, textvariable=self.volume_data_result_str_var),
                                  col_num=LL_MIDDLE_COLUMN, row_num=dd)

        # Create the unit label
        dd = create_widget_object(ttk.Label(thyroid_exam_frame, text='ml'),
                                  col_num=L_MIDDLE_COLUMN, row_num=dd)

        # Update the status log
        self.ui_update_log('Thyroid exam page added')

        # endregion

        # Define the widgets used to populate the developer window
        # region

        # Define the row counter
        ff = 0

        # Define publish controller statuses button
        self.publish_controller_statuses_button = \
            ttk.Button(developer_frame,
                       text=START_PUBLISHING_CONTROLLER_STATUS_VALUES,
                       command=self.publish_controller_statuses_button_callback)
        ff = create_widget_object(self.publish_controller_statuses_button,
                                  col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=ff, increment_row=True)

        # Define the controller selector label
        ff = create_widget_object(ttk.Label(developer_frame, text="Select\nController"),
                                  col_num=LEFT_COLUMN, row_num=ff, row_span=DOUBLE_ROW)

        # Define the controller selector variable
        self.pid_selector = IntVar()

        # Define the pid controller radio buttons
        ff = create_widget_object(Radiobutton(developer_frame, text="x-lin-trj", variable=self.pid_selector,
                                              value=0, command=self.pid_controller_selection_callback),
                                  col_num=L_MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(Radiobutton(developer_frame, text="y-lin-img", variable=self.pid_selector,
                                              value=1, command=self.pid_controller_selection_callback),
                                  col_num=MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(Radiobutton(developer_frame, text="z-lin-force", variable=self.pid_selector,
                                              value=2, command=self.pid_controller_selection_callback),
                                  col_num=R_MIDDLE_COLUMN, row_num=ff, increment_row=True)
        ff = create_widget_object(Radiobutton(developer_frame, text="x-ang-img", variable=self.pid_selector,
                                              value=3, command=self.pid_controller_selection_callback),
                                  col_num=L_MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(Radiobutton(developer_frame, text="y-ang-pos", variable=self.pid_selector,
                                              value=4, command=self.pid_controller_selection_callback),
                                  col_num=MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(Radiobutton(developer_frame, text="z-ang-pos", variable=self.pid_selector,
                                              value=5, command=self.pid_controller_selection_callback),
                                  col_num=R_MIDDLE_COLUMN, row_num=ff, increment_row=True)

        # Define a separator
        ff = create_horizontal_separator(developer_frame, ff)

        # Define p, i, d labels
        ff = create_widget_object(ttk.Label(developer_frame, text="P", anchor=CENTER, justify=CENTER),
                                  col_num=L_MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Label(developer_frame, text="I", anchor=CENTER, justify=CENTER),
                                  col_num=MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Label(developer_frame, text="D", anchor=CENTER, justify=CENTER),
                                  col_num=R_MIDDLE_COLUMN, row_num=ff, increment_row=True)

        # Define current controller gain label
        ff = create_widget_object(ttk.Label(developer_frame, text="Current Values:"),
                                  col_num=LEFT_COLUMN, row_num=ff)

        # Define current controller gain variables
        self.p_gain_var = StringVar(developer_frame, "0.000")
        self.i_gain_var = StringVar(developer_frame, "0.000")
        self.d_gain_var = StringVar(developer_frame, "0.000")
        ff = create_widget_object(ttk.Label(developer_frame, textvariable=self.p_gain_var,
                                            anchor=CENTER, justify=CENTER),
                                  col_num=L_MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Label(developer_frame, textvariable=self.i_gain_var,
                                            anchor=CENTER, justify=CENTER),
                                  col_num=MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Label(developer_frame, textvariable=self.d_gain_var,
                                            anchor=CENTER, justify=CENTER),
                                  col_num=R_MIDDLE_COLUMN, row_num=ff, increment_row=True)

        # Define a separator
        ff = create_horizontal_separator(developer_frame, ff)

        # Define set to label
        ff = create_widget_object(ttk.Label(developer_frame, text="Set to:"), col_num=LEFT_COLUMN, row_num=ff)

        # Define p gain entry field
        self.p_gain_entry = ttk.Entry(developer_frame, validate=ALL, validatecommand=(validation_command, '%P'),
                                      justify=CENTER, width=5)
        self.p_gain_entry.insert(0, "0.000")
        ff = create_widget_object(self.p_gain_entry, col_num=L_MIDDLE_COLUMN, row_num=ff)

        # Define i gain entry field
        self.i_gain_entry = ttk.Entry(developer_frame, validate=ALL, validatecommand=(validation_command, '%P'),
                                      justify=CENTER, width=5)
        self.i_gain_entry.insert(0, "0.000")
        ff = create_widget_object(self.i_gain_entry, col_num=MIDDLE_COLUMN, row_num=ff)

        # Define d gain entry field
        self.d_gain_entry = ttk.Entry(developer_frame, validate=ALL, validatecommand=(validation_command, '%P'),
                                      justify=CENTER, width=5)
        self.d_gain_entry.insert(0, "0.000")
        ff = create_widget_object(self.d_gain_entry, col_num=R_MIDDLE_COLUMN, row_num=ff)

        # Define set values button
        ff = create_widget_object(
            ttk.Button(developer_frame, text="Set Values", command=self.pid_value_setting_callback),
            col_num=RIGHT_COLUMN, row_num=ff, increment_row=True)

        # Define a separator
        ff = create_horizontal_separator(developer_frame, ff)

        # Define a separator
        ff = create_horizontal_separator(developer_frame, ff)

        # Define select image destination button
        self.select_image_destination_directory = ttk.Button(developer_frame,
                                                             text='Select Location of Saved Images',
                                                             command=self.send_save_images_destination)
        ff = create_widget_object(self.select_image_destination_directory, col_num=LEFT_COLUMN,
                                  col_span=FOUR_COLUMN + 1, row_num=ff)

        # Define save images button
        self.save_images_button = ttk.Button(developer_frame, text=START_SAVING_IMAGES,
                                             command=self.send_image_saving_command_callback)
        ff = create_widget_object(self.save_images_button, col_num=RL_MIDDLE_COLUMN, col_span=FOUR_COLUMN,
                                  row_num=ff,
                                  increment_row=True)

        ff = create_horizontal_separator(developer_frame, ff)

        # Define the image selection label
        ff = create_widget_object(ttk.Label(developer_frame, text="Select the images\n to display:"),
                                  col_num=LEFT_COLUMN, col_span=TWO_COLUMN, row_num=ff)

        # Define the variables to store the selections
        self.show_visualization_variables.update({
            SHOW_RECOLOR: IntVar(value=0),
            SHOW_BLUR: IntVar(value=0),
            SHOW_CENTROIDS_ONLY: IntVar(value=0),
            SHOW_CENTROIDS_CROSS_ONLY: IntVar(value=0),
            SHOW_MASK_CENTROIDS_CROSS_OVERLAY: IntVar(value=0),
            SHOW_SKIN_APPROXIMATION: IntVar(value=0)
        })

        # Define the widgets
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Recolored',
                                                  variable=self.show_visualization_variables[SHOW_RECOLOR],
                                                  command=lambda visualization=SHOW_RECOLOR:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=L_MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Pre-processed',
                                                  variable=self.show_visualization_variables[SHOW_BLUR],
                                                  command=lambda visualization=SHOW_BLUR:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Centroids',
                                                  variable=self.show_visualization_variables[SHOW_CENTROIDS_ONLY],
                                                  command=lambda visualization=SHOW_CENTROIDS_ONLY:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=R_MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Centroids\n & Cross',
                                                  variable=self.show_visualization_variables[
                                                      SHOW_CENTROIDS_CROSS_ONLY],
                                                  command=lambda visualization=SHOW_CENTROIDS_CROSS_ONLY:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=RIGHT_COLUMN, row_num=ff, increment_row=True)
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Mask, Centroids,\n & Cross',
                                                  variable=self.show_visualization_variables[
                                                      SHOW_MASK_CENTROIDS_CROSS_OVERLAY],
                                                  command=lambda visualization=SHOW_MASK_CENTROIDS_CROSS_OVERLAY:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=L_MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Balance Line',
                                                  variable=self.show_visualization_variables[
                                                      SHOW_SKIN_APPROXIMATION],
                                                  command=lambda visualization=SHOW_SKIN_APPROXIMATION:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=MIDDLE_COLUMN, row_num=ff, increment_row=True)

        ff = create_horizontal_separator(developer_frame, ff)

        # Define the mask selection label
        ff = create_widget_object(ttk.Label(developer_frame, text="Select the masks\n to display:"),
                                  col_num=LEFT_COLUMN, col_span=TWO_COLUMN, row_num=ff)

        # Define the selection variables
        self.show_result_mask_variable = IntVar(value=0)
        self.show_post_processed_mask_variable = IntVar(value=0)
        self.show_sure_foreground_mask_variable = IntVar(value=0)
        self.show_sure_background_mask_variable = IntVar(value=0)
        self.show_probable_foreground_mask_variable = IntVar(value=0)
        self.show_initialization_mask_variable = IntVar(value=0)
        self.show_grabcut_user_initialization_0_mask_variable = IntVar(value=0)

        # Define the widgets
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Result',
                                                  variable=self.show_result_mask_variable,
                                                  command=lambda visualization=SHOW_MASK:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=L_MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Post-processed',
                                                  variable=self.show_post_processed_mask_variable,
                                                  command=lambda visualization=SHOW_POST_PROCESSED_MASK:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Sure-\nforeground',
                                                  variable=self.show_sure_foreground_mask_variable,
                                                  command=lambda visualization=SHOW_SURE_FOREGROUND:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=R_MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Sure-\nbackground',
                                                  variable=self.show_sure_background_mask_variable,
                                                  command=lambda visualization=SHOW_SURE_BACKGROUND:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=RIGHT_COLUMN, row_num=ff, increment_row=True)
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Probable-\nforeground',
                                                  variable=self.show_probable_foreground_mask_variable,
                                                  command=lambda visualization=SHOW_PROBABLE_FOREGROUND:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=L_MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='Initialization',
                                                  variable=self.show_initialization_mask_variable,
                                                  command=lambda visualization=SHOW_INITIALIZED_MASK:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=MIDDLE_COLUMN, row_num=ff)
        ff = create_widget_object(ttk.Checkbutton(developer_frame, text='User Initialization',
                                                  variable=self.show_grabcut_user_initialization_0_mask_variable,
                                                  command=lambda visualization=SHOW_GRABCUT_USER_INITIALIZATION_0:
                                                  self.visualization_check_button_callback(
                                                      visualization=visualization)),
                                  col_num=R_MIDDLE_COLUMN, row_num=ff, increment_row=True)

        self.segmentation_phase_button = ttk.Button(developer_frame, text=SET_TO_REST_PHASE,
                                                    command=self.segmentation_phase_button_callback)
        ff = create_widget_object(self.segmentation_phase_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=ff, increment_row=True)

        # Update the status log
        self.ui_update_log('Developer page added')

        # endregion

        # Define the widgets used to populate the experimentation window
        # region

        # Define the row counter
        gg = 0

        # Define the image data stream location button
        self.image_data_stream_location_button = \
            ttk.Button(experimentation_frame,
                       text='Select the Saved Data to Stream',
                       command=self.image_data_stream_location_button_callback)
        gg = create_widget_object(self.image_data_stream_location_button,
                                  col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=gg, increment_row=True)

        # Define the selected location label
        gg = create_widget_object(ttk.Label(experimentation_frame, text='Selected Location:'), col_num=LEFT_COLUMN,
                                  col_span=TWO_COLUMN, row_num=gg)

        # Define the image data stream location variable
        self.image_data_stream_location_str_var = StringVar(experimentation_frame,
                                                            '/home/ben/thyroid_ultrasound_data/'
                                                            'testing_and_validation/raw_images')
        gg = create_widget_object(ttk.Label(experimentation_frame,
                                            textvariable=self.image_data_stream_location_str_var),
                                  col_num=L_MIDDLE_COLUMN, col_span=FULL_WIDTH - TWO_COLUMN,
                                  row_num=gg, increment_row=True)

        # Define the stream in reverse order button
        self.reverse_stream_order_button = ttk.Button(experimentation_frame,
                                                      text=PLAYBACK_STREAM_IN_REVERSE_CHRONOLOGICAL_ORDER,
                                                      command=self.reverse_stream_order_button_callback,
                                                      state=DISABLED)
        gg = create_widget_object(self.reverse_stream_order_button,
                                  col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=gg, increment_row=True)

        # Define the publishing rate entry field
        gg = create_widget_object(ttk.Label(experimentation_frame,
                                            text='Publishing rate:'),
                                  col_num=LEFT_COLUMN, row_num=gg)
        self.stream_data_rate_entry = ttk.Entry(experimentation_frame, validate=ALL,
                                                validatecommand=(validation_command, '%P'),
                                                justify=CENTER, width=5)
        self.stream_data_rate_entry.insert(0, "20.0")
        gg = create_widget_object(self.stream_data_rate_entry, col_num=L_MIDDLE_COLUMN, row_num=gg)
        gg = create_widget_object(ttk.Label(experimentation_frame, text='Hz'), col_num=LR_MIDDLE_COLUMN, row_num=gg)
        gg = create_widget_object(ttk.Button(experimentation_frame, text='Send',
                                             command=self.set_new_streaming_frequency_callback),
                                  col_num=MIDDLE_COLUMN, row_num=gg, increment_row=True)

        gg = create_horizontal_separator(experimentation_frame, gg)

        # Define the request image registration button
        gg = create_widget_object(ttk.Button(experimentation_frame,
                                             text="Request Data to be Registered",
                                             command=self.register_new_data_callback),
                                  col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=gg, increment_row=True)

        gg = create_horizontal_separator(experimentation_frame, gg)

        # Define the patient contact label
        gg = create_widget_object(ttk.Label(experimentation_frame, text="Select the values to override:",
                                            anchor=CENTER, justify=CENTER),
                                  col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=gg, increment_row=True)

        # Define the send patient contact override button
        self.send_patient_contact_override_button = \
            ttk.Button(experimentation_frame,
                       text=START_SENDING_PATIENT_CONTACT_OVERRIDE_VALUE,
                       command=self.send_patient_contact_override_button_callback)
        gg = create_widget_object(self.send_patient_contact_override_button, col_num=LEFT_COLUMN,
                                  col_span=FULL_WIDTH, row_num=gg, increment_row=True)

        # Define the force control override button
        self.send_force_control_override_button = \
            ttk.Button(experimentation_frame,
                       text=START_SENDING_FORCE_CONTROL_OVERRIDE_VALUE,
                       command=self.send_force_control_override_button_callback)
        gg = create_widget_object(self.send_force_control_override_button, col_num=LEFT_COLUMN,
                                  col_span=FULL_WIDTH, row_num=gg, increment_row=True)

        # Define image balanced override button
        # self.is_image_balanced_override_active = False
        self.send_image_balanced_override_button = \
            ttk.Button(experimentation_frame,
                       text=START_SENDING_IMAGE_BALANCED_OVERRIDE_VALUE,
                       command=self.send_image_balanced_override_button_callback)
        gg = create_widget_object(self.send_image_balanced_override_button, col_num=LEFT_COLUMN,
                                  col_span=FULL_WIDTH,
                                  row_num=gg, increment_row=True)

        # Define image centered override button
        # self.is_image_centered_override_active = False
        self.send_image_centered_override_button = \
            ttk.Button(experimentation_frame,
                       text=START_SENDING_IMAGE_CENTERED_OVERRIDE_VALUE,
                       command=self.send_image_centered_override_button_callback)
        gg = create_widget_object(self.send_image_centered_override_button, col_num=LEFT_COLUMN,
                                  col_span=FULL_WIDTH,
                                  row_num=gg, increment_row=True)

        # Define registered data button
        # self.is_registered_data_override_active = False
        self.send_registered_data_override_button = \
            ttk.Button(experimentation_frame,
                       text=START_SENDING_REGISTERED_DATA_OVERRIDE_VALUE,
                       command=self.send_registered_data_override_button_callback)
        gg = create_widget_object(self.send_registered_data_override_button, col_num=LEFT_COLUMN,
                                  col_span=FULL_WIDTH,
                                  row_num=gg, increment_row=True)

        # Define not in contact option
        # self.not_in_contact_radio_button = ttk.Radiobutton(experimentation_frame, text="Not In-contact",
        #                                                    value=NO_BUTTON,
        #                                                    variable=self.select_patient_contact_override_variable)
        # gg = create_widget_object(self.not_in_contact_radio_button, col_num=MIDDLE_COLUMN, col_span=TWO_COLUMN,
        #                           row_num=gg, sticky='', increment_row=True)

        # Define a separator
        # gg = create_horizontal_separator(experimentation_frame, gg)

        # Define a separator
        gg = create_horizontal_separator(experimentation_frame, gg)

        # Define select data label
        gg = create_widget_object(ttk.Label(experimentation_frame,
                                            text="Select the data\nto save for\nthis experiment", anchor=CENTER),
                                  col_num=LEFT_COLUMN, col_span=TWO_COLUMN, row_num=gg, row_span=THREE_COLUMN)

        # Define data option labels
        gg = create_widget_object(ttk.Label(experimentation_frame, text="Save\nrobot pose",
                                            anchor=CENTER, justify=CENTER), col_num=L_MIDDLE_COLUMN, row_num=gg)
        gg = create_widget_object(ttk.Label(experimentation_frame, text="Save\nrobot force",
                                            anchor=CENTER, justify=CENTER), col_num=LR_MIDDLE_COLUMN, row_num=gg)
        gg = create_widget_object(ttk.Label(experimentation_frame, text="Save\nraw images",
                                            anchor=CENTER, justify=CENTER), col_num=MIDDLE_COLUMN, row_num=gg)
        gg = create_widget_object(ttk.Label(experimentation_frame, text="Save\nimage data",
                                            anchor=CENTER, justify=CENTER), col_num=RL_MIDDLE_COLUMN, row_num=gg)
        gg = create_widget_object(ttk.Label(experimentation_frame, text="Save image\ncentroid",
                                            anchor=CENTER, justify=CENTER), col_num=R_MIDDLE_COLUMN, row_num=gg)
        gg = create_widget_object(ttk.Label(experimentation_frame, text="Save skin\nerror",
                                            anchor=CENTER, justify=CENTER), col_num=RR_MIDDLE_COLUMN,
                                  row_num=gg, increment_row=True)

        # Define data option variables
        self.save_robot_pose_variable = IntVar()
        self.save_robot_force_variable = IntVar()
        self.save_raw_image_variable = IntVar()
        self.save_image_data_objects_variable = IntVar()
        self.save_image_centroid_variable = IntVar()
        self.save_skin_error_variable = IntVar()

        # Define data yes options
        self.save_robot_pose_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes", value=YES_BUTTON,
                                                          variable=self.save_robot_pose_variable)
        gg = create_widget_object(self.save_robot_pose_yes_button, col_num=L_MIDDLE_COLUMN, row_num=gg, sticky="")

        self.save_robot_force_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes", value=YES_BUTTON,
                                                           variable=self.save_robot_force_variable)
        gg = create_widget_object(self.save_robot_force_yes_button, col_num=LR_MIDDLE_COLUMN, row_num=gg, sticky="")

        self.save_raw_image_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes", value=YES_BUTTON,
                                                         variable=self.save_raw_image_variable)
        gg = create_widget_object(self.save_raw_image_yes_button, col_num=MIDDLE_COLUMN, row_num=gg, sticky="")

        self.save_image_data_objects_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes",
                                                                  value=YES_BUTTON,
                                                                  variable=self.save_image_data_objects_variable)
        gg = create_widget_object(self.save_image_data_objects_yes_button, col_num=RL_MIDDLE_COLUMN, row_num=gg,
                                  sticky="")

        self.save_image_centroid_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes", value=YES_BUTTON,
                                                              variable=self.save_image_centroid_variable)
        gg = create_widget_object(self.save_image_centroid_yes_button, col_num=R_MIDDLE_COLUMN, row_num=gg,
                                  sticky="")

        self.save_skin_error_yes_button = ttk.Radiobutton(experimentation_frame, text="Yes", value=YES_BUTTON,
                                                          variable=self.save_skin_error_variable)
        gg = create_widget_object(self.save_skin_error_yes_button, col_num=RR_MIDDLE_COLUMN, row_num=gg, sticky="",
                                  increment_row=True)

        # Define data no options
        self.save_robot_pose_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                         variable=self.save_robot_pose_variable)
        gg = create_widget_object(self.save_robot_pose_no_button, col_num=L_MIDDLE_COLUMN, row_num=gg, sticky="")

        self.save_robot_force_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                          variable=self.save_robot_force_variable)
        gg = create_widget_object(self.save_robot_force_no_button, col_num=LR_MIDDLE_COLUMN, row_num=gg, sticky="")

        self.save_raw_image_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                        variable=self.save_raw_image_variable)
        gg = create_widget_object(self.save_raw_image_no_button, col_num=MIDDLE_COLUMN, row_num=gg, sticky="")

        self.save_image_data_objects_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                                 variable=self.save_image_data_objects_variable)
        gg = create_widget_object(self.save_image_data_objects_no_button, col_num=RL_MIDDLE_COLUMN, row_num=gg,
                                  sticky="")

        self.save_image_centroid_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                             variable=self.save_image_centroid_variable)
        gg = create_widget_object(self.save_image_centroid_no_button, col_num=R_MIDDLE_COLUMN, row_num=gg,
                                  sticky="")

        self.save_skin_error_no_button = ttk.Radiobutton(experimentation_frame, text="No", value=NO_BUTTON,
                                                         variable=self.save_skin_error_variable)
        gg = create_widget_object(self.save_skin_error_no_button, col_num=RR_MIDDLE_COLUMN, row_num=gg, sticky="",
                                  increment_row=True)

        # Define saving data button
        self.is_experiment_data_saving_active = False
        self.save_experiment_data_button = ttk.Button(experimentation_frame,
                                                      text=START_SAVING_EXPERIMENT_DATA,
                                                      command=self.save_experiment_data_button_callback)
        gg = create_widget_object(self.save_experiment_data_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                  row_num=gg, increment_row=True)

        # Define a status label
        self.all_data_saved_text_variable = StringVar()
        gg = create_widget_object(ttk.Label(experimentation_frame, textvariable=self.all_data_saved_text_variable),
                                  col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=gg, increment_row=True)

        # Define a separator
        gg = create_horizontal_separator(experimentation_frame, gg)

        # Define noise generator button
        self.create_noise_button = ttk.Button(experimentation_frame, text=START_CREATING_VELOCITY_NOISE,
                                              command=self.create_noise_button_callback)
        gg = create_widget_object(self.create_noise_button, col_num=LEFT_COLUMN, col_span=FULL_WIDTH, row_num=gg,
                                  increment_row=True)

        # Update the status log
        self.ui_update_log('Experimentation page added')

        # endregion

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

    def manual_controls_status_button_callback(self):
        if self.manual_controls_status_button[WIDGET_TEXT] == ACTIVATE_MANUAL_CONTROLS:
            new_state = NORMAL
            new_text = DEACTIVATE_MANUAL_CONTROLS
        elif self.manual_controls_status_button[WIDGET_TEXT] == DEACTIVATE_MANUAL_CONTROLS:
            new_state = DISABLED
            new_text = ACTIVATE_MANUAL_CONTROLS
        else:
            raise ValueError('Widget text was not recognized.')
        widgets_in_robot_controls = [self.positive_x_movement_button, self.negative_x_movement_button,
                                     self.positive_y_movement_button, self.negative_y_movement_button,
                                     self.positive_pitch_movement_button, self.negative_pitch_movement_button,
                                     self.positive_yaw_movement_button, self.negative_yaw_movement_button]
        for widget in widgets_in_robot_controls:
            widget[WIDGET_STATE] = new_state
        self.manual_controls_status_button[WIDGET_TEXT] = new_text

    def robot_control_button_callback(self, axis_and_direction: str):

        # Define a standard message values
        standard_msg_value = 1.0

        # Create the common message
        self.manual_control_velocity_message = Twist()

        if axis_and_direction == NO_MOVEMENT:
            self.manual_control_velocity_message = None
        elif axis_and_direction == POSITIVE_X:
            self.manual_control_velocity_message.linear.x = standard_msg_value
        elif axis_and_direction == NEGATIVE_X:
            self.manual_control_velocity_message.linear.x = -standard_msg_value
        elif axis_and_direction == POSITIVE_Y:
            self.manual_control_velocity_message.linear.y = standard_msg_value
        elif axis_and_direction == NEGATIVE_Y:
            self.manual_control_velocity_message.linear.y = -standard_msg_value
        elif axis_and_direction == POSITIVE_PITCH:
            self.manual_control_velocity_message.angular.y = standard_msg_value
        elif axis_and_direction == NEGATIVE_PITCH:
            self.manual_control_velocity_message.angular.y = -standard_msg_value
        elif axis_and_direction == POSITIVE_YAW:
            self.manual_control_velocity_message.angular.z = standard_msg_value
        elif axis_and_direction == NEGATIVE_YAW:
            self.manual_control_velocity_message.angular.z = -standard_msg_value
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

    def generate_new_image_cropping_button_callback(self) -> None:
        """
        Publishes the command to generate new image crop coordinates.
        :return: None
        """
        self.generate_new_image_cropping_command_service(True)

    def load_existing_image_cropping_button_callback(self) -> None:
        """
        Publishes the command to load an existing image cropping.
        """
        self.load_existing_image_cropping_command_service(True)

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
        self.identify_thyroid_from_points_command_service(True)

    def image_spacing_selection_callback(self):
        self.image_spacing_selection_service(self.image_spacing_variable.get() / 10000)

    def scan_positive_button_callback(self) -> None:
        """
        Publish the command to scan upwards.
        """
        # self.scan_command_publisher.publish(
        #     Float64(float(self.scan_distance_entry.get()) / 100)  # convert cm to m
        # )
        self.create_trajectory_service(float(self.scan_distance_entry.get()) / 100)

        # Update the pose control button
        self.update_pose_control_button()

        # Update the data saving button
        self.save_experiment_data_button_callback(action=START_SAVING_EXPERIMENT_DATA)

    def scan_negative_button_callback(self) -> None:
        """
        Publish the command to scan downwards.
        """
        # self.scan_command_publisher.publish(
        #     Float64(-float(self.scan_distance_entry.get()) / 100)  # convert cm to m
        # )
        self.create_trajectory_service(-float(self.scan_distance_entry.get()) / 100)

        # Update the pose control button
        self.update_pose_control_button()

        # Update the data saving button
        self.save_experiment_data_button_callback(action=START_SAVING_EXPERIMENT_DATA)

    def registered_data_save_location_button_callback(self) -> None:
        """
        Select the directory in which to save the registered data, publishes it, and displays it.
        """
        if len(self.registered_data_save_location_str_var.get()) < 3:
            directory_to_look_in = '/home/ben/thyroid_ultrasound_data/testing_and_validation/registered_data'
        else:
            directory_to_look_in = self.registered_data_save_location_str_var.get()
        selected_directory = askdirectory(initialdir=directory_to_look_in,
                                          title="Select the destination for the exam data.")
        if len(selected_directory) > 3 and isdir(selected_directory):
            self.registered_data_save_location_service(selected_directory)
            self.registered_data_save_location_str_var.set(selected_directory)
            self.scan_positive_button[WIDGET_STATE] = NORMAL
            self.scan_negative_button[WIDGET_STATE] = NORMAL

    def registered_data_load_location_button_callback(self) -> None:
        """
        Select the directory from which to load the data used to generate the volume. Then publish it and display it.
        """
        if len(self.registered_data_load_location_str_var.get()) > 3:
            directory_to_look_in = self.registered_data_load_location_str_var.get()[
                                   :self.registered_data_load_location_str_var.get().rfind('/')]
        else:
            if len(self.registered_data_save_location_str_var.get()) > 3:
                directory_to_look_in = self.registered_data_save_location_str_var.get()
            else:
                directory_to_look_in = '/home/ben/thyroid_ultrasound_data/testing_and_validation/registered_data'
        selected_directory = askdirectory(initialdir=directory_to_look_in,
                                          title="Select the data to load to generate the volume.")
        if len(selected_directory) > 3 and isdir(selected_directory):
            self.nrts_registered_data_load_location_service(selected_directory)
            self.vg_registered_data_load_location_service(selected_directory)
            self.registered_data_load_location_str_var.set(selected_directory)
            if len(self.registered_data_load_location_str_var.get()) > 3 and \
                    len(self.volume_data_save_location_str_var.get()) > 3:
                self.generate_new_volume_button[WIDGET_STATE] = NORMAL

    def volume_data_save_location_button_callback(self) -> None:
        """
        Select the directory in which to save the volume data, publish it, and display it.
        """
        if len(self.volume_data_save_location_str_var.get()) > 3:
            directory_to_look_in = self.volume_data_save_location_str_var.get()
        else:
            directory_to_look_in = "/home/ben/thyroid_ultrasound_data/testing_and_validation/volume_data"
        selected_directory = askdirectory(initialdir=directory_to_look_in,
                                          title="Select the destination for the volume data.")
        if len(selected_directory) > 3 and isdir(selected_directory):
            self.volume_data_save_location_service(selected_directory)
            self.volume_data_save_location_str_var.set(selected_directory)
            if len(self.registered_data_load_location_str_var.get()) > 3 and \
                    len(self.volume_data_save_location_str_var.get()) > 3:
                self.generate_new_volume_button[WIDGET_STATE] = NORMAL

    def volume_data_result_callback(self, msg: Float64):
        self.volume_data_result_str_var.set(str(round(msg.data, 2)))

    def update_pose_control_button(self) -> None:

        # Set that pose feedback is now being used
        self.currently_using_pose_feedback = True

        # Publish that pose feedback is now being used
        self.use_pose_feedback_command_service(self.currently_using_pose_feedback)

        # Update the pose control button
        self.pose_control_button[WIDGET_TEXT] = STOP_POSE_CONTROL
        self.pose_control_button[WIDGET_STATE] = NORMAL

        # Set the state of the trajectory button
        self.trajectory_following_button[WIDGET_TEXT] = PAUSE_TRAJECTORY
        self.trajectory_following_button[WIDGET_STATE] = NORMAL

    def trajectory_complete_handler(self, req: BoolRequestRequest):
        """
        Resets the pose control and experiment data saving buttons.
        """
        if req.value:
            # Set the state to be that the robot is not currently using force_feedback
            self.currently_using_pose_feedback = False

            # Publish the command to stop using force feedback
            self.use_pose_feedback_command_service(self.currently_using_pose_feedback)

            # Set the state of the button
            self.pose_control_button[WIDGET_STATE] = DISABLED

            # Set the state of the trajectory button
            self.trajectory_following_button[WIDGET_STATE] = DISABLED

            # Reset the save experiment data buttons
            self.save_experiment_data_button_callback(action=STOP_SAVING_EXPERIMENT_DATA)
        return BoolRequestResponse(True, NO_ERROR)

    def generate_volume_button_callback(self) -> None:
        """
        Publish the command to generate a volume from the ultrasound images.
        """
        self.nrts_generate_volume_command_service(True)
        self.vg_generate_volume_command_service(True)
        self.volume_data_result_str_var.set('0.00')

    def volume_data_load_location_button_callback(self) -> None:
        if len(self.volume_data_load_location_str_var.get()) > 3:
            directory_to_look_in = self.volume_data_load_location_str_var.get()[
                                   :self.volume_data_load_location_str_var.get().rfind('/')]
        else:
            if len(self.volume_data_save_location_str_var.get()) > 3:
                directory_to_look_in = self.volume_data_save_location_str_var.get()
            else:
                directory_to_look_in = "/home/ben/thyroid_ultrasound_data/testing_and_validation/volume_data"
        selected_directory = askdirectory(initialdir=directory_to_look_in,
                                          title="Select the the volume data to load.")
        if len(selected_directory) > 3 and isdir(selected_directory):
            self.volume_data_load_location_service(selected_directory)
            self.volume_data_load_location_str_var.set(selected_directory)
            self.display_loaded_volume_button[WIDGET_STATE] = NORMAL

    def display_loaded_volume_button_callback(self) -> None:
        self.display_loaded_volume_command_service(True)
        self.volume_data_result_str_var.set('0.00')

    def image_streaming_button_callback(self):
        """
        Toggles if the ultrasound images will be streamed, based on the user input.
        """
        # Get the current text of the button
        button_text = self.image_streaming_button[WIDGET_TEXT]

        # If the button currently says "Start"
        if button_text == START_IMAGE_STREAMING:

            # Publish the command to start filtering images
            self.image_streaming_command_service(True)

            # Set it to say "Stop"
            new_button_text = STOP_IMAGE_STREAMING

            # Don't allow the user to reverse the order while streaming
            self.reverse_stream_order_button[WIDGET_STATE] = DISABLED

        # If the button currently says "Stop"
        else:

            # Publish the command to stop filtering images
            self.image_streaming_command_service(False)

            # Set the button to say "Start"
            new_button_text = START_IMAGE_STREAMING

            # Allow the user to reverse the order
            self.reverse_stream_order_button[WIDGET_STATE] = NORMAL

        # Set the new text of the button
        self.image_streaming_button[WIDGET_TEXT] = new_button_text

    def restart_image_streaming_button_callback(self):
        self.restart_image_streaming_command_service(True)

    def image_control_button_callback(self):
        """
        Toggles if the robot will use image feedback, based on the user input.
        """
        # Get the current text of the button
        button_text = self.image_control_button[WIDGET_TEXT]

        # If the button currently says "Start"
        if button_text == START_IMAGE_CONTROL:

            # Set it to say "Stop"
            new_button_text = STOP_IMAGE_CONTROL

            # Set the state to be that image control is currently being used
            self.currently_using_image_control = True

        # If the button currently says "Stop"
        else:

            # Set the button to say "Stop"
            new_button_text = START_IMAGE_CONTROL

            # Set the state to be that image control is not currently being used
            self.currently_using_image_control = False

        self.use_image_feedback_command_service(self.currently_using_image_control)

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
        elif button_text == STOP_FORCE_CONTROL:

            # Set it to say "Start"
            new_button_text = START_FORCE_CONTROL

            # Set the state to be that the robot is not currently using force_feedback
            self.currently_using_force_feedback = False

        else:
            raise Exception(button_text + " is not a recognized button text.")

        # Publish the command to stop using force feedback
        self.use_force_feedback_command_service(self.currently_using_force_feedback)

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
        self.use_balancing_feedback_command_service(self.currently_using_balancing_feedback)

        # Set the new text of the button
        self.balancing_control_button[WIDGET_TEXT] = new_button_text

    def trajectory_following_button_callback(self):
        """
        Toggles if the robot will use pause or resume following the trajectory, based on the user input.
        """

        # Get the current text of the button
        button_text = self.trajectory_following_button[WIDGET_TEXT]

        # If the button currently says "Pause"
        if button_text == PAUSE_TRAJECTORY:

            # Set it to say "Resume"
            new_button_text = RESUME_TRAJECTORY

            # Set the state to be that the robot is not currently using pose feedback
            self.currently_using_pose_feedback = False

        # If the button currently says "Resume"
        else:

            # Set it to say "Pause"
            new_button_text = PAUSE_TRAJECTORY

            # Set the state to be that the robot is currently using pose feedback
            self.currently_using_pose_feedback = True

        # Publish the command to stop or start using pose feedback
        self.use_pose_feedback_command_service(self.currently_using_pose_feedback)

        # Set the new text of the button
        self.trajectory_following_button[WIDGET_TEXT] = new_button_text

    def pose_control_button_callback(self):
        """
        Toggles if the robot will use pose feedback, based on the user input.
        """

        # Set the state to be that the robot is not currently using force_feedback
        self.currently_using_pose_feedback = False

        # Publish the command to stop using force feedback
        self.use_pose_feedback_command_service(self.currently_using_pose_feedback)

        # Publish the command to clear the trajectory
        # self.clear_trajectory_command_publisher.publish(Bool(True))
        self.clear_trajectory_service(True)

        # Set the state of the button
        self.pose_control_button[WIDGET_STATE] = DISABLED

        # Set the state of the trajectory following button
        self.trajectory_following_button[WIDGET_STATE] = DISABLED

        # Reset the save experiment data buttons
        self.save_experiment_data_button_callback(action=STOP_SAVING_EXPERIMENT_DATA)

    def publish_controller_statuses_button_callback(self) -> None:
        if self.publish_controller_statuses_button[WIDGET_TEXT] == START_PUBLISHING_CONTROLLER_STATUS_VALUES:
            self.publish_controller_statuses_service(True)
            self.publish_controller_statuses_button[WIDGET_TEXT] = STOP_PUBLISHING_CONTROLLER_STATUS_VALUES
        elif self.publish_controller_statuses_button[WIDGET_TEXT] == STOP_PUBLISHING_CONTROLLER_STATUS_VALUES:
            self.publish_controller_statuses_service(False)
            self.publish_controller_statuses_button[WIDGET_TEXT] = START_PUBLISHING_CONTROLLER_STATUS_VALUES
        else:
            raise Exception("Button text was not recognized")

    def pid_controller_selection_callback(self) -> None:
        """
        Publish the pid controller selected on the interface.
        """
        resp: ViewControllerGainsResponse = self.view_controller_gains_service(self.pid_selector.get())
        self.p_gain_var.set(resp.p_gain)
        self.i_gain_var.set(resp.i_gain)
        self.d_gain_var.set(resp.d_gain)

    def speed_selector_callback(self) -> None:
        """
        Publish the speed selected on the interface.
        """
        self.speed_selector_service(self.speed_selector_variable.get() / 100)

    def pid_value_setting_callback(self) -> None:
        """
        Publish the values selected in the interface
        """
        try:
            p_gain = float(self.p_gain_entry.get())
            i_gain = float(self.i_gain_entry.get())
            d_gain = float(self.d_gain_entry.get())
        except ValueError:
            return

        self.set_controller_gains_service(self.pid_selector.get(), p_gain, i_gain, d_gain)

    def send_image_saving_command_callback(self) -> None:

        if self.save_images_button.cget(WIDGET_TEXT) == START_SAVING_IMAGES:
            self.send_image_saving_command_service(True)
            self.save_images_button.configure(text=STOP_SAVING_IMAGES)
        elif self.save_images_button.cget(WIDGET_TEXT) == STOP_SAVING_IMAGES:
            self.send_image_saving_command_service(False)
            self.save_images_button.configure(text=START_SAVING_IMAGES)
        else:
            raise Exception("Something very strange has happened here.")

    def send_save_images_destination(self) -> None:
        self.send_folder_destination_service(askdirectory(initialdir='/home/ben',
                                                          title="Select destination for saved images."))

    def image_data_stream_location_button_callback(self) -> None:
        """
        Select the directory from which to stream images.
        """
        selected_directory = askdirectory(initialdir=self.image_data_stream_location_str_var.get(),
                                          title="Select the destination for the exam data.")
        if len(selected_directory) > 3 and isdir(selected_directory):
            self.image_streaming_location_service(selected_directory)
            self.image_data_stream_location_str_var.set(selected_directory)
            self.image_streaming_button[WIDGET_STATE] = NORMAL
            self.restart_image_streaming_button[WIDGET_STATE] = NORMAL
            self.reverse_stream_order_button[WIDGET_STATE] = NORMAL

    def reverse_stream_order_button_callback(self) -> None:
        if self.reverse_stream_order_button[WIDGET_TEXT] == PLAYBACK_STREAM_IN_REVERSE_CHRONOLOGICAL_ORDER:
            self.reverse_stream_order_service(True)
            self.reverse_stream_order_button[WIDGET_TEXT] = PLAYBACK_STREAM_IN_CHRONOLOGICAL_ORDER
        elif self.reverse_stream_order_button[WIDGET_TEXT] == PLAYBACK_STREAM_IN_CHRONOLOGICAL_ORDER:
            self.reverse_stream_order_service(False)
            self.reverse_stream_order_button[WIDGET_TEXT] = PLAYBACK_STREAM_IN_REVERSE_CHRONOLOGICAL_ORDER
        else:
            raise Exception("Button text was not recognized")

    def set_new_streaming_frequency_callback(self) -> None:
        self.image_streaming_frequency_service(float(self.stream_data_rate_entry.get()))

    def register_new_data_callback(self) -> None:
        self.register_new_data_service(True)

    def send_patient_contact_override_button_callback(self) -> None:
        if self.send_patient_contact_override_button[WIDGET_TEXT] == START_SENDING_PATIENT_CONTACT_OVERRIDE_VALUE:
            # self.is_patient_contact_override_active = True
            self.tm_override_patient_contact_service(True)
            self.rc_override_patient_contact_service(True)
            # self.in_contact_radio_button[WIDGET_STATE] = DISABLED
            # self.not_in_contact_radio_button[WIDGET_STATE] = DISABLED
            self.send_patient_contact_override_button[WIDGET_TEXT] = STOP_SENDING_PATIENT_CONTACT_OVERRIDE_VALUE
        elif self.send_patient_contact_override_button[WIDGET_TEXT] == STOP_SENDING_PATIENT_CONTACT_OVERRIDE_VALUE:
            # self.is_patient_contact_override_active = False
            self.tm_override_patient_contact_service(False)
            self.rc_override_patient_contact_service(False)
            # self.in_contact_radio_button[WIDGET_STATE] = NORMAL
            # self.not_in_contact_radio_button[WIDGET_STATE] = NORMAL
            self.send_patient_contact_override_button[WIDGET_TEXT] = START_SENDING_PATIENT_CONTACT_OVERRIDE_VALUE
        else:
            raise Exception("Button text was not recognized")

    def send_force_control_override_button_callback(self) -> None:
        if self.send_force_control_override_button[WIDGET_TEXT] == START_SENDING_FORCE_CONTROL_OVERRIDE_VALUE:
            self.tm_override_force_control_service(True)
            self.send_force_control_override_button[WIDGET_TEXT] = STOP_SENDING_FORCE_CONTROL_OVERRIDE_VALUE
        elif self.send_force_control_override_button[WIDGET_TEXT] == STOP_SENDING_FORCE_CONTROL_OVERRIDE_VALUE:
            self.tm_override_force_control_service(False)
            self.send_force_control_override_button[WIDGET_TEXT] = START_SENDING_FORCE_CONTROL_OVERRIDE_VALUE
        else:
            raise Exception(
                self.send_force_control_override_button[WIDGET_TEXT] + " is not a recognized button text.")

    def send_registered_data_override_button_callback(self) -> None:
        if self.send_registered_data_override_button[WIDGET_TEXT] == START_SENDING_REGISTERED_DATA_OVERRIDE_VALUE:
            # self.is_registered_data_override_active = True
            self.tm_override_data_registered_service(True)
            self.send_registered_data_override_button[WIDGET_TEXT] = STOP_SENDING_REGISTERED_DATA_OVERRIDE_VALUE
        elif self.send_registered_data_override_button[WIDGET_TEXT] == STOP_SENDING_REGISTERED_DATA_OVERRIDE_VALUE:
            # self.is_registered_data_override_active = False
            self.tm_override_data_registered_service(False)
            self.send_registered_data_override_button[WIDGET_TEXT] = START_SENDING_REGISTERED_DATA_OVERRIDE_VALUE
        else:
            raise Exception("Button text was not recognized")

    def send_image_centered_override_button_callback(self) -> None:
        if self.send_image_centered_override_button[WIDGET_TEXT] == START_SENDING_IMAGE_CENTERED_OVERRIDE_VALUE:
            # self.is_image_centered_override_active = True
            self.tm_override_image_centered_service(True)
            self.send_image_centered_override_button[WIDGET_TEXT] = STOP_SENDING_IMAGE_CENTERED_OVERRIDE_VALUE
        elif self.send_image_centered_override_button[WIDGET_TEXT] == STOP_SENDING_IMAGE_CENTERED_OVERRIDE_VALUE:
            # self.is_image_centered_override_active = False
            self.tm_override_image_centered_service(False)
            self.send_image_centered_override_button[WIDGET_TEXT] = START_SENDING_IMAGE_CENTERED_OVERRIDE_VALUE
        else:
            raise Exception("Button text was not recognized")

    def send_image_balanced_override_button_callback(self) -> None:
        if self.send_image_balanced_override_button[WIDGET_TEXT] == START_SENDING_IMAGE_BALANCED_OVERRIDE_VALUE:
            # self.is_image_balanced_override_active = True
            self.tm_override_image_balanced_service(True)
            self.send_image_balanced_override_button[WIDGET_TEXT] = STOP_SENDING_IMAGE_BALANCED_OVERRIDE_VALUE
        elif self.send_image_balanced_override_button[WIDGET_TEXT] == STOP_SENDING_IMAGE_BALANCED_OVERRIDE_VALUE:
            # self.is_image_balanced_override_active = False
            self.tm_override_image_balanced_service(False)
            self.send_image_balanced_override_button[WIDGET_TEXT] = START_SENDING_IMAGE_BALANCED_OVERRIDE_VALUE
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

            self.all_data_saved_text_variable.set("")

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

    def visualization_check_button_callback(self, visualization: int):
        if visualization in self.show_visualization_variables.keys():
            if visualization in self.show_visualization_services.keys():
                self.show_visualization_services[visualization](visualization,
                                                                bool(self.show_visualization_variables[
                                                                         visualization].get()))
            else:
                raise Exception(str(visualization) + ' is not a recognized visualization service.')
        else:
            raise Exception(str(visualization) + ' is not a recognized visualization variable.')

    def segmentation_phase_button_callback(self):
        if self.segmentation_phase_button[WIDGET_TEXT] == SET_TO_REST_PHASE:
            response: StringRequestResponse = self.set_segmentation_phase_service(REST_PHASE)
            if response.was_succesful:
                self.segmentation_phase_button[WIDGET_TEXT] = SET_TO_GROWTH_PHASE
        elif self.segmentation_phase_button[WIDGET_TEXT] == SET_TO_GROWTH_PHASE:
            response: StringRequestResponse = self.set_segmentation_phase_service(GROWTH_PHASE)
            if response.was_succesful:
                self.segmentation_phase_button[WIDGET_TEXT] = SET_TO_REST_PHASE
        else:
            raise Exception(self.segmentation_phase_button[WIDGET_TEXT] + ' is not recognized.')


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
        try:
            self.current_force_string_var.set(str(round(data.wrench.force.z, 1)))
        except AttributeError:
            pass

    def node_statuses_callback(self, data: log_message):
        self.update_status(data)

    def log_messages_callback(self, data: log_message):
        self.update_log(data)

    def all_data_saved_callback(self, msg: Bool):
        try:
            self.all_data_saved_text_variable.set("All data has been saved.")
        except AttributeError:
            pass

    def remaining_data_callback(self, msg: String):

        if self.all_data_saved_text_variable.get() != "All data has been saved.":
            self.all_data_saved_text_variable.set(msg.data)

    def shutdown_node(self):
        """
        Define the custom shutdown behavior of the node to close the window.
        """
        print("\nShutdown signal received.")
        try:
            self.parent.destroy()
        except TclError:
            pass

    # endregion
    #############################################################################

    #############################################################################
    # Define Helpers
    # region
    def update_status(self, message: log_message):
        timestamp = datetime.fromtimestamp(message.header.stamp.to_time()).strftime('%H:%M:%S')
        if message is not None and message.source in self.node_names_all and \
                self.current_status_messages[message.source] != message.message:
            self.current_status_times[message.source].set(timestamp)
            self.current_status_messages[message.source].set(message.message + '.')

    def update_log(self, message: log_message) -> None:
        """
        Defines a function to update the status label at the bottom of the window with a given message.

        Parameters
        ----------
        message
            The message to be displayed. A period will be added to the end of this message automatically.
            If no message is given, the empty status string will be shown.
        """
        if message is not None and message.source in self.node_names_all:

            timestamp = datetime.fromtimestamp(message.header.stamp.to_time()).strftime('%H:%M:%S')

            if self.current_status_messages[message.source] != message.message:
                # self.current_status_messages[message.source].set(message.message + '.')
                self.status_log_dict[message.source].append(timestamp + ': ' + message.message + '.')
                if len(self.status_log_dict[message.source]) > MAXIMUM_STATUS_MESSAGES:
                    self.status_log_dict[message.source].pop(0)
                self.select_node()
                # self.status_log_string = self.status_log_string + '\n' + message.message + "."
                # self.status_label.configure(state=NORMAL)
                # self.status_label.delete('1.0', END)
                # self.status_label.insert(INSERT, self.status_log_string)
                # self.status_label.configure(state=DISABLED)

    def select_node(self):
        if self.selected_node.get() in self.node_names_all:
            self.status_label.configure(state=NORMAL)
            self.status_label.delete('1.0', END)
            temp_list = copy(self.status_log_dict[self.selected_node.get()])
            delimiter = '\n'
            if len(temp_list) > 1:
                temp_list.reverse()
            self.status_label.insert(INSERT, delimiter.join(temp_list))
            self.status_label.configure(state=DISABLED)

    def ui_update_log(self, this_message: str):
        temp_msg = log_message(source=USER_INTERFACE, message=this_message)
        temp_msg.header.stamp = Time.now()
        self.update_log(temp_msg)

    # endregion
    #############################################################################

    def publishing_loop(self):
        """
        Publishes the different commands at regular intervals.
        """
        if self.previous_image_centering_side_variable_value != self.image_centering_side_variable.get():
            self.previous_image_centering_side_variable_value = self.image_centering_side_variable.get()
            self.image_centering_side_publisher.publish(Int8(self.image_centering_side_variable.get()))
        new_msg = TwistStamped()
        new_msg.header.stamp = Time.now()
        if self.manual_control_velocity_message is not None:
            new_msg.twist = self.manual_control_velocity_message
        self.manual_robot_control_publisher.publish(new_msg)
        root.after(25, self.publishing_loop)


# (ttk.Label(exam_setup_frame, text="Current Set-point (N):"), LEFT_COLUMN, SINGLE_COLUMN, 0, DOUBLE_ROW)

def create_widget_object(widget_object,
                         col_num: int, row_num: int,
                         col_span: int = SINGLE_ROW, row_span: int = SINGLE_COLUMN,
                         ipadx: int = 0, ipady: int = 0,
                         padx: int = 5, pady: int = 5,
                         sticky: str = 'nsew', increment_row: bool = False):
    widget_object.grid(column=col_num,
                       columnspan=col_span,
                       row=row_num,
                       rowspan=row_span,
                       ipadx=ipadx,
                       ipady=ipady,
                       padx=padx,
                       pady=pady,
                       sticky=sticky
                       )
    if increment_row:
        return row_num + 1
    else:
        return row_num


def create_horizontal_separator(frame, row: int):
    return create_widget_object(ttk.Separator(frame), col_num=LEFT_COLUMN, col_span=FULL_WIDTH,
                                row_num=row, padx=0, increment_row=True)


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

    print("Node initialized.")
    print("Press ctrl+c to terminate.")

    # Define the publishing rate
    publishing_rate = 10  # hz

    # Run the publishing loop in the background alongside the TKinter main loop
    root.after(round(1000 / publishing_rate), node.publishing_loop)

    # Run until the window is closed.
    root.mainloop()
