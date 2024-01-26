#!/usr/bin/env python3

"""
File containing UserInterface class.
"""

# Import standard packages
from tkinter import *
from tkinter.scrolledtext import ScrolledText
from tkinter.filedialog import askdirectory
import tkinter.ttk as ttk
from argparse import ArgumentParser

# Import ROS packages
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String, UInt8

# Import custom ROS packages
from thyroid_ultrasound_support.BasicNode import *
from thyroid_ultrasound_support.TopicNames import *

# Define constants used for logging purposes
VERBOSE: int = int(0)
NORMAL: int = int(1)

# Define constant to use as incremental increase and decrease value for force set-point
INCREMENTAL_FORCE_CHANGE: float = 0.1
NUM_DIGITS_OF_FORCE_TO_DISPLAY: int = int(2)

# Define constants for GUI elements
START_IMAGE_STREAMING: str = "Start Image Streaming"
STOP_IMAGE_STREAMING: str = "Stop Image Streaming"
START_IMAGE_FILTER: str = "Start Image Filtering"
STOP_IMAGE_FILTER: str = "Stop Image Filtering"
START_FORCE_CONTROL: str = "Start Force Control"
STOP_FORCE_CONTROL: str = "Stop Force Control"
ENABLE_ROBOT_MOVEMENT: str = "ENABLE Robot Movement"
DISABLE_ROBOT_MOVEMENT: str = "DISABLE Robot Movement"
TEST_FORCE_CONTROL: str = "Test Force\n Profile"
STOP_TEST_FORCE_CONTROL: str = "Stop Testing\n Force Control"
START_SAVING_IMAGES: str = "Start Saving Images"
STOP_SAVING_IMAGES: str = "Stop Saving Images"

# Define constants for parameters of widgets
WIDGET_TEXT: str = 'text'
WIDGET_STATE: str = 'state'

# Define grid geometry constants
FULL_WIDTH: int = int(5)
THREE_COLUMN: int = int(3)
TWO_COLUMN: int = int(2)

SINGLE_COLUMN: int = int(1)
LEFT_COLUMN: int = int(0)
L_MIDDLE_COLUMN: int = int(2)
MIDDLE_COLUMN: int = int(3)
R_MIDDLE_COLUMN: int = int(4)
RIGHT_COLUMN: int = int(5)

SINGLE_ROW: int = int(1)
DOUBLE_ROW: int = int(2)

GRAPHICS_WINDOW: int = int(3)

# Define empty status string
EMPTY_STATUS: str = "STATUS: "

# Define function mode constants
TESTING: int = int(0)
RUNNING: int = int(1)


# TODO - Medium - Add robot controls to allow the user to move the robot around and override the image segmentation
# TODO - Medium - Change topic names to use constants from topics files
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

        # Create a publisher to publish the command to start and stop filtering images
        self.filter_images_command_publisher = Publisher(FILTER_IMAGES, Bool, queue_size=1)

        # Create a publisher to publish the command to use force feedback
        self.use_force_feedback_command_publisher = Publisher(USE_FORCE_FEEDBACK, Bool,
                                                              queue_size=1)

        # Create a publisher to publish the command to start and stop the robot motion
        self.stop_robot_motion_command_publisher = Publisher('/command/stop_motion', Bool, queue_size=1)

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
        self.test_force_profile_publisher = Publisher('/command/test_force_profile', Bool, queue_size=1)

        # Create a publisher to publish the command to load image crop coordinates from a file
        self.load_existing_image_cropping_command_publisher = Publisher(CROP_IMAGE_FROM_TEMPLATE,
                                                                        Bool, queue_size=1)

        # Create a publisher to publish the command to scan upwards
        self.scan_command_publisher = Publisher(CREATE_TRAJECTORY, Float64, queue_size=1)

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

        # Create a subscriber to listen to the external force felt by the robot
        Subscriber(ROBOT_DERIVED_FORCE, WrenchStamped, self.robot_sensed_force_callback)

        # Create a subscriber to hear debug messages
        Subscriber(LOGGING, log_message, self.debug_status_messages_callback)

        # endregion
        # ---------------------------------------

        # -------------------------------------------
        # Create GUI components of the User Interface
        # region

        # Define parameters used in the logic of the GUI
        self.currently_filtering = False
        self.currently_using_force_feedback = False
        self.currently_using_pose_feedback = False

        # Set the title of the window
        self.parent.title("User Interface")

        # Define the frame in which all objects will be created
        main_content_frame = Frame(parent)

        # Define a frame to hold the always visible components
        always_visible_frame = Frame(main_content_frame)

        # Define a notebook to manage the tabs within the GUI
        tab_controller = ttk.Notebook(main_content_frame)

        # Define the frames that will be used in the tabs
        exam_setup_frame = ttk.Frame(tab_controller)
        thyroid_exam_frame = ttk.Frame(tab_controller)
        nodule_exam_frame = ttk.Frame(tab_controller)
        status_logging_frame = ttk.Frame(tab_controller)
        developer_frame = ttk.Frame(tab_controller)

        # Add the frames to the tab controller
        tab_controller.add(exam_setup_frame, text="Exam Setup")
        tab_controller.add(thyroid_exam_frame, text="Thyroid Exam")
        tab_controller.add(nodule_exam_frame, text="Nodule Exam")
        tab_controller.add(status_logging_frame, text="Status Logger")
        tab_controller.add(developer_frame, text="Developer")

        # Define the widgets used in the always_visible_frame
        # region
        # Define the buttons used for image streaming in testing
        self.image_streaming_button = ttk.Button(always_visible_frame, text=START_IMAGE_STREAMING,
                                                 command=self.image_streaming_button_callback)
        self.restart_image_streaming_button = ttk.Button(always_visible_frame, text="Restart Image Streaming",
                                                         command=self.restart_image_streaming_button_callback,
                                                         )

        # List the widgets used for testing in the always visible frame at the bottom
        always_visible_testing_widgets = [
            (self.image_streaming_button, LEFT_COLUMN, SINGLE_COLUMN, 0, 1),
            (self.restart_image_streaming_button, L_MIDDLE_COLUMN, SINGLE_COLUMN, 0, 1),
        ]

        # Define the buttons used for general control
        self.image_filtering_button = ttk.Button(always_visible_frame, text=START_IMAGE_FILTER,
                                                 command=self.image_filtering_button_callback)
        self.force_control_button = ttk.Button(always_visible_frame, text=START_FORCE_CONTROL,
                                               command=self.force_control_button_callback)
        self.allow_robot_movement_button = ttk.Button(always_visible_frame, text=ENABLE_ROBOT_MOVEMENT,
                                                      command=self.allow_robot_movement_button_callback,
                                                      state=DISABLED)

        # List the widgets used for general control in the always visible frame at the bottom
        always_visible_streaming_widgets = [
            (self.image_filtering_button, MIDDLE_COLUMN, SINGLE_COLUMN, 0, SINGLE_ROW),
            (self.force_control_button, R_MIDDLE_COLUMN, SINGLE_COLUMN, 0, SINGLE_ROW),
            (self.allow_robot_movement_button, RIGHT_COLUMN, SINGLE_COLUMN, 0, SINGLE_ROW),
        ]

        # Select which widgets to display in the always_visible_frame based on which function mode the GUI is in
        self.always_visible_frame_widgets = always_visible_streaming_widgets
        if passed_arguments.testing_mode:
            self.always_visible_frame_widgets = always_visible_testing_widgets + self.always_visible_frame_widgets

        # endregion

        # Define the widgets used in the exam_setup_frame
        # region
        validation_command = self.parent.register(self.entry_widget_float_validation)
        self.force_set_point_entry = ttk.Entry(exam_setup_frame, validate=ALL,
                                               validatecommand=(validation_command, '%P'))
        # self.force_set_point_entry.bind('<Return>', self.force_set_point_submit_callback)
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
        self.current_force_label = ttk.Label(exam_setup_frame, textvariable=self.current_force_string_var)
        self.select_image_crop_variable = IntVar()
        self.generate_new_image_cropping_button = ttk.Button(exam_setup_frame, text="Generate a New Image Cropping",
                                                             command=self.generate_new_image_cropping_button_callback,
                                                             state=DISABLED)
        self.load_existing_image_cropping_button = ttk.Button(exam_setup_frame,
                                                              text="Load Existing Image Cropping",
                                                              command=self.load_existing_image_cropping_button_callback,
                                                              state=DISABLED)
        self.imaging_depth_entry = ttk.Entry(exam_setup_frame, validate=ALL,
                                             validatecommand=(validation_command, '%P'))
        self.imaging_depth_entry.insert(0, '5.0')
        self.imaging_depth_submit_callback()

        # List the widgets used to populate the exam_setup_frame
        self.exam_setup_widgets = [
            (ttk.Label(exam_setup_frame, text="Current Set-point (N):"), LEFT_COLUMN, SINGLE_COLUMN, 0, DOUBLE_ROW),
            (self.force_set_point_entry, L_MIDDLE_COLUMN, SINGLE_COLUMN, 0, DOUBLE_ROW),
            (force_set_point_increase_button, MIDDLE_COLUMN, SINGLE_COLUMN, 0, SINGLE_ROW),
            (force_set_point_decrease_button, MIDDLE_COLUMN, SINGLE_COLUMN, 1, SINGLE_ROW),
            (ttk.Button(exam_setup_frame, text="Send", command=self.force_set_point_submit_callback),
             R_MIDDLE_COLUMN, SINGLE_COLUMN, 0, DOUBLE_ROW),
            (self.test_force_profile_button, RIGHT_COLUMN, TWO_COLUMN, 0, 3),
            (ttk.Label(exam_setup_frame, text="Current Force (N):"), LEFT_COLUMN, SINGLE_COLUMN, 2, SINGLE_ROW),
            (self.current_force_label, L_MIDDLE_COLUMN, SINGLE_COLUMN, 2, SINGLE_ROW),
            (ttk.Label(exam_setup_frame, text="Would you like to\ncrop the raw image?"),
             LEFT_COLUMN, SINGLE_COLUMN, 3, SINGLE_ROW),
            (Radiobutton(exam_setup_frame, text="Yes", variable=self.select_image_crop_variable,
                         value=1, command=self.select_image_crop_callback), LEFT_COLUMN, SINGLE_COLUMN, 4, SINGLE_ROW),
            (Radiobutton(exam_setup_frame, text="No", variable=self.select_image_crop_variable,
                         value=0, command=self.select_image_crop_callback), LEFT_COLUMN, SINGLE_COLUMN, 5, SINGLE_ROW),
            (self.generate_new_image_cropping_button, L_MIDDLE_COLUMN, TWO_COLUMN, 4, DOUBLE_ROW),
            (self.load_existing_image_cropping_button, R_MIDDLE_COLUMN, TWO_COLUMN, 4, DOUBLE_ROW),
            (ttk.Label(exam_setup_frame, text="Set the imaging depth (cm)\nof the US scanner:"),
             LEFT_COLUMN, SINGLE_COLUMN, 6, SINGLE_ROW),
            (self.imaging_depth_entry, L_MIDDLE_COLUMN, SINGLE_COLUMN, 6, SINGLE_ROW),
            (ttk.Button(exam_setup_frame, text="Send", command=self.imaging_depth_submit_callback),
             MIDDLE_COLUMN, TWO_COLUMN, 6, SINGLE_ROW),
        ]

        # endregion

        # Define the widgets used to populate the thyroid exam frame
        # region
        self.identify_thyroid_from_points_button = ttk.Button(thyroid_exam_frame,
                                                              text="Identify Thyroid from Points",
                                                              command=self.identify_thyroid_from_points_button_callback,
                                                              )
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
        self.scan_distance_entry = Entry(thyroid_exam_frame, validate=ALL, validatecommand=(validation_command, '%P'))
        self.scan_distance_entry.insert(0, '6.0')
        self.complete_full_scan_button = ttk.Button(thyroid_exam_frame,
                                                    text="Complete\n Full Scan",
                                                    command=self.complete_full_scan_button_callback,
                                                    )
        self.generate_volume_selector_variable = IntVar()
        self.display_volume_selector_variable = IntVar()

        # List the widgets to used to populate the thyroid exam frame
        self.thyroid_exam_widgets = [
            (self.identify_thyroid_from_points_button, LEFT_COLUMN, TWO_COLUMN, 0, 1),
            (self.identify_thyroid_from_template_button, R_MIDDLE_COLUMN, TWO_COLUMN, 0, 1),
            (self.scan_positive_button, LEFT_COLUMN, TWO_COLUMN, 1, 1),
            (self.scan_negative_button, LEFT_COLUMN, TWO_COLUMN, 2, 1),
            (ttk.Label(thyroid_exam_frame, text="Scanning Distance (cm)"), MIDDLE_COLUMN, TWO_COLUMN, 1, 1),
            (self.scan_distance_entry, MIDDLE_COLUMN, TWO_COLUMN, 2, 1),
            (self.complete_full_scan_button, RIGHT_COLUMN, SINGLE_COLUMN, 1, 2),
            (ttk.Label(thyroid_exam_frame, text="Would you like to generate a volume?"),
             LEFT_COLUMN, THREE_COLUMN, 3, 1),
            (Radiobutton(thyroid_exam_frame, text="Yes", variable=self.generate_volume_selector_variable,
                         value=1, command=self.generate_volume_button_callback), R_MIDDLE_COLUMN, SINGLE_COLUMN, 3, 1),
            (Radiobutton(thyroid_exam_frame, text="No", variable=self.generate_volume_selector_variable,
                         value=0, command=self.generate_volume_button_callback), RIGHT_COLUMN, SINGLE_COLUMN, 3, 1),
            (ttk.Label(thyroid_exam_frame, text="Would you like to display the volume?"),
             LEFT_COLUMN, THREE_COLUMN, 4, 1),
            (Radiobutton(thyroid_exam_frame, text="Yes", variable=self.display_volume_selector_variable,
                         value=1, command=self.display_volume_button_callback),
             R_MIDDLE_COLUMN, SINGLE_COLUMN, 4, 1),
            (Radiobutton(thyroid_exam_frame, text="No", variable=self.display_volume_selector_variable,
                         value=0, command=self.display_volume_button_callback),
             RIGHT_COLUMN, SINGLE_COLUMN, 4, 1),
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
            (self.status_label, LEFT_COLUMN, FULL_WIDTH, 0, 1),
        ]

        # endregion

        # List the widgets used to populate the developer window
        # region
        self.pid_selector = IntVar()
        self.p_gain_entry = ttk.Entry(developer_frame, validate=ALL, validatecommand=(validation_command, '%P'))
        self.p_gain_entry.insert(0, "0.000")
        self.p_gain_var = StringVar(developer_frame, "0.000")
        self.i_gain_entry = ttk.Entry(developer_frame, validate=ALL, validatecommand=(validation_command, '%P'))
        self.i_gain_entry.insert(0, "0.000")
        self.i_gain_var = StringVar(developer_frame, "0.000")
        self.d_gain_entry = ttk.Entry(developer_frame, validate=ALL, validatecommand=(validation_command, '%P'))
        self.d_gain_entry.insert(0, "0.000")
        self.d_gain_var = StringVar(developer_frame, "0.000")
        self.save_images_button = ttk.Button(developer_frame, text=START_SAVING_IMAGES,
                                             command=self.send_image_saving_command_callback)
        self.select_image_destination_directory = ttk.Button(developer_frame,
                                                             text='Select Location of Saved Images',
                                                             command=self.send_save_images_destination)

        developer_widgets = [
            (ttk.Label(developer_frame, text="Select\nController"), LEFT_COLUMN, SINGLE_COLUMN, 0, DOUBLE_ROW),
            (Radiobutton(developer_frame, text="x-lin-trj", variable=self.pid_selector,
                         value=0, command=self.pid_controller_selection_callback),
             L_MIDDLE_COLUMN, SINGLE_COLUMN, 0, SINGLE_ROW,),
            (Radiobutton(developer_frame, text="y-lin-img", variable=self.pid_selector,
                         value=1, command=self.pid_controller_selection_callback),
             MIDDLE_COLUMN, SINGLE_COLUMN, 0, SINGLE_ROW,),
            (Radiobutton(developer_frame, text="z-lin-force", variable=self.pid_selector,
                         value=2, command=self.pid_controller_selection_callback),
             R_MIDDLE_COLUMN, SINGLE_COLUMN, 0, SINGLE_ROW,),
            (Radiobutton(developer_frame, text="x-ang-N/A", variable=self.pid_selector,
                         value=3, command=self.pid_controller_selection_callback),
             L_MIDDLE_COLUMN, SINGLE_COLUMN, 1, SINGLE_ROW,),
            (Radiobutton(developer_frame, text="y-ang-N/A", variable=self.pid_selector,
                         value=4, command=self.pid_controller_selection_callback),
             MIDDLE_COLUMN, SINGLE_COLUMN, 1, SINGLE_ROW,),
            (Radiobutton(developer_frame, text="z-ang-N/A", variable=self.pid_selector,
                         value=5, command=self.pid_controller_selection_callback),
             R_MIDDLE_COLUMN, SINGLE_COLUMN, 1, SINGLE_ROW,),
            (ttk.Label(developer_frame, text="P"), L_MIDDLE_COLUMN, SINGLE_COLUMN, 2, SINGLE_ROW),
            (ttk.Label(developer_frame, text="I"), MIDDLE_COLUMN, SINGLE_COLUMN, 2, SINGLE_ROW),
            (ttk.Label(developer_frame, text="D"), R_MIDDLE_COLUMN, SINGLE_COLUMN, 2, SINGLE_ROW),
            (ttk.Label(developer_frame, text="Current Values:"), LEFT_COLUMN, SINGLE_COLUMN, 3, SINGLE_ROW),
            (ttk.Label(developer_frame, textvariable=self.p_gain_var), L_MIDDLE_COLUMN, SINGLE_COLUMN, 3, SINGLE_ROW),
            (ttk.Label(developer_frame, textvariable=self.i_gain_var), MIDDLE_COLUMN, SINGLE_COLUMN, 3, SINGLE_ROW),
            (ttk.Label(developer_frame, textvariable=self.d_gain_var), R_MIDDLE_COLUMN, SINGLE_COLUMN, 3, SINGLE_ROW),
            (ttk.Label(developer_frame, text="Set to:"), LEFT_COLUMN, SINGLE_COLUMN, 4, SINGLE_ROW),
            (self.p_gain_entry, L_MIDDLE_COLUMN, SINGLE_COLUMN, 4, SINGLE_ROW),
            (self.i_gain_entry, MIDDLE_COLUMN, SINGLE_COLUMN, 4, SINGLE_ROW),
            (self.d_gain_entry, R_MIDDLE_COLUMN, SINGLE_COLUMN, 4, SINGLE_ROW),
            (ttk.Button(developer_frame, text="Set Values", command=self.pid_value_setting_callback),
             RIGHT_COLUMN, SINGLE_COLUMN, 4, SINGLE_ROW),
        ]
        # Only allow the user to save the images if the user interface is not in testing mode
        if not passed_arguments.testing_mode:
            developer_widgets.append((self.select_image_destination_directory, L_MIDDLE_COLUMN, THREE_COLUMN,
                                      5, SINGLE_ROW))
            developer_widgets.append((self.save_images_button, MIDDLE_COLUMN, SINGLE_COLUMN, 6, SINGLE_ROW))

        # endregion

        # Add the widgets to each frame
        list_of_list_of_widgets = [
            self.always_visible_frame_widgets,
            self.exam_setup_widgets,
            self.thyroid_exam_widgets,
            self.nodule_exam_widgets,
            status_logging_widgets,
            developer_widgets
        ]
        for list_of_widgets in list_of_list_of_widgets:
            self.add_widgets(list_of_widgets)

        # Add the parent frame as the only grid object in the window
        main_content_frame.grid(column=0, row=0)

        # Add the two frames to the main frame
        tab_controller.grid(column=LEFT_COLUMN, columnspan=FULL_WIDTH, row=0)
        always_visible_frame.grid(column=LEFT_COLUMN, columnspan=FULL_WIDTH, row=1)

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

    def scan_negative_button_callback(self) -> None:
        """
        Publish the command to scan downwards.
        """
        self.scan_command_publisher.publish(
            Float64(-float(self.scan_distance_entry.get()) / 100)  # convert cm to m
        )

    def complete_full_scan_button_callback(self) -> None:
        """
        Publishes the command to complete a full scan.
        """
        self.complete_full_scan_command_publisher.publish(Bool(True))

    def generate_volume_button_callback(self) -> None:
        """
        Publish the command to generate a volume from the ultrasound images.
        """
        if bool(self.generate_volume_selector_variable.get()):
            self.generate_volume_command_publisher.publish(Bool(True))
        else:
            self.generate_volume_command_publisher.publish(Bool(False))

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

    def image_filtering_button_callback(self):
        """
        Toggles if the ultrasound images will be filtered, based on the user input.
        """
        # Get the current text of the button
        button_text = self.image_filtering_button[WIDGET_TEXT]

        # If the button currently says "Start"
        if button_text == START_IMAGE_FILTER:

            # Publish the command to start filtering images
            self.filter_images_command_publisher.publish(Bool(True))

            # Set it to say "Stop"
            new_button_text = STOP_IMAGE_FILTER

            # Set the state to be that the image is currently being filtered
            self.currently_filtering = True

        # If the button currently says "Stop"
        else:

            # Publish the command to stop filtering images
            self.filter_images_command_publisher.publish(Bool(False))

            # Set the button to say "Stop"
            new_button_text = START_IMAGE_FILTER

            # Set the state to be that the image is not currently being filtered
            self.currently_filtering = False

        # Set the new text of the button
        self.image_filtering_button[WIDGET_TEXT] = new_button_text

        # Check to see if the state of the robot command button should be toggled
        self.robot_toggle_state_change()

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

        # Check to see if the state of the robot command button should be toggled
        self.robot_toggle_state_change()

    def allow_robot_movement_button_callback(self):
        """
        Toggles if the robot will move, based on user input.
        """

        # Get the current text of the button
        button_text = self.allow_robot_movement_button[WIDGET_TEXT]

        # If the button currently says "Start"
        if button_text == ENABLE_ROBOT_MOVEMENT:

            # Set it to say "Stop"
            new_button_text = DISABLE_ROBOT_MOVEMENT

            # Set the state to be that the robot is currently moving
            self.currently_using_pose_feedback = True

        # If the button currently says "Stop"
        else:

            # Set it to say "Start"
            new_button_text = ENABLE_ROBOT_MOVEMENT

            # Set the state to be that the robot is currently not moving
            self.currently_using_pose_feedback = False

        # Publish the command to use pose feedback based on the button state
        self.use_pose_feedback_command_publisher.publish(Bool(self.currently_using_pose_feedback))

        # Set the new text of the button
        self.allow_robot_movement_button[WIDGET_TEXT] = new_button_text

        # Toggle the state of the other buttons based on the state of the robot movement
        self.robot_movement_toggle_other_button_states()

    def robot_toggle_state_change(self):
        """
        Toggle the state of the robot movement button, based on the state of the image filtering and force feedback.
        """

        # If image filtering and force feedback are both on, allow the button to be clicked
        if self.currently_filtering and self.currently_using_force_feedback:
            self.allow_robot_movement_button[WIDGET_STATE] = NORMAL

        # Otherwise, disable it
        else:
            self.allow_robot_movement_button[WIDGET_STATE] = DISABLED

    def robot_movement_toggle_other_button_states(self):
        """
        Toggle the states of the image filtering and force control buttons based on the robot moving state.
        """

        # If the robot is currently moving, disable the image filtering and force control buttons
        if self.currently_using_pose_feedback:
            new_state = DISABLED

        # If the robot is not currently moving, enable the image filtering and force control buttons
        else:
            new_state = NORMAL

        # Set the states of the buttons
        self.image_filtering_button[WIDGET_STATE] = new_state
        self.force_control_button['state'] = new_state

    def pid_controller_selection_callback(self) -> None:
        """
        Publish the pid controller selected on the interface.
        """
        self.pid_controller_publisher.publish(UInt8(self.pid_selector.get()))

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

    def debug_status_messages_callback(self, data: String):
        self.update_status(data.data)

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
            self.status_log_string = self.status_log_string + "\n" + message.message + "."
            self.status_label.configure(state=NORMAL)
            self.status_label.delete('1.0', END)
            self.status_label.insert(INSERT, self.status_log_string)
            self.status_label.configure(state=DISABLED)

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
            widget[0].grid(column=widget[1], columnspan=widget[2], row=widget[3], rowspan=widget[4])

    def disable_enable_pages(self, always_visible_page: bool = None,
                             exam_setup_page: bool = None,
                             thyroid_exam_page: bool = None,
                             nodule_exam_page: bool = None,
                             ) -> None:
        """
        Disables or enables all the widgets on a given frame based on the given status.
        """

        # For each page status, enable/disable each widget accordingly
        for widget_list, page_status in zip([self.always_visible_frame_widgets,
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

    def publishing_loop(self):
        """
        Publishes the different commands at regular intervals.
        """
        self.filter_images_command_publisher.publish(Bool(self.currently_filtering))
        self.use_force_feedback_command_publisher.publish(Bool(self.currently_using_force_feedback))
        self.use_pose_feedback_command_publisher.publish(Bool(self.currently_using_pose_feedback))
        root.after(100, self.publishing_loop)


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
