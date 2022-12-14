#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, WrenchStamped
# from franka_msgs import FrankaState
# from rv_msgs.msg import ManipulatorState
from std_msgs.msg import Float64, String, Bool
from numpy import sign, array, zeros, sqrt, sum
from motion_constants import *
from motion_helpers import *


class HighLevelController:
    def __init__(self) -> None:

        self.cartesian_position_history = []
        # self.image_centroid_error_history = []
        # self.external_force_history = []
        self.thyroid_in_image_status = True
        self.is_thyroid_centered = False
        self.was_last_goal_position_reached = False
        # self.desired_end_effector_force = 0.1  # N
        # self.allowable_centroid_error = .1
        # self.acceptable_cartesian_error = .1
        self.standard_scan_step = array([0.01, 0.0, 0.0])
        self.move_goal = None

        # Command publishers
        self.goal_pose_publisher: rospy.Publisher = None
        self.desired_force_publisher: rospy.Publisher = None
        self.stop_motion_publisher: rospy.Publisher = None
        self.active_image_centering_status_publisher: rospy.Publisher = None

        # Status subscribers
        self.is_thyroid_in_image_status_subscriber: rospy.Subscriber = None
        self.is_thyroid_centered_status_subscriber: rospy.Subscriber = None
        self.was_last_goal_reached_status_subscriber: rospy.Subscriber = None

        # self.velocity_publisher = None
        # self.centroid_error_subscriber = None
        # self.thyroid_in_image_subscriber = None
        # self.robot_state_subscriber = None
        # self.force_readings_subscriber = None

        # initialize ros node
        rospy.init_node('motion_controller')

        # create publishers and subscribers
        self.init_publishers_and_subscribers()

    # --------------------------------------------
    # Callback Functions
    # --------------------------------------------

    # capture if the thyroid is in the current image
    def is_thyroid_in_image_status_callback(self, data: Bool):
        self.thyroid_in_image_status = data.data

    # capture if the thyroid is centered in the image
    def is_thyroid_centered_status_callback(self, data: Bool):
        self.is_thyroid_centered = data.data

    # capture the status of the motion to reach the last sent goal point
    def was_last_goal_reached_status_callback(self, data: Bool):
        self.was_last_goal_position_reached = data.data

    # --------------------------------------------
    # Define ROS features
    # --------------------------------------------

    # create publisher and subscriber objects
    def init_publishers_and_subscribers(self):

        # Create a publisher to publish goal positions
        self.goal_pose_publisher = rospy.Publisher('/command/goal_pose', TwistStamped, queue_size=1)

        # Create a publisher to publish the desired applied force
        self.desired_force_publisher = rospy.Publisher('/command/desired_force', WrenchStamped, queue_size=1)

        # Create a publisher to publish a stop-motion command
        self.stop_motion_publisher = rospy.Publisher('/command/stop_motion', Bool, queue_size=1)

        # Create a publisher to publish the status of the image centering motions
        self.active_image_centering_status_publisher = rospy.Publisher('/command/center_image', Bool, queue_size=1)

        # Create a subscriber to check if the thyroid is in the image
        self.is_thyroid_in_image_status_subscriber = rospy.Subscriber('/status/thyroid_shown', Bool,
                                                                      self.is_thyroid_in_image_status_callback)

        # Create a subscriber to check if the image is centered
        self.is_thyroid_centered_status_subscriber = rospy.Subscriber('/status/thyroid_centered', Bool,
                                                                      self.is_thyroid_centered_status_callback)

        # Create a subscriber to check if the last goal sent was reached
        self.was_last_goal_reached_status_subscriber = rospy.Subscriber('/status/goal_reached', Bool,
                                                                        self.was_last_goal_reached_status_callback)

    # --------------------------------------------
    # Helper functions
    # --------------------------------------------


if __name__ == '__main__':

    # create motion_controller object and start up ROS objects
    controller = HighLevelController()

    # --------------------------------------------
    # Define state machine parameters
    # --------------------------------------------

    # initialize status of procedure
    procedure_complete_flag = False

    # initialize the state of the procedure
    procedure_state = CHECK_SETUP  # beginning of procedure
    previous_procedure_state = None

    # initialize flag indicating if in waypoint finding or scanning portion of procedure
    current_objective = WAYPOINT_FINDING

    # intialize direction of scanning
    current_direction = DIRECTION_TORSO

    # --------------------------------------------
    # Define variables to store results from procedure
    # --------------------------------------------

    # initialize empty stored sate for start of procedure
    procedure_origin = array([])

    # initialize empty array to store the path waypoints
    procedure_waypoints = []

    # Set rate for publishing new velocities
    rate = rospy.Rate(100)  # hz

    # loop until the routine has been finished or interrupted
    while not rospy.is_shutdown() and not procedure_complete_flag:

        if procedure_state == CHECK_SETUP:

            # check that the thyroid is in the image
            if not controller.thyroid_in_image_status:
                procedure_complete_flag = True

            else:
                print("pass")
                # change states to center the thyroid in the image
                previous_procedure_state = procedure_state
                procedure_state = CENTER_IMAGE

        if procedure_state == CENTER_IMAGE:

            # if the image is not centered
            if not controller.is_thyroid_centered:

                # Publish that the image needs to be centered
                controller.active_image_centering_status_publisher.publish(Bool(True))

            else:

                # Publish that active image centering is no longer needed
                controller.active_image_centering_status_publisher.publish(Bool(False))

                # if the list of procedure_waypoints list is empty, save the current position as the origin
                if len(procedure_waypoints) == 0:
                    procedure_origin = controller.cartesian_position_history[0][1]

                # set placement index based on direction of movement
                if current_direction == DIRECTION_HEAD:
                    placement_index = -1
                elif current_direction == DIRECTION_TORSO:
                    placement_index = 0
                else:
                    placement_index = 0

                # save the current position as a path waypoint to follow in the future
                procedure_waypoints.insert(placement_index, controller.cartesian_position_history[0][1])

                # set the next state of the procedure
                previous_procedure_state = procedure_state
                procedure_state = SET_GOAL

        if procedure_state == SET_GOAL:

            # if the current objective of the procedure is to find waypoints,
            # set goal point as offset from current position
            if current_objective == WAYPOINT_FINDING:

                # set direction of offset based on current direction of motion
                x_offset = controller.standard_scan_step * current_direction

                # calculate new goal position
                controller.move_goal = controller.cartesian_position_history[0][1] + x_offset

            elif current_objective == SCANNING:

                # check to make sure there are more waypoints to travel to
                if len(procedure_waypoints) > 0:

                    # pop out the last waypoint as the goal position
                    controller.move_goal = procedure_waypoints.pop(-1)

                else:

                    # exit the procedure
                    current_objective = EXITING
                    controller.move_goal = procedure_origin
            else:
                # set the new goal position as the origin
                controller.move_goal = procedure_origin

            # set the next state of the procedure
            previous_procedure_state = procedure_state
            procedure_state = MOVE_TO_GOAL

        if procedure_state == MOVE_TO_GOAL:

            # if the current goal has not been reached
            if not controller.was_last_goal_position_reached:

                # generate a new pose message
                new_pose = TwistStamped()

                # fill out message with goal point
                new_pose.twist.linear.x = controller.move_goal[0]
                new_pose.twist.linear.y = controller.move_goal[1]
                new_pose.twist.linear.z = controller.move_goal[2]

                # Publish the goal pose
                controller.goal_pose_publisher.publish(TwistStamped())

            else:

                # Publish a message telling the robot to stop moving
                controller.stop_motion_publisher.publish(Bool(True))

                # save the previous state
                previous_procedure_state = previous_procedure_state

                # if procedure is complete
                if current_objective == EXITING:
                    procedure_state = EXIT_PROCEDURE

                # if finding waypoints
                if current_objective == WAYPOINT_FINDING:
                    procedure_state = CHECK_FOR_THYROID

                # if scanning for images
                if current_objective == SCANNING:
                    procedure_state = SAVE_IMAGE

        if procedure_state == CHECK_FOR_THYROID:

            # if the thyroid is in the image, center the image on the thyroid
            if controller.thyroid_in_image_status:

                previous_procedure_state = procedure_state
                procedure_state = CENTER_IMAGE

            else:

                # save the previous procedure state
                previous_procedure_state = procedure_state

                # if looking for waypoints and heading towards the torso,
                # reverse direction and head back to the origin
                if current_objective == WAYPOINT_FINDING and current_direction == DIRECTION_TORSO:
                    current_direction = DIRECTION_HEAD
                    procedure_state = MOVE_TO_GOAL

                    # set the next movement goal as the origin
                    controller.move_goal = procedure_origin + controller.standard_scan_step * current_direction

                # if looking for waypoints and heading towards the head,
                # switch to scanning at each discovered waypoint and reverse direction
                elif current_objective == WAYPOINT_FINDING and current_direction == DIRECTION_HEAD:
                    current_direction = DIRECTION_TORSO
                    current_objective = SCANNING
                    procedure_state = SET_GOAL

        if procedure_state == SAVE_IMAGE:
            # save the current image

            # save the current robot pose

            # save the image mask

            # save the approximate area of the thyroid in image

            previous_procedure_state = procedure_state
            procedure_state = SET_GOAL

        if procedure_state == EXIT_PROCEDURE:
            procedure_complete_flag = True
