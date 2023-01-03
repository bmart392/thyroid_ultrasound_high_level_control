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
        self.thyroid_shown = False
        self.thyroid_centered = False
        self.goal_reached = False
        self.current_pose = zeros(7)
        # self.desired_end_effector_force = 0.1  # N
        # self.allowable_centroid_error = .1
        # self.acceptable_cartesian_error = .1
        self.standard_scan_step = array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.move_goal = None

        # initialize ros node
        rospy.init_node('high_level_controller')

        # Status subscribers
        self.thyroid_shown_subscriber = rospy.Subscriber('/status/thyroid_shown', Bool, self.thyroid_shown_callback)
        self.thyroid_centered_subscriber = rospy.Subscriber('/status/thyroid_centered', Bool,
                                                            self.thyroid_centered_callback)
        self.goal_reached_subscriber = rospy.Subscriber('/status/goal_reached', Bool, self.goal_reached_callback)
        self.current_pose_subscriber = rospy.Subscriber('/status/current_pose', PoseStamped, self.current_pose_callback)

        # Goal publishers
        self.goal_pose_publisher = rospy.Publisher('/goal/pose', PoseStamped, queue_size=1)
        self.goal_force_publisher = rospy.Publisher('/goal/force', WrenchStamped, queue_size=1)
        
        # Command Publishers
        self.stop_motion_publisher = rospy.Publisher('/command/stop_motion', Bool, queue_size=1)
        self.center_image_publisher = rospy.Publisher('/command/center_image', Bool, queue_size=1)
        self.move_to_goal_publisher = rospy.Publisher('/command/move_to_goal', Bool, queue_size=1)
        self.use_force_feedback_publisher = rospy.Publisher('/command/use_force_feedback', Bool, queue_size=1)
        self.filter_images_publisher = rospy.Publisher('/command/filter_images', Bool, queue_size=1)

        # self.velocity_publisher = None
        # self.centroid_error_subscriber = None
        # self.thyroid_in_image_subscriber = None
        # self.robot_state_subscriber = None
        # self.force_readings_subscriber = None

    # --------------------------------------------
    # Callback Functions
    # --------------------------------------------

    # capture if the thyroid is in the current image
    def thyroid_shown_callback(self, data: Bool):
        self.thyroid_shown = data.data

    # capture if the thyroid is centered in the image
    def thyroid_centered_callback(self, data: Bool):
        self.thyroid_centered = data.data

    # capture the status of the motion to reach the last sent goal point
    def goal_reached_callback(self, data: Bool):
        self.goal_reached = data.data

    def current_pose_callback(self, data: PoseStamped):
        self.current_pose[0] = data.pose.position.x
        self.current_pose[1] = data.pose.position.y
        self.current_pose[2] = data.pose.position.z
        self.current_pose[3] = data.pose.orientation.x
        self.current_pose[4] = data.pose.orientation.y
        self.current_pose[5] = data.pose.orientation.z
        self.current_pose[6] = data.pose.orientation.w


if __name__ == '__main__':

    # create motion_controller object and start up ROS objects
    controller = HighLevelController()

    print("Node Initialized. Press CTRL+C to terminate.")
    print("Waiting for other nodes to start.")

    # wait for other nodes to start and send messages
    rospy.sleep(5)

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

    # initialize direction of scanning
    current_direction = DIRECTION_TORSO

    # --------------------------------------------
    # Define variables to store results from procedure
    # --------------------------------------------

    # initialize empty stored state for start of procedure
    procedure_origin = array([])

    # initialize empty array to store the path waypoints
    procedure_waypoints = []

    # save the current waypoint the robot is at
    current_waypoint = None

    # Set rate for publishing new velocities
    rate = rospy.Rate(100)  # hz

    # loop until the routine has been finished or interrupted
    while not rospy.is_shutdown() and not procedure_complete_flag:

        if procedure_state == CHECK_SETUP:

            # check that the thyroid is in the image
            if not controller.thyroid_shown:
                "Procedure is not ready to start. Thyroid is not shown."
                procedure_complete_flag = True

            else:
                print("Procedure is ready to start.")
                # change states to center the thyroid in the image
                previous_procedure_state = procedure_state
                procedure_state = CENTER_IMAGE

        if procedure_state == CENTER_IMAGE:

            # if the image is not centered
            if not controller.thyroid_centered:
                # Publish that the image needs to be centered
                controller.center_image_publisher.publish(Bool(True))

            else:

                # Publish that active image centering is no longer needed
                controller.center_image_publisher.publish(Bool(False))

                # if the list of procedure_waypoints list is empty, save the current position as the origin
                if len(procedure_waypoints) == 0:
                    procedure_origin = controller.current_pose  # controller.cartesian_position_history[0][1]

                # set placement index based on direction of movement
                if current_direction == DIRECTION_HEAD:
                    placement_index = -1
                elif current_direction == DIRECTION_TORSO:
                    placement_index = 0
                else:
                    placement_index = 0

                # save the current position as a path waypoint to follow in the future
                procedure_waypoints.insert(placement_index, controller.current_pose)

                # save the current waypoint of the robot
                current_waypoint = controller.current_pose

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
                controller.move_goal = current_waypoint + x_offset

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
            if not controller.goal_reached:

                # generate a new pose message
                new_pose = PoseStamped()

                # fill out message with goal point
                new_pose.pose.position.x = controller.move_goal[0]
                new_pose.pose.position.y = controller.move_goal[1]
                new_pose.pose.position.z = controller.move_goal[2]

                # Publish the goal pose
                controller.goal_pose_publisher.publish(new_pose)

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
            if controller.thyroid_shown:

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
