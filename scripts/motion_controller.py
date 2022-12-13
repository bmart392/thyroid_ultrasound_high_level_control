#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, WrenchStamped
# from franka_msgs import FrankaState
# from rv_msgs.msg import ManipulatorState
from std_msgs.msg import Float64, String, Bool
from numpy import sign, array, zeros, sqrt, sum
from motion_constants import *
from motion_helpers import *

class motion_controller:
    def __init__(self) -> None:
        self.cartesian_position_history = []
        self.image_centroid_error_history = []
        self.external_force_history = []
        self.thyroid_in_image_status = False
        self.desired_end_effector_force = 0.1  # N
        self.allowable_centroid_error = .1
        self.acceptable_cartesian_error = .1
        self.standard_scan_step = array([0.8, 0.0, 0.0])
        self.move_goal = None
        self.velocity_publisher = None
        self.centroid_error_subscriber = None
        self.thyroid_in_image_subscriber = None
        self.robot_state_subscriber = None
        self.force_readings_subscriber = None

        # initialize ros node
        rospy.init_node('motion_controller')
        
        # create publishers and subscribers
        self.init_publishers_and_subscirbers()

    #--------------------------------------------
    # Callback Functions
    #--------------------------------------------

    # capture the cartesian position of the robot
    def cartesian_position_callback(data: FrankaState, self):
                
        # limit the number of previous values stored to 5
        if len(self.cartesian_position_history) >= 5:
            self.cartesian_position_history.pop()

        # add the new value to the list
        self.cartesian_position_history.insert(0,
            (data.header.stamp.sec,
            array([data.O_T_EE[3],
            data.O_T_EE[7],
            data.O_T_EE[11]]))
            )

    # capture the error of the position of image centroid
    def centroid_error_callback(data: Float64, self):

        # remove oldest error value if more 5 have already been saved
        if len(self.image_centroid_error_history) >= 5:
            self.image_centroid_error_history.pop()
        
        # save the new error value
        self.image_centroid_error_history.append((0, array([0, data.data, 0])))

    # capture the current force felt by the robot
    def force_value_callback(data: WrenchStamped, self):

        # remove the oldest force value if more than 5 have already been saved
        if len(self.external_force_history) >= 5:
            self.external_force_history.pop()
        
        # save the new error value
        self.external_force_history.insert((data.head.time.sec, array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])))

    # capture if the thyroid is in the current image
    def thyroid_in_image_callback(data: Bool, self):
        self.thyroid_in_image_status = data.data
    
    #--------------------------------------------
    # Control Input Calculation Functions
    #--------------------------------------------

    # calculate the control input based on the image error
    def image_error_calculate_control_input(self):
        k_p = .1   
        min_control_speed = 0.001 # m/s
        max_control_speed = 0.025 # m/s
        error = self.image_centroid_error_history[0]
        return pd_controller(k_p, 0, error, 0, min_control_speed, max_control_speed)

    # calculate the control input based on the force error
    def force_error_calculate_control_input(self):
        k_p = .1
        k_d = 0
        min_control_speed = .001  # m/s
        max_control_speed = .025  # m/s
        error, error_dot = calculate_error(self.desired_end_effector_force, self.external_force_history)
        return pd_controller(k_p, k_d, error, error_dot, min_control_speed, max_control_speed)

    # calculate the control input based on the cartesian position error
    def cartesian_position_calculate_control_input(self):
        k_p = .1
        k_d = 0.
        min_control_speed = .001  # m/s
        max_control_speed = .025  # m/s
        error, error_dot  = calculate_error(self.move_goal, self.cartesian_position_history)
        return pd_controller(k_p, k_d, error, error_dot, min_control_speed, max_control_speed)

    #--------------------------------------------
    # Define ROS features
    #--------------------------------------------
    
    # create publisher and subscriber objects
    def init_publishers_and_subscirbers(self):
        # Create the publisher to publish the desired joint velocities
        self.velocity_publisher = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)

        # Create a subscriber to listen to the error gathered from ultrasound images
        self.centroid_error_subscriber = rospy.Subscriber('/image_data/centroid_error', Float64, self.centroid_error_callback)

        # Create a subscriber to listen to the error gathered from ultrasound images
        self.thyroid_in_image_subscriber = rospy.Subscriber('/image_data/thyroid_in_image', Bool, self.thyroid_in_image_callback)

        # Create a subscriber to listen to the robot state
        self.robot_state_subscriber = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.cartesian_position_callback)

        # Create a subscriber to listen to the force readings from the robot
        self.force_readings_subscriber = rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.force_value_callback)

if __name__ == '__main__':

    # create motion_controller object and start up ROS objects
    controller = motion_controller()

    #--------------------------------------------
    # Define state machine parameters
    #--------------------------------------------

    # initialize status of procedure
    procedure_complete_flag = False

    # initialize the state of the procedure
    procedure_state = CHECK_SETUP  # beginning of procedure
    previous_procedure_state = None

    # initialize flag indicating if in waypoint finding or scanning portion of procedure
    current_objective = WAYPOINT_FINDING

    # intialize direction of scanning
    current_direction = DIRECTION_TORSO

    #--------------------------------------------
    # Define variables to store results from procedure
    #--------------------------------------------

    # initialize empty stored sate for start of procedure
    procedure_origin = array([])

    # initialize empty array to store the path waypoints
    procedure_waypoints = []

    # Set rate for publishing new velocities
    rate = rospy.Rate(100) #hz

    # loop until the routine has been finished or interrupted
    while not rospy.is_shutdown() and not procedure_complete_flag:

        if procedure_state == CHECK_SETUP:

            # check that the thyroid is in the image
            if not controller.thyroid_in_image_status:
                rospy.shutdown()

            # change states to center the thyroid in the image
            previous_procedure_state = procedure_state
            procedure_state = CENTER_IMAGE
            pass
            """# Create a velocity message that will instruct the robot to
            # move in the direction of the control inputs.
            velocity = TwistStamped()
            velocity.twist.linear.x = control_input_x
            velocity.twist.linear.y = control_input_y
            velocity.twist.linear.z = control_input_z

            # Publish the velocity message to the Panda driver at a
            # frequency of 100Hz
            velocity_publisher.publish(velocity)

            # if the control input from the image based error is not 0
            if control_input_x + control_input_y + control_input_z == 0:
                
                # save current position as origin of the procedure
                procedure_origin = cartesian_position_history

                # set flag indicating first time through moving
                first_pass_move_to_next_scan = True

                # set the next state
                procedure_state = MOVE_TO_NEXT_SCAN"""

        if procedure_state == CENTER_IMAGE:

            # check if the thyroid is in the image and if not act accordingly
            if controller.thyroid_in_image_status:
            
                # move the robot to center the centroid within the allowable error
                if abs(controller.image_centroid_error_history[0]) > controller.allowable_centroid_error:

                    # calculate required control input
                    centroid_control_inputs = controller.image_error_calculate_control_input()

                    # calculate force control inputs
                    force_control_inputs = controller.force_error_calculate_control_input()

                    # generate a message to use to send control inputs
                    centroid_correction_velocity = TwistStamped()

                    # assign values to the message
                    centroid_correction_velocity.twist.linear.x = centroid_control_inputs[0] + force_control_inputs[0]
                    centroid_correction_velocity.twist.linear.y = centroid_control_inputs[1] + force_control_inputs[1]
                    centroid_correction_velocity.twist.linear.z = centroid_control_inputs[2] + force_control_inputs[2]

                    # publish the message
                    controller.velocity_publisher.publish(centroid_correction_velocity)

                else:
                    
                    # if the list of procedure_waypoints list is empty, save the current position as the origin
                    if len(procedure_waypoints) == 0:
                        procedure_origin = controller.cartesian_position_history[0][1]

                    # set placement index based on direction of movement
                    if current_direction == DIRECTION_HEAD:
                        placement_index = -1
                    elif current_direction == DIRECTION_TORSO:
                        placement_index == 0
                    else:
                        placement_index == 0

                    # save the current position as a path waypoint to follow in the future
                    procedure_waypoints.insert(placement_index, controller.cartesian_position_history[0][1])

                    # set the next state of the procedure
                    previous_procedure_state = procedure_state
                    procedure_state = MOVE_TO_NEXT_SCAN
            
            else:

                # check if all waypoints have been found
                if current_objective == WAYPOINT_FINDING and current_direction == DIRECTION_TORSO:
                    current_direction = DIRECTION_HEAD
                    previous_procedure_state = procedure_state
                    procedure_state = MOVE_TO_ORIGIN

        if procedure_state == MOVE_TO_NEXT_SCAN:
            
            if first_pass_move_to_next_scan:

                # calculate goal origin to be approximately 2 inches from the current pose 
                # based on the desired direction
                if current_direction == DIRECTION_TORSO:
                    x_offset = controller.standard_scan_step
                elif current_direction == DIRECTION_HEAD:
                    x_offset = -1 * controller.standard_scan_step
                else:
                    x_offset = zeros(3)

                controller.move_goal = controller.cartesian_position_history[0][1] + x_offset

                # set control inputs to zero
                velocity = TwistStamped()

                # reset the flag
                first_pass_move_to_next_scan = False
            
            else:

                if sqrt(sum(controller.move_goal - controller.cartesian_position_history[0] ** 2)) > controller.acceptable_cartesian_error: 
                
                    # create a message to send control velocities
                    velocity = TwistStamped()

                    # calculate positional control inputs
                    positional_control_inputs = controller.cartesian_position_calculate_control_input()
                    
                    # calculate force control inputs = calculate_force_control_inputs
                    force_control_inputs = controller.force_error_calculate_control_input()

                    velocity.twist.linear.x = positional_control_inputs[0] + force_control_inputs[0]
                    velocity.twist.linear.y = positional_control_inputs[1] + force_control_inputs[1]
                    velocity.twist.linear.z = positional_control_inputs[2] + force_control_inputs[2]

                else:

                    # create an empty message
                    velocity = TwistStamped()

                    previous_procedure_state = procedure_state
                    procedure_state = CENTER_IMAGE
            
            # Publish the velocity message
            controller.velocity_publisher.publish(velocity)

        if procedure_state == MOVE_TO_ORIGIN:
            pass

        if procedure_state == FOLLOW_WAYPOINTS:
            
            # check if a new waypoint needs to be travelled to
            if reach_new_waypoint:
                controller.move_goal = procedure_waypoints.pop(0)
            



        # Program starts execution - robot is placed in the right spot
        # CHECK SETUP - robot checks if thyroid is in place
        # IMAGE_CENTERING - robot  centers itself based on image error to within threshold
        # SET ORIGIN - robot sets current position as origin
        # ADD WAYPOINT - robot adds the current position as a waypoint
        # SET GOAL - robot sets new goal as standard offset from current position in current direction
        # MOVE TO GOAL - robot moves to goal
        # CHECK FOR THYROID - check if the thyroid is in the image, change direction accordingly
        # IMAGE CENTERING - robot auto centers itself
        # ADD WAYPOINT - robot adds the current position as a waypoint
        # SET GOAL - robot sets next goal
        # MOVE TO GOAL - robot moves to goal
        # CHECK FOR THYROID (no thyroid this time) - check if the thyroid is in the image, change direction of motion
        # MOVE TO ORIGIN - move back to the starting point
        # SET GOAL - robot sets next goal
        # MOVE TO GOAL - robot moves to goal
        # CHECK FOR THYROID - check if the thyroid is in the image, change current objective accordingly
        # IMAGE CENTERING - robot auto centers itself
        # ADD WAYPOINT - robot adds the current position as a waypoint
        # SET GOAL - robot sets next goal
        # CHECK FOR THYROID (no thyroid this time) - check if the thyroid is in the image, change current objective to scanning
        # MOVE TO WAYPOINT - pop off last waypoint and go to it
        # SAVE IMAGE SCAN - save image at current point
        # MOVE TO WAYPOINT - pop off last waypoint and go to it
        # SAVE IMAGE SCAN - save image at current point
        # MOVE TO WAYPOINT (no more waypoints) - go to end of procedure state
        # END OF PROCEDURE - move back to origin, save data, save logs, exit
        # 

    velocity = TwistStamped()
    velocity_publisher.publish(velocity)