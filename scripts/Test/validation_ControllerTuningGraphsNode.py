"""
Contains the code for validating the ControllerTuningGraphsNode.
"""

# Import standard python packages
from csv import DictReader
from std_msgs.msg import Header

# Import standard ROS packages

# Import custom python packages

# Import custom ROS packages
from ControllerTuningGraphsNode import ControllerTuningGraphsNode, Rate, WrenchStamped, Float64, Float64Stamped, \
    TwistStamped, Time

#####################
# Prep the validation
# region

# Create the node
test_node = ControllerTuningGraphsNode()

# Define a rospy rate to control how fast data is loaded into the node
looping_rate = Rate(30)  # Hz

# Define constants for the headers used in the CSV file
TIME = 'Time'
DERIVED_FORCE = 'DerivedForce'
FORCE_SET_POINT = 'ForceSetPoint'
POSITION_ERROR = 'PositionError'
IMAGE_ERROR = 'ImageError'
PATIENT_CONTACT_ERROR = 'PatientContactError'

# Define lists for each value
time_list = []
derived_force_list = []
force_set_point_list = []
position_error_list = []
image_error_list = []
patient_contact_error_list = []

# Load data from a csv file into a set of equal size arrays for each parameter
source_file = open('/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_high_level_control/scripts'
                   '/Test/CSV/TestFakeData.csv',
                   mode='r')
source_file_reader = DictReader(source_file)
for row in source_file_reader:
    time_list.append(float(row[TIME]))
    derived_force_list.append(float(row[DERIVED_FORCE]))
    force_set_point_list.append(float(row[FORCE_SET_POINT]))
    position_error_list.append(float(row[POSITION_ERROR]))
    image_error_list.append(float(row[IMAGE_ERROR]))
    patient_contact_error_list.append(float(row[PATIENT_CONTACT_ERROR]))

# endregion
#####################

# For each timestep and value,
for time, derived_force, force_set_point, position_error, image_error, patient_contact_error in zip(
        time_list, derived_force_list, force_set_point_list, position_error_list, image_error_list,
        patient_contact_error_list
):
    # Get the current time stamp
    time_stamp = Time.now()

    # Create the appropriate message for each parameter
    # and send each to the node
    derived_force_msg = WrenchStamped()
    derived_force_msg.header.stamp = time_stamp
    derived_force_msg.wrench.force.z = derived_force
    test_node.robot_force_callback(derived_force_msg)
    test_node.robot_force_set_point_callback(Float64(data=force_set_point))

    position_error_msg = Float64Stamped()
    position_error_msg.header.stamp = time_stamp
    position_error_msg.data.data = position_error
    test_node.position_error_callback(position_error_msg)

    image_error_msg = TwistStamped()
    image_error_msg.header.stamp = time_stamp
    image_error_msg.twist.linear.x = image_error
    test_node.image_error_callback(image_error_msg)

    patient_contact_error_msg = Float64Stamped()
    patient_contact_error_msg.header.stamp = time_stamp
    patient_contact_error_msg.data.data = patient_contact_error
    test_node.patient_contact_callback(position_error_msg)

    # Wait
    looping_rate.sleep()
