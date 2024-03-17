# Import standard python packages
from csv import DictReader
from matplotlib.pyplot import show, subplots, rc, figure
from numpy import array
from statistics import median, stdev, mean

# Import custom python packages
from ExperimentalDataRecorder import MESSAGE_ID, STAMP_SECS, STAMP_NSECS, POSE_X, POSE_Y, POSE_Z, \
    POSE_ROLL, POSE_PITCH, POSE_YAW, WAYPOINT_REACHED
from thyroid_ultrasound_robot_control_support.Helpers.calc_inverse import calc_inverse
from thyroid_ultrasound_robot_control_support.Helpers.calc_straight_line_distance import calc_straight_line_distance
from thyroid_ultrasound_robot_control_support.Helpers.calc_transformation_from_rpy import calc_transformation_from_rpy

# Set the default size for plot elements
rc('font', size=10)  # controls default text sizes
rc('axes', titlesize=14)  # font size of the axes title
rc('axes', labelsize=10)  # font size of the x and y labels
rc('xtick', labelsize=6)  # font size of the tick labels
rc('ytick', labelsize=6)  # font size of the tick labels
rc('legend', fontsize=10)  # legend font size
rc('figure', titlesize=24)  # font size of the figure title
rc('figure', dpi=300)  # Set the DPI of the figure

# Define helpful constants
PROBE_X_DISTANCE: str = 'Probe X Position w.r.t.\nStarting Pose (m)'
PROBE_Y_DISTANCE: str = 'Probe Y Position w.r.t.\nStarting Pose (m)'
PROBE_Z_DISTANCE: str = 'Probe Z Position w.r.t.\nStarting Pose (m)'
STRAIGHT_LINE_DISTANCE: str = 'Straight Line Distance (m)'
COMBINED_STAMP: str = 'Elapsed Time (s)'

# Define the source of the information
time_stamps = ['2024-02-19_13-29-32-391129']
line_colors = []

time_stamp = '2024-03-09_18-36-25-368610'
POSE_FILE_PATH: str = '/home/ben/thyroid_ultrasound_data/experimentation/PoseControl' \
                      '/' + time_stamp + '_experiment/Pose_' + time_stamp + '.csv'

# Create a figure and axes to plot the results
fig = figure()
axes_grid_space = fig.add_gridspec(3, 2, wspace=1.)

# Define a counter to determine which plot to plot on
axis_counter = 0

# Title the figure
# fig.suptitle('Force Control Experiment')

# Define a temporary dictionary to store each result
temp_results_dict = {MESSAGE_ID: [], COMBINED_STAMP: [], POSE_X: [], POSE_Y: [], POSE_Z: [],
                     POSE_ROLL: [], POSE_PITCH: [], POSE_YAW: [], WAYPOINT_REACHED: []}
# Define final dictionary to store each result
final_results_dict = {MESSAGE_ID: [], COMBINED_STAMP: [], POSE_X: [], POSE_Y: [], POSE_Z: [],
                      POSE_ROLL: [], POSE_PITCH: [], POSE_YAW: [],
                      STRAIGHT_LINE_DISTANCE: [], PROBE_X_DISTANCE: [], PROBE_Y_DISTANCE: [], PROBE_Z_DISTANCE: []}
waypoints = {COMBINED_STAMP: [], POSE_X: []}
# Open the file
source_file = open(POSE_FILE_PATH, mode='r')
source_file_reader = DictReader(source_file)

# Create variables to store the starting pose of the test
first_pass = True
starting_pose = array([])

# Read in the data
for row in source_file_reader:

    # Capture the starting pose if it is the first time through the loop
    if first_pass:
        starting_pose = calc_transformation_from_rpy(xyz=(float(row[POSE_X]), float(row[POSE_Y]), float(row[POSE_Z])),
                                                     rpy=(float(row[POSE_ROLL]), float(row[POSE_PITCH]), float(row[POSE_YAW])))
        first_pass = False
    # Pull the data from the test
    temp_results_dict[MESSAGE_ID].append(int(row[MESSAGE_ID]))
    temp_results_dict[COMBINED_STAMP].append((int(row[STAMP_SECS])) + (int(row[STAMP_NSECS])) / 10 ** 9)
    temp_results_dict[POSE_X].append(float(row[POSE_X]))
    temp_results_dict[POSE_Y].append(float(row[POSE_Y]))
    temp_results_dict[POSE_Z].append(float(row[POSE_Z]))
    temp_results_dict[POSE_ROLL].append(float(row[POSE_ROLL]))
    temp_results_dict[POSE_PITCH].append(float(row[POSE_PITCH]))
    temp_results_dict[POSE_YAW].append(float(row[POSE_YAW]))
    temp_results_dict[WAYPOINT_REACHED].append(bool(int(row[WAYPOINT_REACHED])))

# Find the smallest values from the data
min_stamp = min(temp_results_dict[COMBINED_STAMP])

# Calculate the inverse matrix of the starting pose
inverse_starting_pose = calc_inverse(starting_pose)

# Improve the data for graphing
for message_id, seconds_stamp, pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw, waypoint_reached in zip(
        temp_results_dict[MESSAGE_ID],
        temp_results_dict[COMBINED_STAMP],
        temp_results_dict[POSE_X],
        temp_results_dict[POSE_Y],
        temp_results_dict[POSE_Z],
        temp_results_dict[POSE_ROLL],
        temp_results_dict[POSE_PITCH],
        temp_results_dict[POSE_YAW],
        temp_results_dict[WAYPOINT_REACHED]):
    # Pull the data from the previous array
    final_results_dict[MESSAGE_ID].append(message_id)
    final_results_dict[COMBINED_STAMP].append(seconds_stamp - min_stamp)
    final_results_dict[POSE_X].append(pose_x)
    final_results_dict[POSE_Y].append(pose_y)
    final_results_dict[POSE_Z].append(pose_z)
    final_results_dict[POSE_ROLL].append(pose_roll)
    final_results_dict[POSE_PITCH].append(pose_pitch)
    final_results_dict[POSE_YAW].append(pose_yaw)
    final_results_dict[STRAIGHT_LINE_DISTANCE].append(calc_straight_line_distance((starting_pose[0][3],
                                                                              starting_pose[1][3],
                                                                              starting_pose[2][3]),
                                                                             (pose_x, pose_y, pose_z)))

    # Convert the current pose to a transformation matrix
    current_pose = calc_transformation_from_rpy(xyz=(pose_x, pose_y, pose_z),
                                                rpy=(-pose_roll, pose_pitch, pose_yaw))

    # Calculate the current pose in terms of the starting pose
    position = inverse_starting_pose @ array([[current_pose[0, 3]],
                                              [current_pose[1, 3]],
                                              [current_pose[2, 3]],
                                              [1]])

    # Pull out the distance in X from the starting pose
    final_results_dict[PROBE_X_DISTANCE].append(abs(position[0][0]))
    final_results_dict[PROBE_Y_DISTANCE].append(abs(position[1][0]))
    final_results_dict[PROBE_Z_DISTANCE].append(abs(position[2][0]))

    if waypoint_reached:
        waypoints[COMBINED_STAMP].append(seconds_stamp - min_stamp)
        waypoints[POSE_X].append(abs(position[0][0]))

distance_between_waypoints = [waypoints[POSE_X][ii] - waypoints[POSE_X][ii - 1] for ii in range(2, len(waypoints[POSE_X]))]
clean_distance_between_waypoints = []
for distance in distance_between_waypoints:
    if abs(distance) > 10 ** -4:
        clean_distance_between_waypoints.append(distance)
print(mean(clean_distance_between_waypoints))
print(median(clean_distance_between_waypoints))
print(stdev(clean_distance_between_waypoints))

# Define the sampling rate for the points included in the plot
point_sampling_rate = 2

# Define variables for convenience
common_x = final_results_dict[COMBINED_STAMP][::point_sampling_rate]

# axes.set_title("No Force Control")
# ax_0 = fig.add_subplot(axes_grid_space[0, 0:2])
# ax_0.scatter(common_x, final_results_dict[POSE_X][::point_sampling_rate], s=0.5, c='r')
# ax_0.set_xlabel(COMBINED_STAMP)
# ax_0.set_ylabel('Probe X Position\nw.r.t. Origin (m)')
# ax_0.grid(axis='y')
#
# ax_1 = fig.add_subplot(axes_grid_space[0, 2:4])
# ax_1.scatter(common_x, final_results_dict[POSE_Y][::point_sampling_rate], s=0.5, c='r')
# ax_1.set_xlabel(COMBINED_STAMP)
# ax_1.set_ylabel('Probe Y Position\nw.r.t. Origin (m)')
# ax_1.grid(axis='y')
#
# ax_2 = fig.add_subplot(axes_grid_space[0, 4:6])
# ax_2.scatter(common_x, final_results_dict[POSE_Z][::point_sampling_rate], s=0.5, c='r')
# ax_2.set_xlabel(COMBINED_STAMP)
# ax_2.set_ylabel('Probe Z Position\nw.r.t. Origin (m)')
# ax_2.grid(axis='y')


# ax_3 = fig.add_subplot(axes_grid_space[1, 0:3])
# ax_3.scatter(common_x, final_results_dict[STRAIGHT_LINE_DISTANCE][::point_sampling_rate], s=0.5, c='r')
# ax_3.set_xlabel(COMBINED_STAMP)
# ax_3.set_ylabel(STRAIGHT_LINE_DISTANCE)
# ax_3.grid(axis='y')

ax_4 = fig.add_subplot(axes_grid_space[:, :])
ax_4.scatter(common_x, final_results_dict[PROBE_X_DISTANCE][::point_sampling_rate],
             s=0.5, c='red', label='X Axis')
ax_4.scatter(common_x, final_results_dict[PROBE_Y_DISTANCE][::point_sampling_rate],
             s=0.5, c='green', label='Y Axis')
ax_4.scatter(common_x, final_results_dict[PROBE_Z_DISTANCE][::point_sampling_rate],
             s=0.5, c='blue', label='Z Axis')
ax_4.legend()
ax_4.scatter(waypoints[COMBINED_STAMP], waypoints[POSE_X], s=6, c='black', marker="D")
ax_4.set_xlabel(COMBINED_STAMP)
ax_4.set_ylabel("Distance Travelled w.r.t. Initial Pose (m)")
ax_4.grid(axis='y')


# axes[0][0].scatter(common_x, final_results_dict[POSE_X][::point_sampling_rate], s=0.5, c='r')
# axes[0][0].set_xlabel(STAMP_SECS)
# axes[0][0].set_ylabel('Robot Position w.r.t. Origin (m)')
# axes[0][0].grid(axis='y')
# axes[0][0].scatter(common_x, final_results_dict[POSE_Y][::point_sampling_rate], s=0.5, c='r')
# axes[0][1].set_xlabel(STAMP_SECS)
# axes[0][1].set_ylabel(POSE_Y)
# axes[0][1].grid(axis='y')
# axes[1][0].scatter(common_x, final_results_dict[POSE_Z][::point_sampling_rate], s=0.5, c='r')
# axes[1][0].set_xlabel(STAMP_SECS)
# axes[1][0].set_ylabel(POSE_Z)
# axes[1][0].grid(axis='y')
# axes[1][1].scatter(common_x, final_results_dict[PROBE_X_DISTANCE][::point_sampling_rate], s=0.5, c='r')
# axes[1][1].set_xlabel(PROBE_X_DISTANCE)
# axes[1][1].set_ylabel(POSE_Z)
# axes[1][1].grid(axis='y')

show()
