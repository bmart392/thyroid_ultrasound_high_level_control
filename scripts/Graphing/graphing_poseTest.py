# Import standard python packages
from csv import DictReader, DictWriter
from matplotlib.pyplot import show, subplots, rc
from numpy import array
from statistics import stdev, mean

# Import custom python packages
from ExperimentalDataRecorder import MESSAGE_ID, STAMP_SECS, STAMP_NSECS, \
    POSE_X, POSE_Y, POSE_Z, POSE_ROLL, POSE_PITCH, POSE_YAW, WAYPOINT_REACHED
from Graphing.read_recorded_data_csv import read_recorded_data_csv, COMBINED_STAMP, CONTROLLED, UNCONTROLLED, \
    FIG_WIDTH, FIG_HEIGHT
from thyroid_ultrasound_robot_control_support.Helpers.calc_inverse import calc_inverse
from thyroid_ultrasound_robot_control_support.Helpers.calc_transformation_from_rpy import calc_transformation_from_rpy

# ----------------------------------------------------------------------------------------------------------------------

# Define the timestamps of the files to use as sources
time_stamps = ['2024-03-09_18-36-25-368610',
               '2024-03-09_18-58-57-675893',
               '2024-03-09_19-00-06-682067']

# Select which file to plot
file_index_to_plot = 2

# ----------------------------------------------------------------------------------------------------------------------

# Set the default size for plot elements
rc('font', size=10)  # controls default text sizes
rc('axes', titlesize=14)  # font size of the axes title
rc('axes', labelsize=10)  # font size of the x and y labels
rc('xtick', labelsize=6)  # font size of the tick labels
rc('ytick', labelsize=6)  # font size of the tick labels
rc('legend', fontsize=10)  # legend font size
rc('figure', titlesize=24)  # font size of the figure title
rc('figure', dpi=300)  # Set the DPI of the figure

# Create a figure and axes to plot the results
fig, axes = subplots(nrows=1, ncols=1, figsize=(FIG_WIDTH, FIG_HEIGHT))

# ----------------------------------------------------------------------------------------------------------------------

# Define helpful constants
PROBE_X_DISTANCE: str = 'Probe X Distance w.r.t.\nStarting Pose (m)'
PROBE_Y_DISTANCE: str = 'Probe Y Distance w.r.t.\nStarting Pose (m)'
PROBE_Z_DISTANCE: str = 'Probe Z Distance w.r.t.\nStarting Pose (m)'

# Define constants to use in the results file
DESIRED_DISTANCE: str = 'Desired Distance (mm)'
DISTANCE_MEAN: str = 'Mean Distance (mm)'
DISTANCE_STDEV: str = 'Standard Deviation of Distance (mm)'
PITCH_ERROR: str = 'Error of Pose - Pitch (deg)'
PITCH_STDEV: str = 'Standard Deviation of Error of Pose - Pitch (deg)'
YAW_ERROR: str = 'Error of Pose - Yaw (deg)'
YAW_STDEV: str = 'Standard Deviation of Error of Pose - Yaw (deg)'
NUMBER_OF_DATA_POINTS: str = 'Number of Data Points'

# ----------------------------------------------------------------------------------------------------------------------
# Define final dictionary to store each result
final_results_dict = {COMBINED_STAMP: [], PROBE_X_DISTANCE: [], PROBE_Y_DISTANCE: [], PROBE_Z_DISTANCE: [],
                      WAYPOINT_REACHED + COMBINED_STAMP: [], WAYPOINT_REACHED + POSE_X: []}

# Define a flag to note that it is the first data file
is_first_data_file = True

# Open the results file
results_file = open('/home/ben/thyroid_ultrasound_data/experimentation/PoseControl/results.csv', mode='w')
results_file_writer = DictWriter(results_file, delimiter=',',
                                 fieldnames=[DESIRED_DISTANCE, DISTANCE_MEAN, DISTANCE_STDEV,
                                             PITCH_ERROR, PITCH_STDEV, YAW_ERROR, YAW_STDEV,
                                             NUMBER_OF_DATA_POINTS])
results_file_writer.writeheader()

# ----------------------------------------------------------------------------------------------------------------------

for file_index, time_stamp in enumerate(time_stamps):

    pose_file_path = '/home/ben/thyroid_ultrasound_data/experimentation/PoseControl' \
                     '/' + time_stamp + '_experiment/Pose_' + time_stamp + '.csv'

    # Determine if the data from this file should be saved
    save_data_to_plot = file_index == file_index_to_plot

    # Define a dictionary in which to store the result data
    waypoint_data = {PROBE_X_DISTANCE: [], POSE_PITCH: [], POSE_YAW: []}

    # Define variables to store information about the starting pose
    starting_pose = None
    inverse_starting_pose = None
    previous_distance = None
    pitch_at_start = None
    yaw_at_start = None

    # Read in the data from the source file
    data = read_recorded_data_csv(file_path=pose_file_path,
                                  headings=[MESSAGE_ID, STAMP_SECS, STAMP_NSECS,
                                            POSE_X, POSE_Y, POSE_Z,
                                            POSE_ROLL, POSE_PITCH, POSE_YAW,
                                            WAYPOINT_REACHED],
                                  sort_heading=MESSAGE_ID,
                                  additional_headings_to_zero=[STAMP_SECS])

    # For each data point,
    for i in range(len(data[WAYPOINT_REACHED])):

        # Convert the current pose to a transformation matrix
        this_pose = calc_transformation_from_rpy(xyz=(data[POSE_X][i], data[POSE_Y][i], data[POSE_Z][i]),
                                                 rpy=(-data[POSE_ROLL][i], data[POSE_PITCH][i], data[POSE_YAW][i]))

        try:
            # Calculate the current pose in terms of the starting pose
            position = inverse_starting_pose @ array([[this_pose[0, 3]], [this_pose[1, 3]], [this_pose[2, 3]], [1]])

            # Only add data to be graphed if it is the first data file
            if save_data_to_plot:
                # Update the final results dictionary
                final_results_dict[COMBINED_STAMP].append(data[COMBINED_STAMP][i])
                final_results_dict[PROBE_X_DISTANCE].append(abs(position[0][0]))
                final_results_dict[PROBE_Y_DISTANCE].append(abs(position[1][0]))
                final_results_dict[PROBE_Z_DISTANCE].append(abs(position[2][0]))

        except ValueError:
            position = None

        if data[WAYPOINT_REACHED][i]:

            # If the starting pose is known,
            try:

                # Find the magnitude of the x distance
                x_distance = abs(position[0][0])

                if abs(x_distance - previous_distance) > 10 ** -5:

                    # Add the relevant data to the final results dictionary
                    if save_data_to_plot:
                        final_results_dict[WAYPOINT_REACHED + COMBINED_STAMP].append(data[COMBINED_STAMP][i])
                        final_results_dict[WAYPOINT_REACHED + POSE_X].append(x_distance)

                    # Add the data to the waypoints dictionary
                    waypoint_data[PROBE_X_DISTANCE].append(x_distance - previous_distance)
                    waypoint_data[POSE_PITCH].append(data[POSE_PITCH][i] - pitch_at_start)
                    waypoint_data[POSE_YAW].append(data[POSE_YAW][i] - yaw_at_start)

                    # Update the previous distance
                    previous_distance = x_distance

            # Otherwise,
            except TypeError:
                # Define the starting pose
                starting_pose = calc_transformation_from_rpy(
                    xyz=(data[POSE_X][i], data[POSE_Y][i], data[POSE_Z][i]),
                    rpy=(-data[POSE_ROLL][i], data[POSE_PITCH][i], data[POSE_YAW][i]))

                # Calculate the inverse matrix of the starting pose
                inverse_starting_pose = calc_inverse(starting_pose)

                # Save the values of the first position
                previous_distance = 0
                pitch_at_start = data[POSE_PITCH][i]
                yaw_at_start = data[POSE_YAW][i]

    # Calculate the mean distance for convenience
    mean_distance_between_waypoints = mean(waypoint_data[PROBE_X_DISTANCE]) * 10 ** 3

    # Write the data into the results file
    results_file_writer.writerow({DESIRED_DISTANCE: round(mean_distance_between_waypoints, 0),
                                  DISTANCE_MEAN: round(mean_distance_between_waypoints, 3),
                                  DISTANCE_STDEV: round(stdev(waypoint_data[PROBE_X_DISTANCE]) * 10 ** 3, 3),
                                  PITCH_ERROR: round(mean(waypoint_data[POSE_PITCH]), 3),
                                  PITCH_STDEV: round(stdev(waypoint_data[POSE_PITCH]), 3),
                                  YAW_ERROR: round(mean(waypoint_data[POSE_YAW]), 3),
                                  YAW_STDEV: round(stdev(waypoint_data[POSE_YAW]), 3),
                                  NUMBER_OF_DATA_POINTS: len(waypoint_data[PROBE_X_DISTANCE])})

# Close the results file
results_file.close()

# Define the sampling rate for the points included in the plot
point_sampling_rate = 10

# Define the common X axis for all the data
common_x = final_results_dict[COMBINED_STAMP][::point_sampling_rate]

# Plot the data for each axis as a line
axes.plot(common_x, final_results_dict[PROBE_X_DISTANCE][::point_sampling_rate],
          linewidth=1, color='red', label='X Axis')
axes.plot(common_x, final_results_dict[PROBE_Y_DISTANCE][::point_sampling_rate],
          linewidth=1, color='green', label='Y Axis')
axes.plot(common_x, final_results_dict[PROBE_Z_DISTANCE][::point_sampling_rate],
          linewidth=1, color='blue', label='Z Axis')

# Add the legend to the axis
axes.legend()

# Add markers for when each waypoint was reached
axes.scatter(final_results_dict[WAYPOINT_REACHED + COMBINED_STAMP], final_results_dict[WAYPOINT_REACHED + POSE_X],
             s=6, c='black', marker="D")

# Set the axis labels
axes.set_xlabel(COMBINED_STAMP)
axes.set_ylabel("Distance Travelled w.r.t. Initial Pose (m)")

# Set the grid
axes.grid(axis='y')

# Show the graph
show()
