# Import standard python packages
from matplotlib.pyplot import show, subplots, rc
from numpy.polynomial import Polynomial
from statistics import median, stdev, mean

# Import custom python packages
from ExperimentalDataRecorder import MESSAGE_ID, STAMP_SECS, STAMP_NSECS, FORCE, POSE_Z
from Graphing.read_recorded_data_csv import read_recorded_data_csv, COMBINED_STAMP, CONTROLLED, UNCONTROLLED

# Set the default size for plot elements
rc('font', size=10)  # controls default text sizes
rc('axes', titlesize=18)  # font size of the axes title
rc('axes', labelsize=14)  # font size of the x and y labels
rc('xtick', labelsize=10)  # font size of the tick labels
rc('ytick', labelsize=10)  # font size of the tick labels
rc('legend', fontsize=10)  # legend font size
rc('figure', titlesize=24)  # font size of the figure title
rc('figure', dpi=300)  # Set the DPI of the figure

# Select which graphs to produce
create_force_graph = True
create_pose_graph = True

# Define the sources of the information and choose the colors
BASIC_SOURCE_PATH: str = '/home/ben/thyroid_ultrasound_data/experimentation/ForceControl/'
time_stamps = ['2024-02-19_13-05-58-231870', '2024-02-19_13-07-40-867184', '2024-02-19_13-08-56-948372',
               '2024-02-19_13-10-24-970993', '2024-02-19_13-11-58-761400', '2024-02-19_13-13-17-885949']
data_types = [UNCONTROLLED, UNCONTROLLED, UNCONTROLLED, CONTROLLED, CONTROLLED, CONTROLLED]
line_colors = [(171, 22, 43), (109, 109, 109), (0, 46, 109), (94, 115, 97), (199, 138, 61), (124, 101, 105)]
data_color_adjustment_value = 50  # pts
# BASIC_SOURCE_PATH: str = '/home/ben/thyroid_ultrasound_data/experimentation/ForceControl/'
# time_stamps = ['2024-03-07_23-47-34-898445']
# data_types = [CONTROLLED]
# data_colors = ['lightskyblue']
# line_colors = ['deepskyblue']

# Define the list of data_colors
data_colors = []

# Adjust the line colors to be floats between 0 and 1
for ii in range(len(line_colors)):
    line_colors[ii] = tuple([value / 255 for value in line_colors[ii]])
    data_colors.append(tuple([value + (data_color_adjustment_value / 255) for value in line_colors[ii]]))

if create_force_graph:
    # Make sure all data is the right size
    if len(time_stamps) != len(data_colors) != len(line_colors) != len(data_types):
        raise Exception("Data sources and number of colors do not match for the uncontrolled data.")

    # Create a figure and axes to plot the results
    fig, axes = subplots(nrows=1, ncols=1)

    # Define a counter to for the trial number
    trial_number = 1

    # Define a list to store a complete set of controlled data
    completed_controlled_data = []

    # Define a list to store the changes in the uncontrolled test
    uncontrolled_data_change = []

    # For each file
    for time_stamp, data_color, line_color, data_type in zip(time_stamps, data_colors,
                                                             line_colors, data_types):

        # Build the path according to the data type
        if data_type == CONTROLLED:
            source_path = (BASIC_SOURCE_PATH + 'Controlled/' + time_stamp + '_experiment/Force_' + time_stamp + '.csv')
        elif data_type == UNCONTROLLED:
            source_path = (BASIC_SOURCE_PATH + 'Uncontrolled/' + time_stamp + '_experiment/Force_' + time_stamp + '.csv')
        else:
            raise Exception(str(data_type) + ' is not a recognized data type.')

        # Read the data from the file
        data = read_recorded_data_csv(file_path=source_path,
                                      headings=[MESSAGE_ID, STAMP_SECS, STAMP_NSECS, FORCE],
                                      sort_heading=MESSAGE_ID,
                                      additional_headings_to_zero=[STAMP_SECS])

        # Define the sampling rate for the points included in the plot
        point_sampling_rate = 20

        # Define variables for convenience
        data_x = data[COMBINED_STAMP][::point_sampling_rate]
        data_y = data[FORCE][::point_sampling_rate]
        data_y_std_dev = stdev(data_y)

        # Calculate the line of best fit for the data
        line_estimation = Polynomial.fit(data_x, data_y, deg=8)
        estimated_data_y = [line_estimation(x) for x in data_x]

        shade_data_y_std_dev = stdev([y_real - y_estimate for y_real, y_estimate in zip(data_y, estimated_data_y)])

        # Plot the line of best fit
        axes.plot(data_x, estimated_data_y,
                  color=line_color, label='Trial ' + str(trial_number) + ' Data')
        axes.fill_between(data_x, [y - shade_data_y_std_dev for y in estimated_data_y],
                          [y + shade_data_y_std_dev for y in estimated_data_y],
                          alpha=0.35, facecolor=data_color)

        # Update the uncontrolled data change
        if data_type == UNCONTROLLED:
            uncontrolled_data_change = uncontrolled_data_change + [data[FORCE][-1] - data[FORCE][0]]

        # Update the complete controlled data list
        if data_type == CONTROLLED:
            completed_controlled_data = completed_controlled_data + data[FORCE]

        # Increment the trial number
        trial_number = trial_number + 1

    if len(completed_controlled_data) > 0:
        print('Median Force Value: ' + str(median(completed_controlled_data)))
        if len(completed_controlled_data) > 1:
            print("Standard Deviation of Force Value: " + str(stdev(completed_controlled_data)))
    if len(uncontrolled_data_change) > 0:
        print('Mean Uncontrolled Change: ' + str(mean(uncontrolled_data_change)))
        if len(uncontrolled_data_change) > 1:
            print("Standard Deviation of Uncontrolled Change: " + str(stdev(uncontrolled_data_change)))

    # Set the parameters for the figure
    axes.grid(axis='y')
    axes.set_xlabel(COMBINED_STAMP)
    axes.set_ylabel(FORCE)

    # Add the legend
    axes.legend()

if create_pose_graph:

    # Make sure all data is the right size
    if len(time_stamps) != len(data_colors) != len(line_colors) != len(data_types):
        raise Exception("Data sources and number of colors do not match for the uncontrolled data.")

    # Create a figure and axes to plot the results
    fig, axes = subplots(nrows=1, ncols=1)

    # Define a counter to for the trial number
    trial_number = 1

    # Define a list to store a complete set of controlled data
    completed_controlled_data = []

    # Define a list to store the changes in the uncontrolled test
    uncontrolled_data_change = []

    # For each file
    for time_stamp, data_color, line_color, data_type in zip(time_stamps, data_colors,
                                                             line_colors, data_types):

        # Build the path according to the data type
        if data_type == CONTROLLED:
            source_path = (BASIC_SOURCE_PATH + 'Controlled/' + time_stamp + '_experiment/Pose_' + time_stamp + '.csv')
        elif data_type == UNCONTROLLED:
            source_path = (
                        BASIC_SOURCE_PATH + 'Uncontrolled/' + time_stamp + '_experiment/Pose_' + time_stamp + '.csv')
        else:
            raise Exception(str(data_type) + ' is not a recognized data type.')

        # Read the data from the file
        data = read_recorded_data_csv(file_path=source_path,
                                      headings=[MESSAGE_ID, STAMP_SECS, STAMP_NSECS, POSE_Z],
                                      sort_heading=MESSAGE_ID,
                                      additional_headings_to_zero=[STAMP_SECS])

        # Define the sampling rate for the points included in the plot
        point_sampling_rate = 20

        # Define variables for convenience
        data_x = data[COMBINED_STAMP][::point_sampling_rate]
        data_y = data[POSE_Z][::point_sampling_rate]
        data_y_std_dev = stdev(data_y)

        # Calculate the line of best fit for the data
        line_estimation = Polynomial.fit(data_x, data_y, deg=8)
        estimated_data_y = [line_estimation(x) for x in data_x]

        shade_data_y_std_dev = stdev([y_real - y_estimate for y_real, y_estimate in zip(data_y, estimated_data_y)])

        # Plot the line of best fit
        axes.plot(data_x, estimated_data_y,
                  color=line_color, label='Trial ' + str(trial_number) + ' Data')
        axes.fill_between(data_x, [y - shade_data_y_std_dev for y in estimated_data_y],
                          [y + shade_data_y_std_dev for y in estimated_data_y],
                          alpha=0.35, facecolor=data_color)

        # Increment the trial number
        trial_number = trial_number + 1

    # Set the parameters for the figure
    axes.grid(axis='y')
    axes.set_xlabel(COMBINED_STAMP)
    axes.set_ylabel('Probe Position in Z Axis\nw.r.t. Robot Origin (m)')

    # Add the legend
    axes.legend()

# Show the plot
show()

