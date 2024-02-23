# Import standard python packages
from csv import DictReader
from matplotlib.pyplot import show, subplots, rc
from matplotlib.patches import Patch
from numpy.polynomial import Polynomial

# Import custom python packages
from ExperimentalDataRecorder import MESSAGE_ID, STAMP_SECS, STAMP_NSECS, FORCE

# Define the header used for the combined time-stamp
COMBINED_STAMP: str = 'Elapsed Time (s)'

# Define the sources of the information
BASIC_SOURCE_PATH: str = '/home/ben/thyroid_ultrasound_data/experimentation/ForceControl/'
uncontrolled_time_stamp = '2024-02-19_13-05-58-231870'
uncontrolled_source_path: str = BASIC_SOURCE_PATH + 'Uncontrolled/' + uncontrolled_time_stamp + \
                                '_experiment/Force_' + uncontrolled_time_stamp + '.csv'
controlled_time_stamp = '2024-02-19_13-10-24-970993'
controlled_source_path: str = BASIC_SOURCE_PATH + 'Controlled/' + controlled_time_stamp + \
                              '_experiment/Force_' + controlled_time_stamp + '.csv'

# Set the default size for plot elements
rc('font', size=10)  # controls default text sizes
rc('axes', titlesize=18)  # font size of the axes title
rc('axes', labelsize=14)  # font size of the x and y labels
rc('xtick', labelsize=10)  # font size of the tick labels
rc('ytick', labelsize=10)  # font size of the tick labels
rc('legend', fontsize=10)  # legend font size
rc('figure', titlesize=24)  # font size of the figure title
rc('figure', dpi=300)  # Set the DPI of the figure

# Create a figure and axes to plot the results
fig, axes = subplots(nrows=1, ncols=1)

# Define a counter to determine which plot to plot on
axis_counter = 0

# Title the figure
# fig.suptitle('Force Control Experiment')

# Define temporary dictionaries to store each result
uncontrolled_temp_results_dict = {MESSAGE_ID: [], COMBINED_STAMP: [], FORCE: []}
controlled_temp_results_dict = {MESSAGE_ID: [], COMBINED_STAMP: [], FORCE: []}

# Define final dictionaries to store each result
uncontrolled_final_results_dict = {MESSAGE_ID: [], COMBINED_STAMP: [], FORCE: []}
controlled_final_results_dict = {MESSAGE_ID: [], COMBINED_STAMP: [], FORCE: []}

# Open the files
uncontrolled_source_file = open(uncontrolled_source_path, mode='r')
uncontrolled_source_file_reader = DictReader(uncontrolled_source_file)
controlled_source_file = open(controlled_source_path, mode='r')
controlled_source_file_reader = DictReader(controlled_source_file)

# Read in the data
for uncontrolled_row, controlled_row in zip(uncontrolled_source_file_reader, controlled_source_file_reader):
    # Pull the data from the uncontrolled test
    uncontrolled_temp_results_dict[MESSAGE_ID].append(int(uncontrolled_row[MESSAGE_ID]))
    uncontrolled_temp_results_dict[COMBINED_STAMP].append((int(uncontrolled_row[STAMP_SECS])) +
                                                      (int(uncontrolled_row[STAMP_NSECS])) / 10 ** 9)
    uncontrolled_temp_results_dict[FORCE].append(float(uncontrolled_row[FORCE]))

    # Pull the data from the controlled test
    controlled_temp_results_dict[MESSAGE_ID].append(int(controlled_row[MESSAGE_ID]))
    controlled_temp_results_dict[COMBINED_STAMP].append((int(controlled_row[STAMP_SECS])) +
                                                    (int(controlled_row[STAMP_NSECS])) / 10 ** 9)
    controlled_temp_results_dict[FORCE].append(float(controlled_row[FORCE]))

# Find the smallest values from the data
uncontrolled_min_stamp = min(uncontrolled_temp_results_dict[COMBINED_STAMP])
controlled_min_stamp = min(controlled_temp_results_dict[COMBINED_STAMP])

# Reset the time stamp to start from 0
for uncontrolled_message_id, uncontrolled_seconds_stamp, uncontrolled_force, \
    controlled_message_id, controlled_seconds_stamp, controlled_force in zip(
        uncontrolled_temp_results_dict[MESSAGE_ID],
        uncontrolled_temp_results_dict[COMBINED_STAMP],
        uncontrolled_temp_results_dict[FORCE],
        controlled_temp_results_dict[MESSAGE_ID],
        controlled_temp_results_dict[COMBINED_STAMP],
        controlled_temp_results_dict[FORCE]):

    # Pull the data from the uncontrolled test
    uncontrolled_final_results_dict[MESSAGE_ID].append(uncontrolled_message_id)
    uncontrolled_final_results_dict[COMBINED_STAMP].append(uncontrolled_seconds_stamp - uncontrolled_min_stamp)
    uncontrolled_final_results_dict[FORCE].append(uncontrolled_force)

    # Pull the data from the controlled test
    controlled_final_results_dict[MESSAGE_ID].append(controlled_message_id)
    controlled_final_results_dict[COMBINED_STAMP].append(controlled_seconds_stamp - controlled_min_stamp)
    controlled_final_results_dict[FORCE].append(controlled_force)

# Define the sampling rate for the points included in the plot
point_sampling_rate = 3

# Define variables for conveneience
uncontrolled_x = uncontrolled_final_results_dict[COMBINED_STAMP][::point_sampling_rate]
uncontrolled_y = uncontrolled_final_results_dict[FORCE][::point_sampling_rate]

controlled_x = controlled_final_results_dict[COMBINED_STAMP][::point_sampling_rate]
controlled_y = controlled_final_results_dict[FORCE][::point_sampling_rate]

# Plot the data
axes.scatter(uncontrolled_x, uncontrolled_y, s=0.5, c='lightgreen', label='Uncontrolled - Raw Data')
axes.scatter(controlled_x, controlled_y, s=0.5, c='lightcoral', label='Controlled - Raw Data')

# Calculate the line of best fit for the data
uncontrolled_line_estimation = Polynomial.fit(uncontrolled_x, uncontrolled_y, deg=8)
controlled_line_estimation = Polynomial.fit(controlled_x, controlled_y, deg=7)

# uncontrolled_line = poly1d(uncontrolled_line_estimation)
axes.plot(uncontrolled_x, [uncontrolled_line_estimation(x) for x in uncontrolled_x], 'g--',
          label='Uncontrolled - Line of Best Fit')
axes.plot(controlled_x, [controlled_line_estimation(x) for x in controlled_x], 'r--',
          label='Controlled - Line of Best Fit')

# Set the parameters for the figure
axes.grid(axis='y')
axes.set_xlabel(COMBINED_STAMP)
axes.set_ylabel(FORCE)

# Define patches to use for the legend
uncontrolled_raw_data = Patch(color='lightgreen', label='Uncontrolled Test - Raw Data')
uncontrolled_line = Patch(color='green', label='Uncontrolled Test - Line of Best Fit')
controlled_raw_data = Patch(color='lightcoral', label='Controlled Test - Raw Data')
controlled_line = Patch(color='red', label='Controlled Test - Line of Best Fit')

# Add the lengend
axes.legend()

show()
