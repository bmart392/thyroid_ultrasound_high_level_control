
# Import standard python packages
from csv import DictReader
from matplotlib.pyplot import show, subplots, rc

# Import custom python packages
from ExperimentalDataRecorder import MESSAGE_ID, STAMP_SECS, STAMP_NSECS, FORCE

# Define the source of the information
FORCE_FILE_PATH: str = '/home/ben/thyroid_ultrasound_data/experimentation/' \
                       '2024-02-07_09-52-54-095983_experiment/Force_2024-02-07_09-52-54-095983.csv'

# Set the default size for plot elements
rc('font', size=10)         # controls default text sizes
rc('axes', titlesize=30)    # font size of the axes title
rc('axes', labelsize=24)    # font size of the x and y labels
rc('xtick', labelsize=16)   # font size of the tick labels
rc('ytick', labelsize=16)   # font size of the tick labels
rc('legend', fontsize=10)   # legend font size
rc('figure', titlesize=36)  # font size of the figure title

# Create a figure and axes to plot the results
fig, axes = subplots(nrows=1, ncols=1)

# Define a counter to determine which plot to plot on
axis_counter = 0

# Title the figure
fig.suptitle('Force Control Experiment')

# Define a dictionary to store each result
results_dict = {MESSAGE_ID: [], STAMP_SECS: [], FORCE: []}
# Open the file
source_file = open(FORCE_FILE_PATH, mode='r')
source_file_reader = DictReader(source_file)
# Save the values from the first time through
first_message_id = None
first_time_sec = None
first_time_nsec = None

# Read in the data
for row in source_file_reader:

    if first_message_id is None:
        first_message_id = int(row[MESSAGE_ID])
    if first_time_sec is None:
        first_time_sec = int(row[STAMP_SECS])
    if first_time_nsec is None:
        first_time_nsec = int(row[STAMP_NSECS])

    results_dict[MESSAGE_ID].append(int(row[MESSAGE_ID]) - first_message_id)
    results_dict[STAMP_SECS].append((int(row[STAMP_SECS]) - first_time_sec) + (int(row[STAMP_NSECS]) - first_time_nsec) / 10 ** 9)
    results_dict[FORCE].append(float(row[FORCE]))

axes.set_title("No Force Control")
axes.scatter(results_dict[STAMP_SECS], results_dict[FORCE])
axes.set_xlabel(STAMP_SECS)
axes.set_ylabel(FORCE)

show()

