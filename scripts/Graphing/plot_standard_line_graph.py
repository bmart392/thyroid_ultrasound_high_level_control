# Import standard python packages
from matplotlib.pyplot import show, subplots, rc
from numpy.polynomial import Polynomial
from statistics import median, stdev, mean

# Import custom python packages
from ExperimentalDataRecorder import MESSAGE_ID, STAMP_SECS, STAMP_NSECS, \
    POSE_X, POSE_Y, POSE_Z, POSE_ROLL, POSE_PITCH, POSE_YAW, \
    FORCE, IMAGE_CENTROID, SKIN_ERROR, WAYPOINT_REACHED, \
    FORCE_PREFIX, POSE_PREFIX, CENTROID_PREFIX, SKIN_ERROR_PREFIX

from Graphing.read_recorded_data_csv import read_recorded_data_csv, COMBINED_STAMP, CONTROLLED, UNCONTROLLED, \
    FIG_WIDTH, FIG_HEIGHT

# Define the default values to use when plotting
DEFAULT_SIX_COLORS: tuple = ((171, 22, 43), (109, 109, 109), (0, 46, 109),
                             (94, 115, 97), (199, 138, 61), (124, 101, 105))
DEFAULT_COLOR_ADJUSTMENT_VALUE: float = 50
DEFAULT_SIX_DATA_TYPES: tuple = (UNCONTROLLED, UNCONTROLLED, UNCONTROLLED, CONTROLLED, CONTROLLED, CONTROLLED)


def plot_standard_line_graph(basic_source_path: str, time_stamps: tuple, heading_to_plot: str,
                             estimation_line_deg: int, point_sampling_rate: int,
                             data_types: tuple = DEFAULT_SIX_DATA_TYPES,
                             line_colors: tuple = DEFAULT_SIX_COLORS,
                             data_color_adjustment_value: float = DEFAULT_COLOR_ADJUSTMENT_VALUE,
                             y_label_text: str = None, statistics_text_to_show: str = None, show_plot: bool = False):
    """
    Creates a standard plot for comparing one data point over time between multiple tests.

    Parameters
    ----------
    basic_source_path :
        The absolute path to where the data for one set of controlled and uncontrolled experiments can be found.
    time_stamps :
        The time stamp of the files to pull.
    heading_to_plot :
        The data to pull from the CSV file. Must be one of the standard headers.
    estimation_line_deg :
        The degree of the polynomial used to estimate the data.
    point_sampling_rate :
        The rate at which to select points from the data to plot
    data_types :
        The types of data to be plotted. Must be either CONTROLLED or UNCONTROLLED.
    line_colors :
        The colors to use to plot each line.
    data_color_adjustment_value :
        The adjustment value to use to get the shade color.
    y_label_text :
        Text to show as the label for the y-axis rather than the heading.
    statistics_text_to_show :
        The text to show in the statistics printout.
    show_plot :
        An option to show the plot.

    """

    # Select the proper file prefix based on the header to plot
    if any([heading_to_plot == x for x in [POSE_X, POSE_Y, POSE_Z, POSE_ROLL, POSE_PITCH, POSE_YAW]]):
        file_prefix = POSE_PREFIX
    elif heading_to_plot == FORCE:
        file_prefix = FORCE_PREFIX
    elif heading_to_plot == IMAGE_CENTROID:
        file_prefix = CENTROID_PREFIX
    elif heading_to_plot == SKIN_ERROR:
        file_prefix = SKIN_ERROR_PREFIX
    else:
        raise Exception(heading_to_plot + ' is not a recognized header.')


    # Redefine the line colors data
    line_colors = list(line_colors)

    # Define the list of data_colors
    data_colors = []

    # Adjust the line colors to be floats between 0 and 1
    for ii in range(len(line_colors)):
        line_colors[ii] = tuple([value / 255 for value in line_colors[ii]])
        data_colors.append(tuple([value + (data_color_adjustment_value / 255) for value in line_colors[ii]]))

    # Set the default size for plot elements
    rc('font', size=10)  # controls default text sizes
    rc('axes', titlesize=18)  # font size of the axes title
    rc('axes', labelsize=14)  # font size of the x and y labels
    rc('xtick', labelsize=10)  # font size of the tick labels
    rc('ytick', labelsize=10)  # font size of the tick labels
    rc('legend', fontsize=10)  # legend font size
    rc('figure', titlesize=24)  # font size of the figure title
    rc('figure', dpi=300)  # Set the DPI of the figure

    # Make sure all data is the right size
    if len(time_stamps) != len(data_colors) != len(line_colors) != len(data_types):
        raise Exception("Data sources and number of colors do not match for the uncontrolled data.")

    # Create a figure and axes to plot the results
    fig, axes = subplots(nrows=1, ncols=1, figsize=(FIG_WIDTH, FIG_HEIGHT))

    # Define a counter to for the trial number
    controlled_trial_number = 1
    uncontrolled_trial_number = 1

    # Define a list to store a complete set of controlled data
    completed_controlled_data = []

    # Define a list to store the changes in the uncontrolled test
    uncontrolled_data_change = []

    # For each file
    for time_stamp, data_color, line_color, data_type in zip(time_stamps, data_colors,
                                                             line_colors, data_types):

        # Build the path according to the data type
        if data_type == CONTROLLED:
            source_path = (basic_source_path + 'Controlled/' + time_stamp + '_experiment/' + file_prefix +
                           time_stamp + '.csv')
        elif data_type == UNCONTROLLED:
            source_path = (
                        basic_source_path + 'Uncontrolled/' + time_stamp + '_experiment/' + file_prefix +
                        time_stamp + '.csv')
        else:
            raise Exception(str(data_type) + ' is not a recognized data type.')

        # Read the data from the file
        data = read_recorded_data_csv(file_path=source_path,
                                      headings=[MESSAGE_ID, STAMP_SECS, STAMP_NSECS, heading_to_plot],
                                      sort_heading=MESSAGE_ID,
                                      additional_headings_to_zero=[STAMP_SECS])

        # Define variables for convenience
        data_x = data[COMBINED_STAMP][::point_sampling_rate]
        data_y = data[heading_to_plot][::point_sampling_rate]
        data_y_std_dev = stdev(data_y)

        # Calculate the line of best fit for the data
        line_estimation = Polynomial.fit(data_x, data_y, deg=estimation_line_deg)
        estimated_data_y = [line_estimation(x) for x in data_x]

        shade_data_y_std_dev = stdev([y_real - y_estimate for y_real, y_estimate in zip(data_y, estimated_data_y)])

        # Set the line-style and trial number to use
        if data_type == CONTROLLED:
            line_style = 'dashed'
            display_trial_number = controlled_trial_number
        elif data_type == UNCONTROLLED:
            line_style = 'solid'
            display_trial_number = uncontrolled_trial_number
        else:
            raise Exception("Line style given is not a recognized style.")

        # Plot the line of best fit
        axes.plot(data_x, estimated_data_y,
                  color=line_color, linestyle=line_style,
                  label='Trial ' + str(display_trial_number) + ' - ' + data_type)
        axes.fill_between(data_x, [y - shade_data_y_std_dev for y in estimated_data_y],
                          [y + shade_data_y_std_dev for y in estimated_data_y],
                          alpha=0.35, facecolor=data_color)

        # Update the uncontrolled data change
        if data_type == UNCONTROLLED:
            uncontrolled_data_change = uncontrolled_data_change + [data[heading_to_plot][-1] - data[heading_to_plot][0]]

            # Increment the trial number
            uncontrolled_trial_number = uncontrolled_trial_number + 1

        # Update the complete controlled data list
        if data_type == CONTROLLED:
            completed_controlled_data = completed_controlled_data + data[heading_to_plot]

            # Increment the trial number
            controlled_trial_number = controlled_trial_number + 1

    if statistics_text_to_show is not None:

        if len(completed_controlled_data) > 0:
            print(statistics_text_to_show + ' - Controlled --- Mean: ' +
                  str(round(mean(completed_controlled_data), 4)))
            print(statistics_text_to_show + ' - Controlled --- Median: ' +
                  str(round(median(completed_controlled_data), 4)))
            if len(completed_controlled_data) > 1:
                print(statistics_text_to_show + ' - Controlled --- Std Dev: ' +
                      str(round(stdev(completed_controlled_data), 4)))
        if len(uncontrolled_data_change) > 0:
            print(statistics_text_to_show + ' - Uncontrolled - Mean: ' +
                  str(round(mean(uncontrolled_data_change), 4)))
            print(statistics_text_to_show + ' - Uncontrolled - Median: ' +
                  str(round(median(uncontrolled_data_change), 4)))
            if len(uncontrolled_data_change) > 1:
                print(statistics_text_to_show + ' - Uncontrolled - Std Dev: ' +
                      str(round(stdev(uncontrolled_data_change), 4)))

        print('-' * 10)

    # Set the parameters for the figure
    axes.grid(axis='y')
    axes.set_xlabel(COMBINED_STAMP)
    if y_label_text is not None:
        axes.set_ylabel(y_label_text)
    else:
        axes.set_ylabel(heading_to_plot)

    # Add the legend
    axes.legend()

    # Show the plot if desired
    if show_plot:
        show()
