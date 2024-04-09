"""
Contains the code for validating that the ImageBasedUserInput ui_node is successful.
"""

# Import standard python packages
from numpy import save, load, append
from imageio.v3 import imwrite
from csv import DictWriter, DictReader
from numpy.polynomial import Polynomial
from statistics import mean, median, stdev
from matplotlib.pyplot import subplots, savefig, rc, show
from os.path import isdir
from os import mkdir
from cv2 import imshow, waitKey, imread, resize, INTER_CUBIC

# Import standard ROS packages

# Import custom python packages
from thyroid_ultrasound_imaging_support.RegisteredData.load_folder_of_saved_registered_data import \
    load_folder_of_saved_registered_data
from thyroid_ultrasound_imaging_support.RegisteredData.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents
from thyroid_ultrasound_imaging_support.UserInput.user_input_polygon_points import user_input_polygon_points
from thyroid_ultrasound_imaging_support.Boundaries.create_convex_triangles_from_points import \
    create_convex_triangles_from_points
from thyroid_ultrasound_imaging_support.Boundaries.create_mask_array_from_triangles import \
    create_mask_array_from_triangles
from thyroid_ultrasound_imaging_support.Validation.calculate_dice_score import calculate_dice_score
from Graphing.read_recorded_data_csv import read_recorded_data_csv, CONTROLLED, UNCONTROLLED, FIG_WIDTH, FIG_HEIGHT
from thyroid_ultrasound_imaging_support.Visualization.create_mask_overlay_array import create_mask_overlay_array, \
    COLORIZED

# Set the default size for plot elements
rc('font', size=10)  # controls default text sizes
rc('axes', titlesize=18)  # font size of the axes title
rc('axes', labelsize=14)  # font size of the x and y labels
rc('xtick', labelsize=10)  # font size of the tick labels
rc('ytick', labelsize=10)  # font size of the tick labels
rc('legend', fontsize=10)  # legend font size
rc('figure', titlesize=24)  # font size of the figure title
rc('figure', dpi=300)  # Set the DPI of the figure


########################################################################################################################


# Define helper function to create directories
def create_directory(root, sub_directory=None):
    full_path = root
    if sub_directory is not None:
        full_path = full_path + sub_directory
    if not isdir(full_path):
        mkdir(full_path)
    return full_path


########################################################################################################################


# Select actions to perform
view_images: bool = False
extract_cropped_images: bool = False
define_ground_truth_masks: bool = False
extract_segmentation_masks: bool = False
create_overlay_comparison_images: bool = False
generate_comparison_results: bool = False
graph_results: bool = True

# Define location of the source registered data that has been segmented in non-real-time
SOURCE_DATA_PATH: str = '/home/ben/thyroid_ultrasound_data/testing_and_validation/volume_data' \
                        '/VolumeData_2024-03-17_17-07-06-622496_2024-03-17_17-55-49-772717'

# Define the location to save all data
SAVE_DATA_BASIC_PATH: str = '/home/ben/thyroid_ultrasound_data/experimentation/NonRealTimeSegmentation'

# Define the paths in which the data to graph is located
SAVED_RESULTS_FILES_NAMES: tuple = ('SegmentationAnalysis_2024-03-17_01-24-02-350700_2024-03-17_17-43-48-276775',
                                    'SegmentationAnalysis_2024-03-17_16-49-50-810014_2024-03-17_17-49-08-315923',
                                    'SegmentationAnalysis_2024-03-17_16-58-20-798648_2024-03-17_17-52-00-042309',
                                    'SegmentationAnalysis_2024-03-17_17-02-25-272295_2024-03-17_17-53-40-494210',
                                    'SegmentationAnalysis_2024-03-17_17-07-06-622496_2024-03-17_17-55-49-772717')

########################################################################################################################

# Define the colors to use for graphing
line_colors = [(171, 22, 43), (109, 109, 109), (0, 46, 109), (94, 115, 97), (199, 138, 61), (124, 101, 105)]
data_color_adjustment_value = 50  # pts

# Define the list of data_colors
data_colors = []

# Adjust the line colors to be floats between 0 and 1
for ii in range(len(line_colors)):
    line_colors[ii] = tuple([value / 255 for value in line_colors[ii]])
    data_colors.append(tuple([value + (data_color_adjustment_value / 255) for value in line_colors[ii]]))

#######################################################################################################################

# Define the prefix for naming the root folder
ROOT_FOLDER_PREFIX: str = 'SegmentationAnalysis'

# Define the prefix to use for the cropped image files
CROPPED_IMAGES_DIRECTORY: str = 'CroppedImages'
CROPPED_IMAGES_PREFIX: str = 'CroppedImage'
CROPPED_IMAGES_FILE_EXTENSION: str = '.png'

# Define the prefix to use for the ground truth masks
GROUND_TRUTH_MASKS_DIRECTORY: str = 'GroundTruths'
GROUND_TRUTH_MASKS_PREFIX: str = 'GroundTruth'

# Define the prefix to use for the non-real time segmentation masks
NON_REAL_TIME_SEGMENTATION_MASKS_DIRECTORY: str = 'NonRealTimeSegmentationMasks'
NON_REAL_TIME_SEGMENTATION_MASKS_PREFIX: str = 'NonRealTimeSegmentationMask'

# Define the prefix to use for the comparison overlay image
COMPARISON_OVERLAY_IMAGE_DIRECTORY: str = 'ComparisonOverlayImages'
COMPARISON_OVERLAY_IMAGE_PREFIX: str = 'ComparisonOverlayImage'
COMPARISON_OVERLAY_IMAGE_FILE_EXTENSION: str = '.png'

# Define the extensions used for the results
RESULTS_FILES_DATA_POINTS_PREFIX: str = 'ResultsDataPoints'
RESULTS_FILES_DATA_POINTS_EXTENSION: str = '.csv'
RESULTS_FILES_STATISTICS_Prefix: str = 'ResultsStatistics'
RESULTS_FILES_STATISTICS_EXTENSION: str = '.txt'

# Define the headers to use in the CSV file of results
HEADER_IMAGE_NUMBER: str = 'Image # in Sequence'
HEADER_DICE_SCORE: str = 'DICE score of Segmentation'

# Define the prefixes to use in the text file of statistical results
PREFIX_MEAN: str = 'Mean DICE Score of Data Set: '
PREFIX_MEDIAN: str = 'Median DICE Score of Data Set: '
PREFIX_STD_DEV: str = 'Standard Deviation of DICE Score of Data Set: '

# Define the prefix and extension to use for the result images
GRAPH_IMAGE_PREFIX: str = 'ComparisonGraphImage'
GRAPH_IMAGE_EXTENSION: str = '.png'

#######################################################################################################################

# Pull the timestamp out of the source data file name
source_data_time_stamp = SOURCE_DATA_PATH[SOURCE_DATA_PATH.rfind('/'):][
                         SOURCE_DATA_PATH[SOURCE_DATA_PATH.rfind('/'):].find('_'):]

# Add a backslash to the source root, if it is not included
root_folder_path = SAVE_DATA_BASIC_PATH
if SAVE_DATA_BASIC_PATH[-1] != '/':
    root_folder_path = root_folder_path + '/'

# Create the path to the root folder
root_folder_path = create_directory(root_folder_path, ROOT_FOLDER_PREFIX + source_data_time_stamp + '/')

# Define the path to the cropped images and create it if necessary
CROPPED_IMAGES_PATH: str = create_directory(root_folder_path, CROPPED_IMAGES_DIRECTORY)

# Define the locations of the ground truth data
GROUND_TRUTH_MASKS_PATH: str = create_directory(root_folder_path, GROUND_TRUTH_MASKS_DIRECTORY)

# Define the locations of the segmentation masks
NON_REAL_TIME_SEGMENTATION_MASKS_PATH: str = create_directory(root_folder_path,
                                                              NON_REAL_TIME_SEGMENTATION_MASKS_DIRECTORY)

# Define the location to save the results
COMPARISON_OVERLAY_IMAGES_PATH: str = create_directory(root_folder_path, COMPARISON_OVERLAY_IMAGE_DIRECTORY)

# Define the location to save the results
SAVE_RESULTS_FILES_PATH: str = root_folder_path

# Define the location to save the resulting graph as an image
GRAPH_IMAGE_PATH: str = root_folder_path

# Define the saved results files paths
SAVED_RESULTS_FILES_PATHS: tuple = tuple([SAVE_DATA_BASIC_PATH + '/' + path for path in SAVED_RESULTS_FILES_NAMES])

#######################################################################################################################

if view_images or extract_cropped_images or define_ground_truth_masks or extract_segmentation_masks:

    # Load the registered data
    image_data_objects = [registered_data.image_data for registered_data in
                          load_folder_of_saved_registered_data(SOURCE_DATA_PATH)]

    # Loop through each registered data object
    for image_data_object in image_data_objects:

        # Define the time stamp suffix for file names
        time_stamp_suffix = '_' + str(image_data_object.image_capture_time.secs) + \
                            '_' + str(image_data_object.image_capture_time.nsecs)

        # If the purpose is just to view the registered data
        if view_images:
            # Display the images
            imshow("Registered Data Images", image_data_object.original_image)
            waitKey(-1)

        # If the purpose is to extract the cropped images,
        if extract_cropped_images:
            # Create the full path of the image
            imwrite(
                uri=CROPPED_IMAGES_PATH + '/' + CROPPED_IMAGES_PREFIX + time_stamp_suffix + CROPPED_IMAGES_FILE_EXTENSION,
                image=image_data_object.cropped_image)

        # If the purpose is to define ground truth masks,
        if define_ground_truth_masks:
            # Define the list to store the points that define the foreground
            list_of_foreground_points = None

            # Capture the foreground of the image from the user
            list_of_points_for_foreground_polygon = user_input_polygon_points(
                image_data_object,
                "foreground ground truth",
                display_result=True,
                list_of_points=list_of_foreground_points)

            # Convert the points of the foreground polygons to triangles
            list_of_foreground_triangles = create_convex_triangles_from_points(
                list_of_points_for_foreground_polygon)

            # Generate the ground-truth mask using the triangles selected by the user
            ground_truth_mask = create_mask_array_from_triangles(
                list_of_foreground_triangles,
                image_data_object.cropped_image.shape[:2])

            # Save the ground truth mask
            save(file=GROUND_TRUTH_MASKS_PATH + '/' + GROUND_TRUTH_MASKS_PREFIX + time_stamp_suffix,
                 arr=ground_truth_mask)

        # If the purpose is to extract the segmentation masks,
        if extract_segmentation_masks:
            # Save the ground truth mask
            save(file=(NON_REAL_TIME_SEGMENTATION_MASKS_PATH + '/' + NON_REAL_TIME_SEGMENTATION_MASKS_PREFIX +
                       time_stamp_suffix),
                 arr=image_data_object.image_mask)

if create_overlay_comparison_images or generate_comparison_results:

    # Read in the ground truth masks
    ground_truth_masks = [load(path) for path in generate_ordered_list_of_directory_contents(GROUND_TRUTH_MASKS_PATH,
                                                                                             sort_indices=(0, 1))]

    # Read in the result masks
    segmentation_masks = [load(path) for path in
                          generate_ordered_list_of_directory_contents(NON_REAL_TIME_SEGMENTATION_MASKS_PATH,
                                                                      sort_indices=(0, 1))]

    if create_overlay_comparison_images:

        # Read in the cropped images
        cropped_images = [imread(path) for path in
                          generate_ordered_list_of_directory_contents(CROPPED_IMAGES_PATH,
                                                                      sort_indices=(0, 1))]

        # Gather the time stamps from the cropped images
        cropped_image_stamps = [path[path.rfind('/') + 1 + len(CROPPED_IMAGES_PREFIX)
                                     :-len(CROPPED_IMAGES_FILE_EXTENSION)] for path in
                                generate_ordered_list_of_directory_contents(CROPPED_IMAGES_PATH,
                                                                            sort_indices=(0, 1))]

        for cropped_image, cropped_image_stamp, \
            ground_truth_mask, segmentation_mask in zip(cropped_images, cropped_image_stamps,
                                                        ground_truth_masks, segmentation_masks):
            # Expand the masks the size of the image
            ground_truth_mask_expanded = resize(ground_truth_mask,
                                                dsize=(cropped_image.shape[1], cropped_image.shape[0]),
                                                interpolation=INTER_CUBIC)
            segmentation_mask_expanded = resize(segmentation_mask,
                                                dsize=(cropped_image.shape[1], cropped_image.shape[0]),
                                                interpolation=INTER_CUBIC)

            # Create the overlay masks
            ground_truth_overlay = create_mask_overlay_array(cropped_image,
                                                             overlay_mask=ground_truth_mask_expanded,
                                                             overlay_method=COLORIZED,
                                                             overlay_color=(0, 35, 0))
            segmentation_mask_overlay = create_mask_overlay_array(cropped_image,
                                                                  overlay_mask=segmentation_mask_expanded,
                                                                  overlay_method=COLORIZED,
                                                                  overlay_color=(0, 0, 35))

            stitched_image = append(ground_truth_overlay, segmentation_mask_overlay, axis=1)

            # Save the ground truth mask
            imwrite(uri=(COMPARISON_OVERLAY_IMAGES_PATH + '/' + COMPARISON_OVERLAY_IMAGE_PREFIX + cropped_image_stamp +
                         COMPARISON_OVERLAY_IMAGE_FILE_EXTENSION),
                    image=stitched_image)

            imshow("Stiched", stitched_image)
            waitKey(-1)

    if generate_comparison_results:
        # Create a csv file in which to write the results
        data_points_file = open(SAVE_RESULTS_FILES_PATH + RESULTS_FILES_DATA_POINTS_PREFIX +
                                RESULTS_FILES_DATA_POINTS_EXTENSION, mode='w')
        data_points_file_writer = DictWriter(data_points_file,
                                             fieldnames=[HEADER_IMAGE_NUMBER, HEADER_DICE_SCORE])
        data_points_file_writer.writeheader()

        # Create a text file in which to write the statistical results
        statistics_file = open(SAVE_RESULTS_FILES_PATH + RESULTS_FILES_STATISTICS_Prefix +
                               RESULTS_FILES_STATISTICS_EXTENSION, mode='w')

        # Define a list in which to store the DICE scores
        dice_scores = []

        # Define an iterator to note which image in the series is being analyzed
        image_number = 0

        # Loop through each pair
        for ground_truth, segmentation in zip(ground_truth_masks, segmentation_masks):
            # Calculate the DICE score
            score = calculate_dice_score(ground_truth, segmentation)

            # Write the iterator and DICE score to the file
            data_points_file_writer.writerow({HEADER_IMAGE_NUMBER: image_number, HEADER_DICE_SCORE: score})

            # Append the DICE score to a list
            dice_scores.append(score)

            # Increment the iterator
            image_number = image_number + 1

        # Calculate the mean, median, and standard deviation of the results
        mean_score = mean(dice_scores)
        median_score = median(dice_scores)
        score_std_dev = stdev(dice_scores)

        # Write the statistical results to the text file
        for prefix, value in zip([PREFIX_MEAN, PREFIX_MEDIAN, PREFIX_STD_DEV],
                                 [mean_score, median_score, score_std_dev]):
            statistics_file.write(prefix + str(value) + '\n')

        # Close the text and csv files
        data_points_file.close()
        statistics_file.close()

if graph_results:

    # Create a figure and axes to plot the results
    fig, axes = subplots(nrows=1, ncols=1, figsize=(FIG_WIDTH, FIG_HEIGHT))

    # Define a file counter
    trial_counter = 1

    # For each data file given
    for data_points_file_path in SAVED_RESULTS_FILES_PATHS:

        # Define a variable to save all of the data
        total_data = []

        # If it is a valid path
        if isdir(data_points_file_path):
            # # Open the data points file
            # data_points_file = open(data_points_file_path + '/' + RESULTS_FILES_DATA_POINTS_PREFIX +
            #                         RESULTS_FILES_DATA_POINTS_EXTENSION, mode='r')
            # data_points_file_reader = DictReader(data_points_file)

            # Read in the data
            data = read_recorded_data_csv(file_path=(data_points_file_path + '/' + RESULTS_FILES_DATA_POINTS_PREFIX +
                                                     RESULTS_FILES_DATA_POINTS_EXTENSION),
                                          headings=[HEADER_IMAGE_NUMBER, HEADER_DICE_SCORE],
                                          create_combined_stamp=False)

            # Define variables for convenience
            data_x = data[HEADER_IMAGE_NUMBER]
            data_y = data[HEADER_DICE_SCORE]

            # Add the new data to the total data variable
            total_data = total_data + data_y

            # Calculate the line of best fit for the data
            line_estimation = Polynomial.fit(data_x, data_y, deg=3)
            estimated_data_y = [line_estimation(x) for x in data_x]

            # Calculate the standard deviation of the estimate from the real data
            shade_data_y_std_dev = stdev([y_real - y_estimate for y_real, y_estimate in zip(data_y, estimated_data_y)])

            # # Plot the line of best fit
            # axes.plot(data_x, estimated_data_y,
            #           color=line_colors[trial_counter - 1])  # , label='Trial ' + str(trial_counter) + ' Estimate')
            #
            # # Shade around the line of best fit
            # axes.fill_between(data_x, [y - shade_data_y_std_dev for y in estimated_data_y],
            #                   [y + shade_data_y_std_dev for y in estimated_data_y],
            #                   alpha=0.35, facecolor=data_colors[trial_counter - 1])

            # Plot the line between the data
            axes.plot(data[HEADER_IMAGE_NUMBER], data[HEADER_DICE_SCORE], label='Trial ' + str(trial_counter) + ' Data',
                      color=data_colors[trial_counter - 1])

            # Plot the individual data points
            axes.scatter(data_x, data_y, s=10, color=line_colors[trial_counter - 1])

            # Increment the file counter
            trial_counter = trial_counter + 1

    # Calculate the mean, median, and standard deviation of the results
    print('Mean: ' + str(mean(total_data)))
    print('Median: ' + str(median(total_data)))
    print('Standard Deviation: ' + str(stdev(total_data)))

    # Set the parameters for the figure
    axes.grid(axis='y')
    axes.set_ylim([0, 1.0])
    axes.set_xlabel(HEADER_IMAGE_NUMBER)
    axes.set_ylabel(HEADER_DICE_SCORE)

    # Add the legend
    axes.legend()

    # Show the figure
    show()

    # Save the figure
    # savefig(GRAPH_IMAGE_PATH + GRAPH_IMAGE_PREFIX + GRAPH_IMAGE_EXTENSION)
