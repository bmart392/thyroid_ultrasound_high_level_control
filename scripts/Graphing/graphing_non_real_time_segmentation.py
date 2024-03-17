"""
Contains the code for validating that the ImageBasedUserInput ui_node is successful.
"""

# Import standard python packages
from numpy import array, save, load
from imageio.v3 import imwrite
from csv import DictWriter, DictReader
from statistics import mean, median, stdev
from matplotlib.pyplot import subplots, savefig
from os.path import isdir
from cv2 import imshow, waitKey

# Import standard ROS packages
from std_msgs.msg import Bool

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_image_files import \
    load_folder_of_image_files
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageData.convert_image_message_to_array import convert_image_message_to_array
from thyroid_ultrasound_imaging_support.ImageFilter.ImageFilterGrabCut import ImageFilterGrabCut, COLOR_GRAY
from thyroid_ultrasound_imaging_support.RegisteredData.load_folder_of_saved_registered_data import \
    load_folder_of_saved_registered_data
from thyroid_ultrasound_imaging_support.RegisteredData.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents
from thyroid_ultrasound_imaging_support.RegisteredData.RegisteredData import RegisteredData
from thyroid_ultrasound_imaging_support.UserInput.user_input_polygon_points import user_input_polygon_points
from thyroid_ultrasound_imaging_support.Boundaries.create_convex_triangles_from_points import \
    create_convex_triangles_from_points
from thyroid_ultrasound_imaging_support.Boundaries.create_mask_array_from_triangles import \
    create_mask_array_from_triangles
from thyroid_ultrasound_imaging_support.Validation.calculate_dice_score import calculate_dice_score
from Graphing.read_recorded_data_csv import read_recorded_data_csv

# Import custom ROS packages

########################################################################################################################

# Select actions to perform
view_images: bool = True
extract_cropped_images: bool = False
define_ground_truth_masks: bool = False
extract_segmentation_masks: bool = False
generate_comparison_results: bool = False
graph_results: bool = False

# ------------------------ !!! END ALL PATHS WITH '/' !!! --------------------------------------------------------------

# Define base file locations
THYROID_ULTRASOUND_DATA_PATH: str = '/home/ben/thyroid_ultrasound_data/'

# Define location of the source registered data that has been segmented in non-real-time
NON_REAL_TIME_SEGMENTED_REGISTERED_DATA_PATH: str = '/home/ben/thyroid_ultrasound_data/testing_and_validation/' \
                                                    'volume_data/VolumeData_1710652687_558489084'
# '/home/ben/thyroid_ultrasound_data/testing_and_validation/registered_data/Exam_0000000_0001/'

# Define the location to of the cropped images
CROPPED_IMAGES_PATH: str = '/home/ben/thyroid_ultrasound_data/experimentation/NonRealTimeSegmentation/Exam_0000000_0001/'

# Define the locations of the ground truth data
GROUND_TRUTH_MASKS_PATH: str = '/home/ben/thyroid_ultrasound_data/experimentation/NonRealTimeSegmentation/Exam_0000000_0001/'

# Define the locations of the segmentation masks
NON_REAL_TIME_SEGMENTATION_MASKS_PATH: str = '/home/ben/thyroid_ultrasound_data/experimentation/NonRealTimeSegmentation/Exam_0000000_0001/'

# Define the location to save the results
SAVE_RESULTS_FILES_PATH: str = '/home/ben/thyroid_ultrasound_data/experimentation/NonRealTimeSegmentation/Exam_0000000_0001/'

# Define the paths in which the data to graph is located
SAVED_RESULTS_FILES_PATHS: tuple = ('', '')

# Define the location to save the resulting graph as an image
GRAPH_IMAGE_PATH: str = ''

#######################################################################################################################

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

# Define the prefix to use for the cropped image files
CROPPED_IMAGES_PREFIX: str = 'CroppedImage'
CROPPED_IMAGES_FILE_EXTENSION: str = '.png'

# Define the prefix to use for the ground truth masks
GROUND_TRUTH_MASKS_PREFIX: str = 'GroundTruth'

# Define the prefix to use for the non-real time segmentation masks
NON_REAL_TIME_SEGMENTATION_MASKS_PREFIX: str = 'NonRealTimeSegmentationMasks'

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

#######################################################################################################################

if view_images or extract_cropped_images or define_ground_truth_masks or extract_segmentation_masks:

    # Load the registered data
    registered_data_objects = load_folder_of_saved_registered_data(NON_REAL_TIME_SEGMENTED_REGISTERED_DATA_PATH)

    # Loop through each registered data object
    for registered_data in registered_data_objects:

        # Note the type of data
        registered_data: RegisteredData

        # Define the time stamp suffix for file names
        time_stamp_suffix = '_' + str(registered_data.image_data.image_capture_time.secs) + \
                            '_' + str(registered_data.image_data.image_capture_time.nsecs)

        # If the purpose is just to view the registered data
        if view_images:
            # Display the images
            imshow("Registered Data Images", registered_data.image_data.original_image)
            waitKey(-1)

        # If the purpose is to extract the cropped images,
        if extract_cropped_images:
            # Create the full path of the image
            imwrite(uri=CROPPED_IMAGES_PATH + CROPPED_IMAGES_PREFIX + time_stamp_suffix + CROPPED_IMAGES_FILE_EXTENSION,
                    image=registered_data.image_data.cropped_image)

        # If the purpose is to define ground truth masks,
        if define_ground_truth_masks:
            # Define the list to store the points that define the foreground
            list_of_foreground_points = None

            # Capture the foreground of the image from the user
            list_of_points_for_foreground_polygon = user_input_polygon_points(
                registered_data.image_data,
                "foreground ground truth",
                display_result=True,
                list_of_points=list_of_foreground_points)

            # Convert the points of the foreground polygons to triangles
            list_of_foreground_triangles = create_convex_triangles_from_points(
                list_of_points_for_foreground_polygon)

            # Generate the ground-truth mask using the triangles selected by the user
            ground_truth_mask = create_mask_array_from_triangles(
                list_of_foreground_triangles,
                registered_data.image_data.cropped_image.shape[:2])

            # Save the ground truth mask
            save(file=GROUND_TRUTH_MASKS_PATH + GROUND_TRUTH_MASKS_PREFIX + time_stamp_suffix,
                 arr=ground_truth_mask)

        # If the purpose is to extract the segmentation masks,
        if extract_segmentation_masks:
            # Save the ground truth mask
            save(file=NON_REAL_TIME_SEGMENTATION_MASKS_PATH + NON_REAL_TIME_SEGMENTATION_MASKS_PREFIX +
                      time_stamp_suffix,
                 arr=registered_data.image_data.image_mask)

if generate_comparison_results:
    # Create a csv file in which to write the results
    data_points_file = open(SAVE_RESULTS_FILES_PATH + RESULTS_FILES_DATA_POINTS_PREFIX +
                            RESULTS_FILES_DATA_POINTS_EXTENSION, mode='w')
    data_points_file_writer = DictWriter(data_points_file, delimeter=',',
                                         fieldnames=[HEADER_IMAGE_NUMBER, HEADER_DICE_SCORE])
    data_points_file_writer.writeheader()

    # Create a text file in which to write the statistical results
    statistics_file = open(SAVE_RESULTS_FILES_PATH + RESULTS_FILES_STATISTICS_Prefix +
                           RESULTS_FILES_STATISTICS_EXTENSION, mode='w')

    # Read in the ground truth masks
    ground_truth_masks = [load(path) for path in generate_ordered_list_of_directory_contents(GROUND_TRUTH_MASKS_PATH,
                                                                                             sort_indices=(0, 1))]

    # Read in the result masks
    segmentation_masks = [load(path) for path in
                          generate_ordered_list_of_directory_contents(NON_REAL_TIME_SEGMENTATION_MASKS_PATH,
                                                                      sort_indices=(0, 1))]

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
    fig, axes = subplots(nrows=1, ncols=1)

    # Define a file counter
    trial_counter = 1

    # For each data file given
    for data_points_file_path in SAVED_RESULTS_FILES_PATHS:

        # If it is a valid path
        if isdir(data_points_file_path):

            # Open the data points file
            data_points_file = open(data_points_file_path + RESULTS_FILES_DATA_POINTS_PREFIX +
                                    RESULTS_FILES_DATA_POINTS_EXTENSION, mode='r')
            data_points_file_reader = DictReader(data_points_file)

            # Read in the data
            data = read_recorded_data_csv(file_path=(SAVE_RESULTS_FILES_PATH + RESULTS_FILES_DATA_POINTS_PREFIX +
                                                     RESULTS_FILES_DATA_POINTS_EXTENSION),
                                          headings=[HEADER_IMAGE_NUMBER, HEADER_DICE_SCORE],
                                          create_combined_stamp=False)

            # Plot the data
            axes.scatter(data[HEADER_IMAGE_NUMBER], data[HEADER_DICE_SCORE],
                         size=2, color=data_colors[trial_counter - 1])
            axes.plot(data[HEADER_IMAGE_NUMBER], data[HEADER_DICE_SCORE], label='Trial ' + str(trial_counter),
                      color=line_colors[trial_counter - 1])

            # Increment the file counter
            trial_counter = trial_counter + 1

    # Set the parameters for the figure
    axes.grid(axis='y')
    axes.set_xlabel(HEADER_IMAGE_NUMBER)
    axes.set_ylabel(HEADER_DICE_SCORE)

    # Add the legend
    axes.legend()

    # Save the figure
    savefig(GRAPH_IMAGE_PATH)
