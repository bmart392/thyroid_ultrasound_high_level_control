import imageio.v3 as iio
from pygifsicle import optimize

from thyroid_ultrasound_imaging_support.RegisteredData.generate_ordered_list_of_directory_contents import \
    generate_ordered_list_of_directory_contents

RESULT_FOLDER: str = '/home/ben/Documents/Thesis/Visuals'
SOURCE_FOLDER: str = '/home/ben/thyroid_ultrasound_data/experimentation'

# Define the image sources to use
folders = ['BalanceControl', 'BalanceControl', 'BalanceControl', 'BalanceControl', 'BalanceControl', 'BalanceControl']
control_types = ['Uncontrolled', 'Uncontrolled', 'Uncontrolled', 'Controlled', 'Controlled', 'Controlled']
timestamps = ['2024-03-10_10-41-54-154715', '2024-03-10_10-44-54-134848', '2024-03-10_10-47-30-414361',
              '2024-03-10_10-54-15-624089', '2024-03-10_11-56-29-382349', '2024-03-10_12-04-31-643835']
down_sampling_rates = [15, 15, 15, 16, 28, 15]
# folders = ['ImageControl', 'ImageControl', 'ImageControl', 'ImageControl', 'ImageControl', 'ImageControl']
# control_types = ['Uncontrolled', 'Uncontrolled', 'Uncontrolled', 'Controlled', 'Controlled', 'Controlled']
# timestamps = ['2024-03-10_14-15-50-095346', '2024-03-10_14-21-52-993521', '2024-03-10_14-31-34-704117',
#               '2024-03-10_12-47-29-098133', '2024-03-10_14-08-58-467311', '2024-03-10_14-12-28-780537']
# down_sampling_rates = [15, 15, 15, 16, 28, 15]

for folder, control_type, timestamp, down_sampling_rate in zip(folders, control_types,
                                                               timestamps, down_sampling_rates):

    # Generate the path to the images
    source_path = SOURCE_FOLDER + '/' + folder + '/' + control_type + '/' + \
                  timestamp + '_experiment/RawImages_' + timestamp

    # Define the name of the gif
    gif_name = folder + '_' + control_type + '_' + timestamp + '.gif'

    # Generate the name of the result gif
    gif_path = RESULT_FOLDER + '/' + gif_name

    # Capture the images in the source folder in order
    images = [iio.imread(path) for path in generate_ordered_list_of_directory_contents(
                directory_path=source_path,
                sort_indices=(0, 1))[::down_sampling_rate]]

    # Print the number of images included in the gif
    print('Number of images in ' + gif_name + ': ' + str(len(images)))

    # Create the gif
    iio.imwrite(gif_path, images)

    # Optimize the gif
    # optimize(gif_path)