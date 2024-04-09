# Import custom python packages
from ExperimentalDataRecorder import FORCE, POSE_Z, POSE_ROLL, SKIN_ERROR, IMAGE_CENTROID
from Graphing.plot_standard_line_graph import plot_standard_line_graph

# ----------------------------------------------------------------------------------------------------------------------

# Define the sources of the force information
force_source_path: str = '/home/ben/thyroid_ultrasound_data/experimentation/ForceControl/'
force_time_stamps = ['2024-02-19_13-05-58-231870', '2024-02-19_13-07-40-867184', '2024-02-19_13-08-56-948372',
                     '2024-02-19_13-10-24-970993', '2024-02-19_13-11-58-761400', '2024-02-19_13-13-17-885949']

# Define the sources of the balance information
balance_source_path: str = '/home/ben/thyroid_ultrasound_data/experimentation/BalanceControl/'
balance_time_stamps = ['2024-03-10_10-41-54-154715', '2024-03-10_10-44-54-134848', '2024-03-10_10-47-30-414361',
                       '2024-03-10_10-51-05-732145', '2024-03-10_10-54-15-624089', '2024-03-10_12-04-31-643835']

# Define the sources of the image information
image_source_path: str = '/home/ben/thyroid_ultrasound_data/experimentation/ImageControl/'
Image_time_stamps = ['2024-03-10_14-15-50-095346', '2024-03-10_14-21-52-993521', '2024-03-10_14-31-34-704117',
                     '2024-03-10_12-47-29-098133', '2024-03-10_14-08-58-467311', '2024-03-10_14-12-28-780537']

# ----------------------------------------------------------------------------------------------------------------------

# Print a divider for clarity
print('-' * 10)

# Create force graph
plot_standard_line_graph(basic_source_path=force_source_path, time_stamps=tuple(force_time_stamps),
                         heading_to_plot=FORCE, estimation_line_deg=8, point_sampling_rate=20,
                         statistics_text_to_show='Force')
# Create pose graph
plot_standard_line_graph(basic_source_path=force_source_path, time_stamps=tuple(force_time_stamps),
                         heading_to_plot=POSE_Z, estimation_line_deg=8, point_sampling_rate=20)
# Create balance graph
plot_standard_line_graph(basic_source_path=balance_source_path, time_stamps=tuple(balance_time_stamps),
                         heading_to_plot=SKIN_ERROR, estimation_line_deg=4, point_sampling_rate=1,
                         y_label_text='Slope of Approximation Line', statistics_text_to_show='Balance Error')
# Create pose graph
plot_standard_line_graph(basic_source_path=balance_source_path, time_stamps=tuple(balance_time_stamps),
                         heading_to_plot=POSE_ROLL, estimation_line_deg=8, point_sampling_rate=20,
                         y_label_text='Roll Angle of Probe\n w.r.t. Robot Origin (deg)')

# Create centroid graph
plot_standard_line_graph(basic_source_path=image_source_path, time_stamps=tuple(Image_time_stamps),
                         heading_to_plot=IMAGE_CENTROID, estimation_line_deg=12, point_sampling_rate=1,
                         y_label_text='Centroid Error (px)', statistics_text_to_show='Centroid Error',
                         show_plot=True)
