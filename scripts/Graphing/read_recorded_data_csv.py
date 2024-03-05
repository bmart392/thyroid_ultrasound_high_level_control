# Import standard packages
from os.path import exists
from csv import DictReader

from ExperimentalDataRecorder import STAMP_SECS, STAMP_NSECS

# Define the accepted sort types
SORT_ASCENDING: str = 'Sort Ascending'
SORT_DESCENDING: str = 'Sort Descending'

# Define the header used for the combined time-stamp
COMBINED_STAMP: str = 'Elapsed Time (s)'

# Define the types of data that will be graphed
CONTROLLED = 'controlled'
UNCONTROLLED = 'uncontrolled'


def read_recorded_data_csv(file_path: str,
                           headings: list,
                           sort_heading: str = None,
                           sort_type: str = SORT_ASCENDING,
                           zero_sort_column: bool = False,
                           additional_headings_to_zero: list = None,
                           create_combined_stamp: bool = True):
    # Ensure that the path to the file is valid
    if not exists(file_path):
        raise Exception(file_path + ' is not a valid path.')

    # Ensure that the path points to CSV file
    if file_path[-4:] != '.csv':
        raise Exception(file_path + ' does not point to a CSV file.')

    # Ensure that the headings list contains headings
    if not len(headings) > 0:
        raise Exception("The list of headings cannot be of length zero.")

    # Create a place to store the first data values
    first_row_data_values = {}

    # If additional headings should be zeroed
    if additional_headings_to_zero is not None:

        # Ensure that any additional headings are also in the list of headings
        for heading in additional_headings_to_zero:
            if heading not in headings:
                raise Exception(heading + ' is not in the list of headings.')

        # Add the appropriate fields to the first data dictionary
        for heading in additional_headings_to_zero:
            first_row_data_values[heading] = None

    # Open the CSV file and create a reader for it
    source_file = open(file_path, mode='r')
    source_file_reader = DictReader(source_file)

    # Define where the result will be stored
    final_data_dictionary = {}

    # Add the correct entries into the dictionary
    for heading in headings:
        final_data_dictionary[heading] = []

    # If the data is not being sorted
    if sort_heading is None:
        # Define a flag indicating it is the first row in the dictionary
        first_row_flag = True

        # Add the headings to the dictionary
        for row in source_file_reader:

            # If values should be zeroed
            if additional_headings_to_zero is not None:
                # If it is the first row, pull out the values
                if first_row_flag:
                    for heading in additional_headings_to_zero:
                        first_row_data_values[heading] = float(row[heading])
                        first_row_flag = False

            # Add the value from the given heading to the final dictionary
            for heading in headings:

                # Zero the value if needed
                if len(first_row_data_values.keys()) > 0 and heading in first_row_data_values.keys():
                    final_data_dictionary[heading].append(float(row[heading]) - first_row_data_values[heading])

                # Otherwise add the value as is
                else:
                    final_data_dictionary[heading].append(float(row[heading]))

    else:

        # Ensure that the sort_heading is in the list of headings
        if sort_heading not in headings:
            raise Exception(sort_heading + ' is not contained in list of headings.')

        # Define a flag to indicate if the sort order should be reversed
        if sort_type == SORT_ASCENDING:
            reverse_sort_order = False
        elif sort_type == SORT_DESCENDING:
            reverse_sort_order = True
        else:
            raise Exception(sort_type + ' is not a recognized sorting type.')

        # Define the temporary location where data will be stored
        first_data_dictionary = {}

        # Add the headings to the dictionary
        for row in DictReader(source_file):

            # Define the temporary row dictionary
            row_dictionary = {}

            # Add the value from the given list of headings to the row dictionary
            for heading in headings:
                row_dictionary[heading] = row[heading]

            # Add the new values to the first dictionary
            first_data_dictionary[row[sort_heading]] = row_dictionary

        # Sort the keys of the first dictionary
        sorted_keys = sorted([int(x) for x in first_data_dictionary.keys()], reverse=reverse_sort_order)

        # Add the sort column to zero if needed
        if zero_sort_column:
            first_row_data_values[sort_heading] = None

        # Capture the value from the first row for each heading
        for heading in first_row_data_values.keys():
            first_row_data_values[heading] = float(first_data_dictionary[str(sorted_keys[0])][heading])

        # Create a sorted version of the final dictionary
        for key in [str(x) for x in sorted_keys]:

            # Add the value from the given heading to the final dictionary
            for heading in headings:

                # Zero the value if needed
                if len(first_row_data_values.keys()) > 0 and heading in first_row_data_values.keys():
                    final_data_dictionary[heading].append(float(first_data_dictionary[key][heading]) -
                                                          first_row_data_values[heading])

                # Otherwise add the value as is
                else:
                    final_data_dictionary[heading].append(float(first_data_dictionary[key][heading]))

    # If a combined stamp should be created
    if create_combined_stamp:

        # Ensure the proper headings are included
        if STAMP_SECS not in final_data_dictionary.keys() or STAMP_NSECS not in final_data_dictionary.keys():
            raise Exception('Cannot create a combined time stamp because proper headings are not in file.')

        # Add a combined stamp field
        final_data_dictionary[COMBINED_STAMP] = []

        # Make sure the first stamp is truly at zero
        first_stamp_offset = final_data_dictionary[STAMP_SECS][0] + (final_data_dictionary[STAMP_NSECS][0] / 10 ** 9)

        # Calculate the combined time stamp and add it to the result
        for sec, nsec in zip(final_data_dictionary[STAMP_SECS], final_data_dictionary[STAMP_NSECS]):
            final_data_dictionary[COMBINED_STAMP].append((sec + (nsec / 10 ** 9)) - first_stamp_offset)

    return final_data_dictionary

# if __name__ == '__main__':
#     BASIC_SOURCE_PATH: str = '/home/ben/thyroid_ultrasound_data/experimentation/ForceControl/'
#     controlled_time_stamp = '2024-02-19_13-10-24-970993'
#     controlled_source_path: str = (BASIC_SOURCE_PATH + 'Controlled/' + controlled_time_stamp +
#                                    '_experiment/Force_' + controlled_time_stamp + '.csv')
#     read_recorded_data_csv(file_path=controlled_source_path,
#                            headings=[MESSAGE_ID, STAMP_SECS, STAMP_NSECS, FORCE],
#                            sort_heading=None,
#                            sort_type=SORT_ASCENDING,
#                            zero_sort_column=False,
#                            additional_headings_to_zero=[STAMP_SECS])

