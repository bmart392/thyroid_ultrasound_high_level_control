#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, WrenchStamped
# from franka_msgs import FrankaState
# from rv_msgs.msg import ManipulatorState
from std_msgs.msg import Float64, String, Bool
from numpy import sign, array, zeros, sqrt, sum
import tkinter as tk


class TestInterface:

    def __init__(self):

        # Create empty window
        self.window = tk.Tk(className="Topic Helper")
        self.window.rowconfigure(13, minsize=50)
        self.window.columnconfigure([0, 1, 2], minsize=50)

        current_row = 0

        subscribers_label = tk.Label(text="Subscribers")
        subscribers_label.grid(row=current_row, column=0, columnspan=3)

        current_row = current_row + 1

        current_row = self.create_labels(["Topic Name", "Received Value"], [1, 2], current_row)

        """subscribers_topic_name_label = tk.Label(text="Topic Name", padx=10)
        subscribers_topic_name_label.grid(row=current_row, column=0)
        subscribers_published_value_label = tk.Label(text="Received Value", padx=10)
        subscribers_published_value_label.grid(row=current_row, column=1, columnspan=2)

        current_row = current_row + 1"""

        subscriber_topic_names = [
            "/status/thyroid_shown",
            "/status/thyroid_centered",
            "/status/goal_centered"
        ]

        subscriber_message_types = [
            "Bool",
            "Bool",
            "Bool"
        ]

        self.subscriber_value_labels = []

        for topic_name, message_type in zip(subscriber_topic_names, subscriber_message_types):
            new_label = tk.Label(text=topic_name)
            new_label.grid(row=current_row, column=0)

            new_label = tk.Label(text=str(False))
            new_label.grid(row=current_row, column=1, columnspan=2)

            self.subscriber_value_labels.append(new_label)

            current_row = current_row + 1

        empty_space_label = tk.Label(text="")
        empty_space_label.grid(row=current_row, column=0, columnspan=4)

        current_row = current_row + 1

        publishers_label = tk.Label(text="Publishers")
        publishers_label.grid(row=current_row, column=0, columnspan=3)
        current_row = current_row + 1

        current_row = self.create_labels(["Topic Name", "Published Value", "Publishing Status"], [1, 1, 1], current_row)


        """# Define publisher message type options
        message_types = [
            "Bool",
            "TwistStamped",
            "WrenchStamped"
        ]

        # Create two encompassing frames
        publisher_frame = tk.Frame(master=self.window)
        subscriber_frame = tk.Frame(master=self.window)

        # Create publisher sub-frames
        publisher_label_frame = tk.Frame(master=publisher_frame)
        publisher_create_frame = tk.Frame(master=publisher_frame)
        publisher_create_topic_name_frame = tk.Frame(master=publisher_create_frame)
        publisher_create_message_type_frame = tk.Frame(master=publisher_create_frame)
        publisher_create_button_frame = tk.Frame(master=publisher_create_frame)
        self.publisher_display_frame = tk.Frame(master=publisher_frame)

        # Create subscriber sub-frames
        subscriber_label_frame = tk.Frame(master=subscriber_frame)
        subscriber_create_frame = tk.Frame(master=subscriber_frame)
        subscriber_create_topic_name_frame = tk.Frame(master=subscriber_create_frame)
        subscriber_create_message_type_frame = tk.Frame(master=subscriber_create_frame)
        subscriber_create_button_frame = tk.Frame(master=subscriber_create_frame)
        self.subscriber_display_frame = tk.Frame(master=subscriber_frame)

        # Fill in publisher label frame
        publisher_label = tk.Label(master=publisher_label_frame, text="Publishers")
        publisher_label.pack()

        # Fill in publisher creator frame
        publisher_create_topic_name_label = tk.Label(master=publisher_create_topic_name_frame, text="Topic Name")
        publisher_create_topic_name_label.pack(side=tk.LEFT, fill=tk.BOTH)

        publisher_create_topic_name_entry = tk.Entry(master=publisher_create_topic_name_frame)
        publisher_create_topic_name_entry.pack(side=tk.RIGHT, fill=tk.BOTH)

        publisher_create_message_type_label = tk.Label(master=publisher_create_message_type_frame, text="Message Type")
        publisher_create_message_type_label.pack(side=tk.LEFT, fill=tk.BOTH)

        publisher_create_message_type_variable = tk.StringVar(self.window)
        publisher_create_message_type_entry = tk.OptionMenu(publisher_create_message_type_frame,
                                                            publisher_create_message_type_variable,
                                                            *message_types)
        publisher_create_message_type_entry.pack(side=tk.RIGHT, fill=tk.BOTH)

        publisher_create_button = tk.Button(publisher_create_button_frame,
                                            text="Create", command=self.create_new_publisher)
        publisher_create_button.pack(fill=tk.BOTH)

        # Create blank publisher display frame
        self.publisher_display_frame.rowconfigure(0, minsize=50)
        self.publisher_display_frame.columnconfigure([0, 1, 2, 3], minsize=50)
        publisher_display_name_label = tk.Label(text="Topic Name")
        publisher_display_value_label = tk.Label(text="Published Value")
        publisher_display_status_label = tk.Label(text="Publisher Status")
        publisher_display_delete_label = tk.Label(text="")

        # Fill in subscriber label frame
        subscriber_label = tk.Label(master=subscriber_label_frame, text="Subscribers")
        subscriber_label.pack()

        # Fill in subscriber creator frame
        subscriber_create_topic_name_label = tk.Label(master=subscriber_create_topic_name_frame, text="Topic Name")
        subscriber_create_topic_name_label.pack(side=tk.LEFT, fill=tk.BOTH)

        subscriber_create_topic_name_entry = tk.Entry(master=subscriber_create_topic_name_frame)
        subscriber_create_topic_name_entry.pack(side=tk.RIGHT, fill=tk.BOTH)

        subscriber_create_message_type_label = tk.Label(master=subscriber_create_message_type_frame,
                                                        text="Message Type")
        subscriber_create_message_type_label.pack(side=tk.LEFT, fill=tk.BOTH)

        subscriber_create_message_type_variable = tk.StringVar(self.window)
        subscriber_create_message_type_entry = tk.OptionMenu(subscriber_create_message_type_frame,
                                                             subscriber_create_message_type_variable,
                                                             *message_types)
        subscriber_create_message_type_entry.pack(side=tk.RIGHT, fill=tk.BOTH)

        subscriber_create_button = tk.Button(subscriber_create_button_frame,
                                             text="Create", command=self.create_new_subscriber)
        subscriber_create_button.pack(fill=tk.BOTH)

        # Pack out all frames
        publisher_label_frame.pack(fill=tk.BOTH, expand=True)
        publisher_create_topic_name_frame.pack(fill=tk.BOTH, expand=True)
        publisher_create_message_type_frame.pack(fill=tk.BOTH, expand=True)
        publisher_create_button_frame.pack(fill=tk.BOTH, expand=True)
        publisher_create_frame.pack(fill=tk.BOTH, expand=True)
        self.publisher_display_frame.pack(fill=tk.BOTH, expand=True)
        publisher_frame.pack(fill=tk.BOTH, expand=True)

        subscriber_label_frame.pack(fill=tk.BOTH, expand=True)
        subscriber_create_topic_name_frame.pack(fill=tk.BOTH, expand=True)
        subscriber_create_message_type_frame.pack(fill=tk.BOTH, expand=True)
        subscriber_create_button_frame.pack(fill=tk.BOTH, expand=True)
        subscriber_create_frame.pack(fill=tk.BOTH, expand=True)
        self.subscriber_display_frame.pack(fill=tk.BOTH, expand=True)
        subscriber_frame.pack(fill=tk.BOTH, expand=True)"""

        self.window.mainloop()

    def create_new_publisher(self):
        pass

    def create_new_subscriber(self):
        pass

    @staticmethod
    def create_labels(label_texts, column_widths, current_row_index, list_of_labels=None):
        current_column_index = 0
        for label_text, column_width in zip(label_texts, column_widths):
            new_label = tk.Label(text=label_text, background="green")
            new_label.grid(row=current_row_index, column=current_column_index, columnspan=column_width)
            current_column_index = current_column_index + column_width

            if list_of_labels is not None:
                list_of_labels.append(new_label)

        if list_of_labels is not None:
            return current_row_index + 1, list_of_labels

        return current_row_index + 1


if __name__ == '__main__':
    TestInterface()

    """window = tk.Tk()

    frame_a = tk.Frame()
    frame_b = tk.Frame()

    label_a = tk.Label(master=frame_a, text="I'm in Frame A")
    label_a.pack()

    label_b = tk.Label(master=frame_b, text="I'm in Frame B")
    label_b.pack()

    frame_a.pack()
    frame_b.pack()

    window.mainloop()"""
