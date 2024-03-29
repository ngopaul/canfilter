#!/usr/bin/env python

# Author: Paul Ngo
# Initial Date: Apr 18, 2022
# License: MIT License

#   Permission is hereby granted, free of charge, to any person obtaining
#   a copy of this software and associated documentation files
#   (the "Software"), to deal in the Software without restriction, including
#   without limitation the rights to use, copy, modify, merge, publish,
#   distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject
#   to the following conditions:

#   The above copyright notice and this permission notice shall be
#   included in all copies or substantial portions of the Software.

"""
Allow the user to specify when the car goes into different discrete states.

This allows distinct modes to be filtered out.
"""
import sys
import time
import pickle
import os

from utils import *
from threading import Lock
import rospy
from std_msgs.msg import String, Header, Float64


class CarMode(Enum):
    UNKNOWN = -1
    PARKING = 0
    REVERSE = 1
    DRIVE_NO_CRUISE = 2
    DRIVE_CRUISE_ENABLED = 3
    DRIVE_CRUISE_ACTIVE = 4
    OTHER = 5


mode = CarMode.UNKNOWN
# Map (Bus, MessageID) to a list of the boundaries within that ID which are cruise control on (but no value is set)
cruise_enabled_messages = {}
# Map (Bus, MessageID) to a list of the boundaries within that ID which are cruise control on AND active (speed value is set)
cruise_active_messages = {}
# Map (Bus, MessageID) to a list of binary values representing whether that bit has changed yet
changed_messages = {}
# Map (Bus, MessageID) to the binary value recorded from the CAN bus.
previous_messages = {}
# Map messages to their maximum lengths in bits
message_id_to_bit_length = {}

mode_lock = Lock()
known_mode_index = 0


def process_can_message(message: Dict):
    """
    Process a can message, filtering out irrelevant information
    :param message:
    :return:
    """
    # if the state is unknown, then we're in the middle of switching states. Ignore it.
    if mode == CarMode.UNKNOWN:
        return

    bus = message["Bus"]
    message_id = message["MessageID"]
    size_in_bytes = int(len(message["Message"]) // 2)
    size_in_bits = size_in_bytes * 8
    previous_max_bit_length = message_id_to_bit_length[message_id]
    message_id_to_bit_length[message_id] = max(size_in_bits, message_id_to_bit_length.get(message_id, 0))
    max_bit_length = message_id_to_bit_length[message_id]
    
    if max_bit_length > previous_max_bit_length:
        # pad the other arrays to the correct length
        pad_width = (max_bit_length - previous_max_bit_length, 0)
        if bus_msg_id in changed_messages:
            changed_messages[bus_msg_id] = np.pad(changed_messages[bus_msg_id], pad_width, constant_values=False)
        if bus_msg_id in previous_messages:
            previous_messages[bus_msg_id] = np.pad(previous_messages[bus_msg_id], pad_width, constant_values=False)
        if bus_msg_id in cruise_enabled_messages:
            cruise_enabled_messages[bus_msg_id] = np.pad(cruise_enabled_messages[bus_msg_id], pad_width, constant_values=False)
        if bus_msg_id in cruise_active_messages:
            cruise_active_messages[bus_msg_id] = np.pad(cruise_active_messages[bus_msg_id], pad_width, constant_values=False)
        
    
    current_message = hex_to_binary(message["Message"], pad_to=max_bit_length, return_np_bool_array=True)
    bus_msg_id = (bus, message_id)

    if mode == CarMode.DRIVE_CRUISE_ENABLED:
        if bus_msg_id not in previous_messages:
            # a new message appeared when we entered the DRIVE_CRUISE_ENABLED state, this message must be relevant
            cruise_enabled_messages[bus_msg_id] = np.ones(max_bit_length).astype(bool)
        else:
            if bus_msg_id not in cruise_enabled_messages:
                cruise_enabled_messages[bus_msg_id] = np.zeros(max_bit_length).astype(bool)
            # otherwise, the relevant bits are only the ones that were not changing before, but also have changed now
            cruise_enabled_messages[bus_msg_id] |= ~changed_messages[bus_msg_id] & \
                                                  (previous_messages[bus_msg_id] != current_message)
    elif mode == CarMode.DRIVE_CRUISE_ACTIVE:
        if bus_msg_id not in previous_messages:
            # a new message appeared when we entered the DRIVE_CRUISE_ACTIVE state, this message must be relevant
            cruise_active_messages[bus_msg_id] = np.ones(64).astype(bool)
        else:
            if bus_msg_id not in cruise_active_messages:
                cruise_active_messages[bus_msg_id] = np.zeros(max_bit_length).astype(bool)
            # otherwise, the relevant bits are only the ones that were not changing before, but also have changed now
            cruise_active_messages[bus_msg_id] |= ~changed_messages[bus_msg_id] & \
                                                  (previous_messages[bus_msg_id] != current_message)
    elif bus_msg_id in previous_messages:
        # make sure the cruise messages don't include anything that changed either; but skip the first time around
        if bus_msg_id not in cruise_enabled_messages:
            cruise_enabled_messages[bus_msg_id] = np.zeros(max_bit_length).astype(bool)
        cruise_enabled_messages[bus_msg_id] &= (previous_messages[bus_msg_id] == current_message)
        if bus_msg_id not in cruise_active_messages:
            cruise_active_messages[bus_msg_id] = np.zeros(max_bit_length).astype(bool)
        cruise_active_messages[bus_msg_id] &= (previous_messages[bus_msg_id] == current_message)

    if bus_msg_id not in previous_messages:
        # we haven't seen the message before, initialize as messages have changed yet
        changed_messages[bus_msg_id] = np.zeros(max_bit_length).astype(bool)
    else:
        # log the change in messages
        changed_messages[bus_msg_id] |= (previous_messages[bus_msg_id] != current_message)
    previous_messages[bus_msg_id] = current_message


def process_ros_message(data, known_mode_time_and_state=None):
    time_i, bus, message_id, message, message_length = data.data.split(" ")
    time_i = float(time_i)
    message_id = int(message_id)

    global known_mode_index, mode
    if known_mode_time_and_state:
        while known_mode_index < len(known_mode_time_and_state) and \
                known_mode_time_and_state[known_mode_index][0] <= time_i:
            mode_lock.acquire()
            try:
                mode = known_mode_time_and_state[known_mode_index][1]
            finally:
                mode_lock.release()
            print(f"known_mode_time {known_mode_time_and_state[known_mode_index][0]} < present time {time_i}: "
                  f"CarMode changed to: {mode}")
            known_mode_index += 1

    process_can_message(
        {
            "Time": time,
            "Bus": bus,
            "MessageID": message_id,
            "Message": message
        }
    )


def main(argv):
    global mode
    ns = rospy.get_namespace()  # Retrieve namespace this way appends '/' at the end as well,
    ns = ns[0:-1]

    known_mode_times = argv[0]
    msg_topic_name = argv[1]
    output_loc = argv[2]
    known_mode_time_and_state = None
    if not known_mode_times == "None":
        print(f"Running with known_mode_times from file {known_mode_times}.")
        known_mode_time_and_state = []
        with open(known_mode_times, 'r') as f:
            for line in f.readlines():
                time_i, mode = line.strip().split(" ")
                time_i = float(time_i)
                mode = CarMode.__members__[mode]
                known_mode_time_and_state.append((time_i, mode))
    else:
        print("Using user input to determine the car mode.")

    rospy.init_node('mode_setter', anonymous=True)
    subscriber = rospy.Subscriber(msg_topic_name, String, lambda data: process_ros_message(data, known_mode_time_and_state))

    print("Known mode times:", known_mode_times)

    if known_mode_times == "None":
        while not rospy.is_shutdown():
            print("Possible CarMode Values:")
            for val in CarMode.__members__.values():
                print(val.__repr__)
            print(f"Current CarMode is: {mode}.")
            new_mode = input("Input Next CarMode: ")

            if new_mode in ['q', 'exit', 'quit', 'stop']:
                break

            mode_lock.acquire()
            try:
                mode = CarMode(int(new_mode))
            finally:
                mode_lock.release()
    else:
        rospy.spin()

    with open(os.path.join(output_loc, "cruise_enabled_messages.pickle"), "wb") as f:
        pickle.dump(cruise_enabled_messages, f)
    with open(os.path.join(output_loc, "cruise_active_messages.pickle"), "wb") as f:
        pickle.dump(cruise_active_messages, f)

    print("cruise_enabled_messages")
    for k, v in cruise_enabled_messages.items():
        if any(v):
            print(f"{k}: {v}")
    print("cruise_active_messages")
    for k, v in cruise_active_messages.items():
        if any(v):
            print(f"{k}: {v}")


if __name__ == '__main__':
    main(sys.argv[1:])
