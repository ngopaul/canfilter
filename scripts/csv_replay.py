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

from std_msgs.msg import String, Header, Float64
import pandas as pd
import rospy
import sys, time
import numpy as np


class csv_replayer():
    def __init__(self, ns, csvfile, time_col, bus_col, msg_id_col, msg_col, msg_length_col, realtime, msg_topic_name,
                 **kwargs):
        print(f"Initializing csv_replay, ns: {ns}, csvfile: {csvfile}, time_col: {time_col}, "
              f"msg_id_col: {msg_id_col}, msg_col: {msg_col}")
        self.ns = ns
        rospy.init_node("replayer", anonymous=True)

        ## Publishers
        if realtime:
            # we know that the pub-sub (4.8GHz processor) can handle the messages being streamed at a CONSTANT rate
            # faster than realtime on all messages. (w/ blocking pub-sub, gets through 100s of real messages in 59.8s)
            # But this doesn't account for the timing of how the messages are burst on the realtime vehicle.
            # Therefore, make the queue size to handle a burst of all ~200 messages broadcasting at the same time
            self.msg_publisher = rospy.Publisher(msg_topic_name, String, queue_size=200)
        else:
            # don't drop a single message - block until each message is processed
            self.msg_publisher = rospy.Publisher(msg_topic_name, String, queue_size=None)

        self.data = pd.DataFrame()
        dataframe = pd.read_csv(csvfile)
        dataframe.dropna(inplace=True)

        map_csv_columns_to_desired_name = {
            time_col: "Time",
            bus_col: "Bus",
            msg_id_col: "MessageID",
            msg_col: "Message",
            msg_length_col: "MessageLength"
        }

        for csv_column_name, desired_column_name in map_csv_columns_to_desired_name.items():
            if csv_column_name in dataframe.columns:
                self.data[desired_column_name] = dataframe[csv_column_name]
            else:
                raise KeyError("{} column not available in {}".format(csv_column_name, csvfile))

        # Check for monotonicity of time
        time_diff = np.diff(self.data['Time'])
        if not np.all(time_diff) >= 0:
            raise ValueError("Time is not monotonically increasing in the provided dataset")

        # print("Data:", self.data)

        self.current_time = None
        self.next_time = None

    def publish(self):
        """
        Publish Function
        """

        self.current_time = self.data.iloc[0]['Time']
        next_bus = self.data.iloc[0]['Bus']
        next_msg_id = self.data.iloc[0]['MessageID']
        next_msg = self.data.iloc[0]['Message']
        next_msg_length = int(self.data.iloc[0]['MessageLength'])  # is default float, force to int

        # print(f"Time: {self.current_time}, Message: {next_msg}")

        if self.data.shape[0] == 1:
            self.next_time = -1  # -1 will signify that it is time to step publishing when last row has been read
            return

        self.next_time = self.data.iloc[1]['Time']

        # Remove the row just read from the dataframe
        self.data = self.data.iloc[1:]

        new_message = f"{self.current_time} {next_bus} {next_msg_id} {next_msg} {next_msg_length}"

        self.msg_publisher.publish(new_message)


def main(argv):
    ns = rospy.get_namespace()  # Retrieve namespace this way appends '/' at the end as well,
    ns = ns[0:-1]

    csvfile = argv[0]
    time_col = argv[1]
    bus_col = argv[2]
    msg_id_col = argv[3]
    msg_col = argv[4]
    msg_length_col = argv[5]
    realtime = argv[6]
    msg_topic_name = argv[7]
    realtime = (realtime == 'true')
    node = csv_replayer(ns, csvfile, time_col, bus_col, msg_id_col, msg_col, msg_length_col, realtime, msg_topic_name)

    # rate = rospy.Rate(20) # 20 Hz publish rate

    while not rospy.is_shutdown():
        # print(f"{rospy.get_param('execute', False)}")
        # if rospy.get_param("/execute", False):
        node.publish()
        if node.next_time == -1:
            break
        deltaT = node.next_time - node.current_time
        if realtime:
            time.sleep(deltaT)


if __name__ == '__main__':
    main(sys.argv[1:])
