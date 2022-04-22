# canfilter

This ROS package implements a way for two people to quickly decode cruise control 
messages on a vehicle, live. It is testable by replaying a CSV file with CAN messages in it.

To test with such a CSV file, it must have the headers:

    Time | Bus | MessageID | Message | MessageLength

where time is the UNIX timestamp, bus is the bus on which the CAN message was delivered
(0 default), MessageID is the ID of the CAN message, Message is the hexademical string 
containing the message payload, and MessageLength is the length of the message in bytes.

The quickstart and commands sections below describe to test with a CSV file, as well as
testing live on a vehicle. If testing live on a vehicle, a ROS node must be publishing
to the relevant ROS topic as described in the quickstart.

This ROS package was built with catkin. 

`git clone https://github.com/ngopaul/canfilter.git` in `~/catkin_ws/src/` before
running commands.

## Requirements

- ROS. Tested with ROS Noetic on Ubuntu 20.04 LTSm, using catkin.
- Python 3
- [CAN-to-ROS]("https://github.com/jmscslgroup/can_to_ros/") if running on a vehicle

## Setup Commands

    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    cd ~/catkin_ws/src/canfilter
    rosmake canfilter
    cd ~/catkin_ws/
    catkin_make
    cd ~/catkin_ws/src/canfilter

## Run Commands

If you set `known_mode_times`, the `known_mode_times` file should be populated as demonstrated
in the example. The possible states are:

- UNKNOWN = -1
- PARKING = 0
- REVERSE = 1
- DRIVE_NO_CRUISE = 2
- DRIVE_CRUISE_ENABLED = 3
- DRIVE_CRUISE_ACTIVE = 4
- OTHER = 5

Otherwise, the terminal in which you run `roslaunch` will prompt you live to change the mode,
in which you must type the integer of the mode you wish to change to. While running this live on 
a vehicle ALWAYS switch to UNKNOWN (-1) before physically pressing something in the vehicle,
then after changing to that mode in the vehicle, switch to that mode in the program. This
will ensure you never incorrectly label a mode.

    roslaunch launch/canfilter.launch known_mode_times:="/home/azureorbit/catkin_ws/src/canfilter/known_mode_times.txt" realtime:=false

    roslaunch launch/canfilter.launch

## Quickstart (csv_replay)

1. Change the argument `csvfile` in `canfilter.launch` to the location of the driving file
`2020-07-08-15-15-54_2T3MWRFVXLW056972_CAN_Messages.csv`. This the the file for which 
`known_mode_times.txt` is accurate. File is Rahul Bhadani's.
2. Change the argument `output_loc` in `canfilter.launch` to the location of the output
folder where the pickle files of message_relevancy_dict should be saved.
3. Run the setup commands
4. Run one of the run commands. To get faster results, set `realtime` to false, otherwise
the program will attempt to simulate real-time playback of the CAN messages.
5. Use a python terminal and pickle to load the pickled `cruise_enabled_messages.pickle`
and `cruise_active_messages.pickle`. These files can be passed directly into `score_messages()` 
in utils.py to return a score for each message ID.


## Quickstart (on vehicle)

Run these commands as the passenger in the vehicle, giving instructions to the driver as 
necessary.

1. Change the argument `output_loc` in `canfilter.launch` to the location of the output
folder where the pickle files of message_relevancy_dict should be saved (`azureorbit` is the 
name of my personal machine). 
2. In [CAN-to-ROS]("https://github.com/jmscslgroup/can_to_ros/"), modify `src/vehicle_interface.cpp`,
[line 180 - 183](https://github.com/jmscslgroup/can_to_ros/blob/8783cee220b919eca2785576b99ccdde86c23656/src/vehicle_interface.cpp#L180-L182),
to remove the "if" statement. We want to publish ALL messages. 
3. Comment out the `csv_replay.py` node in `canfilter.launch`
4. Run the [setup commands](#setup-commands)
5. Start CAN-to-ROS. Verify that `/realtime_raw_data` is publishing with the command, 
`rostopic echo -c /realtime_raw_data`.
6. Run the second [run command](#run-commands) (with no arguments). This will start listening to the ROS topic 
with the raw CAN messages. By default, the car mode will be UNKNOWN (-1). A prompt will show
up asking for the Car Mode to change to.
7. As the driver drives the vehicle, the passenger should direct them as to when they should
switch to a different mode. The passenger should always change to mode UNKNOWN (-1) before
telling the driver to press anything that would change a mode on the vehicle. After the driver
has made the mode change, the passenger can then type the integer corresponding to the new mode.
8. Enter `exit` or `quit` to exit listening. Then, the pickle files of message_relevancy_dict 
should be saved to `output_loc`.

## Example python script to process output pickled files

After the ROS processes have finished, two pickled files are generated according to the
command-line argument `output_loc`. These pickled files are message relevancy dictionaries
which store a mapping from message ID to bit-wise relevancy of each bit in the message (`True`
if possibly relevant, `False` otherwise).

    import pickle
    with open("cruise_active_messages.pickle", "rb") as f:
      cruise_active_messages = pickle.load(f)

    with open("cruise_enabled_messages.pickle", "rb") as f:
      cruise_enabled_messages = pickle.load(f)

    from scripts.utils import *

    cruise_active_scores = score_messages(cruise_active_messages)
    cruise_enabled_scores = score_messages(cruise_enabled_messages)

    cruise_active_scores.head()
    cruise_enabled_scores.head()