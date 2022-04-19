# canfilter

## Setup

    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    rosmake canfilter
    cd ~/catkin_ws/
    catkin_make
    cd ~/catkin_ws/src/canfilter


## Run

    roslaunch launch/canfilter.launch known_mode_times:="/home/azureorbit/catkin_ws/src/canfilter/known_mode_times.txt" realtime:=false`

    roslaunch launch/canfilter.launch
