#!/usr/bin/env sh

# Absolute path to this script, e.g. /home/user/bin/foo.sh
SCRIPT=$(readlink -f "$0")
# Absolute path this script is in, thus /home/user/bin
SCRIPTPATH=$(dirname "$SCRIPT")

# $SCRIPTPATH/sample_webcam_publisher.sh; $SCRIPTPATH/sample_webcam_subscriber.sh; 
ros2 run turtlesim turtlesim_node & ros2 run turtlesim turtle_teleop_key;