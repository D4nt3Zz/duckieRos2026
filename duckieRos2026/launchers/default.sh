#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# launching app
chmod +x $DT_REPO_PATH/packages/mission_controller/src/mission_controller_node.py
chmod +x $DT_REPO_PATH/packages/lane_follow/src/lane_follow_node.py
chmod +x $DT_REPO_PATH/packages/led_emitter/src/led_emitter_node.py


dt-exec roslaunch mission_controller mission_controller_node.launch veh:=myduckiebot02
# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join