#! /bin/bash
# This script safely launches ros nodes with buffer time to allow param server population
tmux split-window "roslaunch kuka_arm target_description.launch" &
sleep 3 &&
tmux split-window "roslaunch kuka_arm cafe.launch" &
sleep 3 &&
tmux split-window "roslaunch kuka_arm spawn_target.launch" &
sleep 5 &&
tmux split-window "rosrun kuka_arm IK_server.py" &&
#x-terminal-emulator -e roslaunch kuka_arm inverse_kinematics.launch
roslaunch kuka_arm inverse_kinematics.launch
