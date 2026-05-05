#!/bin/bash
# Source this file to enter the handheld_mapping workspace
# Usage: source /home/argen/my_robot_ws/handheld_mapping_ws/setup_env.sh

source /opt/ros/humble/setup.bash
source /home/argen/my_robot_ws/handheld_mapping_ws/install/setup.bash

echo "[handheld_mapping] Environment ready."
echo "  Launch: ros2 launch handheld_mapping handheld_mapping.launch.py lidar_model:=TminiPro"
echo "  Save:   ros2 run handheld_mapping save_map"
