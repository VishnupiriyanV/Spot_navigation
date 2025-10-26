#!/bin/bash

echo "🤖 Simple Spot Robot in Gazebo"
echo "==============================="

cd /home/enma/pudusu/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "🚀 Launching Spot in Gazebo..."

ros2 launch champ_bringup spot_bringup.launch.py \
    world_file:=/home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_gazebo/worlds/empty_room.sdf \
    x:=0.0 y:=0.0 z:=0.5
