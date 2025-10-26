#!/bin/bash
# Quick Spot Robot Launcher - One Command Does Everything
cd /home/enma/pudusu/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && echo "🚀 Starting Spot Robot..." && ros2 launch champ_bringup spot_bringup.launch.py world_file:=/home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_gazebo/worlds/empty_room.sdf x:=0.0 y:=0.0 z:=0.5 &
sleep 25 && echo "🎨 Starting RViz..." && rviz2 -d /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/rviz/default_view.rviz &
sleep 5 && echo "🤖 Starting robot movement..." && python3 /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/scripts/simple_working_movement.py
