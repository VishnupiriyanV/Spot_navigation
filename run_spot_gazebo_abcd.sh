#!/bin/bash

echo "🚀 Starting Spot Robot in Gazebo with A→B→C→D Navigation"
echo "======================================================="

cd /home/enma/pudusu/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✅ Environment sourced"

cleanup() {
    echo ""
    echo "🛑 Shutting down..."
    pkill -f "ros2 launch"
    pkill -f "rviz2"
    pkill -f "python3.*"
    pkill -f "gz sim"
    echo "✅ Cleanup complete"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "🎮 Launching Spot simulation in Gazebo..."

ros2 launch champ_bringup spot_bringup.launch.py \
    world_file:=/home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_gazebo/worlds/empty_room.sdf \
    x:=0.0 y:=0.0 z:=0.5 &

echo "⏳ Waiting for Gazebo to load (25 seconds)..."
sleep 25

echo "🎨 Launching RViz..."
rviz2 -d /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/rviz/default_view.rviz &

sleep 5

echo "🚶 Starting A→B→C→D waypoint navigation..."
echo ""
echo "The robot will walk through waypoints:"
echo "  📍 A → B → C → D"
echo "  🦵 With visible leg movements"
echo "  🎯 Large joint motions for clear visualization"
echo ""

python3 /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/scripts/visible_walking_abcd.py

echo ""
echo "🎉 A→B→C→D navigation completed!"
echo "🟢 Gazebo and RViz are still running"
echo "💡 Press Ctrl+C to shutdown everything"

while true; do
    sleep 1
done
