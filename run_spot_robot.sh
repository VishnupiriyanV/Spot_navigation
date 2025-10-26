#!/bin/bash

# Complete Spot Robot Launch Script
# This single command launches everything and runs the robot

echo "🤖 Starting Spot Robot Navigation System..."
echo "================================================"

# Set up environment
export ROS_DOMAIN_ID=0
cd /home/enma/pudusu/ros2_ws

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✅ Environment configured"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down Spot Robot system..."
    pkill -f "ros2 launch"
    pkill -f "rviz2"
    pkill -f "python3.*simple_working_movement"
    pkill -f "ign gazebo"
    echo "✅ Cleanup complete"
    exit 0
}

# Set up cleanup trap
trap cleanup SIGINT SIGTERM

echo "🚀 Launching Spot simulation..."

# Launch the simulation in background
ros2 launch champ_bringup spot_bringup.launch.py \
    world_file:=/home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_gazebo/worlds/empty_room.sdf \
    x:=0.0 y:=0.0 z:=0.5 &

LAUNCH_PID=$!

echo "⏳ Waiting for simulation to initialize (30 seconds)..."
sleep 30

# Check if launch is still running
if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "❌ Simulation failed to start"
    exit 1
fi

echo "✅ Simulation started successfully"

echo "🎨 Launching RViz visualization..."

# Launch RViz in background
rviz2 -d /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/rviz/default_view.rviz &

RVIZ_PID=$!

echo "⏳ Waiting for RViz to load (5 seconds)..."
sleep 5

echo "✅ RViz launched successfully"

echo "🎯 Starting robot movement demonstration..."
echo ""
echo "The robot will now perform:"
echo "  1️⃣ Standing position"
echo "  2️⃣ Leg lifting sequences"
echo "  3️⃣ Walking cycles"
echo "  4️⃣ Victory dance"
echo ""

# Run the movement script
python3 /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/scripts/simple_working_movement.py

echo ""
echo "🎉 Robot movement demonstration completed!"
echo ""
echo "🔧 Available commands while system is running:"
echo "   - Waypoint Navigation: python3 /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/scripts/waypoint_navigation_timed.py"
echo "   - Joint Walking: python3 /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/scripts/joint_walking_controller.py"
echo "   - Simple Movement: python3 /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/scripts/simple_working_movement.py"
echo ""
echo "💡 Press Ctrl+C to shutdown everything"
echo ""

# Keep the script running to maintain the simulation
echo "🟢 System is running. Simulation and RViz are active."
echo "   You can now control the robot or just observe the simulation."

# Wait for user to interrupt
while true; do
    sleep 1
done
