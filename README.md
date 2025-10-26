# Spot Robot Navigation

Boston Dynamics Spot robot simulation with A→B→C→D waypoint navigation in Gazebo.

## Installation

### Prerequisites
```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install Gazebo Ignition
sudo apt install gz-garden

# Install dependencies
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher
sudo apt install ros-humble-controller-manager ros-humble-joint-trajectory-controller
```

### Build Workspace
```bash
cd /home/enma/pudusu/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Setup

### Environment
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source /home/enma/pudusu/ros2_ws/install/setup.bash

# Set domain ID (optional)
export ROS_DOMAIN_ID=0
```

### Verify Installation
```bash
# Check packages
ros2 pkg list | grep champ

# Test launch file
ros2 launch champ_bringup spot_bringup.launch.py --show-args
```

## Usage

### Option 1: Complete A→B→C→D Navigation
```bash
/home/enma/pudusu/run_spot_gazebo_abcd.sh
```

### Option 2: Manual Control
```bash
# Terminal 1: Launch simulation
/home/enma/pudusu/run_spot_gazebo_simple.sh

# Terminal 2: Run navigation
source /home/enma/pudusu/ros2_ws/install/setup.bash
python3 /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/scripts/visible_walking_abcd.py
```

### Option 3: Individual Scripts
```bash
source /home/enma/pudusu/ros2_ws/install/setup.bash

# Simple movement test
python3 /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/scripts/simple_working_movement.py

# Timed waypoint navigation
python3 /home/enma/pudusu/ros2_ws/src/spot_ros2_ign/champ_bringup/scripts/waypoint_navigation_timed.py
```

## Controls

- **Start**: Run any launch script
- **Stop**: Press Ctrl+C
- **Monitor**: Check Gazebo window for robot movement
- **Debug**: Check terminal output for status

## Troubleshooting

### Common Issues
```bash
# If packages not found
source /home/enma/pudusu/ros2_ws/install/setup.bash

# If Gazebo doesn't start
pkill -f "gz sim"
pkill -f "ros2 launch"

# Rebuild if needed
cd /home/enma/pudusu/ros2_ws
colcon build --packages-select champ_bringup spot_navigation
```

### Expected Behavior
- Gazebo opens with empty room environment
- Spot robot spawns at origin
- Robot performs visible walking motions
- Console shows navigation progress

## Files Structure
```
/home/enma/pudusu/
├── run_spot_gazebo_abcd.sh     # Complete launch
├── run_spot_gazebo_simple.sh   # Basic launch
├── ros2_ws/
│   └── src/spot_ros2_ign/
│       └── champ_bringup/
│           └── scripts/
│               ├── visible_walking_abcd.py
│               ├── simple_working_movement.py
│               └── waypoint_navigation_timed.py
└── empty_room.sdf              # Simulation world
```

Robot executes A→B→C→D rectangular path with quadruped gait patterns and large joint movements for clear visualization.
