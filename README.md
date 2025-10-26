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

### Clone Repository
```bash
# Create workspace
mkdir -p ~/spot_navigation_ws/src
cd ~/spot_navigation_ws/src

# Clone Spot ROS 2 repository
git clone https://github.com/chvmp/champ.git
git clone https://github.com/chvmp/champ_setup_assistant.git

# Clone this navigation package
git clone <your-repository-url>
```

### Build Workspace
```bash
cd ~/spot_navigation_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Setup

### Environment Configuration
```bash
# Add to ~/.bashrc for permanent setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/spot_navigation_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# Reload terminal or source manually
source ~/.bashrc
```

### Verify Installation
```bash
# Check packages
ros2 pkg list | grep champ

# Test launch file syntax
ros2 launch champ_bringup spot_bringup.launch.py --show-args
```

## Usage

### Quick Start
```bash
# Make scripts executable
chmod +x ~/spot_navigation_ws/scripts/*.sh

# Run complete A→B→C→D navigation
~/spot_navigation_ws/scripts/run_spot_gazebo_abcd.sh
```

### Manual Control
```bash
# Terminal 1: Launch simulation
ros2 launch champ_bringup spot_bringup.launch.py use_simulator:=true

# Terminal 2: Run navigation (in new terminal)
source ~/spot_navigation_ws/install/setup.bash
ros2 run spot_navigation waypoint_navigator
```

### Custom Waypoints
```bash
# Edit waypoint coordinates in:
# ~/spot_navigation_ws/src/spot_navigation/scripts/waypoint_navigator.py

# Rebuild after changes
cd ~/spot_navigation_ws
colcon build --packages-select spot_navigation
```

## Configuration

### Modify Waypoints
Edit [src/spot_navigation/scripts/waypoint_navigator.py](cci:7://file:///home/enma/puthu_folder/spot_navigation_ws/src/spot_navigation/scripts/waypoint_navigator.py:0:0-0:0):
```python
self.waypoints = [
    {'x': 1.0, 'y': -1.0, 'z': 0.0, 'name': 'A'},  # Start point
    {'x': 3.0, 'y': -1.0, 'z': 0.0, 'name': 'B'},  # Forward
    {'x': 3.0, 'y': 1.0, 'z': 0.0, 'name': 'C'},   # Left
    {'x': 1.0, 'y': 1.0, 'z': 0.0, 'name': 'D'}    # Back
]
```

### Adjust Movement Parameters
Edit movement scripts to change:
- Walking speed: Modify `duration` parameters
- Joint angles: Adjust position arrays
- Gait cycles: Change `cycles` parameter

## Scripts

### Available Launch Scripts
```bash
# Complete navigation with visualization
./scripts/run_spot_gazebo_abcd.sh

# Basic simulation only
./scripts/run_spot_gazebo_simple.sh

# Navigation with RViz
./scripts/run_spot_with_rviz.sh
```

### Individual Movement Scripts
```bash
# Basic movement demonstration
ros2 run spot_navigation simple_movement

# Visible A→B→C→D walking
ros2 run spot_navigation visible_walking_abcd

# Timed waypoint navigation
ros2 run spot_navigation waypoint_navigation_timed
```

## Troubleshooting

### Build Issues
```bash
# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean build
rm -rf build install log
colcon build
```

### Runtime Issues
```bash
# Kill existing processes
pkill -f "gz sim"
pkill -f "ros2 launch"

# Check ROS environment
printenv | grep ROS

# Verify packages
ros2 pkg list | grep -E "(champ|spot)"
```

### Common Problems

**Gazebo doesn't start**: Check graphics drivers and try headless mode
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

**Robot doesn't move**: Verify controllers are loaded
```bash
ros2 control list_controllers
```

**Package not found**: Source workspace
```bash
source ~/spot_navigation_ws/install/setup.bash
```

## Customization

### Create New Movement Patterns
1. Copy existing script: `cp src/spot_navigation/scripts/waypoint_navigator.py src/spot_navigation/scripts/my_pattern.py`
2. Modify waypoints and movement logic
3. Rebuild: `colcon build --packages-select spot_navigation`
4. Run: `ros2 run spot_navigation my_pattern`

### Add New Worlds
1. Place SDF file in: `src/spot_navigation/worlds/`
2. Update launch file world parameter
3. Rebuild and test

### Integration with Nav2
For full navigation stack integration:
```bash
# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Use navigation launch file
ros2 launch spot_navigation spot_with_nav2.launch.py
```
