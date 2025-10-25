#!/usr/bin/env python3

"""
Complete Gazebo Launch File for Spot Robot Navigation
This launch file sets up everything needed to run Spot in Gazebo with the empty room
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'world_file',
            default_value='/home/enma/pudusu/gazebo_empty_room.sdf',
            description='Path to the Gazebo world file'
        ),
        DeclareLaunchArgument(
            'robot_x',
            default_value='0.0',
            description='Initial X position of the robot'
        ),
        DeclareLaunchArgument(
            'robot_y', 
            default_value='0.0',
            description='Initial Y position of the robot'
        ),
        DeclareLaunchArgument(
            'robot_z',
            default_value='0.5',
            description='Initial Z position of the robot'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Whether to launch Gazebo GUI'
        )
    ]
    
    # Get launch configurations
    world_file = LaunchConfiguration('world_file')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y') 
    robot_z = LaunchConfiguration('robot_z')
    use_rviz = LaunchConfiguration('use_rviz')
    gui = LaunchConfiguration('gui')
    
    # Set Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            '/home/enma/pudusu'
        ])
    )
    
    # Launch Gazebo with the world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_file, ' -r -v 4'],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Robot description (using a simple URDF for now)
    robot_description_content = """
    <?xml version="1.0"?>
    <robot name="simple_spot">
        <link name="base_link">
            <visual>
                <geometry>
                    <box size="1.0 0.5 0.3"/>
                </geometry>
                <material name="yellow">
                    <color rgba="1 1 0 1"/>
                </material>
            </visual>
            <collision>
                <geometry>
                    <box size="1.0 0.5 0.3"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="30.0"/>
                <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
            </inertial>
        </link>
    </robot>
    """
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'spot_robot',
            '-topic', '/robot_description',
            '-x', robot_x,
            '-y', robot_y, 
            '-z', robot_z
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge for basic topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        output='screen'
    )
    
    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription(
        launch_args + [
            gazebo_resource_path,
            gazebo_launch,
            robot_state_publisher,
            spawn_robot,
            bridge,
            rviz_node
        ]
    )
