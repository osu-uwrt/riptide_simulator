import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PathJoinSubstitution, TextSubstitution

import os

def generate_launch_description():

    # Read in the vehicle's namespace through the command line or use the default value one is not provided
    robot = LaunchConfiguration("robot")

    # declare the path to the robot's vehicle description file
    config = PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'config',
        LaunchConfiguration("robot_yaml")
    ])

    # declare the path to the robot's vehicle description file
    obsstacleConfig = PathJoinSubstitution([
        get_package_share_directory('riptide_mapping2'),
        'config','config.yaml'
    ])

    # declare the path to collision data folder
    collision = PathJoinSubstitution([
        get_package_share_directory('c_simulator'),
        'collision_files'
    ])

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="talos",
            description="Name of the vehicle",
        ),
        
        DeclareLaunchArgument('robot_yaml', default_value=[
                              LaunchConfiguration("robot"), '.yaml']),

        launch.actions.GroupAction([
            launch_ros.actions.PushRosNamespace(
                LC("robot")
            ),
            # Launch simulator
            launch_ros.actions.Node(
                package="c_simulator",
                executable="physics_simulator",
                name="physics_simulator",
                output="screen",
                parameters=[
                    {"obstacle_config": obsstacleConfig},
                    {"collision_folder": collision},
                    {"vehicle_config": config},
                    {"robot": robot},
                ]
            ),
        ], scoped=True)
    ])
