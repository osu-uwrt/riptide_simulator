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
    package_src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(get_package_share_directory("camera_faker")))))
    # declare the path to shader data folder
    shaderFolder = PathJoinSubstitution([
        package_src_dir,
        'src',
        "riptide_simulator",
        'camera_faker',
        'shaders'
    ])

    # declare the path to texture data folder
    textureFolder = PathJoinSubstitution([
        package_src_dir,
        'src',
        "riptide_simulator",
        'camera_faker',
        'textures'
    ])

    # declare the path to model data folder
    modelFolder = PathJoinSubstitution([
        package_src_dir,
        'src',
        "riptide_simulator",
        'camera_faker',
        'models'
    ])

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="talos",
            description="Name of the vehicle",
        ),

        launch.actions.GroupAction([
            launch_ros.actions.PushRosNamespace(
                LC("robot")
            ),
            # Launch simulator
            launch_ros.actions.Node(
                package="camera_faker",
                executable="zed_faker",
                name="zed_faker",
                output="screen",
                parameters=[
                    {"shader_folder": shaderFolder},
                    {"texture_folder": textureFolder},
                    {"model_folder": modelFolder},
                    {"robot": robot},
                ]
            ),
        ], scoped=True)
    ])
