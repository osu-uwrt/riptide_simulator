"""Vehicle navigation (EKF, depth converter, robot description) on simulated time.

Wraps riptide_hardware2 navigation.launch.py with use_sim_time so it follows the
/clock published by physics_simulator. Use this when running navigation next to
the simulator without the full riptide_bringup2 simulation.launch.py.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    navigation_launch = os.path.join(
        get_package_share_directory("riptide_hardware2"), "launch", "navigation.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="talos"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            GroupAction(
                [
                    SetParameter(
                        name="use_sim_time",
                        value=ParameterValue(LC("use_sim_time"), value_type=bool),
                    ),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(navigation_launch),
                        launch_arguments=[("robot", LC("robot"))],
                    ),
                ],
                scoped=True,
                forwarding=True,
            ),
        ]
    )
