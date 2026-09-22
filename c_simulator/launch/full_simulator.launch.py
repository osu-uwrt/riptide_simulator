import launch
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import os

physics_launch = os.path.join(
    get_package_share_directory('c_simulator'), 'launch', 'physics_simulator.launch.py')
camera_launch = os.path.join(
    get_package_share_directory('camera_faker'), 'launch', 'zedfaker.launch.py')
rviz_launch = os.path.join(
    get_package_share_directory('riptide_rviz'), 'launch', 'rviz_start.launch.py')


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="talos",
            description="Name of the vehicle",
        ),
        DeclareLaunchArgument("sync_odom", default_value="false"),
        DeclareLaunchArgument("with_camera_faker", default_value="true"),
        DeclareLaunchArgument("with_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Run every simulator node on the physics node's /clock"),
        DeclareLaunchArgument("real_time_factor", default_value="1.0",
                              description="Simulated seconds per wall second"),
        GroupAction([
            # Applies to the camera faker and RViz below; the physics launch
            # sets it for its own nodes.
            SetParameter(name='use_sim_time',
                         value=ParameterValue(LC('use_sim_time'), value_type=bool)),
            # Physics simulator
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(physics_launch),
                launch_arguments=[
                    ('robot', LC('robot')),
                    ('sync_odom', LC('sync_odom')),
                    ('use_sim_time', LC('use_sim_time')),
                    ('real_time_factor', LC('real_time_factor')),
                ]
            ),
            # Camera faker
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(camera_launch),
                condition=IfCondition(LC("with_camera_faker")),
                launch_arguments=[
                    ('robot', LC('robot'))
                ]
            ),
            # RViz
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(rviz_launch),
                condition=IfCondition(LC("with_rviz")),
                launch_arguments=[
                    ('robot', LC('robot'))
                ]
            ),
        ], scoped=True, forwarding=True),
    ])
