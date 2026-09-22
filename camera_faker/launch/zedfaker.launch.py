"""Launch the pool viewer and AprilTag detector for the full simulator."""

import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC, TextSubstitution
from riptide_sim_config.launching import arguments, prepare
from launch.actions import OpaqueFunction


def generate_launch_description():
    robot = LC("robot")
    return launch.LaunchDescription(
        arguments()
        + [
            OpaqueFunction(function=prepare),
            DeclareLaunchArgument(
                "robot",
                default_value="talos",
                description="Name of the vehicle",
            ),
            DeclareLaunchArgument(
                "with_apriltag",
                default_value="true",
                description="Run the AprilTag detector (keeps the forward camera subscribed)",
            ),
            launch.actions.GroupAction(
                [
                    launch_ros.actions.PushRosNamespace(LC("robot")),
                    # The viewer reads vehicle and camera poses from the simulation TF tree.
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory("camera_faker"),
                                "launch",
                                "pool_viewer.launch.py",
                            )
                        ),
                        launch_arguments={
                            "robot": robot,
                            "year": LC("year"),
                            "scenario": LC("scenario"),
                            "resolved_config": LC("resolved_config"),
                        }.items(),
                    ),
                    # Run the same detector configuration used on the real calibration
                    # board, against the image and CameraInfo produced above.
                    launch_ros.actions.Node(
                        package="apriltag_ros",
                        executable="apriltag_node",
                        name="apriltag_36h11",
                        condition=IfCondition(LC("with_apriltag")),
                        namespace="apriltag",
                        output="screen",
                        remappings=[
                            (
                                "image_rect",
                                [
                                    TextSubstitution(text="/"),
                                    LC("robot"),
                                    TextSubstitution(text="/ffc/zed_node/left/image_rect_color"),
                                ],
                            ),
                            (
                                "camera_info",
                                [
                                    TextSubstitution(text="/"),
                                    LC("robot"),
                                    TextSubstitution(text="/ffc/zed_node/left/camera_info"),
                                ],
                            ),
                        ],
                        parameters=[
                            {
                                "image_transport": "raw",
                                "family": "36h11",
                                "size": 0.508,
                                "max_hamming": 0,
                                "z_up": True,
                            }
                        ],
                    ),
                    launch_ros.actions.Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="surface_frame_node",
                        condition=IfCondition(LC("with_apriltag")),
                        arguments=[
                            "0",
                            "0.4572",
                            "0",
                            "0",
                            "-1.5707",
                            "-1.5707",
                            "tag36h11:0",
                            "estimated_origin_frame",
                        ],
                    ),
                ],
                scoped=True,
            ),
        ]
    )
