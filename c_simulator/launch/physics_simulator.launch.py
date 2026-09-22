"""Launch the plant and the selected year behavior from one resolved profile."""

from pathlib import Path
import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from riptide_sim_config.launching import arguments, run, prepare


def nodes(context):
    path, meta = run(context)

    def value(name):
        return LC(name).perform(context)

    params = {
        key + "_config": str(path / (key + ".yaml"))
        for key in ("vehicle", "hydrodynamics", "simulator", "mapping", "scene", "task")
    }
    params.update(
        robot=meta["namespace"],
        collision_folder=str(Path(get_package_share_directory("c_simulator")) / "collision_files"),
    )
    for key in (
        "physics_step",
        "real_time_factor",
        "clock_publish_rate",
        "gyro_rate",
        "gyro_noise_stddev",
        "gyro_variance",
    ):
        params[key] = float(value(key))
    params["random_seed"] = int(value("random_seed"))
    for key in ("sensor_noise", "collisions", "sync_odom", "use_sim_time"):
        params[key] = value(key).lower() in ("true", "1")
    result = [
        Node(
            package="c_simulator",
            executable="physics_simulator",
            name="physics_simulator",
            namespace=meta["namespace"],
            output="screen",
            parameters=[str(path / "sensors.yaml"), params],
        )
    ]
    if meta["enabled"] or meta["robot_behavior"]:
        result.append(
            Node(
                package="c_simulator",
                executable="task_simulator.py",
                name="task_simulator",
                namespace=meta["namespace"],
                output="screen",
                parameters=[params],
            )
        )
    return result


def generate_launch_description():
    return launch.LaunchDescription(
        arguments()
        + [
            OpaqueFunction(function=prepare),
            DeclareLaunchArgument(
                "robot",
                default_value="talos",
                description="Name of the vehicle",
            ),
            DeclareLaunchArgument("hydrodynamics_config", default_value=""),
            DeclareLaunchArgument("mapping_config", default_value=""),
            DeclareLaunchArgument("physics_step", default_value="0.002"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Run the simulator nodes on the /clock the physics node publishes",
            ),
            DeclareLaunchArgument(
                "real_time_factor",
                default_value="1.0",
                description="Simulated seconds per wall second; also settable at runtime with ros2 param set",
            ),
            DeclareLaunchArgument(
                "clock_publish_rate",
                default_value="500.0",
                description="/clock publication rate in simulated Hz",
            ),
            DeclareLaunchArgument("random_seed", default_value="7"),
            DeclareLaunchArgument("sensor_noise", default_value="true"),
            DeclareLaunchArgument("sensor_config", default_value=""),
            DeclareLaunchArgument("gyro_rate", default_value="500.0"),
            DeclareLaunchArgument("gyro_noise_stddev", default_value="0.00017453292519943296"),
            DeclareLaunchArgument(
                "gyro_variance",
                default_value=PythonExpression(["max(1e-9, (", LC("gyro_noise_stddev"), ") ** 2)"]),
                description="FOG rate variance [(rad/s)^2]; defaults to simulated noise variance",
            ),
            DeclareLaunchArgument("collisions", default_value="true"),
            DeclareLaunchArgument("with_tasks", default_value="true"),
            DeclareLaunchArgument("task_config", default_value=""),
            DeclareLaunchArgument(
                "sync_odom", default_value="false", description="Use ideal sensor data (debug only)"
            ),
            OpaqueFunction(function=nodes),
        ]
    )
