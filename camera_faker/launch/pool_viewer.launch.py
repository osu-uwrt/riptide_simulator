"""OpenGL RoboSub viewer. Physics continues to run in c_simulator."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration as LC, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from riptide_sim_config.launching import arguments as profile_arguments, run
from riptide_sim_config.profiles import read


def generate_launch_description():
    package = Path(share("camera_faker"))
    # navigation.launch.py uses the ZED description when available. Otherwise
    # supply optical joints beneath the robot camera mounts, never truth TF.
    try:
        share("zed_wrapper")
        publish_camera_optical_tf = "false"
    except PackageNotFoundError:
        publish_camera_optical_tf = "true"
    arguments = [
        DeclareLaunchArgument(
            "camera_compute",
            default_value="",
            description="Camera compute backend: auto or cpu; empty uses camera_settings (default auto)",
        ),
        DeclareLaunchArgument(
            "camera_settings",
            default_value="",
            description="YAML startup settings for camera scales, depth model, and water appearance",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="",
            description="Empty selects the model from the robot profile",
        ),
        DeclareLaunchArgument("payload_model", default_value=""),
        DeclareLaunchArgument("launcher_model", default_value=""),
        DeclareLaunchArgument("claw_model", default_value=""),
        DeclareLaunchArgument("status_lights_config", default_value=""),
        DeclareLaunchArgument("thruster_visuals_config", default_value=""),
        DeclareLaunchArgument("depth_preview", default_value="false"),
        DeclareLaunchArgument(
            "show_tf",
            default_value="false",
            description="Show TF axes and frame names in the observer viewport",
        ),
        DeclareLaunchArgument("show_scorecard", default_value="false"),
        DeclareLaunchArgument("task_config", default_value=""),
        DeclareLaunchArgument("lighting", default_value="outdoor"),
        DeclareLaunchArgument("robot", default_value="talos"),
        DeclareLaunchArgument(
            "demo",
            default_value="false",
            description="Preview only; publishes no sensor data or vehicle TF",
        ),
        DeclareLaunchArgument("demo_task", default_value="gate"),
        DeclareLaunchArgument("initial_focus", default_value="Vehicle"),
        DeclareLaunchArgument(
            "publish_camera_optical_tf",
            default_value=publish_camera_optical_tf,
            description="Supply robot optical joints when no ZED description publishes them",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Hide window (an OpenGL display is still required)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Follow the physics simulator /clock so image stamps match its TF",
        ),
        DeclareLaunchArgument(
            "camera_scale",
            default_value="1.0",
            description="1.0 = native ZED resolution; 0.5 = half width/height",
        ),
        DeclareLaunchArgument(
            "camera_preview_width",
            default_value="480",
            description="Maximum side-preview width; sensor and primary views keep camera_scale resolution",
        ),
        DeclareLaunchArgument(
            "profile",
            default_value="false",
            description="Log camera render sizes and viewer timing",
        ),
        DeclareLaunchArgument("mapping_config", default_value=""),
        DeclareLaunchArgument("scene_config", default_value=""),
        DeclareLaunchArgument("ffc_calibration", default_value=""),
        DeclareLaunchArgument("dfc_calibration", default_value=""),
        DeclareLaunchArgument("exit_after_frames", default_value="0"),
        DeclareLaunchArgument("screenshot_path", default_value=""),
        DeclareLaunchArgument(
            "point_cloud_overlay",
            default_value="0",
            description="Draw published point clouds in the pool view: 0 off, 1 ffc, 2 dfc, 3 both",
        ),
        DeclareLaunchArgument(
            "detections",
            default_value="false",
            description="Draw yolo_orientation markers at the true camera pose their image was rendered from",
        ),
    ]
    return LaunchDescription(profile_arguments() + arguments + [OpaqueFunction(function=viewer)])


def viewer(context):
    path, meta = run(context)
    package = Path(share("camera_faker"))
    robot = read(path / "vehicle.yaml")
    defaults = meta["viewer"]

    def selected(key):
        return LC(key).perform(context) or defaults.get(key, "")

    camera_params = {}
    compute = LC("camera_compute").perform(context)
    if compute:
        camera_params["camera_compute"] = compute
    for camera in robot.get("sim_cameras", []):
        camera_params[camera["name"] + ".config"] = camera.get("config", "")
        camera_params[camera["name"] + ".calibration_file"] = context.launch_configurations.get(
            camera["name"] + "_calibration", ""
        ) or camera.get("calibration", "")
    return [
        Node(
            package="camera_faker",
            executable="pool_viewer",
            name="pool_viewer",
            namespace="/" + meta["namespace"],
            output="screen",
            parameters=([selected("camera_settings")] if selected("camera_settings") else [])
            + [
                camera_params,
                {
                    "robot": meta["namespace"],
                    "robot_model": ParameterValue(LC("robot_model"), value_type=str),
                    "payload_model": ParameterValue(selected("payload_model"), value_type=str),
                    "launcher_model": ParameterValue(selected("launcher_model"), value_type=str),
                    "claw_model": ParameterValue(selected("claw_model"), value_type=str),
                    "status_lights_config": ParameterValue(selected("status_lights_config"), value_type=str),
                    "thruster_visuals_config": ParameterValue(selected("thruster_visuals_config"), value_type=str),
                    "task_config": str(path / "task.yaml"),
                    "depth_preview": ParameterValue(LC("depth_preview"), value_type=bool),
                    "show_tf": ParameterValue(LC("show_tf"), value_type=bool),
                    "show_scorecard": ParameterValue(LC("show_scorecard"), value_type=bool),
                    "lighting.profile": LC("lighting"),
                    "demo_task": LC("demo_task"),
                    "initial_focus": LC("initial_focus"),
                    "publish_camera_optical_tf": ParameterValue(
                        LC("publish_camera_optical_tf"), value_type=bool
                    ),
                    "demo": ParameterValue(LC("demo"), value_type=bool),
                    "headless": ParameterValue(LC("headless"), value_type=bool),
                    "use_sim_time": ParameterValue(LC("use_sim_time"), value_type=bool),
                    "camera_scale": ParameterValue(LC("camera_scale"), value_type=float),
                    "camera_preview_width": ParameterValue(
                        LC("camera_preview_width"), value_type=int
                    ),
                    "profile": ParameterValue(LC("profile"), value_type=bool),
                    "exit_after_frames": ParameterValue(LC("exit_after_frames"), value_type=int),
                    "screenshot_path": ParameterValue(LC("screenshot_path"), value_type=str),
                    "detections": ParameterValue(LC("detections"), value_type=bool),
                    "point_cloud.overlay": ParameterValue(
                        LC("point_cloud_overlay"), value_type=int
                    ),
                    "vehicle_config": str(path / "vehicle.yaml"),
                    "shader_folder": str(package / "shaders/pool"),
                    "texture_folder": str(package / "textures"),
                    "riptide_mesh_folder": meta["mesh_root"],
                    "marker_config": str(path / "markers.yaml"),
                    "mapping_config": str(path / "mapping.yaml"),
                    "scene_config": str(path / "scene.yaml"),
                },
            ],
        )
    ]
