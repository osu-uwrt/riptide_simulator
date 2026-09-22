"""OpenGL RoboSub viewer. Physics continues to run in c_simulator."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = Path(share('camera_faker'))
    hardware = Path(share('riptide_hardware2')) / 'cfg'
    # navigation.launch.py uses the ZED description when available. Otherwise
    # supply optical joints beneath the robot camera mounts, never truth TF.
    try:
        share('zed_wrapper')
        publish_camera_optical_tf = 'false'
    except PackageNotFoundError:
        publish_camera_optical_tf = 'true'
    # A symlink install follows the editable scene; a binary install uses its copy.
    source = Path(__file__).resolve().parents[2]
    camera_settings = Path(__file__).resolve().parents[1] / 'config/cameras.yaml'
    scene = source / 'scene_info.yaml'
    mapping = source / 'config.yaml'
    arguments = [
        DeclareLaunchArgument('camera_settings', default_value=str(
            camera_settings if camera_settings.exists() else package / 'config/cameras.yaml'),
                              description='YAML startup settings for camera scales, depth model, and water appearance'),
        DeclareLaunchArgument('robot_model', default_value='',
                              description='Empty selects the lightweight Talos model'),
        DeclareLaunchArgument('payload_model', default_value=str(package / 'models/payloads/projectile.glb')),
        DeclareLaunchArgument('launcher_model', default_value=str(package / 'models/payloads/launcher.glb')),
        DeclareLaunchArgument('claw_model', default_value=str(package / 'models/claw/gripper.glb')),
        DeclareLaunchArgument('depth_preview', default_value='false'),
        DeclareLaunchArgument('show_tf', default_value='false',
                              description='Show TF axes and frame names in the observer viewport'),
        DeclareLaunchArgument('show_scorecard', default_value='false'),
        DeclareLaunchArgument('task_config', default_value=str(package / 'config/talos_tasks.yaml')),
        DeclareLaunchArgument('lighting', default_value='outdoor'),
        DeclareLaunchArgument('robot', default_value='talos'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='Preview only; publishes no sensor data or vehicle TF'),
        DeclareLaunchArgument('demo_task', default_value='gate'),
        DeclareLaunchArgument('initial_focus', default_value='Vehicle'),
        DeclareLaunchArgument('publish_camera_optical_tf', default_value=publish_camera_optical_tf,
                              description='Supply robot optical joints when no ZED description publishes them'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Hide window (an OpenGL display is still required)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Follow the physics simulator /clock so image stamps match its TF'),
        DeclareLaunchArgument('camera_scale', default_value='1.0',
                              description='1.0 = native ZED resolution; 0.5 = half width/height'),
        DeclareLaunchArgument('camera_preview_width', default_value='480',
                              description='Maximum side-preview width; sensor and primary views keep camera_scale resolution'),
        DeclareLaunchArgument('profile', default_value='false',
                              description='Log camera render sizes and viewer timing'),
        DeclareLaunchArgument('mapping_config', default_value=str(
            mapping if mapping.exists() else package / 'config/simulation.yaml')),
        DeclareLaunchArgument('scene_config', default_value=str(
            scene if scene.exists() else package / 'config/scene_info.yaml')),
        DeclareLaunchArgument('ffc_calibration', default_value=''),
        DeclareLaunchArgument('dfc_calibration', default_value=''),
        DeclareLaunchArgument('exit_after_frames', default_value='0'),
        DeclareLaunchArgument('screenshot_path', default_value=''),
        DeclareLaunchArgument('point_cloud_overlay', default_value='0',
                              description='Draw published point clouds in the pool view: 0 off, 1 ffc, 2 dfc, 3 both'),
        DeclareLaunchArgument('detections', default_value='false',
                              description='Draw yolo_orientation markers at the true camera pose their image was rendered from'),
    ]
    return LaunchDescription(arguments + [Node(
        package='camera_faker', executable='pool_viewer', name='pool_viewer',
        namespace=['/', LC('robot')], output='screen', parameters=[LC('camera_settings'), {
            'robot': LC('robot'),
            'robot_model': ParameterValue(LC('robot_model'), value_type=str),
            'payload_model': LC('payload_model'),
            'launcher_model': LC('launcher_model'),
            'claw_model': LC('claw_model'),
            'task_config': LC('task_config'),
            'depth_preview': ParameterValue(LC('depth_preview'), value_type=bool),
            'show_tf': ParameterValue(LC('show_tf'), value_type=bool),
            'show_scorecard': ParameterValue(LC('show_scorecard'), value_type=bool),
            'lighting.profile': LC('lighting'),
            'demo_task': LC('demo_task'),
            'initial_focus': LC('initial_focus'),
            'publish_camera_optical_tf': ParameterValue(LC('publish_camera_optical_tf'), value_type=bool),
            'demo': ParameterValue(LC('demo'), value_type=bool),
            'headless': ParameterValue(LC('headless'), value_type=bool),
            'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool),
            'camera_scale': ParameterValue(LC('camera_scale'), value_type=float),
            'camera_preview_width': ParameterValue(LC('camera_preview_width'), value_type=int),
            'profile': ParameterValue(LC('profile'), value_type=bool),
            'exit_after_frames': ParameterValue(LC('exit_after_frames'), value_type=int),
            'screenshot_path': ParameterValue(LC('screenshot_path'), value_type=str),
            'detections': ParameterValue(LC('detections'), value_type=bool),
            'point_cloud.overlay': ParameterValue(LC('point_cloud_overlay'), value_type=int),
            'ffc.calibration_file': ParameterValue(LC('ffc_calibration'), value_type=str),
            'dfc.calibration_file': ParameterValue(LC('dfc_calibration'), value_type=str),
            'ffc.config': str(hardware / 'ffc_config.yaml'),
            'dfc.config': str(hardware / 'dfc_config.yaml'),
            'vehicle_config': PathJoinSubstitution([
                share('riptide_descriptions2'), 'config', [LC('robot'), '.yaml']]),
            'shader_folder': str(package / 'shaders/pool'),
            'texture_folder': str(package / 'textures'),
            'riptide_mesh_folder': str(Path(share('riptide_meshes')) / 'meshes'),
            'marker_config': str(Path(share('riptide_rviz')) / 'config/markers.yaml'),
            'mapping_config': LC('mapping_config'),
            'scene_config': LC('scene_config'),
        }])])
