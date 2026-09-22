import launch
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PathJoinSubstitution, PythonExpression

from pathlib import Path


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    editable_mapping=Path(__file__).resolve().parents[2]/'config.yaml'
    mapping_default=str(editable_mapping if editable_mapping.exists() else Path(get_package_share_directory('c_simulator'))/'config/simulation.yaml')
    editable_scene = Path(__file__).resolve().parents[2] / 'scene_info.yaml'
    sceneConfig = str(editable_scene if editable_scene.exists() else
                      Path(get_package_share_directory('c_simulator')) / 'config/scene_info.yaml')

    # declare the path to the robot's vehicle description file
    vehicle_config = PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'config',
        LaunchConfiguration("robot_yaml")
    ])

    # declare the path to the simulator's description file
    simulation_config = PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'config',
        "simulator.yaml"
    ])

    # declare the path to collision data folder
    collisionFolder = PathJoinSubstitution([
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
        DeclareLaunchArgument('hydrodynamics_config',default_value=PathJoinSubstitution([
            get_package_share_directory('c_simulator'),'config',[LC('robot'),'_hydrodynamics.yaml']])),
        DeclareLaunchArgument('mapping_config',default_value=mapping_default),
        DeclareLaunchArgument('physics_step',default_value='0.002'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
            description='Run the simulator nodes on the /clock the physics node publishes'),
        DeclareLaunchArgument('real_time_factor', default_value='1.0',
            description='Simulated seconds per wall second; also settable at runtime with ros2 param set'),
        DeclareLaunchArgument('clock_publish_rate', default_value='500.0',
            description='/clock publication rate in simulated Hz'),
        DeclareLaunchArgument('random_seed',default_value='7'),
        DeclareLaunchArgument('sensor_noise',default_value='true'),
        DeclareLaunchArgument('sensor_config', default_value=PathJoinSubstitution([
            get_package_share_directory('c_simulator'), 'config', [LC('robot'), '_sensors.yaml']])),
        DeclareLaunchArgument('gyro_rate', default_value='500.0'),
        DeclareLaunchArgument('gyro_noise_stddev', default_value='0.00017453292519943296'),
        DeclareLaunchArgument('gyro_variance', default_value=PythonExpression([
            'max(1e-9, (', LC('gyro_noise_stddev'), ') ** 2)']),
            description='FOG rate variance [(rad/s)^2]; defaults to simulated noise variance'),
        DeclareLaunchArgument('collisions',default_value='true'),
        DeclareLaunchArgument('with_tasks',default_value='true'),
        DeclareLaunchArgument('task_config',default_value=PathJoinSubstitution([
            get_package_share_directory('c_simulator'),'config','talos_tasks.yaml'])),
        DeclareLaunchArgument(
            'sync_odom', default_value='false',
            description='Use ideal sensor data (debug only)'),

        launch.actions.GroupAction([
            launch_ros.actions.PushRosNamespace(
                LC("robot")
            ),
            launch_ros.actions.SetParameter(
                name='use_sim_time',
                value=ParameterValue(LC('use_sim_time'), value_type=bool)),
            # Launch simulator
            launch_ros.actions.Node(
                package="c_simulator",
                executable="physics_simulator",
                name="physics_simulator",
                output="screen",
                parameters=[
                    LC('sensor_config'),
                    {"scene_config": sceneConfig},
                    {"task_config": LC('task_config')},
                    {"hydrodynamics_config": LC('hydrodynamics_config')},
                    {"mapping_config": LC('mapping_config')},
                    {"physics_step": ParameterValue(LC('physics_step'),value_type=float)},
                    {"real_time_factor": ParameterValue(LC('real_time_factor'), value_type=float)},
                    {"clock_publish_rate": ParameterValue(LC('clock_publish_rate'), value_type=float)},
                    {"random_seed": ParameterValue(LC('random_seed'),value_type=int)},
                    {"sensor_noise": ParameterValue(LC('sensor_noise'),value_type=bool)},
                    {"gyro_rate": ParameterValue(LC('gyro_rate'), value_type=float)},
                    {"gyro_noise_stddev": ParameterValue(LC('gyro_noise_stddev'), value_type=float)},
                    {"gyro_variance": ParameterValue(LC('gyro_variance'), value_type=float)},
                    {"collisions": ParameterValue(LC('collisions'),value_type=bool)},
                    {"collision_folder": collisionFolder},
                    {"vehicle_config": vehicle_config},
                    {"simulator_config": simulation_config},
                    {"robot": robot},
                    {"sync_odom": ParameterValue(
                        LaunchConfiguration("sync_odom"), value_type=bool)},
                ]
            ),
            launch_ros.actions.Node(
                package='c_simulator', executable='task_simulator.py',
                name='task_simulator', output='screen', condition=IfCondition(LC('with_tasks')),
                parameters=[{'robot': robot, 'vehicle_config': vehicle_config,
                             'hydrodynamics_config': LC('hydrodynamics_config'),
                             'mapping_config': LC('mapping_config'), 'task_config': LC('task_config')}]),
        ], scoped=True)
    ])
