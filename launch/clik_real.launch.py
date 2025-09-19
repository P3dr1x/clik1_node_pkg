import os  # Importa il modulo os per manipolare i percorsi di file
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

import pathlib


# Ottiene la descrizione del robot dall'URDF usando il comando xacro
def generate_launch_description(): 

    # Launch Arguments
    use_sim = LaunchConfiguration('use_sim', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    real_system = LaunchConfiguration('real_system', default='true')
    robot_model = LaunchConfiguration('robot_model', default='mobile_wx250s')
    robot_name = LaunchConfiguration('robot_name', default='mobile_wx250s')
    xs_driver_logging_level = LaunchConfiguration('xs_driver_logging_level', default='INFO')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    motor_configs = LaunchConfiguration('motor_configs', default=PathJoinSubstitution([
        FindPackageShare('clik1_node_pkg'), 'config', 'mobile_wx250s.yaml'
    ]))
    mode_configs = LaunchConfiguration('mode_configs', default=PathJoinSubstitution([
        FindPackageShare('clik1_node_pkg'), 'config', 'modes.yaml'
    ]))
    load_configs = LaunchConfiguration('load_configs', default='true')
    #robot_description = LaunchConfiguration('robot_description', default='')

    # Micro XRCE Agent args (PX4 → ROS 2 bridge over serial)
    px4_agent_dev = LaunchConfiguration('px4_agent_dev', default='/dev/USB1')
    px4_agent_baud = LaunchConfiguration('px4_agent_baud', default='921600')

    # Paths to models
    pkg_share = get_package_share_directory('clik1_node_pkg')  

    # Carica il file URDF COMPLETO (drone + braccio)
    # full_urdf_path = pathlib.Path(pkg_share, 'model', 't960a.urdf.xacro')
    # full_robot_description = full_urdf_path.read_text()

    full_robot_description = Command([
        'xacro', ' ',
        os.path.join(pkg_share, 'model', 't960a_real.urdf.xacro')
    ])

    # Nota: non usiamo un URDF "solo arm" qui; pubblichiamo la descrizione completa.

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': ParameterValue(full_robot_description, value_type=str),
                     'use_sim_time': use_sim_time}],
    )
    # Nodo driver Interbotix (hardware reale)
    xs_sdk_node = Node(
        package='interbotix_xs_sdk',
        executable='xs_sdk',
        name='xs_sdk',
        #namespace=robot_name,
        parameters=[{
            'motor_configs': motor_configs,
            'mode_configs': mode_configs,
            'load_configs': load_configs,
            'xs_driver_logging_level': xs_driver_logging_level,
        }],
        condition=UnlessCondition(use_sim),
        output='screen',
    )

    # Avvio Micro XRCE Agent (necessario per esporre i topic PX4 su ROS 2)
    microxrce_agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'serial', '--dev', px4_agent_dev, '-b', px4_agent_baud
        ],
        output='screen',
        condition=IfCondition(real_system)
    )

    world_to_base_link_broadcaster = Node(
        package='clik1_node_pkg',
        executable='world_to_base_link_broadcaster',
        name='world_to_base_link_broadcaster',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'real_system': real_system}
        ]
    )

    real_drone_pose_pub = Node(
        package='clik1_node_pkg',
        executable='real_drone_pose_pub',
        name='real_drone_pose_pub',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(real_system)
    )

    # Nodo CLIK
    clik_uam_node = Node(
        package='clik1_node_pkg',
        executable='clik_uam_node',
        name='clik_uam_node',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'real_system': real_system},
            {'use_gazebo_pose': False},
        ],
    )

    # Includi la visualizzazione RViz (da lanciare con ritardo)
    visual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_share, 'launch', 'clik_uam_visual.launch.py'
        )),
        launch_arguments={'real_system': 'true'}.items()
    )

    # Lancia il visual 10s dopo l'avvio di xs_sdk_node
    delayed_visual = RegisterEventHandler(
        OnProcessStart(
            target_action=xs_sdk_node,
            on_start=[
                TimerAction(period=10.0, actions=[visual_launch])
            ]
        )
    )

    return LaunchDescription([   # lista dei nodi da lanciare
        DeclareLaunchArgument('use_sim', default_value='false', choices=['true', 'false'], description='Se true, usa il driver simulato.'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'], description='Se true, usa il clock simulato.'),
    # px4_repo_path non serve in real-time hardware
        DeclareLaunchArgument('motor_configs', default_value=PathJoinSubstitution([
            FindPackageShare('clik1_node_pkg'), 'config', 'mobile_wx250s.yaml'
        ]), description="Percorso al file di configurazione dei motori."),
        DeclareLaunchArgument('mode_configs', default_value=PathJoinSubstitution([
            FindPackageShare('clik1_node_pkg'), 'config', 'modes.yaml'
        ]), description="Percorso al file di configurazione delle modalità."),
        DeclareLaunchArgument('load_configs', default_value='true', choices=['true', 'false'], description='Se true, carica i valori iniziali dei registri.'),
        #DeclareLaunchArgument('robot_description', default_value='', description='Descrizione URDF del robot.'),
        DeclareLaunchArgument('robot_model', default_value='mobile_wx250s', description='Modello del robot.'),
        DeclareLaunchArgument('robot_name', default_value='mobile_wx250s', description='Nome del robot.'),
        DeclareLaunchArgument('xs_driver_logging_level', default_value='INFO', choices=['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'], description='Livello di log del driver XS.'),
        DeclareLaunchArgument('real_system', default_value='true', choices=['true','false'], description='Se true avvia il nodo real_drone_pose_pub e il broadcaster usa la posa reale.'),
        DeclareLaunchArgument('px4_agent_dev', default_value='/dev/USB1', description='Dispositivo seriale PX4 (es. /dev/ttyACM0, /dev/USB1).'),
        DeclareLaunchArgument('px4_agent_baud', default_value='921600', description='Baudrate per MicroXRCEAgent.'),
        # Be sure that MicroXRCEAgent is exposing PX4 topic on ROS2
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false'], description='Lancia RViz se true.'),
    #microxrce_agent,
    real_drone_pose_pub,
    world_to_base_link_broadcaster,
    xs_sdk_node,
    robot_state_publisher,
    delayed_visual
    ])
