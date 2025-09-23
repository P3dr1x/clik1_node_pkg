import os  # Importa il modulo os per manipolare i percorsi di file
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
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
    use_sim = LaunchConfiguration('use_sim', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    real_system = LaunchConfiguration('real_system', default='false')
    px4_repo_path = LaunchConfiguration('px4_repo_path', default=os.path.expanduser('~/PX4-Autopilot'))
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

    # Avvio del processo PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_t960a'],  # SITL simulation target
        #cwd='/home/mattia/PX4-Autopilot',     # Path alla tua repo PX4
        cwd=px4_repo_path,
        output='screen'
    )

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_sim_system_plugin_path = LaunchConfiguration(
        'gz_sim_system_plugin_path',
        default=os.path.expanduser('~/gz_ros2_control_ws/install/gz_ros2_control/lib')
    )

    # Set environment variable for Gazebo 
    # Costruisci un percorso portabile espandendo "~" e usa os.pathsep per concatenare più directory
    _gz_plugin_dir = os.path.expanduser('~/gz_ros2_control_ws/install/gz_ros2_control/lib')
    _existing_gz_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH')
    if _existing_gz_plugin_path:
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = _existing_gz_plugin_path + os.pathsep + _gz_plugin_dir
    else:
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = _gz_plugin_dir

    # Set environment variable for Gazebo (corretto)
    # set_gz_plugin_path = SetEnvironmentVariable(
    #     name='GZ_SIM_SYSTEM_PLUGIN_PATH',
    #     value=gz_sim_system_plugin_path
    # )

    # Paths to models
    pkg_share = get_package_share_directory('clik1_node_pkg')  

    # Carica il file URDF COMPLETO (drone + braccio)
    # full_urdf_path = pathlib.Path(pkg_share, 'model', 't960a.urdf.xacro')
    # full_robot_description = full_urdf_path.read_text()

    full_robot_description = Command([
        'xacro', ' ',
        os.path.join(pkg_share, 'model', 't960a.urdf.xacro')
    ])

    # File controller ros2_control (SITL)
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('clik1_node_pkg'),
        'config',
        'mobile_wx250s_joint_pos_ctrl.yaml',
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': ParameterValue(full_robot_description, value_type=str),
                     'use_sim_time': use_sim_time}],
    )
    # Spawner ros2_control: joint_state_broadcaster e arm_controller
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers,
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--param-file',
            robot_controllers,
        ],
        output='screen',
    )


    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/model/t960a_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
                   #'/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V', # non funziona
                   '/model/t960a/xyz@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/model/t960a_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'], # [ indica che il bridge è da gazebo a ros
        output='screen'
    )

    # Pubblica la configurazione di sleep via CLI (una volta sola)
    sleep_pose_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/arm_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            '{data: [0.0, -1.8, 1.55, 0.0, 0.8, 0.0]}'
        ],
        output='screen'
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

    # Nodo CLIK in SITL: usa posa da Gazebo e pubblica comandi verso ros2_control
    clik_uam_node = Node(
        package='clik1_node_pkg',
        executable='clik_uam_node',
        name='clik_uam_node',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'real_system': False},  # False: pubblica su /arm_controller/commands
            {'use_gazebo_pose': True},
        ],
    )

    return LaunchDescription([   # lista dei nodi da lanciare
        DeclareLaunchArgument('use_sim', default_value='true', choices=['true', 'false'], description='Se true, usa il driver simulato.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Se true, usa il clock simulato.'),
        DeclareLaunchArgument('px4_repo_path', default_value=os.path.expanduser('~/PX4-Autopilot'), description='Percorso alla cartella PX4-Autopilot'),
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
        DeclareLaunchArgument('real_system', default_value='false', choices=['true','false'], description='Se true avvia il nodo real_drone_pose_pub e il broadcaster usa la posa reale.'),
        # Be sure that MicroXRCEAgent is exposing PX4 topic on ROS2
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false'], description='Lancia RViz se true.'),
    px4_sitl,
    bridge,
    robot_state_publisher,
    joint_state_broadcaster_spawner,
    arm_controller_spawner,
    world_to_base_link_broadcaster,
    real_drone_pose_pub,
    # Dopo lo spawn di arm_controller, invia la sleep pose e avvia clik_uam_node dopo 5s
    RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                sleep_pose_pub,
                #TimerAction(period=10.0, actions=[clik_uam_node])
            ],
        )
    ),
    ])
