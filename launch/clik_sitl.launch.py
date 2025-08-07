import os  # Importa il modulo os per manipolare i percorsi di file
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction, IncludeLaunchDescription
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
    use_sim = LaunchConfiguration('use_sim', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
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
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.path.join(
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        '/home/mattia/gz_ros2_control_ws/install/gz_ros2_control/lib'
    )

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

    # Carica il file URDF contenente solo l'arm
    arm_urdf_path = pathlib.Path(pkg_share, 'model', 'wx250s_sim.urdf')
    arm_description = arm_urdf_path.read_text()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('clik1_node_pkg'),
            'config',
            'mobile_wx250s_joint_pos_ctrl.yaml',
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': ParameterValue(full_robot_description, value_type=str),
                     'use_sim_time': True}],
    )

    # Adesso startiamo i 3 controller del configuration file (arm, gripper controller e joint state broadcaster) tramite un nodo
    joint_state_broadcaster_spawner = Node(  # questo è un nuovo nodo
        package = "controller_manager",
        executable="spawner",  # Nome dell'eseguibile del nodo
        # namespace='/mobile_wx250s',  # Namespace del nodo
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers,
        ],
    )

    arm_controller_spawner = Node(  # questo è un nuovo nodo
        package = "controller_manager",
        executable="spawner",
        # namespace='/mobile_wx250s', 
        arguments=[
            'arm_controller',
            '--param-file',
            robot_controllers,
        ],
    )


    # gripper_controller_spawner = Node(  # questo è un nuovo nodo
    #     package = "controller_manager",
    #     executable="spawner",
    #     # namespace='/mobile_wx250s', 
    #     arguments=[
    #         '-c',
    #         'controller_manager',
    #         'gripper_controller',
    #     ],
    # )


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

    # Action call to send the arm to sleep pose
    send_sleep_pose = Node(
        package='clik1_node_pkg',
        executable='send_sleep_pose.py',  # Assicurati che sia installato correttamente
        name='send_sleep_pose',
        output='screen'
    )


    # Includi xsarm_control.launch.py
    xsarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model,
            'robot_name': robot_name,
            'use_rviz': use_rviz,
            'motor_configs': motor_configs,
            'mode_configs': mode_configs,
            'load_configs': load_configs,
            #'robot_description': arm_description,
            'use_sim': use_sim,
            'use_sim_time': use_sim_time,
            'xs_driver_logging_level': xs_driver_logging_level,
            'use_robot_state_publisher': 'false',
        }.items(),
    )

    world_to_base_link_broadcaster = Node(
        package='clik1_node_pkg',
        executable='world_to_base_link_broadcaster',
        name='world_to_base_link_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false'], description='Lancia RViz se true.'),
        bridge,
        #xsarm_control_launch,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        #gripper_controller_spawner,
        px4_sitl,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[send_sleep_pose],
            )
        ),
        world_to_base_link_broadcaster,
    ])
