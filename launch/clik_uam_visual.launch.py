import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('clik1_node_pkg')
    
    # Percorso al file di configurazione di RViz
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'clik_uam.rviz')

    # Argomento per decidere se usare la posa da Gazebo
    use_gazebo_pose_arg = DeclareLaunchArgument(
        'use_gazebo_pose',
        default_value='true',
        description='Set to true to use pose from Gazebo, false to use PX4 topics.'
    )

    # # Nodo clik_uam_node
    # clik_uam_node = Node(
    #     package='clik1_node_pkg',
    #     executable='clik_uam_node',
    #     name='clik_uam_node',
    #     output='screen',
    #     parameters=[{
    #         'use_gazebo_pose': LaunchConfiguration('use_gazebo_pose')
    #     }]
    # )

    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    # Nodo robot_state_publisher per pubblicare le trasformate del modello
    # Legge /robot_description e /joint_states
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': open(os.path.join(pkg_share, 'model', 't960a.urdf')).read()}]
    # )

    # Nodo rviz_marker_node
    rviz_marker_node = Node(
        package='clik1_node_pkg',
        executable='rviz_marker_node',
        name='rviz_marker_node',
        output='screen'
    )


    return LaunchDescription([
        use_gazebo_pose_arg,
        #robot_state_publisher_node,
        rviz_node,
        rviz_marker_node,
        # clik_uam_node,
    ])
