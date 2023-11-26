import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Определение путей к файлам
    package_description = "barista_robot_description"
    urdf_file = 'barista_robot_model.urdf.xacro'
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", urdf_file)

    # Создание команды для парсинга URDF для каждого робота
    rick_robot_desc_command = Command(['xacro ', robot_desc_path, ' robot_name:=rick'])
    morty_robot_desc_command = Command(['xacro ', robot_desc_path, ' robot_name:=morty'])

    # Gazebo и RVIZ конфигурация
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_barista_gazebo = get_package_share_directory('barista_robot_description')
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')))
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'barista_robot.rviz')

    # Настройка нод для каждого робота
    rick_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='rick',
        parameters=[{'use_sim_time': True, 'robot_description': rick_robot_desc_command}],
        output="screen"
    )

    morty_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='morty',
        parameters=[{'use_sim_time': True, 'robot_description': morty_robot_desc_command}],
        output="screen"
    )

    # Настройка Joint State Publisher для каждого робота
    rick_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='rick',
        output="screen"
    )

    morty_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='morty',
        output="screen"
    )

    # Настройка RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir])

    # Настройка спавна для каждого робота
    rick_spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='rick',
        output='screen',
        arguments=['-entity', 'rick_robot', '-x', '0', '-y', '0', '-z', '0.2', '-topic', 'robot_description']
    )

    morty_spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='morty',
        output='screen',
        arguments=['-entity', 'morty_robot', '-x', '2', '-y', '0', '-z', '0.2', '-topic', 'robot_description']
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=[os.path.join(pkg_barista_gazebo, 'worlds', 'barista_robot_empty.world'), ''], description='SDF world file'),
        rick_robot_state_publisher_node,
        rick_joint_state_publisher_node,
        rick_spawn_robot,
        morty_robot_state_publisher_node,
        morty_joint_state_publisher_node, 
        morty_spawn_robot,
        rviz_node,
        gazebo
    ])
