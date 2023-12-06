import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    # Path to files folder
    urdf_file = 'barista_robot_model.urdf.xacro'

    package_description = "barista_robot_description"

    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", urdf_file)

    # Parsing to each robot name
    rick_robot_desc_command = Command(['xacro ', robot_desc_path, ' robot_name:=rick'])
    morty_robot_desc_command = Command(['xacro ', robot_desc_path, ' robot_name:=morty'])

    # Gazebo and Rviz config
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_barista_gazebo = get_package_share_directory('barista_robot_description')

    install_dir = get_package_prefix(package_description)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in barista_robot_description package
    gazebo_models_path = os.path.join(pkg_barista_gazebo, 'meshes')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')))

    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'barista_robot.rviz')

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    rick_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='rick',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': rick_robot_desc_command, 'frame_prefix': "rick/"}],
        output="screen"
    )

    morty_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='morty',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': morty_robot_desc_command, 'frame_prefix': "morty/"}],
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir])

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

    static_tf_pub_rick = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'rick/odom']
    )

    static_tf_pub_morty = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'morty/odom']
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=[os.path.join(pkg_barista_gazebo, 'worlds', 'barista_robot_empty.world'), ''], description='SDF world file'),
        rick_robot_state_publisher_node,
        rick_spawn_robot,
        morty_robot_state_publisher_node,
        static_tf_pub_morty,
        static_tf_pub_rick,
        morty_spawn_robot,
        rviz_node,
        gazebo
    ])
