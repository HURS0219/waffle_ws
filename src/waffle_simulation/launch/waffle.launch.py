import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('waffle_simulation')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(pkg_share, 'worlds', 'maze.world')
    default_rviz = os.path.join(pkg_share, 'config', 'waffle.rviz')

    model = 'waffle'
    urdf_path = os.path.join(
        tb3_gazebo_share, 'urdf', 'turtlebot3_{}.urdf'.format(model))
    sdf_path = os.path.join(
        tb3_gazebo_share, 'models',
        'turtlebot3_{}'.format(model), 'model.sdf')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # Launch configuration variables
    world = LaunchConfiguration('world', default=default_world)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.75')
    y_pose = LaunchConfiguration('y_pose', default='0.75')
    yaw = LaunchConfiguration('yaw', default='0.0')
    open_rviz = LaunchConfiguration('open_rviz', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to the SDF world file to load',
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_x_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.75',
        description='Spawn x position (default: maze start cell)')
    declare_y_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.75',
        description='Spawn y position (default: maze start cell)')
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', default_value='0.0', description='Spawn yaw in radians')
    declare_open_rviz_cmd = DeclareLaunchArgument(
        'open_rviz',
        default_value='true',
        description='Start rviz2 automatically if true',
    )
    declare_use_gui_cmd = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start the Gazebo GUI (gzclient) if true',
    )

    set_turtlebot3_model_cmd = SetEnvironmentVariable(
        'TURTLEBOT3_MODEL', model)

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(use_gui),
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model,
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw,
        ],
        output='screen',
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(open_rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_world_cmd,
        declare_use_sim_time_cmd,
        declare_x_cmd,
        declare_y_cmd,
        declare_yaw_cmd,
        declare_open_rviz_cmd,
        declare_use_gui_cmd,
        set_turtlebot3_model_cmd,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        rviz_cmd,
    ])
