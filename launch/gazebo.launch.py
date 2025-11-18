import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path


def generate_launch_description():
    bringup_dir = get_package_share_directory('differential_robot')
    world = os.path.join(bringup_dir , "world", "depot.sdf")
    urdf_file  =  os.path.join(bringup_dir, 'src', 'description', 'robot.urdf')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'gazebo.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc}
        ])
    
#    joint_state_publisher_gui_node = Node(
#        package='joint_state_publisher_gui',
#        executable='joint_state_publisher_gui',
#        name='joint_state_publisher_gui',
#    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(bringup_dir, 'world'),
            str(Path(bringup_dir).parent.resolve())
        ])
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(bringup_dir, 'config', 'gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": ["-r -v 4 ", world]}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "robot",
            "-topic",
            "/robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    frame_id_converter_node = Node(
        package='differential_robot',
        executable='laser_scan_frame_id_convertor',
        name='laser_scan_frame_id_convertor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='True',description='Use sim time if true'),
        DeclareLaunchArgument('urdf_file',default_value=os.path.join(bringup_dir, 'src', 'description', 'robot.urdf'),description='Whether to start RVIZ'),
        DeclareLaunchArgument('use_robot_state_pub',default_value='True',description='Whether to start the robot state publisher'),
        gz_resource_path,
        gz_sim,bridge,
        start_robot_state_publisher_cmd,
        # joint_state_publisher_gui_node,
        spawn_entity,
        rviz_node, 
        frame_id_converter_node,
    ])
