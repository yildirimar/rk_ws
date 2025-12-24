from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('rk_demo')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot_arm.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path,'config','view.rviz')

    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_path, '..')
    )

    return LaunchDescription([

        set_gz_resource_path,


        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ]
        ),

        # Gazebo başlat
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py')]
            ),
            launch_arguments={
                'gz_args': '-r empty.sdf'
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{
                # **EKLEME: Fixed Frame'i 'world' olarak ayarla**
                'fixed_frame': 'world' 
            }],
            arguments=['-d', rviz_config_path],
        ),



        # Robot State Publisher (robot_description paramı üretir)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command([
                    "xacro ", urdf_path
                ]),
            }, "use_sim_time = true"]
        ),

        # Gazebo'ya spawn et
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-topic', 'robot_description'
            ],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['position_controller'],
            parameters=['robot_description']
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            parameters=['robot_description',]
        )

    ])