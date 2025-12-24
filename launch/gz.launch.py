from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('rk_demo')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot_arm.urdf.xacro')

    # --- FIX STARTS HERE ---
    # We need to tell Gazebo where the "rk_demo" package is located.
    # get_package_share_directory returns: .../install/rk_demo/share/rk_demo
    # We need the PARENT directory: .../install/rk_demo/share
    pkg_share_parent = os.path.dirname(pkg_path)
    
    # Check if the variable already exists so we append to it, rather than overwrite
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path = pkg_share_parent + ':' + os.environ['GZ_SIM_RESOURCE_PATH']
    else:
        gz_resource_path = pkg_share_parent

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=gz_resource_path
    )
    # --- FIX ENDS HERE ---

    return LaunchDescription([
        # 1. Register the environment variable FIRST
        set_resource_path,


        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ]
        ),


        # 2. Start Gazebo
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

        # 3. Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command([
                    "xacro ", urdf_path
                ])
            }]
        ),

        # 4. Spawn Robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-topic', 'robot_description'
            ],
            output='screen'
        ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['position_controller'],
        #     parameters=['robot_description']
        # ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            parameters=['robot_description']
        ),


        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            parameters=['robot_description',]
        )


    ])