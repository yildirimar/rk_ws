import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
# Hata düzeltmesi için gerekli import
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Paketinizin share dizininin yolunu al
    pkg_path = get_package_share_directory('rk_demo')

    # URDF dosyasının tam yolunu oluştur
    # DİKKAT: Dosya adınız 'robot_arm.urdf.xacro' değilse bu satırı güncelleyin
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot_arm.urdf.xacro')

    # RViz konfigürasyon dosyasının tam yolunu oluştur
    rviz_config_path = os.path.join(pkg_path, 'config', 'view.rviz')

    # robot_description parametresini oluştur
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # LaunchDescription objesini oluştur ve node'ları ekle
    return LaunchDescription([

        # Robot modelini yayınlayan state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # Eklemleri kontrol etmek için GUI arayüzü
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),

        # RViz'i, kaydedilmiş konfigürasyon dosyası ile başlat
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