from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vocabulary',
            default_value=PathJoinSubstitution([
                FindPackageShare('orbslam3_ros2'),
                'vocabulary',  # Assuming your vocab file is in the vocabulary directory
                'ORBvoc.txt'   # Replace with your actual vocabulary file name
            ]),
            description='Path to the ORB_SLAM3 vocabulary file'
        ),
        DeclareLaunchArgument(
            'yaml_file',
            default_value='stereo-inertial-rescaled.yaml',
            description='Name of the ORB_SLAM3 YAML configuration file'
        ),
        DeclareLaunchArgument('namespace', default_value='SM2', description='namespace of node'),
        
        Node(
            package='orbslam3_ros2',
            executable='stereo-inertial',
            name='stereo_inertial_orbslam3',
            namespace=LaunchConfiguration('namespace'),

            output='screen',
            arguments=[
                LaunchConfiguration('vocabulary'),
                PathJoinSubstitution([
                    FindPackageShare('orbslam3_ros2'),
                    'config',  
                    'stereo-inertial',
                    LaunchConfiguration('yaml_file')
                ]),
                'True'
                'False'
            ],
            remappings=[
                ('camera/left', '/SM2/left/image_rescaled'),
                ('camera/right', '/SM2/right/image_rescaled'),
                ('/imu' , '/SM2/imu/data_raw')  
            ]
        )
    ])
