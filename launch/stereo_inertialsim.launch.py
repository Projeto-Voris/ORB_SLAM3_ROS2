from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess

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
            default_value='stereo-inertialsim.yaml',
            description='Name of the ORB_SLAM3 YAML configuration file'
        ),
        
        Node(
            package='orbslam3_ros2',
            executable='stereo-inertial',
            name='stereo_inertial_orbslam3',
            namespace='orbslam3_inertial',
            output='screen',
            arguments=[
                LaunchConfiguration('vocabulary'),
                PathJoinSubstitution([
                    FindPackageShare('orbslam3_ros2'),
                    'config',  
=                    LaunchConfiguration('yaml_file')
                ]),
                'False'
                'False'
            ],
            remappings=[
                ('camera/left', '/stereo_left'),
                ('camera/right', '/stereo_right'),
                ('/imu' , '/model/bluerov2_heavy/imu')
            ]
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                    '--yaw', '-1.570796327',
                    '--roll', '-1.5707963270',
                    '--pitch', '0',
                    '--frame-id', 'map',
                    '--child-frame-id', 'orbslam3'],
            output='screen',
        ),

    ])
