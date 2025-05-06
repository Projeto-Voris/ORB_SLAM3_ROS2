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
            default_value='25red.yaml',
            description='Name of the ORB_SLAM3 YAML configuration file'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='SM2',
            description='Namespace of system'
        ),
        DeclareLaunchArgument(
            'rescale',
            default_value='False',
            description='Rescale Image'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='SM2',
            description='Namespace of system'
        ),
        DeclareLaunchArgument(
            'pangolin',
            default_value='False',
            description='Use the viewer'
        ),
        
        Node(
            package='orbslam3_ros2',
            executable='mono',
            namespace=LaunchConfiguration('namespace'),
            name='mono_orbslam3',
            output='screen',
            arguments=[
                LaunchConfiguration('vocabulary'),
                PathJoinSubstitution([
                    FindPackageShare('orbslam3_ros2'),
                    'config',  # Assuming your config files are in the config directory
                    'monocular',
                    LaunchConfiguration('yaml_file')  # Use the file name directly
                ]),
                LaunchConfiguration('pangolin')
            ],
            parameters=[{'rescale': LaunchConfiguration('rescale')}],
            remappings=[
                ('camera', '/SM2/left/image_raw')
            ]
        )
    ])
