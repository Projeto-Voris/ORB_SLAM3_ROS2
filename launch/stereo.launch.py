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
            'pangolin',
            default_value="False",
            description='Use the viewer'
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
            default_value='True',
            description='Rescale Image'
        ),
        DeclareLaunchArgument(
            'parent_frame_id',
            default_value='SM2/base_link',
            description='Parent link of SLAM frame'
        ),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='SM2/left_camera_link',
            description='link of SLAM frame'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='orbslam3',
            description='PointCloud SLAM link'
        ),
        
        Node(
            package='orbslam3_ros2',
            executable='stereo',
            name='stereo_orbslam3',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[
                LaunchConfiguration('vocabulary'),
                PathJoinSubstitution([
                    FindPackageShare('orbslam3_ros2'),
                    'config',  
                    'stereo',
                    LaunchConfiguration('yaml_file')
                ]),
                'True',
                LaunchConfiguration('pangolin')
            ],
            parameters=[{
                'rescale': LaunchConfiguration('rescale'),
                'parent_frame_id': LaunchConfiguration('parent_frame_id'),
                'child_frame_id': LaunchConfiguration('child_frame_id'),
                'frame_id': LaunchConfiguration('frame_id')
            }],
            remappings=[
                ('camera/left', 'left/image_raw'),
                ('camera/right', 'right/image_raw')
            ]
        )
    ])
