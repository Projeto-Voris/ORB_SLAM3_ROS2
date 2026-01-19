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
            default_value='tbuggy.yaml',
            description='Name of the ORB_SLAM3 YAML configuration file'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='orbslam3',
            description='Namespace of system'
        ),
        DeclareLaunchArgument(
            'rescale',
            default_value='True',
            description='Rescale Image'
        ),
        DeclareLaunchArgument(
            'pangolin',
            default_value='False',
            description='Use the viewer'
        ),
        DeclareLaunchArgument(
            'parent_frame_id',
            default_value='tbuggy/base_footprint',
            description='Parent link of SLAM frame'
        ),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='tbuggy/camera_front',
            description='link of SLAM frame'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='orbslam3',
            description='PointCloud SLAM link'
        ),
        DeclareLaunchArgument(
            'ENU_publish',
            default_value='False',
            description='Publish poses in ENU frame'
        ),
        DeclareLaunchArgument(
            'tracked_points',
            default_value='True',
            description='Publish tracked image'
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
            parameters=[{'rescale': LaunchConfiguration('rescale'),
                        'tracked_points': LaunchConfiguration('tracked_points'),
                        'parent_frame_id': LaunchConfiguration('parent_frame_id'),
                        'child_frame_id': LaunchConfiguration('child_frame_id'),
                        'frame_id': LaunchConfiguration('frame_id'),
                        'ENU_publish': LaunchConfiguration('ENU_publish')}],
            remappings=[
                ('camera', '/tbuggy/camera_front/image_raw'),  # Remap the camera topic to the video frames topic
                #('pose', '/mavros/vision_pose/pose'),
            ]
        )
    ])
