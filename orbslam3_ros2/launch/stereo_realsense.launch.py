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
                'vocabulary',
                'ORBvoc.txt'
            ]),
            description='Path to the ORB_SLAM3 vocabulary file'
        ),
        DeclareLaunchArgument(
            'pangolin',
            default_value='True',
            description='Use the viewer'
        ),
        DeclareLaunchArgument(
            'yaml_file',
            # !!! IMPORTANT !!!
            # This must be a YAML file calibrated for the
            # D455's INFRARED (infra1/infra2) stereo sensors,
            # NOT the color camera.
            default_value='realsense_stereo_infra.yaml',
            description='Name of the ORB_SLAM3 YAML configuration file'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='SM2',
            description='Namespace of system'
        ),
        DeclareLaunchArgument(
            'rescale',
            default_value='false',
            description='Rescale Image'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='orbslam3',
            description='frame_id'
        ),
        DeclareLaunchArgument(
            'parent_frame_id',
            default_value='camera_link',
            description='parent_frame_id'
        ),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='camera_infra1_optical_frame',
            description='child_frame_id'
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
=                    LaunchConfiguration('yaml_file')
                ]),
                'True',
                LaunchConfiguration('pangolin')
            ],
            parameters=[
                {'rescale': LaunchConfiguration('rescale')},
                {'frame_id': LaunchConfiguration('frame_id')},
                {'parent_frame_id': LaunchConfiguration('parent_frame_id')},
                {'child_frame_id': LaunchConfiguration('child_frame_id')}
                ],
            remappings=[
                # Map the node's internal 'camera/left' topic to the
                # RealSense's rectified infra1 image stream.
                ('camera/left', '/VORIS/SM2/infra1/image_rect_raw'),
                
                # Map the node's internal 'camera/right' topic to the
                # RealSense's rectified infra2 image stream.
                ('camera/right', '/VORIS/SM2/infra2/image_rect_raw')
            ]
        )
    ])
