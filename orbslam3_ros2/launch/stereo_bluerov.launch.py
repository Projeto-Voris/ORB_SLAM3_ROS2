from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vocabulary', default_value=PathJoinSubstitution([FindPackageShare('orbslam3_ros2'),'vocabulary','ORBvoc.txt' ]), description='Path to ORB_SLAM3 vocabulary file'),
        DeclareLaunchArgument('pangolin', default_value="False", description='Use the viewer'),
        DeclareLaunchArgument('yaml_file', default_value='stereo_bluerov_uw_bw.yaml', description='Name of the ORB_SLAM3 YAML configuration file'),
        DeclareLaunchArgument('namespace', default_value='Passive', description='Namespace of system'),
        DeclareLaunchArgument('rescale', default_value='True', description='Rescale Image'),
        DeclareLaunchArgument('parent_frame_id', default_value='base_link', description='Parent link of SLAM frame'),
        DeclareLaunchArgument('child_frame_id', default_value='Passive/left_camera_link', description='link of SLAM frame'),
        DeclareLaunchArgument('frame_id', default_value='map', description='PointCloud SLAM link'),
        DeclareLaunchArgument('ENU_publish', default_value='True', description='Publish poses in ENU frame'),
        DeclareLaunchArgument('tracked_points', default_value='True', description='Publish tracked points?'),
        DeclareLaunchArgument('pose', default_value='pose', description='Pose topic name'),
        DeclareLaunchArgument('left_image', default_value=['left/image_raw'], description='stereo left image'),
        DeclareLaunchArgument('right_image', default_value=['right/image_raw'], description='stereo right image'),
        
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
                    LaunchConfiguration('yaml_file')]),
                'True',
                LaunchConfiguration('pangolin'),
                '--with-saver'
            ],
            parameters=[{
                'rescale': LaunchConfiguration('rescale'),
                'parent_frame_id': LaunchConfiguration('parent_frame_id'),
                'child_frame_id': LaunchConfiguration('child_frame_id'),
                'frame_id': LaunchConfiguration('frame_id'),
                'ENU_publish': LaunchConfiguration('ENU_publish'),
                'tracked_points': LaunchConfiguration('tracked_points')
            }],
            remappings=[
                ('camera/left', LaunchConfiguration('left_image')),
                ('camera/right', LaunchConfiguration('right_image')),
                ('pose', LaunchConfiguration('pose'))
            ]
        )
    ])
