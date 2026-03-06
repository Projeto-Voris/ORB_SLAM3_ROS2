from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='Passive',
            description='Namespace of system'
        ),
        DeclareLaunchArgument(
            'rescale',
            default_value='True',
            description='Rescale Image'
        ),
        DeclareLaunchArgument(
            'parent_frame_id',
            default_value='base_link',
            description='Parent link of SLAM frame'
        ),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='Passive/left_camera_link',
            description='link of SLAM frame'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='PointCloud SLAM link'
        ),
        DeclareLaunchArgument('left_image', default_value=['left/image_raw'], description='stereo left image'),
        DeclareLaunchArgument('right_image', default_value=['right/image_raw'], description='stereo right image'),
        DeclareLaunchArgument('voc_file', default_value='/home/jetson/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/vocabulary/ORBvoc.txt', 
                  description='Caminho para o vocabulário ORB'),
        DeclareLaunchArgument('settings_file', default_value='/home/jetson/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/config/lab_bw.yaml', 
                  description='Caminho para o settings .yaml'),
        Node(
            package='orbslam3_ros2',
            executable='stereo-inertial',
            name='stereo_inertial_orbslam3',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'voc_file': LaunchConfiguration('voc_file'),
                'settings_file': LaunchConfiguration('settings_file'),
                'rescale': LaunchConfiguration('rescale'),
                'do_rectify': True,
                'ENU_publish': True,
                'parent_frame_id': LaunchConfiguration('parent_frame_id'),
                'child_frame_id': LaunchConfiguration('child_frame_id'),
                'frame_id': LaunchConfiguration('frame_id')
            }],
            remappings=[
                ('camera/left', 'left/image_raw'),
                ('camera/right', 'right/image_raw'),
                ('/imu' , 'imu/data_raw')  
            ]
        )
    ])
