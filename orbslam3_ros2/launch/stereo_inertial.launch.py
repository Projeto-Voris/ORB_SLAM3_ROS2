from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ComposableNode 
from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
import os

def launch_setup(context, *args, **kwargs):
    composable_nodes = [
        ComposableNode(
                    package='orbslam3_ros2',
                    plugin='orbslam3_ros2::StereoSlamNode',
                    name='slam_stereo_node',
                    namespace=LaunchConfiguration('namespace'),
                    parameters=[{
                        'voc_file': LaunchConfiguration('voc_file'),
                        'settings_file': LaunchConfiguration('settings_file'),
                        'ENU_publish': True,
                        'tf_publish': False,
                        'resize_factor': 0.25,
                        'frame_id': LaunchConfiguration('frame_id'),
                        'parent_frame_id': LaunchConfiguration('parent_frame_id'),
                        'child_frame_id': LaunchConfiguration('child_frame_id'),
                        'tracked_points': True,
                    }],
                    remappings=[
                        ('camera/left', "/Passive/left/image_raw"),
                        ('camera/right', "/Passive/right/image_raw"),
                        ('pose_cov', 'slam/pose_cov'),
                        ('imu', 'imu/data_raw')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
    ]
    
    composable = ComposableNodeContainer(
        name='passive_stereo_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container_mt', # IMPORTANTE: Multi-Threaded para performance
        composable_node_descriptions=composable_nodes,
        output='screen',
    )
    return [composable]
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
        DeclareLaunchArgument('voc_file', default_value=f'/home/{os.getenv("USER")}/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/vocabulary/ORBvoc.txt', 
                  description='Caminho para o vocabulário ORB'),
        DeclareLaunchArgument('settings_file', default_value=f'/home/{os.getenv("USER")}/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/config/lab_bw.yaml', 
                  description='Caminho para o settings .yaml'),

    ])
