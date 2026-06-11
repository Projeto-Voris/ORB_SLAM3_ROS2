from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ComposableNode 
from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

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
                        'do_rectify': True,
                        'ENU_publish': True,
                        'tf_publish': False,
                        'resize_factor': 0.25,
                        'frame_id': 'map',
                        'parent_frame_id': 'base_link',
                        'child_frame_id': 'Passive/left_camera_link',
                        'tracked_points': True,
                        'clahe': True,
                    }],
                    remappings=[
                        ('camera/left', "/Passive/left/image_raw"),
                        ('camera/right', "/Passive/right/image_raw"),
                        ('pose_cov', 'slam/pose_cov')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
    ]
    if LaunchConfiguration('save_stereo').perform(context) == 'true':
        composable_nodes.append(ComposableNode(
                    package='orbslam3_ros2',
                    plugin='slam::ImageSaver',
                    name='stereo_image_saver',
                    namespace=LaunchConfiguration('namespace'),
                    parameters=[{
                        'saving_path': '/home/jetson/stereo_images',
                        'apply_clahe': True,
                        'clahe_tiles': 5.0,
                        'clahe_clip': 5.0,
                    }],
                    remappings=[
                        ('camera/left', "/Passive/left/image_raw"),
                        ('camera/right', "/Passive/right/image_raw"),
                        ('odometry', '/mavros/local_position/odom')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ))
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
        DeclareLaunchArgument('namespace',default_value='debug',description='Namespace of system' ),
        DeclareLaunchArgument('save_stereo', default_value='true', description='Habilitar salvamento de imagens stereo'),
        DeclareLaunchArgument('voc_file', default_value='/home/jetson/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/vocabulary/ORBvoc.txt', 
                  description='Caminho para o vocabulário ORB'),
        DeclareLaunchArgument('settings_file', default_value='/home/jetson/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/config/stereo_bluerov.yaml', 
                  description='Caminho para o settings .yaml'),

        OpaqueFunction(function=launch_setup),
    ])
