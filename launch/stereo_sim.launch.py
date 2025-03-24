from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=['orbslam3']
        ),
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
            default_value="True",
            description='Use the viewer'
        ),
        DeclareLaunchArgument(
            'yaml_file',
            default_value='sm2_stereosim.yaml',
            description='Name of the ORB_SLAM3 YAML configuration file'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value=['orbslam3']
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=['orbslam3']
        ),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value=['left_camera_link']
        ),

        Node(
            package='orbslam3_ros2',
            executable='stereo',
            name='stereo_orbslam3',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            arguments=[
                LaunchConfiguration('vocabulary'),
                PathJoinSubstitution([
                    FindPackageShare('orbslam3_ros2'),
                    'config',  
                    'stereo',
                    LaunchConfiguration('yaml_file')
                ]),
                'False',
                LaunchConfiguration('pangolin')
            ],
            remappings=[
                ('camera/left', '/stereo_left'),
                ('camera/right', '/stereo_right')
            ],
            parameters = [{'frame_id':LaunchConfiguration('frame_id'), 'child_frame_id':LaunchConfiguration('child_frame_id')}]
        ),
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                    # '--yaw', '-1.570796327',
                    # '--roll', '-1.5707963270',
                    # '--pitch', '0',
                    '--frame-id', 'map',
                    '--child-frame-id', 'orbslam3'],
            output='screen',
        ),
        TimerAction(
            actions=[ExecuteProcess(
                cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                     '--yaw', '-1.570796327',
                     '--roll', '-1.5707963270',
                     '--pitch', '0',
                     '--frame-id', 'left_camera_link',
                     '--child-frame-id', 'left_camera_frame'],
                output='screen'
            )],
            period='1'
        )
    ])
