import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # Enable CycloneDDS with shared memory
    cyclone_dds_uri = SetEnvironmentVariable(
        name='CYCLONEDDS_URI',
        value='<CycloneDDS><Domain><SharedMemory><Enable>true</Enable></SharedMemory></Domain></CycloneDDS>'
    )

    # Or use Fast-DDS shared memory
    fastdds_uri = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE',
        value=os.path.join(os.path.dirname(__file__), 'fastdds_shm.xml')
    )

    stereo_node = Node(
        package='orbslam3_ros2',
        executable='stereo',
        name='stereo_slam',
        arguments=['path_to_vocab', 'path_to_settings', 'do_rectify'],
        output='screen'
    )

    saver_node = Node(
        package='orbslam3_ros2',
        executable='stereo_image_saver',
        name='stereo_image_saver',
        parameters=[{'save_directory': '/tmp/stereo_images'}],
        output='screen'
    )

    return LaunchDescription([
        cyclone_dds_uri,
        stereo_node,
        saver_node
    ])
