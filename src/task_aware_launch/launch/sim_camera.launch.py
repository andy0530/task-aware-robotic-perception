from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    sim_share = get_package_share_directory('task_aware_sim')
    world_path = os.path.join(sim_share, 'worlds', 'table_camera.sdf')

    launch_share = get_package_share_directory('task_aware_launch')
    rviz_config = os.path.join(launch_share, 'rviz', 'camera_view.rviz')

    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-r',
            world_path,
            '--render-engine', 'ogre'
        ],
        output='screen'
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/fixed_camera/rgb/image'],
        remappings=[('/fixed_camera/rgb/image', '/camera/color/image_raw'),],
        output='screen'
    )

    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '1.8',     # x y z (meters)
            '0', '1.5708', '0',  # roll pitch yaw (radians)  <-- adjust if needed
            'world',
            'fixed_camera/camera_link/rgb_camera'
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        image_bridge,
        static_tf_world,
        rviz,
    ])
