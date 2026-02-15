from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    sim_share = get_package_share_directory('task_aware_sim')
    world_path = os.path.join(sim_share, 'worlds', 'table_v1.sdf')

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
    
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/fixed_camera/rgb/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/fixed_camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/fixed_camera/rgb/image', '/camera/color/image_raw'),
            ('/fixed_camera/rgb/camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )


    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '1.8',     # x y z (meters)
            '0', '1.5708', '0',  # roll pitch yaw (radians)
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
    
    yolo_node = Node(
        package='task_aware_perception',
        executable='yolo_node',
        output='screen',
        parameters=[{
            'venv_path': '/home/nd/venvs/task_aware_ml',
            'device': 'cuda',
            'conf': 0.35,
        }],
    )

    sort_tracker = Node(
        package='task_aware_tracking',
        executable='sort_tracker',
        output='screen',
        parameters=[{
            'iou_thresh': 0.3,
            'max_age': 10,
        }],
    )

    dashboard = Node(
        package='task_aware_tools',
        executable='dashboard',
        output='screen',
        parameters=[{
            'refresh_hz': 5.0,
        }],
    )
    
    return LaunchDescription([
        gz_sim,
        camera_bridge,
        static_tf_world,
        rviz,
        yolo_node,
        sort_tracker,
        dashboard
    ])
