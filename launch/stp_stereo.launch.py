from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # TF 변환 설정 (map -> base_link)
         Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             name='static_transform_publisher_map_to_base_link',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
         ),
        # TF 변환 설정 (base_link -> left_camera_frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_to_left_camera',
            arguments=['-0.03825', '0', '0', '0', '0', '0', 'base_link', 'left_camera_frame']
        ),
        # TF 변환 설정 (base_link -> right_camera_frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_to_right_camera',
            arguments=['0.03825', '0', '0', '0', '0', '0', 'base_link', 'right_camera_frame']
        ),

    ])
