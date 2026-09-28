from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    world_node = Node(
        package='grid_world',
        executable='grid_world_node',
        name='grid_world_node',
        output='screen',
    )

    solver_node = Node(
        package='grid_world',
        executable='grid_world_solver',
        name='grid_world_solver',
        output='screen',
    )

    viewer_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/world_img'],
        output='screen'
    )

    robot_viewer_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/robot_view_img'],
        output='screen'
    )

    return LaunchDescription([
        world_node,
        solver_node,
        viewer_node,
        robot_viewer_node,
    ])