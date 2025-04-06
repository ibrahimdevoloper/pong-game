import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('pong_game')
    robot_description_path = os.path.join(package_dir, 'resource', 'robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'main.2.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    keyboard_control_publisher = Node(
        package='pong_game',
        executable='keyboard_control_publisher',
        # name='keyboard_control_publisher',
        # output='screen',
        # arguments=[],
        # parameters=[
        #     {'robot_description': robot_description_path},
        # ]
    )

    camera_feed_publisher = Node(
        package='pong_game',
        executable='camera_feed_publisher',
    #     name='camera_feed_publisher',
    #     output='screen',
    #     arguments=[],
    #     parameters=[
    #         {'robot_description': robot_description_path},
    #     ]
     )
    
    behaviour_control_publisher = Node(
        package='pong_game',
        executable='behaviour_control_publisher',
    #     name='behaviour_control_publisher',
    #     output='screen',
    #     arguments=[],
    #     parameters=[
    #         {'robot_description': robot_description_path},
    #     ]
    )


    return LaunchDescription([
        webots,
        my_robot_driver,
        keyboard_control_publisher,
        camera_feed_publisher,
        behaviour_control_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])