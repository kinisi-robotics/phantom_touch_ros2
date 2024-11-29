import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

USE_SIM_TIME = True

GUI = False
CONTROL = False


def load(path):

    with open(path, 'r') as urdf_file:
        urdf_content = urdf_file.read()

    return urdf_content

def generate_launch_description():

    # Path to URDF file
    urdf_path = os.path.join(
        get_package_share_directory("phantom_touch_description"), "urdf/phantom_touch.urdf"
    )

    urdf = load(urdf_path)

    if GUI:
        joint_state_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        # namespace=LaunchConfiguration('namespace'),
        output="screen",
        parameters=[{"use_sim_time": USE_SIM_TIME, "robot_description": urdf}],
        remappings=[
                ('/joint_states', '/TouchA/joint_states')
        ]
    )

    ld = LaunchDescription([
        
        # Declare a namespace argument with a default value
        DeclareLaunchArgument(
            'namespace',
            default_value='touch',
            description='Namespace for the robot_state_publisher node'
        )]
        )

    ld.add_action(robot_state_publisher)

    if CONTROL:
        ld.add_action(joint_state_node)

    return ld
