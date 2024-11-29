import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable

USE_SIM_TIME = True

GUI = False
CONTROL = GUI


def load(path):

    with open(path, 'r') as urdf_file:
        urdf_content = urdf_file.read()

    return urdf_content

def generate_launch_description():

    # prefix = LaunchConfiguration('prefix')

    phantom_xacro_file = os.path.join(
        get_package_share_directory("phantom_touch_description"),
        "urdf",
        "phantom_touch_instance.xacro.urdf",
    )

    robot_descriptionA = Command(
        [FindExecutable(name="xacro"), " ", phantom_xacro_file, f" prefix:='touch_a_'"]
    )

    robot_descriptionB = Command(
        [FindExecutable(name="xacro"), " ", phantom_xacro_file, f" prefix:='touch_b_'"]
    )

    if GUI:
        joint_state_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            # namespace="touch_",
            output="screen",
        )

    robot_state_publisher_A = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="TouchA",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM_TIME, "robot_description": ParameterValue(robot_descriptionA, value_type=str)}],
        remappings=[
                ('/joint_states', '/TouchA/joint_states')
        ]
    )

    robot_state_publisher_B = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="TouchB",
        output="screen",
        parameters=[{"use_sim_time": USE_SIM_TIME, "robot_description": ParameterValue(robot_descriptionB, value_type=str)}],
        remappings=[
                ('/joint_states', '/TouchB/joint_states')
        ]
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "-0.25", "0.0", "0.0", "0.0", "0.0", "touch_a_base", "touch_b_base"],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "-0.25", "0.0", "0.0", "0.0", "0.0", "touch_a_base", "touch_b_base"],
    )

    static_tf_node_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "-0.25", "0.0", "0.0", "0.0", "0.0", "touch_a_base", "base_link"],
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_A)
    ld.add_action(robot_state_publisher_B)
    ld.add_action(static_tf_node)
    ld.add_action(static_tf_node_base_link)

    

    if CONTROL:
        ld.add_action(joint_state_node)

    return ld
