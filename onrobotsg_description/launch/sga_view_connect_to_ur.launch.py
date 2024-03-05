import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[ "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "tool0", "gripper_adapter_link"]
    )

    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory("onrobotsg_description"), "urdf", "sga", "onrobotsg_a.urdf.xacro"))
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="gripper",
        output="screen",
        parameters=[robot_description])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory("onrobotsg_description"), "rviz", "onrobotsg_view.rviz")],
        parameters=[robot_description])

    return LaunchDescription([static_tf, rviz_node, robot_state_pub_node])
