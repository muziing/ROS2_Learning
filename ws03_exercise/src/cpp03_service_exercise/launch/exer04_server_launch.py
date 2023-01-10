from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    t = Node(package="turtlesim", executable="turtlesim_node")
    server = Node(package="cpp03_service_exercise", executable="exer04_action_server")
    node_list = [t, server]
    return LaunchDescription(node_list)
