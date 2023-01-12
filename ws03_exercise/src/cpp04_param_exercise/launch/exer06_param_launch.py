from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    t = Node(package="turtlesim", executable="turtlesim_node")
    param = Node(package="cpp04_param_exercise", executable="exer06_param")
    node_list = [t, param]
    return LaunchDescription(node_list)
