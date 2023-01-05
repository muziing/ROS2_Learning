from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 1.turtlesim_node
    turtle_node = Node(package="turtlesim", executable="turtlesim_node")

    # 2.自定义的服务端
    server_node = Node(package="cpp02_service_exercise", executable="exer02_server")

    return LaunchDescription([turtle_node, server_node])
