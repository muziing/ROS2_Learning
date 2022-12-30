from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    turtle_node_1 = Node(
        package="turtlesim", executable="turtlesim_node", name="turtle1"
    )
    turtle_node_2 = Node(
        package="turtlesim", executable="turtlesim_node", namespace="t1"
    )
    turtle_node_3 = Node(
        package="turtlesim", executable="turtlesim_node", namespace="t1", name="turtle1"
    )

    return LaunchDescription([turtle_node_1, turtle_node_2, turtle_node_3])
