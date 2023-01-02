from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 修改节点名称
    turtle_node_1 = Node(
        package="turtlesim", executable="turtlesim_node", name="turtle1"
    )
    turtle_node_2 = Node(
        package="turtlesim", executable="turtlesim_node", namespace="t1"
    )
    turtle_node_3 = Node(
        package="turtlesim", executable="turtlesim_node", namespace="t1", name="turtle1"
    )

    # 修改话题名称
    turtle_node_4 = Node(
        package="turtlesim", executable="turtlesim_node", namespace="t1"
    )  # 通过修改命名空间，也可以对应修改话题名称
    turtle_node_5 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtle1",
        remappings=[("/turtle1/cmd_vel", "/cmd_vel")],
    )

    # return LaunchDescription([turtle_node_1, turtle_node_2, turtle_node_3])
    return LaunchDescription([turtle_node_4, turtle_node_5])
