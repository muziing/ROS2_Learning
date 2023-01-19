"""
需求: 创建三个turtlesim_node,前两个划分为一组、第三个单独一组
"""

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    # 创建3个节点
    t1 = Node(package="turtlesim", executable="turtlesim_node", name="t1")
    t2 = Node(package="turtlesim", executable="turtlesim_node", name="t2")
    t3 = Node(package="turtlesim", executable="turtlesim_node", name="t3")

    # 分组
    group1 = GroupAction([PushRosNamespace("g1"), t1, t2])  # 设置当前组命名空间，以及包含的节点
    group2 = GroupAction([PushRosNamespace("g2"), t3])

    node_list = [group1, group2]
    return LaunchDescription(node_list)
