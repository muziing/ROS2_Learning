import rclpy
from rclpy.node import Node

# 自定义类
class MyNode(Node):
    def __init__(self):
        super().__init__("hello_node_py")
        self.get_logger().info("Hello World!")


def main():

    # 方式1（不推荐）
    # # 初始化 ROS2
    # rclpy.init()

    # # 创建节点
    # node = rclpy.create_node("helloworld_py_node")

    # # 输出文本
    # node.get_logger().info("Hello World!")

    # # 释放资源
    # rclpy.shutdown()

    # 方式2（推荐）
    rclpy.init()
    node = MyNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
