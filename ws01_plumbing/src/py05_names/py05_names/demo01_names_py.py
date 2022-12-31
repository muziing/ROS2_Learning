import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        # 通过编程方式设置命名空间
        super().__init__("my_node_py", namespace="t1_ns_py")


def main():
    rclpy.init()
    rclpy.spin(MyNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
