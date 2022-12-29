"""
1.创建参数对象
2.解析参数
"""

import rclpy
from rclpy.node import Node


class MyParam(Node):
    def __init__(self):
        super().__init__("mu_param_node")
        self.get_logger().info("参数API使用")

        # 创建参数对象
        p1 = rclpy.Parameter("car_name", value="Tiger")
        p2 = rclpy.Parameter("width", value=1.5)
        p3 = rclpy.Parameter("wheels", value=2)

        # 解析参数
        self.get_logger().info(f"car_name = {p1.value}")
        self.get_logger().info(f"width = {p2.value}")
        self.get_logger().info(f"wheels = {p3.value}")

        self.get_logger().info(f"key = {p1.name}")


def main():
    rclpy.init()
    MyParam()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
