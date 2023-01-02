import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time


class MyNode(Node):
    def __init__(self):
        super().__init__("time_calculate_node_py")
        self.demo_opt()

    def demo_opt(self):
        t1 = Time(seconds=20)
        t2 = Time(seconds=15)

        du1 = Duration(seconds=7)
        du2 = Duration(seconds=13)

        self.get_logger().info(f"t1 >= t2 ? {t1 >= t2}")
        self.get_logger().info(f"du2 > du1 ? {du2 > du1}")

        du3 = t1 - t2
        t4 = t1 + du1
        self.get_logger().info(f"du3 = {du3.nanoseconds} nsec")
        self.get_logger().info(f"t4 = {t4.nanoseconds} nsec")

        self.get_logger().info(f"du2 > du1 ? {du2 > du1}")


def main():
    rclpy.init()
    MyNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
