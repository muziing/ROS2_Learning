import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration


class TimeNode(Node):
    def __init__(self):
        super().__init__("time_node_py")
        # self.demo_rate()
        # self.demo_time()
        self.demo_duration()

    def demo_rate(self):
        # 1.创建 Rate 对象
        self.rate = self.create_rate(1.0)

        # 2.调用sleep函数 --- 会导致程序阻塞
        # while rclpy.ok():
        #     self.get_logger().info("++++++++++++++++")
        #     self.rate.sleep()

        # 创建子线程实现运行频率控制
        thread = threading.Thread(target=self.do_something)
        thread.start()

    def do_something(self):
        while rclpy.ok():
            self.get_logger().info("++++++++++++++++")
            self.rate.sleep()

    def demo_time(self):
        # 1.创建time对象
        t1 = Time(seconds=5, nanoseconds=500000000)
        right_now = self.get_clock().now()

        # 2.调用time函数
        self.get_logger().info(
            f"sec = {t1.seconds_nanoseconds()[0]} nsec = {t1.seconds_nanoseconds()[1]}"
        )
        self.get_logger().info(
            f"sec = {right_now.seconds_nanoseconds()[0]} nsec = {right_now.nanoseconds}"
        )

    def demo_duration(self):
        # 1.创建Duration对象
        du1 = Duration(seconds=10, nanoseconds=500000000)

        # 2.调用函数
        self.get_logger().info(f"nsec = {du1.nanoseconds}")


def main():
    rclpy.init()
    rclpy.spin(TimeNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
