import rclpy
from rclpy.node import Node
import threading


class TimeNode(Node):
    def __init__(self):
        super().__init__("time_node_py")
        self.demo_rate()

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


def main():
    rclpy.init()
    rclpy.spin(TimeNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
