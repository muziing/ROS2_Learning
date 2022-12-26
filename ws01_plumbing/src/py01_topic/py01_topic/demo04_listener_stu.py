"""
需求：订阅发布方发布的消息，并在终端输出
流程：
    1.导包
    2.初始化 ROS2 客户端
    3.自定义节点类
        3-1.创建订阅方
        3-2.解析并输出数据
    4.调用spin函数,并传入节点对象
    5.资源释放
"""

import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

# 3.自定义节点类
class ListenerStu(Node):
    def __init__(self):
        super().__init__("listener_stu_node_py")
        self.get_logger().info("订阅方创建了...")

        # 3-1.创建订阅方
        self.subscription = self.create_subscription(
            Student, "chatter_stu", self.do_cb, 10
        )

    def do_cb(self, stu):
        """
        回调函数 \n
        :param stu: 接收到的学生消息
        """

        # 3-2.解析并输出数据
        self.get_logger().info(f"name={stu.name} age={stu.age} height={stu.height}")


def main():
    rclpy.init()
    rclpy.spin(ListenerStu())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
