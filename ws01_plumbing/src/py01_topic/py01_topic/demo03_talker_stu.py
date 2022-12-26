"""
需求：以固定频率发布学生信息
步骤:
    1.导包
    2.初始化 ROS2 客户端
    3.定义节点类
        3-1.创建消息发布方
        3-2.创建定时器
        3-3.组织消息并发布学生信息
    4.调用spin函数,并传入节点对象
    5.释放资源
"""

import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

# 3.定义节点类
class TalkerStu(Node):
    def __init__(self):
        super().__init__("talker_stu_node_py")
        self.get_logger().info("发布方创建了...")

        # 3-1.创建消息发布方
        self.publisher = self.create_publisher(Student, "chatter_stu", 10)

        # 3-2.创建定时器
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # 3-3.组织消息并发布
        stu = Student()
        stu.name = "李四"
        stu.age = 21
        stu.height = 1.7
        self.publisher.publish(stu)
        self.get_logger().info(f"发布的数据:{stu}")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TalkerStu())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
