"""
需求：以某个固定频率发送文本"Hello world!"。文本后缀编号,每发送一条消息,编号递增1。
步骤:
    1.导包
    2.初始化 ROS2 客户端
    3.定义节点类
        3-1.创建消息发布方
        3-2.创建定时器
        3-3.组织消息并发布
    4.调用spin函数,并传入节点对象
    5.释放资源
"""

# 导包
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 3.自定义节点类
class Talker(Node):
    def __init__(self):
        super().__init__("talker_node_py")

        self.get_logger().info("发布方创建了...")

        # 设置计数器
        self.count = 0

        # 3-1.创建消息发布方
        self.publisher = self.create_publisher(String, "chatter", 10)

        # 3-2.创建定时器
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # 3-3.组织消息并发布
        message = String()
        message.data = f"Hello World!{str(self.count)}"
        self.publisher.publish(message)
        self.count += 1
        self.get_logger().info(f"发布的数据:{message.data}")


def main(args=None):

    # 2.初始化 ROS2 客户端
    rclpy.init(args=args)

    # 4.调用spin函数
    rclpy.spin(Talker())

    # 5.释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()
