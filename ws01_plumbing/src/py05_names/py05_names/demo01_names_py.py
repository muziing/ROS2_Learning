import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyNode(Node):
    def __init__(self):
        # 通过编程方式设置命名空间
        super().__init__("zhenkeng_py", namespace="zuoxie_py")

        # 全局话题
        # self.pub = self.create_publisher(String, "/shi", 10)

        # 相对话题
        # self.pub = self.create_publisher(String, "kaihui", 10)

        # 私有话题
        self.pub = self.create_publisher(String, "~/vip", 10)


def main():
    rclpy.init()
    rclpy.spin(MyNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
