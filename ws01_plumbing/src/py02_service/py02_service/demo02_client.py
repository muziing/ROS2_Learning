"""
需求:编写客户端,发送两个整型变量作为请求数据,并处理响应结果
流程:
    1.前提:main函数中判断提交的参数是否正确
    2.初始化 ROS2 客户端
    3.自定义节点类:
        3-1.创建客户端
        3-2.连接服务器(如果客户端无法连接到服务端,则不能发送请求)
        3-3.发送请求
    4.创建对象
        4-1.调用连接服务的函数,根据连接结果做进一步处理
        4-2.连接服务后,调用请求发送函数
        4-3.再处理响应结果
    5.资源释放
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from base_interfaces_demo.srv import AddInts


class AddIntsClient(Node):
    def __init__(self):
        super().__init__("add_ints_client_node_py")

        # 3-1.创建客户端
        self.client = self.create_client(AddInts, "add_ints")
        self.get_logger().info("客户端已创建")

        self.future = None

        # 3-2.连接服务器
        self.connect_server()

    def connect_server(self):
        while not self.client.wait_for_service(2.0):
            self.get_logger().info("服务连接中...")

    def send_request(self):
        request = AddInts.Request()
        request.num1 = int(sys.argv[1])
        request.num2 = int(sys.argv[2])
        # 3-3.发送请求
        self.future = self.client.call_async(request)


def main():

    # 校验操作
    if len(sys.argv) != 3:
        get_logger("rclpy").error("请提交两个整型数据!")
        return 1

    rclpy.init()

    client = AddIntsClient()  # 4-1.创建客户端对象与连接服务
    client.send_request()  # 4-2.发送请求

    # 4-3.处理响应
    rclpy.spin_until_future_complete(client, client.future)
    try:
        response = client.future.result()
        client.get_logger().info(f"响应结果:{response.sum}")
    except Exception as e:
        client.get_logger().error(f"服务响应失败,异常信息为:{e}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
