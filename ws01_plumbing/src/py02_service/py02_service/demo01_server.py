"""
需求:创建服务端,解析客户端提交的数据并响应
流程:
    3.自定义节点
        3-1.创建服务类
        3-2.编写回调函数,处理请求并产生响应
    4.调用spin函数,并传入节点对象
"""

import rclpy
from rclpy.node import Node
from base_interfaces_demo.srv import AddInts


class AddIntsServer(Node):
    def __init__(self):
        super().__init__("add_ints_server_node_py")
        self.get_logger().info("服务端已创建")
        # 创建服务端
        self.create_service(AddInts, "add_ints", self.add)

    def add(self, request, response):
        response.sum = request.num1 + request.num2
        self.get_logger().info(f"{request.num1} + {request.num2} = {response.sum}")
        return response


def main():
    rclpy.init()
    rclpy.spin(AddIntsServer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
