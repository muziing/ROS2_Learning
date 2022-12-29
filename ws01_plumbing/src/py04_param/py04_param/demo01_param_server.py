"""
需求：创建参数服务端，并操作参数（增删改查）
流程：
    自定义节点类
    1.增
    2.查
    3.改
    4.删
"""

import rclpy
from rclpy.node import Node


class ParamServer(Node):
    def __init__(self):
        # 如果允许删除参数，那么需要提前声明
        super().__init__("param_server_node_py", allow_undeclared_parameters=True)
        self.get_logger().info("参数服务端已创建")

    # 1.增
    def declare_param(self):
        self.get_logger().info("----------新增参数----------")
        self.declare_parameter("car_name", "tiger")
        self.declare_parameter("width", 1.55)
        self.declare_parameter("wheels", 5)

        # 设置未声明的参数，前提：allow_undeclared_parameters=True
        self.set_parameters([rclpy.Parameter("height", value=1.6)])

    # 2.查
    def get_param(self):
        self.get_logger().info("----------查询参数----------")

        # 获取指定参数
        car_name = self.get_parameter("car_name")
        self.get_logger().info(f"{car_name.name} = {car_name.value}")

        # 获取多个参数
        params = self.get_parameters(["car_name", "wheels", "width", "height"])
        for par in params:
            self.get_logger().info(f"{par.name} == {par.value}")

        # 判断是否包含某个参数
        self.get_logger().info(f"是否包含car_name: {self.has_parameter('car_name')}")
        self.get_logger().info(f"是否包含height: {self.has_parameter('height')}")

    # 3.改
    def update_param(self):
        self.get_logger().info("----------修改参数----------")
        self.set_parameters([rclpy.Parameter("car_name", value="dragon")])
        car_name = self.get_parameter("car_name")
        self.get_logger().info(f"修改后{car_name.name} = {car_name.value}")

    # 4.删
    def del_param(self):
        self.get_logger().info("----------删除参数----------")
        self.undeclare_parameter("car_name")  # 可以删除通过declare_param设置的参数值
        self.get_logger().info(f"是否包含car_name: {self.has_parameter('car_name')}")


def main():
    rclpy.init()

    test_node = ParamServer()
    test_node.declare_param()
    test_node.get_param()
    test_node.update_param()
    test_node.del_param()
    rclpy.spin(test_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
