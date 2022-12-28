"""
需求:编写动作服务端,需要解析客户端提交的数字,遍历该数字并累加求和,将最终结果响应回
    客户端,且请求响应过程中需要生成连续反馈
流程:
    2.初始化ROS2客户端
    3.自定义节点类
        3-1.创建动作服务端对象
        3-2.处理提交的目标值(回调函数) --- 已有默认实现
        3-3.处理取消请求(回调函数) --- 已有默认实现
        3-4.生成连续反馈与最终响应(回调函数)
    4.调用spin函数
    5.资源释放
"""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, GoalResponse, ServerGoalHandle
from rclpy.node import Node

from base_interfaces_demo.action import Progress


class ProgressActionServer(Node):
    def __init__(self):
        super().__init__("progress_action_sever_node_py")

        self.get_logger().info("动作通信服务端已创建")

        # 3-1.创建动作服务端对象
        self.server = ActionServer(
            self,
            Progress,
            "get_sum",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    # 3-2.处理提交的目标值
    def goal_callback(self, goal_request):
        if goal_request.num > 1:
            return GoalResponse.ACCEPT
        else:
            self.get_logger().info("收到非法请求,已拒绝")
            return GoalResponse.REJECT

    # 3-3.处理取消请求(回调函数)
    def cancel_callback(self, cancel_request):
        self.get_logger().info("用户中止！")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # 3-4-1.生成连续反馈
        num = goal_handle.request.num
        sum = 0
        for i in range(1, num + 1):
            sum += i
            feedback = Progress.Feedback()
            feedback.progress = i / num
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"连续反馈中,进度{feedback.progress*100:.1f}%")

            # 处理中止
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Progress.Result()

            time.sleep(0.5)

        # 3-4-2.响应最终结果
        goal_handle.succeed()
        result = Progress.Result()
        result.sum = sum
        self.get_logger().info(f"最终计算结果为：{sum}")

        return result


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ProgressActionServer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
