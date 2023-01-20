"""
需求: 为turtlesim_node绑定事件: 
    - 节点启动时, 执行生成新的乌龟;
    - 节点关闭时, 执行日志输出
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    turtle = Node(package="turtlesim", executable="turtlesim_node")

    spawn = ExecuteProcess(
        cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': 8.0, 'y':3.0}\""],
        output="both",
        shell=True,
    )

    # 创建handler
    start_evt_hdl = OnProcessStart(target_action=turtle, on_start=spawn)
    exit_evt_hdl = OnProcessExit(
        target_action=turtle, on_exit=LogInfo(msg="turtlesim_node 已退出")
    )

    # 注册事件
    event_start = RegisterEventHandler(start_evt_hdl)
    event_exit = RegisterEventHandler(exit_evt_hdl)

    return LaunchDescription((turtle, event_start, event_exit))
