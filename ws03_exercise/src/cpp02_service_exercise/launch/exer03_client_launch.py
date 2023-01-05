from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    x = 6
    y = 9
    theta = 0.0
    name = "t2"

    # 1.在目标点生成一只新乌龟
    t_spawn = ExecuteProcess(
        cmd=[
            f"ros2 service call /spawn turtlesim/srv/Spawn \"{{'x': {x},'y': {y}, 'theta': {theta}}}\""
        ],
        output="both",
        shell=True,
    )

    # 2.调用客户端发送目标点坐标
    client_node = Node(
        package="cpp02_service_exercise",
        executable="exer03_client",
        arguments=[str(x), str(y), str(theta)],
    )

    return LaunchDescription([t_spawn, client_node])
