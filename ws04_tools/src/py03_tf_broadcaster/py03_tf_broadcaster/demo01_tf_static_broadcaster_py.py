import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.logging import get_logger
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations


class TFStaticBroadcasterPy(Node):
    def __init__(self, argv):
        super().__init__("tf_static_broadcaster_py_node_py")
        # 1.创建广播对象
        self.broadcaster = StaticTransformBroadcaster(self)
        # 2.组织并发布数据
        self.pub_static_tf(argv)

    def pub_static_tf(self, argv):
        ts = TransformStamped()
        # 设置参数
        ts.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
        ts.header.frame_id = argv[7]  # 设置父坐标系
        ts.child_frame_id = argv[8]  # 设置子坐标系
        # 设置平移
        ts.transform.translation.x = float(argv[1])
        ts.transform.translation.y = float(argv[2])
        ts.transform.translation.z = float(argv[3])
        # 设置四元数
        # 将欧拉角转换成四元数
        qtn = tf_transformations.quaternion_from_euler(
            float(argv[4]), float(argv[5]), float(argv[6])
        )
        ts.transform.rotation.x = qtn[0]
        ts.transform.rotation.y = qtn[1]
        ts.transform.rotation.z = qtn[2]
        ts.transform.rotation.w = qtn[3]

        self.broadcaster.sendTransform(ts)


def main():
    if len(sys.argv) != 9:
        get_logger("rclpy").error("传入的参数不合法")
        return

    rclpy.init()
    rclpy.spin(TFStaticBroadcasterPy(sys.argv))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
