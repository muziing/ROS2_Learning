/*
需求：启动 turtlesim_node 节点，编写程序，发布乌龟(turtle1)相对于窗体(world)的位姿
流程：
    1.创建动态广播器
    2.创建乌龟位姿订阅方
    3.回调函数中获取乌龟位姿，成相对关系，并发布
*/

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TFDynaBroadcaster : public rclcpp::Node
{
public:
    TFDynaBroadcaster() : Node("tf_dyna_broadcaster_node_cpp")
    {
        // 1.创建动态广播器
        brocaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 2.创建乌龟位姿订阅方
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TFDynaBroadcaster::do_pose, this, std::placeholders::_1));
    }

private:
    void do_pose(const turtlesim::msg::Pose &pose)
    {
        // 3.回调函数中获取乌龟位姿，成相对关系，并发布
        // 3-1.组织消息
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "turtle1";
        ts.transform.translation.x = pose.x;
        ts.transform.translation.y = pose.y;
        ts.transform.translation.z = 0.0;
        tf2::Quaternion qtn; // 从欧拉角转换出四元数
        qtn.setRPY(0, 0, pose.theta);
        ts.transform.rotation.x = qtn.x();
        ts.transform.rotation.y = qtn.y();
        ts.transform.rotation.z = qtn.z();
        ts.transform.rotation.w = qtn.w();

        // 3-2.发布
        brocaster_->sendTransform(ts);
    }
    std::shared_ptr<tf2_ros::TransformBroadcaster> brocaster_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFDynaBroadcaster>());
    rclcpp::shutdown();

    return 0;
}
