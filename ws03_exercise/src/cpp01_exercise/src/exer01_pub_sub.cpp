/*
需求：订阅乌龟1的位姿信息，解析出线速度与角速度；生成并发布控制乌龟2运动的速度指令
明确：
    订阅话题：/turtle1/pose
    订阅消息：turtlesim/msg/Pose
            x: 0.0
            y: 0.0
            theta: 0.0
            linear_velocity: 0.0
            angular_velocity: 0.0

    发布话题：/t2/turtle1/cmd_vel
    发布消息：geometry_msgs/msg/Twist
            linear:
              x: 0.0
              y: 0.0
              z: 0.0
            angular:
              x: 0.0
              y: 0.0
              z: 0.0
流程：
    1.创建发布方
    2.创建订阅方（订阅乌龟1位姿，解析速度）
    3.订阅方的回调函数处理速度，生成并发布控制乌龟2的速度指令
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class Exer01PubSub : public rclcpp::Node
{
public:
    Exer01PubSub() : Node("exer01_pub_sub_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "案例1对象创建!");

        // 1.创建发布方
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/t2/turtle1/cmd_vel", 10);

        // 2.创建订阅方（订阅乌龟1位姿，解析速度）
        sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&Exer01PubSub::pose_cb, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;

    void pose_cb(const turtlesim::msg::Pose &pose)
    {
        // 3.订阅方的回调函数: 处理速度，生成并发布控制乌龟2的速度指令
        // 3-1.创建速度指令
        geometry_msgs::msg::Twist twist;
        twist.linear.x = pose.linear_velocity;
        twist.angular.z = -pose.angular_velocity;

        // 3-2.发布
        pub_->publish(twist);
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Exer01PubSub>());
    rclcpp::shutdown();

    return 0;
}
