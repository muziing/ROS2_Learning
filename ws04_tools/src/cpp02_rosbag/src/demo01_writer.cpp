/*
需求：录制控制乌龟运动的速度指令
流程：
    1.创建录制对象
    2.设置磁盘文件
    3.写数据（创建速度订阅方，回调函数中执行写出操作）
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosbag2_cpp/writer.hpp"

class SimpleBagRecorder : public rclcpp::Node
{
public:
    SimpleBagRecorder() : Node("simple_bag_recorder_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "消息录制对象创建");

        // 1.创建录制对象
        writer_ = std::make_unique<rosbag2_cpp::Writer>();

        // 2.设置磁盘文件
        writer_->open("my_bag"); // 相对路径，工作空间的直接子集

        // 3.写数据（创建速度订阅方，回调函数中执行写出操作）
        // writer_->write()
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel", 10,
            std::bind(&SimpleBagRecorder::do_write_msg, this, std::placeholders::_1));
    }

private:
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

    void do_write_msg(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        RCLCPP_INFO(this->get_logger(), "数据写出...");
        writer_->write(msg, "turtle1/cmd_vel", "geometry_msgs/msg/Twist", this->now());
        // std::shared_ptr<rclcpp::SerializedMessage> message, // 被写出的消息
        // const std::string & topic_name,                     // 话题名称
        // const std::string & type_name,                      // 消息类型
        // const rclcpp::Time & time);                         // 时间戳
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleBagRecorder>());
    rclcpp::shutdown();

    return 0;
}
