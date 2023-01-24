/*
需求：读取bag文件数据，并将数据输出至终端
流程：
    1.创建一个回放对象
    2.设置被读取的文件
    3.读消息
    4.关闭文件，释放资源
*/

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SimpleBagPlayer : public rclcpp::Node
{
public:
    SimpleBagPlayer() : Node("simple_bag_play_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "消息回放对象创建");

        // 1.创建回放对象
        reader_ = std::make_unique<rosbag2_cpp::Reader>();

        // 2.设置被读取的文件
        reader_->open("my_bag");

        // 3.读消息
        while (reader_->has_next())
        {
            auto twist = reader_->read_next<geometry_msgs::msg::Twist>();
            RCLCPP_INFO(this->get_logger(), "线速度:%.2f, 角速度:%.2f", twist.linear.x, twist.angular.z);
        }

        // 4.关闭文件
        reader_->close();
    }

private:
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::make_shared<SimpleBagPlayer>();
    rclcpp::shutdown();

    return 0;
}
