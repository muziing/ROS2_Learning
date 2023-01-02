#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TimeNode : public rclcpp::Node
{
public:
    TimeNode() : Node("time_node_cpp")
    {
        this->demo_rate();
    }

private:
    // 演示 Rate 的使用
    void demo_rate()
    {
        // 1.创建 Rate 对象
        rclcpp::Rate rate1(500ms); // 间隔
        rclcpp::Rate rate2(1.0);   // 频率

        // 2.调用 Rate 的 sleep 函数
        while (rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), "----------------");
            // rate1.sleep();
            rate2.sleep();
        }
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimeNode>());
    rclcpp::shutdown();

    return 0;
}
