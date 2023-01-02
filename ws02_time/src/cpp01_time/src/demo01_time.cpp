#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TimeNode : public rclcpp::Node
{
public:
    TimeNode() : Node("time_node_cpp")
    {
        // this->demo_rate();
        this->demo_time();
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

    // 演示 Time 使用
    void demo_time()
    {
        // 1.创建 Time 对象
        rclcpp::Time t1(500000000L);
        rclcpp::Time t2(2, 500000000L);
        // rclcpp::Time right_now = this->get_clock()->now();
        rclcpp::Time right_now = this->now();

        // 2.调用 Time 对象的函数
        RCLCPP_INFO(this->get_logger(), "sec = %.2f, nsec = %ld", t1.seconds(), t1.nanoseconds());
        RCLCPP_INFO(this->get_logger(), "sec = %.2f, nsec = %ld", t2.seconds(), t2.nanoseconds());
        RCLCPP_INFO(this->get_logger(), "sec = %.2f, nsec = %ld", right_now.seconds(), right_now.nanoseconds());
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimeNode>());
    rclcpp::shutdown();

    return 0;
}
