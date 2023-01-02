#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("time_calculate_node_cpp")
    {
        demo_opt();
    }

private:
    // 演示运算符的使用
    void demo_opt()
    {
        rclcpp::Time t1(10, 0);
        rclcpp::Time t2(30, 0);

        rclcpp::Duration du1(8, 0);
        rclcpp::Duration du2(17, 0);

        // 运算
        // 比较运算
        RCLCPP_INFO(this->get_logger(), "t1 >= t2 ? %d", t1 >= t2);     // 0
        RCLCPP_INFO(this->get_logger(), "t1 <= t2 ? %d", t1 <= t2);     // 1
        RCLCPP_INFO(this->get_logger(), "du1 >= du2 ? %d", du1 >= du2); // 0

        // 数学运算
        rclcpp::Duration du3 = t2 - t1;
        rclcpp::Time t3 = t1 + du1;
        rclcpp::Time t4 = t2 - du2;
        RCLCPP_INFO(this->get_logger(), "du3 = %.2f", du3.seconds()); // 20
        RCLCPP_INFO(this->get_logger(), "t3 = %.2f", t3.seconds());   // 18
        RCLCPP_INFO(this->get_logger(), "t4 = %.2f", t4.seconds());   // 13

        rclcpp::Duration du4 = du1 * 3;
        rclcpp::Duration du5 = du1 + du2;
        rclcpp::Duration du6 = du1 - du2;
        RCLCPP_INFO(this->get_logger(), "du4 = %.2f", du4.seconds()); // 24
        RCLCPP_INFO(this->get_logger(), "du5 = %.2f", du5.seconds()); // 25
        RCLCPP_INFO(this->get_logger(), "du6 = %.2f", du6.seconds()); // -9
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::make_shared<MyNode>();
    rclcpp::shutdown();

    return 0;
}
