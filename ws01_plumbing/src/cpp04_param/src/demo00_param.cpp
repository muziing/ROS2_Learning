/*
自定义节点对象
    1.参数对象创建
    2.参数对象解析（获取键、值、将值转换成字符串...）

*/

#include "rclcpp/rclcpp.hpp"

class MyParam : public rclcpp::Node
{
public:
    MyParam() : Node("my_param")
    {
        RCLCPP_INFO(this->get_logger(), "参数API使用");

        // 1.参数对象创建
        rclcpp::Parameter p1("car_name", "tiger");
        rclcpp::Parameter p2("height", 1.68);
        rclcpp::Parameter p3("tyre", 4);

        // 2.参数对象解析（获取键、值、将值转换成字符串……
        // 2-1.获取值
        RCLCPP_INFO(this->get_logger(), "car_name = %s", p1.as_string().c_str());
        RCLCPP_INFO(this->get_logger(), "height = %.2f", p2.as_double());
        RCLCPP_INFO(this->get_logger(), "tyre = %ld", p3.as_int());
        RCLCPP_INFO(this->get_logger(), "value2string = %s", p2.value_to_string().c_str());
        // 2-2.获取键
        RCLCPP_INFO(this->get_logger(), "p1_name = %s", p1.get_name().c_str());
        RCLCPP_INFO(this->get_logger(), "p1_type = %s", p1.get_type_name().c_str());
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto test = std::make_shared<MyParam>();
    rclcpp::shutdown();

    return 0;
}
