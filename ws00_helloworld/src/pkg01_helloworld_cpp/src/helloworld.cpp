#include "rclcpp/rclcpp.hpp"

// 方式1（不推荐）
// int main(int argc, char ** argv)
// {
//   // 初始化 ROS2
//   rclcpp::init(argc, argv);

//   // 创建节点指针
//   auto node = rclcpp::Node::make_shared("helloworld_node_cpp");

//   // 输出文本
//   RCLCPP_INFO(node->get_logger(), "Hello World!");

//   // 释放资源
//   rclcpp::shutdown();

//   return 0;
// }

// 方式2（推荐）

// 自定义类继承 Node
class MyNode: public rclcpp::Node{
public:
    MyNode():Node("hello_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "Hello World!");
    }
};

int main(int argc, char const *argv[])
{
    // 初始化
    rclcpp::init(argc, argv);

    // 实例化自定义类
    auto node = std::make_shared<MyNode>();
    
    // 资源释放
    rclcpp::shutdown();

    return 0;
}
