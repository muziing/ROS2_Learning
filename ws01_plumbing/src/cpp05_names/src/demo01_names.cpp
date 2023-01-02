#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("zhen_keng", "zuoxie") // 通过编程方式设置命名空间"zuoxie""
    {
        // 全局话题：与命名空间、节点名称无关系
        // pub_ = this->create_publisher<std_msgs::msg::String>("/shi", 10);

        // 相对话题
        // pub_ = this->create_publisher<std_msgs::msg::String>("kaihui", 10);

        // 私有话题
        pub_ = this->create_publisher<std_msgs::msg::String>("~/vip", 10);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();

    return 0;
}
