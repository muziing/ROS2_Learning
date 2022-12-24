/*
需求：订阅发布方发布的消息，并在终端输出
流程：
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
        3-1.创建订阅方
        3-2.解析并输出数据
    4.调用spin函数，并传入节点对象指针
    5.资源释放
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 3.自定义节点类;
class Listener : public rclcpp::Node
{
public:
    Listener() : Node("listener_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "订阅方创建!");

        // 3-1.创建订阅方
        subcription_ = this->create_subscription<std_msgs::msg::String>("chartter", 10, std::bind(&Listener::do_cb, this, std::placeholders::_1));
        /*
        create_subscription
        模板:消息类型
        参数: 1.话题名称(和发布方保持一直) 2.QOS(队列长度) 3.回调函数
        返回值:订阅对象指针
        */
    }

private:
    void do_cb(const std_msgs::msg::String &msg)
    {
        // 3-2.解析并输出
        RCLCPP_INFO(this->get_logger(), "订阅到的消息为:%s", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subcription_;
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端
    rclcpp::init(argc, argv);

    // 4.调用spin函数,并传入节点对象指针
    rclcpp::spin(std::make_shared<Listener>());

    // 5.资源释放
    rclcpp::shutdown();
    return 0;
}