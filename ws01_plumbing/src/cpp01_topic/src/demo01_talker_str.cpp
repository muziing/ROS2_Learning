/*
需求：以某个固定频率发送文本“Hello world!”。文本后缀编号，每发送一条消息，编号递增1。
步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类：
        3-1.创建发布方；
        3-2.创建定时器；
        3-3.组织消息并发布；
    4.调用spin函数，并传入节点对象指针；
    5.释放资源

*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker: public rclcpp::Node
{
public:
    Talker():Node("talker_node_cpp"), count_(0)
    {
        // 3-1.创建发布方
        publisher_ = this->create_publisher<std_msgs::msg::String>("chartter", 10);
        /*
        create_publisher
        模板：被发布的消息类型
        参数：1.话题名称 2.QOS（消息队列长度）
        返回值：发布对象指针
        */

        // 3-2.创建定时器
        timer_ = this->create_wall_timer(1s, std::bind(&Talker::on_timer, this));
        /*
        create_wall_timer
        参数：1.时间间隔 2.回调函数
        返回值：定时器对象指针
        */
    }

private:
    void on_timer()
    {
        // 3-3.组织并发布消息
        auto message = std_msgs::msg::String();
        message.data = "Hello world!" + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "发布的消息： '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    // 2.初始化 ROS2 客户端
    rclcpp::init(argc, argv);

    // 4.调用spin函数，并传入节点对象指针
    rclcpp::spin(std::make_shared<Talker>());

    // 5.释放资源
    rclcpp::shutdown();
    
    return 0;
}
