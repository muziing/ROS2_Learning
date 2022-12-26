/*
需求：订阅学生信息，并在终端输出
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
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;

// 3.自定义节点类;
class ListenerStu : public rclcpp::Node
{
public:
    ListenerStu() : Node("listener_stu_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "订阅方创建!");

        // 3-1.创建订阅方
        subcription_ = this->create_subscription<Student>("chartter_stu", 10, std::bind(&ListenerStu::do_cb, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<Student>::SharedPtr subcription_;

    void do_cb(const Student &stu)
    {
        // 3-2.解析并输出
        RCLCPP_INFO(this->get_logger(), "订阅到的学生信息:name=%s, age=%d, height=%.2f", stu.name.c_str(), stu.age, stu.height);
    }
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端
    rclcpp::init(argc, argv);

    // 4.调用spin函数,并传入节点对象指针
    rclcpp::spin(std::make_shared<ListenerStu>());

    // 5.资源释放
    rclcpp::shutdown();
    return 0;
}