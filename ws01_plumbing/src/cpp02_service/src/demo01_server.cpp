/*
需求:编写服务端,接收客户端发送的请求,提取其中两个整型数据,相加后将结果响应回客户端.
步骤:
    1.包含头文件;
    2.初始化 ROS2 节点
    3.定义节点类
        3-1.创建服务端
        3-2.回调函数:解析请求数据,并响应结果
    4.调用spin函数,并传入节点对象指针
    5.释放资源
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;

using std::placeholders::_1;
using std::placeholders::_2;

class AddIntsServer : public rclcpp::Node
{
public:
    AddIntsServer() : Node("add_ints_server_node_cpp")
    {
        // 3-1.创建服务端
        server = this->create_service<AddInts>("add_ints", std::bind(&AddIntsServer::add, this, _1, _2));
        /*
        create_service
        模板:服务接口类型
        参数:1.服务话题 2.回调函数
        返回值:服务对象指针
        */
        RCLCPP_INFO(this->get_logger(), "服务端节点已创建");
    }

private:
    rclcpp::Service<AddInts>::SharedPtr server;

    // 3-2.回调函数:解析请求数据,并响应结果
    void add(const AddInts::Request::SharedPtr req, const AddInts::Response::SharedPtr res)
    {
        res->sum = req->num1 + req->num2;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", req->num1, req->num2, res->sum);
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddIntsServer>());
    rclcpp::shutdown();
    return 0;
}
