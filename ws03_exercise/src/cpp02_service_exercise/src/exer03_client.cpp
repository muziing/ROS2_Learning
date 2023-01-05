/*
需求：客户端需要提交目标点坐标，并解析响应结果
流程：
    1.解析动态传入的数据，作为目标点坐标
    2.自定义节点类
        2-1.构造函数创建客户端
        2-2.客户端连接服务端
        2-3.发送请求数据
    3.调用节点对象指针的相关函数
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

using base_interfaces_demo::srv::Distance;
using namespace std::chrono_literals;

class Exer03Client : public rclcpp::Node
{
public:
    Exer03Client() : Node("exer03_client_node")
    {
        RCLCPP_INFO(this->get_logger(), "案例2客户端创建了");

        // 2-1.构造函数创建客户端
        client_ = this->create_client<Distance>("distance");
    }

    // 2-2.客户端连接服务端
    bool connect_server()
    {

        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "用户中止");
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "服务连接中...");
        }

        return true;
    }

    // 2-3.发送请求数据
    rclcpp::Client<Distance>::FutureAndRequestId send_goal(float x, float y, float theta)
    {
        auto request = std::make_shared<Distance::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;

        return client_->async_send_request(request);
    }

private:
    rclcpp::Client<Distance>::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
    if (argc != 5)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "请提交x坐标、y坐标、航向角theta");
        return 1;
    }

    // 解析提交的参数
    float goal_x = atof(argv[1]);
    float goal_y = atof(argv[2]);
    float goal_theta = atof(argv[3]);

    rclcpp::init(argc, argv);
    auto client = std::make_shared<Exer03Client>();
    bool flag = client->connect_server();
    if (!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接失败");
        return 1;
    }

    // 发送请求并处理响应
    auto future = client->send_goal(goal_x, goal_y, goal_theta);

    // 判断响应结果状态
    if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(client->get_logger(), "两只乌龟间距离为%.2f", future.get()->distance);
    }
    else
    {
        RCLCPP_ERROR(client->get_logger(), "服务响应失败");
    }

    rclcpp::shutdown();
    return 0;
}
