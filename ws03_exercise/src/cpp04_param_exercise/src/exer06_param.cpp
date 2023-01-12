/*
需求：修改turtlesim_node的背景颜色
流程：
    1.创建参数客户端
    2.连接参数服务端
    3.修改参数
*/

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Exer06Param : public rclcpp::Node
{
public:
    Exer06Param() : Node("exer06_param_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "参数客户端已创建");

        // 1.创建参数客户端
        client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/turtlesim");
    }

    // 2.连接参数服务端
    bool connect_server()
    {
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "客户端强制退出");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "服务连接中...");
        }

        return true;
    }

    // 3.更新参数
    void update_param()
    {
        // 背景色渐变
        // 3-1.获取参数
        int red = client_->get_parameter<int>("background_r");

        // 3-2.编写循环，修改参数（通过sleep控制修改频率）
        rclcpp::Rate rate(30.0);
        int count = red;
        while (rclcpp::ok())
        {
            count <= 255 ? red += 5 : red -= 5;
            count += 5;
            if (count > 511)
                count = 0;

            // 修改服务端参数
            client_->set_parameters({rclcpp::Parameter("background_r", red)});
            rate.sleep();
        }
    }

private:
    rclcpp::SyncParametersClient::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    auto client = std::make_shared<Exer06Param>();
    if (!client->connect_server())
    {
        rclcpp::shutdown();
        return 1;
    }
    client->update_param();

    rclcpp::shutdown();
    return 0;
}
