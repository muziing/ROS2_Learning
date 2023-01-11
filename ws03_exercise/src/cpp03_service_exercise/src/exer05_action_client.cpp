/*
需求：向动作服务端发送目标点数据，并处理响应结果
流程：
    1.解析launch文件传入的参数
    2.自定义节点类
        2-1.创建动作客户端
        2-2.连接服务端，发送请求
        2-3.处理目标值相关响应结果
        2-4.处理连续反馈
        2-5.处理最终响应
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "base_interfaces_demo/action/nav.hpp"

using base_interfaces_demo::action::Nav;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class Exer05ActionClient : public rclcpp::Node
{
public:
    Exer05ActionClient() : Node("exer05_action_client_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "动作客户端已创建");

        // 2-1.创建动作客户端
        client_ = rclcpp_action::create_client<Nav>(this, "nav");
    }

    void send_goal(float x, float y, float theta)
    {
        // 2-2.连接服务端
        if (!client_->wait_for_action_server(10s))
        {
            RCLCPP_WARN(this->get_logger(), "服务连接超时");
            return;
        }

        // 2-2.组织并发布数据
        Nav::Goal goal;
        goal.goal_x = x;
        goal.goal_y = y;
        goal.goal_theta = theta;
        rclcpp_action::Client<Nav>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&Exer05ActionClient::goal_response_cb, this, _1);
        options.feedback_callback = std::bind(&Exer05ActionClient::feedback_cb, this, _1, _2);
        options.result_callback = std::bind(&Exer05ActionClient::result_cb, this, _1);
        client_->async_send_goal(goal, options);
    }

private:
    rclcpp_action::Client<Nav>::SharedPtr client_;

    // 2-3.处理目标值相关响应结果
    void goal_response_cb(std::shared_ptr<rclcpp_action::ClientGoalHandle<Nav>> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_WARN(this->get_logger(), "目标请求被拒绝");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "目标值被接受");
        }
    }

    // 2-4.处理连续反馈
    void feedback_cb(std::shared_ptr<rclcpp_action::ClientGoalHandle<Nav>> goal_handle,
                     std::shared_ptr<const Nav::Feedback> feedback)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "剩余%.2f米", feedback->distance);
    }

    void result_cb(const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(),
                        "乌龟最终位姿信息：(%.2f, %.2f, %.2f)",
                        result.result->turtle_x,
                        result.result->turtle_y,
                        result.result->turtle_theta);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "最终响应失败");
        }
    }
};

int main(int argc, char const *argv[])
{
    if (argc != 5)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "请输入合法的目标点数据");
        return 1;
    }

    rclcpp::init(argc, argv);

    auto client = std::make_shared<Exer05ActionClient>();
    client->send_goal(atof(argv[1]), atof(argv[2]), atof(argv[3]));

    rclcpp::shutdown();
    return 0;
}
