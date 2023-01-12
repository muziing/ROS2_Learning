/*
需求：处理客户端发送的请求数据（目标点），控制乌龟向目标点运动，且要连续反馈剩余距离
流程：
    1.创建原生乌龟位姿订阅方，获取当前乌龟坐标
    2.创建速度指令发布方，控制乌龟运动
    3.创建一个动作服务端
    4.解析动作客户端提交的数据是否合法
    5.处理客户端的取消请求操作
    6.实现主逻辑（耗时操作，启动子线程）
    7.子线程中发布速度指令、产生连续反馈，并响应最终结果
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "base_interfaces_demo/action/nav.hpp"

using base_interfaces_demo::action::Nav;
using std::placeholders::_1;
using std::placeholders::_2;

class Exer04ActionServer : public rclcpp::Node
{
public:
    Exer04ActionServer() : Node("exer04_action_server_node_cpp"), x(0.0), y(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "动作服务端已创建");

        // 1.创建一个订阅方（原生乌龟位姿 /turtle/pose）
        sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&Exer04ActionServer::pose_cb, this, _1));

        // 2.创建速度指令发布方，控制乌龟运动
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // 3.创建一个动作服务端
        action_server_ = rclcpp_action::create_server<Nav>(
            this,
            "nav",
            std::bind(&Exer04ActionServer::handle_goal, this, _1, _2),
            std::bind(&Exer04ActionServer::handle_cancel, this, _1),
            std::bind(&Exer04ActionServer::handle_accepted, this, _1));
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_action::Server<Nav>::SharedPtr action_server_;
    float x, y;

    void pose_cb(const turtlesim::msg::Pose::SharedPtr pose)
    {
        x = pose->x;
        y = pose->y;
    }

    // 4.处理请求目标
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Nav::Goal> goal)
    {
        (void)uuid;
        // 提取目标中的 x y 坐标，分别判断是否超出了 [0, 11.08] 的取值范围，如果超出则认为非法
        if (goal->goal_x < 0 || goal->goal_x > 11.08 || goal->goal_y < 0 || goal->goal_y > 11.08)
        {
            RCLCPP_WARN(this->get_logger(), "目标点超出正常取值范围！");
            return rclcpp_action::GoalResponse::REJECT;
        }
        else
        {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
    }

    // 5.处理取消请求
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "任务被取消了！");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 6.主逻辑
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle)
    {
        // 子线程处理主要逻辑
        RCLCPP_INFO(this->get_logger(), "主逻辑开始执行");

        auto result = std::make_shared<Nav::Result>(); // 最终结果
        auto feedback = std::make_shared<Nav::Feedback>();
        geometry_msgs::msg::Twist twist;

        // 6-1.生成连续反馈
        rclcpp::Rate rate(1.0);
        while (true)
        {
            // 如果要取消任务，则需要特殊处理
            if (goal_handle->is_canceling())
            {
                // 设置取消后的最终结果
                goal_handle->canceled(result);
                return;
            }

            // 解析目标点坐标与原生乌龟实时坐标
            float goal_x = goal_handle->get_goal()->goal_x;
            float goal_y = goal_handle->get_goal()->goal_y;

            // 6-2.发布乌龟运动指令
            // 计算剩余距离并发布
            float distance_x = goal_x - x;
            float distance_y = goal_y - y;
            float distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
            feedback->distance = distance;
            goal_handle->publish_feedback(feedback);

            // 根据剩余距离计算速度指令并发布
            float scale = 0.5;
            float linear_x = scale * distance_x;
            float linear_y = scale * distance_y;
            twist.linear.x = linear_x;
            twist.linear.y = linear_y;
            cmd_pub_->publish(twist);

            // 循环结束条件
            if (distance <= 0.05)
            {
                RCLCPP_INFO(this->get_logger(), "已经移动至目标点");
                break;
            }

            rate.sleep();
        }

        // 6-3.生成最终响应
        if (rclcpp::ok())
        {
            result->turtle_x = x;
            result->turtle_y = y;
            goal_handle->succeed(result);
        }
    }

    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle)
    {
        std::thread(std::bind(&Exer04ActionServer::execute, this, goal_handle)).detach();
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Exer04ActionServer>());
    rclcpp::shutdown();

    return 0;
}
