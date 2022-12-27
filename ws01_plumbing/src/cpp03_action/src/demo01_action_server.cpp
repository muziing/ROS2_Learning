/*
需求:编写动作服务端,需要解析客户端提交的数字,遍历该数字并累加求和,将最终结果响应回
    客户端,且请求响应过程中需要生成连续反馈
分析:
    1.创建动作服务端对象
    2.处理提交的目标值
    3.生成连续反馈
    4.响应最终结果
    5.处理取消请求
流程:
    2.初始化ROS2客户端
    3.自定义节点类
        3-1.创建动作服务端对象
        3-2.处理提交的目标值(回调函数)
        3-3.处理取消请求(回调函数)
        3-4.生成连续反馈与最终响应(回调函数)
    4.调用spin函数
    5.资源释放
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;

class ProgressActionServer : public rclcpp::Node
{
public:
    ProgressActionServer() : Node("progress_action_server_node_cpp")
    {
        server_ = rclcpp_action::create_server<Progress>(
            this,
            "get_sum",
            std::bind(&ProgressActionServer::handle_goal, this, _1, _2),
            std::bind(&ProgressActionServer::handle_cancel, this, _1),
            std::bind(&ProgressActionServer::handle_accepted, this, _1));
        /*
        rclcpp_action::Server<ActionT>::SharedPtr create_server<ActionT, NodeT>(
            NodeT node,
            const std::string &name.
            rclcpp_action::Server<ActionT>::GoalCallback handle_goal,
            rclcpp_action::Server<ActionT>::CancelCallback handle_cancel,
            rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted,
            ...
        )
        */
        RCLCPP_INFO(this->get_logger(), "action服务端已创建");
    }

private:
    rclcpp_action::Server<Progress>::SharedPtr server_;

    // 3-2.处理提交的目标值(回调函数)
    // std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Progress::Goal> goal)
    {
        (void)uuid; // 显式声明未使用该参数
        // 业务逻辑:判断提交的数字是否大于1,是则接收,否则拒绝
        if (goal->num <= 1)
        {
            RCLCPP_INFO(this->get_logger(), "提交的目标值必须大于1");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "提交的目标值合法");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 3-3.处理取消请求(回调函数)
    // std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "接收到任务取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 3-4.生成连续反馈与最终响应(回调函数)
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
        // 1.生成连续反馈返回给客户端
        // 获取目标值,遍历,遍历中进行累加,且每次循环中计算进度,并作为连续反馈发布
        int num = goal_handle->get_goal()->num;
        int sum = 0;
        auto feedback = std::make_shared<Progress::Feedback>();
        auto result = std::make_shared<Progress::Result>();
        rclcpp::Rate rate(2.0); // 设置休眠,频率2Hz
        for (int i = 1; i <= num; i++)
        {
            sum += i;
            double progress = i / (double)num; // 计算进度
            feedback->progress = progress;     // 回馈进度

            // void publish_feedback(std::shared_ptr<base_interfaces_demo::action::Progress_Feedback> feedback_msg)
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "连续反馈中,进度:%d%%", (int)(progress * 100));

            // 判断是否接收到了取消请求
            // void canceled(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
            if (goal_handle->is_canceling())
            {
                // 如果接收到了,则终止程序,return
                result->sum = sum;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "客户端中止了任务");
                return;
            }

            rate.sleep(); // 设置休眠,便于观察效果
        }

        // 2.生成最终响应结果
        if (rclcpp::ok())
        {

            result->sum = sum;

            // void succeed(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
            goal_handle->succeed(result);

            RCLCPP_INFO(this->get_logger(), "最终结果:%d", sum);
        }
    }

    // std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
        // 新建子线程.处理耗时的主逻辑实现
        std::thread(std::bind(&ProgressActionServer::execute, this, goal_handle)).detach();
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProgressActionServer>());
    rclcpp::shutdown();
    return 0;
}
