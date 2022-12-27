/*
需求:编写动作客户端,可以发送一个整型数据到服务端,并处理服务端的连续反馈、最终响应结果
流程:
    前提:可以解析终端动态传入的参数
    3.自定义节点类
        3-1.创建动作客户端
        3-2.发送请求
        3-3.处理关于目标值的服务端响应(回调函数)
        3-4.处理连续反馈(回调函数)
        3-5.处理最终响应结果(回调函数)
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using base_interfaces_demo::action::Progress;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ProgressActionClient : public rclcpp::Node
{
public:
    ProgressActionClient() : Node("progress_action_client_node_cpp")
    {
        // 3-1.创建动作客户端
        client_ = rclcpp_action::create_client<Progress>(this, "get_sum");
        /*
        template<typename ActionT, typename NodeT>
        typename Client<ActionT>::SharedPtr
        create_client(
        NodeT node,
        const std::string & name,
        ...)
        */

        RCLCPP_INFO(this->get_logger(), "action客户端端已创建");
    }

    // 3-2.发送请求
    void send_goal(int num)
    {
        // 3-2-1.需要连接到服务端
        if (!client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "服务连接失败");
            return;
        }

        // 3-2-2.发送具体请求
        auto goal = Progress::Goal();
        goal.num = num;
        rclcpp_action::Client<Progress>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&ProgressActionClient::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&ProgressActionClient::feedback_callback, this, _1, _2);
        options.result_callback = std::bind(&ProgressActionClient::result_callback, this, _1);

        auto future = client_->async_send_goal(goal, options);
        /*
        std::shared_future<typename GoalHandle::SharedPtr>
        async_send_goal(const Goal & goal, const SendGoalOptions & options = SendGoalOptions())
        */
    }

    // 3-3.处理关于目标值的服务端响应
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle)
    {
        // std::function<void (typename ClientGoalHandle<ActionT>::SharedPtr)>;
        if (!goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "目标请求被服务端拒绝");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "目标处理中...");
        }
    }

    // 3-4.处理连续反馈
    void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle,
                           const std::shared_ptr<const Progress::Feedback> feedback)
    {
        /*
        std::function<void (
            typename ClientGoalHandle<ActionT>::SharedPtr,
            const std::shared_ptr<const Feedback>)>;
        */
        (void)goal_handle;
        int progress = (int)(feedback->progress * 100);
        RCLCPP_INFO(this->get_logger(), "当前进度:%d%%", progress);
    }

    // 3-5.处理最终响应
    void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult &result)
    {
        // std::function<void (const WrappedResult & result)>
        // 通过状态码判断最终结果状态
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "最终结果为:%d", result.result->sum);
        }
        else if (result.code == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_INFO(this->get_logger(), "被中断!");
        }
        else if (result.code == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_INFO(this->get_logger(), "被取消!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "出现未知异常!");
        }
    }

private:
    rclcpp_action::Client<Progress>::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
    if (argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交一个整型数据");
        return 1;
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<ProgressActionClient>();
    node->send_goal(atoi(argv[1]));
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
