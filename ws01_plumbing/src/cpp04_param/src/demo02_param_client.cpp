/*
需求：创建参数客户端，查询或修改服务端参数
自定义节点类：
    1.创建参数客户端对象
    2.连接服务端
    3.参数查询
    4.修改参数
*/

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamClient : public rclcpp::Node
{
public:
    ParamClient() : Node("param_client_node")
    {
        RCLCPP_INFO(this->get_logger(), "参数客户端已创建");
        // 1.创建参数客户端对象
        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "param_server_node");
        // 参数1：当前对象所依赖的节点  参数2：参数服务端节点名称
        /*
        问题：服务通信通过「服务话题」关联，而为什么参数客户端是通过参数服务端的节点名称建立关联？
        答：
            1.参数服务端启动后，底层封装了多个服务通信的服务端
            2.每个服务端的话题都是采用「/服务节点名称/xxx」的方式命名，xxx为 "list_parameters" "set_parameters"等
            3.参数客户端创建后，也会封装多个服务通信的客户端
            4.客户端为与服务端建立关联，也要使用相同的话题，因此客户端创建时需要传入服务端节点名称
        */
    }

    // 2.连接服务端
    bool connect_server()
    {
        while (!param_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "服务连接中...");
        }
        return true;
    }
    // 3.参数查询
    void get_param()
    {
        RCLCPP_INFO(this->get_logger(), "-----参数查询操作------");

        // 获取某个参数
        std::string car_name = param_client_->get_parameter<std::string>("car_name");
        double car_width = param_client_->get_parameter<double>("width");
        RCLCPP_INFO(this->get_logger(), "car_name = %s", car_name.c_str());
        RCLCPP_INFO(this->get_logger(), "car_width = %.2f", car_width);

        // 获取多个参数
        auto params = param_client_->get_parameters({"car_name", "width", "wheels"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(), "%s = %s", param.get_name().c_str(),
                        param.value_to_string().c_str());
        }

        // 判断是否包含某个参数
        RCLCPP_INFO(this->get_logger(), "是否包含car_name: %d",
                    param_client_->has_parameter("car_name"));
    }

    // 4.修改参数
    void update_param()
    {
        RCLCPP_INFO(this->get_logger(), "-----参数修改操作------");
        param_client_->set_parameters<>({rclcpp::Parameter("car_name", "dragon"),
                                         rclcpp::Parameter("width", 3.0),
                                         rclcpp::Parameter("length", 5.0)});
        
        // 可以设置先前不存在的参数，但前提为服务端设置了rclcpp::NodeOptions().allow_undeclared_parameters(true)
        RCLCPP_INFO(this->get_logger(), "新设置的参数：%.2f", param_client_->get_parameter<double>("length"));
    }

private:
    rclcpp::SyncParametersClient::SharedPtr param_client_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    auto test_client_node = std::make_shared<ParamClient>();
    bool flag = test_client_node->connect_server();
    if (!flag)
    {
        return 0;
    }
    test_client_node->get_param();
    test_client_node->update_param();
    test_client_node->get_param();

    rclcpp::shutdown();
    return 0;
}
