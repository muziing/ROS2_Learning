/*
需求：创建参数服务端，并操作参数（增删改查）
流程：
    自定义节点类
    1.增
    2.查
    3.改
    4.删
*/

#include "rclcpp/rclcpp.hpp"

class ParamServer : public rclcpp::Node
{
public:
    // 需要通过NodeOptions声明允许删除参数
    ParamServer() : Node("param_server_node",
                         rclcpp::NodeOptions().allow_undeclared_parameters(true))
    {
        RCLCPP_INFO(this->get_logger(), "参数服务端已创建");
    }

    // 1.增
    void declare_param()
    {
        RCLCPP_INFO(this->get_logger(), "-------------增------------");
        this->declare_parameter("car_name", "tiger");
        this->declare_parameter("width", 1.55);
        this->declare_parameter("wheels", 5);

        // 也可以通过set_parameter实现添加参数，但前提为rclcpp::NodeOptions().allow_undeclared_parameters(true)
        this->set_parameter(rclcpp::Parameter("color", "red"));
    }

    // 2.查
    void get_param()
    {
        RCLCPP_INFO(this->get_logger(), "-------------查------------");

        // 获取指定参数
        auto car_name = this->get_parameter("car_name");
        RCLCPP_INFO(this->get_logger(), "key = %s, value = %s",
                    car_name.get_name().c_str(), car_name.as_string().c_str());

        // 获取一些参数
        auto params = this->get_parameters({"car_name", "width", "wheels"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(), "%s = %s", param.get_name().c_str(),
                        param.value_to_string().c_str());
        }

        // 判断是否包含
        RCLCPP_INFO(this->get_logger(), "是否包含car_name: %d", this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(), "是否包含height: %d", this->has_parameter("height"));
    }

    // 3.改
    void update_param()
    {
        RCLCPP_INFO(this->get_logger(), "-------------改------------");
        this->set_parameter(rclcpp::Parameter("width", 1.75));
        RCLCPP_INFO(this->get_logger(), "width = %.2f", this->get_parameter("width").as_double());
    }

    // 4.删
    void del_param()
    {
        RCLCPP_INFO(this->get_logger(), "-------------删------------");
        // this->undeclare_parameter("car_name");  // 由declare_parameter()声明的参数不能被删除
        RCLCPP_INFO(this->get_logger(), "是否包含color: %d", this->has_parameter("color"));
        this->undeclare_parameter("color"); // 可以删除未声明而设置的参数
        RCLCPP_INFO(this->get_logger(), "是否包含color: %d", this->has_parameter("color"));
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    auto test_node = std::make_shared<ParamServer>();
    test_node->declare_param();
    test_node->get_param();
    test_node->update_param();
    test_node->del_param();
    rclcpp::spin(test_node);

    rclcpp::shutdown();

    return 0;
}
