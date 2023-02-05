/*
需求：编写静态坐标变换程序，执行时传入两个坐标系的相对位姿关系以及父子级坐标系id，
     程序运行发布静态坐标变换
     ros2 run 包 可执行程序名 x y z roll pitch yaw frame child_frame
流程：
    0.判断传入的参数是否合法
    1.创建广播对象
    2.组织并发布数据
*/

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TFStaticBroadcaster : public rclcpp::Node
{
public:
    TFStaticBroadcaster(char const *argv[]) : Node("tf_static_broadcaster_node")
    {
        // 1.创建广播对象
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // 2.组织并发布数据
        pub_static_tf(argv);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    void pub_static_tf(char const *argv[])
    {
        // 2-1.组织消息
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now(); // 时间戳
        transform.header.frame_id = argv[7];  // 父级坐标系
        transform.child_frame_id = argv[8];   // 子级坐标系
        // 设置偏移量
        transform.transform.translation.x = atof(argv[1]);
        transform.transform.translation.y = atof(argv[2]);
        transform.transform.translation.z = atof(argv[3]);
        // 设置四元数
        // 将欧拉角转换成四元数
        tf2::Quaternion qtn;
        qtn.setRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));
        transform.transform.rotation.x = qtn.x();
        transform.transform.rotation.y = qtn.y();
        transform.transform.rotation.z = qtn.z();
        transform.transform.rotation.w = qtn.w();

        // 2-2.发布消息
        broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char const *argv[])
{
    if (argc != 9)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "传入的参数不合法");
        return 1;
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFStaticBroadcaster>(argv));
    rclcpp::shutdown();
    return 0;
}