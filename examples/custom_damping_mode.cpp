#include "rclcpp/rclcpp.hpp"
#include "go2_control_interface/robot_controller.hpp"
#include <Eigen/Dense>

class CustomDampModeNode : public rclcpp::Node, RobotController
{
public:
    CustomDampModeNode()
    : rclcpp::Node("custom_damp_mode_node")
    , RobotController(static_cast<rclcpp::Node&>(*this))
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CustomDampModeNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        this->send_command(
            Eigen::VectorXd::Zero(12), //q
            Eigen::VectorXd::Zero(12), //v
            Eigen::VectorXd::Zero(12), //tau
            Eigen::VectorXd::Zero(12), //kp
            Eigen::VectorXd::Zero(12) //kd
        );
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::TimerBase::SharedPtr timer_;
    auto node = std::make_shared<CustomDampModeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}