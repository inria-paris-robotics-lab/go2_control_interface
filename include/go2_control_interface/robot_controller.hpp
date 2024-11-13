#pragma once
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"

class RobotController {
public:
    RobotController(rclcpp::Node & node);

    // damping mode of sportsmode
    // disable sports mode
    // goto q_start
    bool init(const Eigen::VectorXd & q_start);

    // Scale all the commands by a ratio
    void set_scaling(double scale);

    // Forward to robot -> topic pub
    // Sanity check
    // Emergency if necessary
    void send_command(const Eigen::VectorXd & q, const Eigen::VectorXd & v, const Eigen::VectorXd & tau, const Eigen::VectorXd & kp, const Eigen::VectorXd & kd);

protected:
    void switch_sportsmode(bool enable);
    void go_to_configuration(const Eigen::VectorXd & q);
    void send_safe_damp();

private:
    double scaling;
    rclcpp::Node& node;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_publisher_;
};