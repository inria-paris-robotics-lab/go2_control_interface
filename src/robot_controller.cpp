#include "go2_control_interface/robot_controller.hpp"

RobotController::RobotController(rclcpp::Node& node)
: scaling(1.0)
, node(node)
{
    this->lowcmd_publisher_ = this->node.create_publisher<unitree_go::msg::LowCmd>("lowcmd", 10);
}

// damping mode of sportsmode
// disable sports mode
// goto q_start
bool RobotController::init(const Eigen::VectorXd & q_start){
    // TODO
}

// Scale all the commands by a ratio
void RobotController::set_scaling(double scale) {
    this->scaling = scale;
}

// Forward to robot -> topic pub
// Sanity check
// Emergency if necessary
void RobotController::send_command(const Eigen::VectorXd & q, const Eigen::VectorXd & v, const Eigen::VectorXd & tau, const Eigen::VectorXd & kp, const Eigen::VectorXd & kd) {
    assert(q.size() == 12);
    assert(v.size() == 12);
    assert(tau.size() == 12);
    assert(kp.size() == 12);
    assert(kd.size() == 12);

    unitree_go::msg::LowCmd msg;
    msg.head[0] = 0xFE;
    msg.head[1] = 0xEF;

    // Unused
    msg.level_flag = 0;
    msg.frame_reserve = 0;
    msg.sn[0] = 0;
    msg.sn[1] = 0;
    msg.bandwidth = 0;
    msg.fan[0] = 0;
    msg.fan[1] = 0;
    msg.reserve = 0;
    for(int i=0;i<12;i++) {
        msg.led[i] = 0;
    }

    // battery
    msg.bms_cmd.off = 0;
    msg.bms_cmd.reserve[0] = 0;
    msg.bms_cmd.reserve[1] = 0;
    msg.bms_cmd.reserve[2] = 0;

    // Version
    msg.sn[0] = 0;
    msg.sn[1] = 0;

    // Gpio
    msg.gpio = 0;

    int i;
    for(i=0;i<12;i++) {
        msg.motor_cmd[i].mode = 0x01; //Set toque mode
        msg.motor_cmd[i].q = q[i];
        msg.motor_cmd[i].dq = v[i];
        msg.motor_cmd[i].tau = scaling * tau[i];
        msg.motor_cmd[i].kp = scaling * kp[i];
        msg.motor_cmd[i].kd = scaling * kd[i];
    }
    for(;i<20;i++) { // Unused motors
        msg.motor_cmd[i].mode = 0x00; //Set passive mode
        msg.motor_cmd[i].q = 0;
        msg.motor_cmd[i].dq = 0;
        msg.motor_cmd[i].tau = 0;
        msg.motor_cmd[i].kp = 0;
        msg.motor_cmd[i].kd = 0;
    }

    // Compute CRC here
    msg.crc = 0x123;

    this->lowcmd_publisher_->publish(msg);
}

void RobotController::switch_sportsmode(bool enable){
    // TODO
}

void RobotController::go_to_configuration(const Eigen::VectorXd & q) {
    // TODO
}

void RobotController::send_safe_damp()  {
    //TODO
}
