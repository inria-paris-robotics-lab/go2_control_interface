#include "go2_control_interface_cpp/robot_interface.hpp"
#include "go2_control_interface_cpp/motor_crc.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include <cstddef>

Go2RobotInterface::Go2RobotInterface(rclcpp::Node & node, const std::array<std::string_view, 12> source_joint_order)
: node_(node)
, is_ready_(false)
, is_safe_(false)
, state_cb_(nullptr)
, state_t_(0)
, idx_source_in_target_(map_indices(source_joint_order, target_joint_order_))
, idx_target_in_source_(map_indices(target_joint_order_, source_joint_order))
{
  // Set up publishers
  watchdog_publisher_ = node.create_publisher<std_msgs::msg::Bool>("/watchdog/arm", 10);
  command_publisher_ = node.create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

  // Read node parameters
  scaling_glob_ = node.declare_parameter("scaling_glob", 1.0);
  scaling_gain_ = node.declare_parameter("scaling_gain", 1.0);
  scaling_ff_ = node.declare_parameter("scaling_ff", 1.0);

  filter_fq_ = node.declare_parameter("joint_filter_fq", -1.0); // By default no filter

  // Subscribe to the /lowstate and /watchdog/is_safe topics
  state_subscription_ = node_.create_subscription<unitree_go::msg::LowState>(
    "/lowstate", 10, std::bind(&Go2RobotInterface::consume_state, this, std::placeholders::_1));
  watchdog_subscription_ = node.create_subscription<std_msgs::msg::Bool>(
    "/watchdog/is_safe", 10, std::bind(&Go2RobotInterface::consume_watchdog, this, std::placeholders::_1));

  // Initialize the command
  initialize_command(cmd_);
};

void Go2RobotInterface::initialize_command(unitree_go::msg::LowCmd & cmd)
{
  cmd.head = {0xFE, 0xEF};
  cmd.level_flag = 0;
  cmd.frame_reserve = 0;
  cmd.sn = {0, 0};
  cmd.bandwidth = 0;
  cmd.fan = {0, 0};
  cmd.reserve = 0;
  cmd.led = std::array<uint8_t, 12>{};

  for (unitree_go::msg::MotorCmd & m_cmd_ : cmd.motor_cmd)
  {
    m_cmd_.mode = 1; // Torque control mode with with internal feed-forward PID
    m_cmd_.q = 0;
    m_cmd_.dq = 0;
    m_cmd_.kp = 0;
    m_cmd_.kd = 0;
    m_cmd_.tau = 0;
  }
};

void Go2RobotInterface::register_callback(void (*callback)(double, Vector12d, Vector12d, Vector12d))
{
  this->state_cb_ = callback;
}

void Go2RobotInterface::send_command(
  const Vector12d & q, const Vector12d & v, const Vector12d & tau, const Vector12d & kp, const Vector12d & kd)
{
  // Check if the robot is ready
  if (!is_ready_)
  {
    throw std::runtime_error("Robot is not ready, cannot send command!");
  }
  else
  {
    send_command_aux(q, v, tau, kp, kd);
  }
};

void Go2RobotInterface::send_command_aux(
  const Vector12d & q, const Vector12d & v, const Vector12d & tau, const Vector12d & kp, const Vector12d & kd)
{
  if (!is_safe_)
  {
    throw std::runtime_error("Robot is not safe, cannot send command!");
  }
  else
  {
    // Set the command
    for (size_t source_idx = 0; source_idx < 12; source_idx++)
    {
      size_t target_idx = idx_source_in_target_[source_idx];
      cmd_.motor_cmd[target_idx].q = q[source_idx];
      cmd_.motor_cmd[target_idx].dq = v[source_idx];
      cmd_.motor_cmd[target_idx].tau = (this->scaling_glob_ * this->scaling_ff_) * tau[source_idx];
      cmd_.motor_cmd[target_idx].kp = (this->scaling_glob_ * this->scaling_gain_) * kp[source_idx];
      cmd_.motor_cmd[target_idx].kd = (this->scaling_glob_ * this->scaling_gain_) * kd[source_idx];
    }

    // CRC the command -- this is a checksum
    get_crc(cmd_);

    // Publish the command
    command_publisher_->publish(cmd_);
  }
};

void Go2RobotInterface::start_async(const Vector12d & q_start, bool goto_config)
{
  // Run aux in a separate thread
  std::thread t(&Go2RobotInterface::start_aux, this, q_start, goto_config);
  t.detach();
}

void Go2RobotInterface::start_aux(const Vector12d & q_start, bool goto_config)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  watchdog_publisher_->publish(msg);

  RCLCPP_INFO(node_.get_logger(), "Waiting for watchdog to be armed...");
  while (!this->is_safe_ && rclcpp::ok())
  {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  if (goto_config)
  {
    RCLCPP_INFO(node_.get_logger(), "Going to start configuration...");
    go_to_configuration(q_start, 5.0);
    RCLCPP_INFO(node_.get_logger(), "Start configuration reached.");
  }
  else
  {
    RCLCPP_INFO(node_.get_logger(), "Skipping start configuration, set command to zero");
    send_command_aux(Vector12d::Zero(), Vector12d::Zero(), Vector12d::Zero(), Vector12d::Zero(), Vector12d::Zero());
  }
  this->is_ready_ = true;
}

void Go2RobotInterface::go_to_configuration(const Vector12d & q_des, double duration_s)
{
  // Check that duration is positive
  if (duration_s <= 0)
  {
    throw std::runtime_error("Duration must be strictly positive!");
  }

  // Sleep for a second to allow the robot to stabilize
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Set up rate limiter to control the robot
  rclcpp::Rate rate(500); // Hz

  // Get the current state
  Vector12d start_q = state_q_;
  rclcpp::Time start_time = node_.now();

  // Interpolate the joint positions
  while (this->is_safe_ && rclcpp::ok())
  {
    // Compute the interpolation factor
    const rclcpp::Duration delta_time = node_.now() - start_time;
    double alpha = delta_time.seconds() / duration_s;

    // Check if the interpolation is complete
    if (alpha >= 1.5)
    {
      break;
    }

    // Clip alpha to max 1.
    alpha = alpha > 1.0 ? 1.0 : alpha;

    // Interpolate the joint positions
    const Vector12d q_step = start_q + alpha * (q_des - start_q);

    // Send the command
    send_command_aux(q_step, Vector12d::Zero(), Vector12d::Zero(), 150. * Vector12d::Ones(), 1. * Vector12d::Ones());

    // Sleep for a while
    rate.sleep();
  }

  // Check if the interpolation is complete by looking at difference
  // between the current and desired joint positions
  for (size_t source_idx = 0; source_idx < 12; source_idx++)
  {
    double error = std::abs(q_des[source_idx] - state_q_[source_idx]);
    if (error > 0.1)
    {
      throw std::runtime_error("Interpolation failed, error is: " + std::to_string(error));
    }
  }
};

void Go2RobotInterface::consume_state(const unitree_go::msg::LowState::SharedPtr msg)
{
  const rclcpp::Time t = node_.now();

  // Copy and re-order motor state
  Vector12d q_meas, dq_meas, ddq_meas;
  for (size_t source_idx = 0; source_idx < 12; source_idx++)
  {
    const size_t target_idx = idx_source_in_target_[source_idx];
    q_meas[source_idx] = msg->motor_state[target_idx].q;
    dq_meas[source_idx] = msg->motor_state[target_idx].dq;
    ddq_meas[source_idx] = msg->motor_state[target_idx].ddq;
  }

  if (this->state_t_.nanoseconds() == 0 || this->filter_fq_ <= 0.)
  {
    // No filtering to do on first point
    state_q_ = q_meas;
    state_dq_ = dq_meas;
    state_ddq_ = ddq_meas;
  }
  else
  {
    const double b = 1. / (1 + 2 * 3.14 * (t - this->state_t_).seconds() * this->filter_fq_);
    state_q_ = (1 - b) * q_meas + b * state_q_;
    state_dq_ = (1 - b) * dq_meas + b * state_dq_;
    state_ddq_ = (1 - b) * ddq_meas + b * state_ddq_;
  }

  this->state_t_ = t;

  // Call user callback if set
  if (this->state_cb_)
  {
    state_cb_(state_t_.seconds(), state_q_, state_dq_, state_ddq_);
  }
}

void Go2RobotInterface::consume_watchdog(const std_msgs::msg::Bool::SharedPtr msg)
{
  this->is_safe_ = msg->data;
}
