#include "robot_interface.hpp"

#include <array>
#include <string>
#include <vector>

#include "motor_crc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

Go2RobotInterface::Go2RobotInterface(
    rclcpp::Node &node,
    const std::array<std::string_view, 12> source_joint_names)
    : node_(node),
      state_(std::make_shared<unitree_go::msg::LowState>()),
      cmd_(std::make_shared<unitree_go::msg::LowCmd>()),
      source_joint_names_(source_joint_names),
      idx_source_in_target_(
          map_indices(source_joint_names_, target_joint_names_)),
      idx_target_in_source_(
          map_indices(target_joint_names_, source_joint_names_)) {
  // Set up publishers
  watchdog_publisher_ =
      node.create_publisher<std_msgs::msg::Bool>("/watchdog/arm", 10);
  command_publisher_ =
      node.create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

  // Subscribe to the /lowstate and /watchdog/is_safe topics
  auto state_callback = [this](const unitree_go::msg::LowState::SharedPtr msg) {
    consume_state(msg);
  };
  state_subscription_ = node_.create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, state_callback);

  auto watchdog_callback = [this](const std_msgs::msg::Bool::SharedPtr msg) {
    consume(msg);
  };
  watchdog_subscription_ = node.create_subscription<std_msgs::msg::Bool>(
      "/watchdog/is_safe", 10, watchdog_callback);

  // Initialize the command
  initialize_command();
};

void Go2RobotInterface::initialize_command() {
  cmd_->head = {0xFE, 0xEF};
  cmd_->level_flag = 0;
  cmd_->frame_reserve = 0;
  cmd_->sn = {0, 0};
  cmd_->bandwidth = 0;
  cmd_->fan = {0, 0};
  cmd_->reserve = 0;
  cmd_->led = std::array<uint8_t, 12>{};

  /* Initialize the motor commands,
   * refer to
   * https://github.com/isaac-sim/IsaacLab/blob/874b7b628d501640399a241854c83262c5794a4b/source/extensions/omni.isaac.lab_assets/omni/isaac/lab_assets/unitree.py#L167
   * for the default values of kp and kd.
   */
  for (auto &m_cmd_ : cmd_->motor_cmd) {
    m_cmd_.mode = 1;
    m_cmd_.q = 0;
    m_cmd_.dq = 0;
    m_cmd_.kp = 0;
    m_cmd_.kd = 0;
    m_cmd_.tau = 0;
  }
};

void Go2RobotInterface::send_command(const std::array<float, 12> &q,
                                     const std::array<float, 12> &v,
                                     const std::array<float, 12> &tau,
                                     const std::array<float, 12> &kp,
                                     const std::array<float, 12> &kd) {
  // Check if the robot is ready
  if (!is_ready_) {
    throw std::runtime_error("Robot is not ready, cannot send command!");
  } else {
    send_command_aux(q, v, tau, kp, kd);
  }
};

void Go2RobotInterface::send_command_aux(const std::array<float, 12> &q,
                                         const std::array<float, 12> &v,
                                         const std::array<float, 12> &tau,
                                         const std::array<float, 12> &kp,
                                         const std::array<float, 12> &kd) {
  if (!is_safe_) {
    throw std::runtime_error("Robot is not safe, cannot send command!");
  } else {
    // Set the command
    for (size_t source_idx = 0; source_idx < 12; source_idx++) {
      size_t target_idx = idx_source_in_target_[source_idx];
      cmd_->motor_cmd[target_idx].q = q[source_idx];
      cmd_->motor_cmd[target_idx].dq = v[source_idx];
      cmd_->motor_cmd[target_idx].tau = tau[source_idx];
      cmd_->motor_cmd[target_idx].kp = kp[source_idx];
      cmd_->motor_cmd[target_idx].kd = kd[source_idx];
    }

    std::cout << "send_command_aux() ------------------" << std::endl;

    // CRC the command -- this is a checksum
    get_crc(*cmd_.get());

    // Publish the command
    command_publisher_->publish(*cmd_.get());
  }
};

void Go2RobotInterface::go_to_configuration_aux(
    const std::array<float, 12> &q_des, float duration_s) {
  // Check that duration is positive
  if (duration_s <= 0) {
    throw std::runtime_error("Duration must be strictly positive!");
  }
  // Zero-array for velocities and torques
  std::array<float, 12> zeroes{};
  std::fill(zeroes.begin(), zeroes.end(), 0.0);

  // Kp and Kd arrays
  std::array<float, 12> kp_array{};
  std::fill(kp_array.begin(), kp_array.end(), 150.0);

  std::array<float, 12> kd_array{};
  std::fill(kd_array.begin(), kd_array.end(), 1.0);

  // Get the current time
  auto start_time = node_.now();

  // Set up rate limiter
  auto rate = rclcpp::Rate(100);  // 100 Hz

  // Sleep for a second to allow the robot to stabilise
  rclcpp::sleep_for(std::chrono::seconds(1));

  auto start_q = state_q_;

  // Interpolate the joint positions
  while (rclcpp::ok()) {
    // Get the current time
    auto current_time = node_.now() - start_time;

    // Calculate the interpolation factor
    float alpha = current_time.seconds() / duration_s;

    // Check if the interpolation is complete
    if (alpha >= 1.5) {
      std::cout << "TIME EXCEEDED WE ARE BREAKING!!!!!! " << std::endl;
      break;
    }

    alpha = alpha > 1.0 ? 1.0 : alpha;

    // Interpolate the joint positions
    std::array<float, 12> q_step{};
    for (size_t source_idx = 0; source_idx < 12; source_idx++) {
      q_step[source_idx] = start_q[source_idx] +
                           alpha * (q_des[source_idx] - start_q[source_idx]);
    }

    std::cout << "q_state --- q_step: " << std::endl;
    for (size_t i = 0; i < 12; i++) {
      std::cout << i << ": " << state_q_[i] << " ---- " << q_step[i] << std::endl;
    }
    std::cout << std::endl;

    // Send the command
    send_command_aux(q_step, zeroes, zeroes, kp_array, kd_array);

    // Sleep for a while
    rate.sleep();
  }

  // Check if the interpolation is complete by looking at difference
  // between the current and desired joint positions
  for (size_t source_idx = 0; source_idx < 12; source_idx++) {
    float error = std::abs(q_des[source_idx] - state_q_[source_idx]);
    if (error > 0.1) {
      throw std::runtime_error("Interpolation failed, error is: " +
                               std::to_string(error));
    }
  }

  RCLCPP_INFO(node_.get_logger(), "Interpolation complete!");
  is_ready_ = true;
};

void Go2RobotInterface::go_to_configuration(const std::array<float, 12> &q_des,
                                            float duration_s) {
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  watchdog_publisher_->publish(msg);
  // Run aux in a separate thread
  std::thread t(&Go2RobotInterface::go_to_configuration_aux, this, q_des,
                duration_s);
  t.detach();
};

void Go2RobotInterface::consume_state(
    const unitree_go::msg::LowState::SharedPtr msg) {
  // Copy the /lowstate message
  state_ = msg;

  // Process the motor states
  for (size_t source_idx = 0; source_idx < 12; source_idx++) {
    size_t target_idx = idx_source_in_target_[source_idx];
    state_q_[source_idx] = state_->motor_state[target_idx].q;
    state_dq_[source_idx] = state_->motor_state[target_idx].dq;
    state_ddq_[source_idx] = state_->motor_state[target_idx].ddq;
    state_tau_[source_idx] = state_->motor_state[target_idx].tau_est;
  }

  // Process the IMU state
  for (size_t i = 0; i < 3; i++) {
    imu_lin_acc_[i] = state_->imu_state.accelerometer[i];
    imu_ang_vel_[i] = state_->imu_state.gyroscope[i];
  }
}

void Go2RobotInterface::consume(const std_msgs::msg::Bool::SharedPtr msg) {
  // Copy the /watchdog/is_safe message
  is_safe_ = msg->data;
}
