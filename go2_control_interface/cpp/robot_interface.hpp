#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

/**
 * Maps the indices of elements in the source array to their corresponding
 * indices in the target array.
 *
 * @tparam T The type of elements in the arrays.
 * @tparam N The size of the arrays.
 * @param source The source array.
 * @param target The target array.
 * @return An array of indices representing the mapping from source to target.
 */
template <typename T, size_t N>
std::array<std::uint8_t, N> map_indices(std::array<T, N> source,
                                        std::array<T, N> target) {
  std::array<std::uint8_t, N> permutation{0};
  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      if (source[i] == target[j]) {
        permutation[i] = j;
        break;
      }
    }
  }
  return permutation;
}

class Go2RobotInterface {
 public:
  Go2RobotInterface(rclcpp::Node &node,
                    const std::array<std::string_view, 12> source_joint_names);

  /**
   * @brief Sends motor commands to the `/lowstate` topic.
   *
   * This method will throw if the robot is still being initialised, or
   * the watchdog `is_safe_` flag is false.
   *
   * All input arrays are in source (i.e. controller) order, and will be
   * reordered to match
   *
   * @param q Target positions,
   * @param v Target velocities,
   * @param tau Feed-forward torques (Nm),
   * @param Kp Proportional coefficients,
   * @param Kd derivative coefficients.
   */
  void send_command(const std::array<float, 12> &q,
                    const std::array<float, 12> &v,
                    const std::array<float, 12> &tau,
                    const std::array<float, 12> &kp,
                    const std::array<float, 12> &kd);

  void start_async(const std::vector<float> &q_start, bool goto_config = true);

  void register_callback(void (*callback)(float, std::vector<float>,
                                          std::vector<float>,
                                          std::vector<float>));

  /**
   * @brief Moves the robot to the initial pose.
   *
   * This function moves the robot to the initial pose by interpolating the
   * joint positions from the current state to the initial pose.
   *
   * @param q_des_ The desired joint positions.
   * @param duration_ms The duration of the interpolation in seconds.
   */
  void go_to_configuration(const std::array<float, 12> &q_des_,
                           float duration_s);

  void go_to_configuration_aux(const std::array<float, 12> &q_des_,
                               float duration_s);

  // Getters
  bool is_ready() const { return is_ready_; }
  bool is_safe() const { return is_safe_; }
  const std::array<float, 12> &get_q() const { return state_q_; }
  const std::array<float, 12> &get_dq() const { return state_dq_; }
  const std::array<float, 12> &get_ddq() const { return state_ddq_; }
  const std::array<float, 12> &get_tau() const { return state_tau_; }
  const std::array<float, 3> &get_lin_acc() const { return imu_lin_acc_; }
  const std::array<float, 3> &get_ang_vel() const { return imu_ang_vel_; }

 private:
  /**
   * @brief Initializes the command structure with default values.
   *
   * This function sets up the command structure `cmd_` with initial values:
   *
   * - `head = {0xEF, 0xEF}`,
   * - `level_flag = 0`,
   * - `frame_reserve = 0`,
   * - `sn = {0, 0}`,
   * - `bandwidth = 0`,
   * - `fan = {0, 0}`,
   * - `reserve = 0`,
   * - `led` is a 12-element array of zeros.
   */
  void initialize_command();

  /**
   * @brief Consumes the state message.
   *
   * This function consumes the state message and stores it in the `state_`
   * member variable.
   *
   * @param msg The LowState message to consume.
   */
  void consume_state(const unitree_go::msg::LowState::SharedPtr msg);

  /**
   * @brief Consumes the `/watchdog/is_safe` message.
   *
   * This function consumes the safety message and stores it in the `is_safe_`
   * member variable.
   *
   * @param msg The Bool message to consume.
   */
  void consume(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Sends motor commands to the `/lowstate` topic.
   *
   * All input arrays are in source (i.e. controller) order, and will be
   * reordered to match
   *
   * @param q Target positions,
   * @param v Target velocities,
   * @param tau Feed-forward torques (Nm),
   * @param Kp Proportional coefficients,
   * @param Kd derivative coefficients.
   */
  void send_command_aux(const std::array<float, 12> &q,
                        const std::array<float, 12> &v,
                        const std::array<float, 12> &tau,
                        const std::array<float, 12> &kp,
                        const std::array<float, 12> &kd);
  /// @brief The node handle
  rclcpp::Node &node_;

  // Safety flags
  volatile bool is_ready_ =
      false;  ///< True if the robot has been successfully initialised.
  volatile bool is_safe_ = true;  ///< True if it is safe to publish commands.

  // Robot state
  // Inertial state
  std::array<float, 3> imu_lin_acc_{};  ///< Linear acceleration
  std::array<float, 3> imu_ang_vel_{};  ///< Angular velocity

  // Joint states (12 joints)
  std::array<float, 12> state_q_{};    ///< Joint positions
  std::array<float, 12> state_dq_{};   ///< Joint velocities
  std::array<float, 12> state_ddq_{};  ///< Joint accelerations
  std::array<float, 12> state_tau_{};  ///< Joint torques (Nm)

  // Messages
  unitree_go::msg::LowState::SharedPtr
      state_;                               ///< Pointer to the LowState message
  unitree_go::msg::LowCmd::SharedPtr cmd_;  ///< Pointer to the LowCmd message

  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for publishing commands

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      watchdog_subscription_;  ///< Subscription to the "/watchdog/is_safe"
                               ///< topic
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      joy_subscription_;  ///< Subscription to the Joy topic
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr
      state_subscription_;  ///< Subscription to the state topic

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      watchdog_publisher_;  ///< Publisher for the "/watchdog/arm" topic
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr
      command_publisher_;  ///< Publisher for the command topic

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;  ///< Handle for the parameter callback

  // Joint names in the order that the controller is expecting them.
  const std::array<std::string_view, 12> source_joint_names_;

  // Joint names in the order that the robot is expecting them.
  static constexpr std::array<std::string_view, 12> target_joint_names_ = {
      "FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",  "FL_hip_joint",
      "FL_thigh_joint", "FL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint",
      "RR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint"};

  // Map indices between source and target joint orderings
  const std::array<uint8_t, 12> idx_source_in_target_;
  const std::array<uint8_t, 12> idx_target_in_source_;
};