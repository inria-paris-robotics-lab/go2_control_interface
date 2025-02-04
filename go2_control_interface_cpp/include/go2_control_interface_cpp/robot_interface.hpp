#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include <Eigen/Dense>
#include <chrono>

/**
 * Maps the indices of elements in the source array to their corresponding
 * indices in the target array.
 *
 * Used for reordering the robot configuration depending on the joint order conventions.
 *
 * @tparam N The size of the arrays.
 * @param source The source order.
 * @param target The target order.
 * @return An array of indices representing the mapping from source to target.
 */
template<size_t N>
std::array<std::uint8_t, N> map_indices(std::array<std::string_view, N> source, std::array<std::string_view, N> target)
{
  std::array<std::uint8_t, N> permutation{0};
  for (size_t i = 0; i < N; i++)
  {
    for (size_t j = 0; j < N; j++)
    {
      if (source[i] == target[j])
      {
        permutation[i] = j;
        break;
      }
    }
  }
  return permutation;
}

class Go2RobotInterface
{
public:
  typedef Eigen::Vector<double, 12> Vector12d;
  typedef std::function<void(double t, Vector12d q, Vector12d dq, Vector12d ddq)> fnStateCb;

  Go2RobotInterface(
    rclcpp::Node & node, const std::array<std::string_view, 12> source_joint_names = default_source_joint_order_);

  /**
   * @brief Sends commands to the robot motors.
   *
   * This method will throw if the robot has not been initialised completely,
   * or if the watchdog `is_safe_` flag is false.
   *
   * All input arrays are in source (i.e. controller) order, and will be
   * reordered to match robot hardware
   *
   * @param q Target positions (rad),
   * @param v Target velocities (rad/s),
   * @param tau Feed-forward torques (Nm),
   * @param Kp Proportional coefficients,
   * @param Kd Derivative coefficients.
   */
  void send_command(
    const Vector12d & q, const Vector12d & v, const Vector12d & tau, const Vector12d & kp, const Vector12d & kd);

  /**
   * @brief Initialize the robot and set it to a start configuration.
   * This function is non-blocking, (and run the start procedure in a different thread).
   *
   * @warning The user has to wait "manually" for the flag `is_ready()` to be set, before trying to send commands to the
   * robot.
   *
   * @note This function also handles the watchdog initialization and expect to be able to communicate with it.
   *
   * @param q_start Start configuration to put the robot in.
   * @param goto_config If set to false, the robot won't go into the start configuration, and will just be initialized
   * without.
   */
  void start_async(const Vector12d & q_start, bool goto_config = true);

  /**
   * @brief Register a callback to receive the robot state as soon as it is published.
   *
   * @param callback The callback function should have the following arguments
   * (double time, Vector12d q_meas, Vector12d a_meas, Vector12d tau_meas)
   */
  void register_callback(fnStateCb callback);

  /// @brief Is the robot initialized properly and ready to control.
  bool is_ready() const
  {
    return is_ready_;
  }

  /// @brief Is the watchdog saying that the robot is safe to control.
  bool is_safe() const
  {
    return is_safe_;
  }

  /// @brief Time (in s) when last state was measured.
  double get_t() const
  {
    return state_t_.seconds();
  }

  /// @brief Last configuration measured.
  const Vector12d & get_q() const
  {
    return state_q_;
  }

  /// @brief Last velocity measured.
  const Vector12d & get_dq() const
  {
    return state_dq_;
  }

  /// @brief Last acceleration measured.
  const Vector12d & get_ddq() const
  {
    return state_ddq_;
  }

private:
  /**
   * @brief Initializes the cmd packet with default values.
   *
   * - `head = {0xEF, 0xEF}`,
   * - `level_flag = 0`,
   * - `frame_reserve = 0`,
   * - `sn = {0, 0}`,
   * - `bandwidth = 0`,
   * - `fan = {0, 0}`,
   * - `reserve = 0`,
   * - `led` is a 12-element array of zeros.
   * - `motor_cmd[].mode = 1` for torque control (with ff-pid)
   */
  void initialize_command(unitree_go::msg::LowCmd & cmd);

  /**
   * @brief ROS callback to consumes the state message and store it in the `state_` member variable.
   */
  void consume_state(const unitree_go::msg::LowState::SharedPtr msg);

  /**
   * @brief ROS callback to keep track of the watchdog 'is_safe' topic.
   *
   * Stores the value in the `is_safe_` member variable.
   */
  void consume_watchdog(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Execute the `start_async` logic in a synchronous/blocking way.
   */
  void start_aux(const Vector12d & q_start, bool goto_config);

  /**
   * @brief Sends motor commands to the `/lowstate` topic.
   *
   * All input arrays are in source (i.e. controller) order, and will be
   * reordered to match.
   *
   * @warning This function doesn't check for the robot to be properly initialized.
   *
   * @param q Target positions,
   * @param v Target velocities,
   * @param tau Feed-forward torques (Nm),
   * @param Kp Proportional coefficients,
   * @param Kd derivative coefficients.
   */
  void send_command_aux(
    const Vector12d & q, const Vector12d & v, const Vector12d & tau, const Vector12d & kp, const Vector12d & kd);

  /**
   * @brief Moves the robot to a given configuration (with position control).
   *
   * This function moves the robot to configuration by interpolating the
   * joint positions from the current state to the goal state.
   * This function call is blocking until the configuration is reached.
   *
   * @param q_des_ The desired joint positions.
   * @param duration_ms The duration of the interpolation in seconds.
   */
  void go_to_configuration(const Vector12d & q_des_, double duration_s);

  /// @brief The node handle
  rclcpp::Node & node_;

  // Safety flags
  volatile bool is_ready_; ///< True if the robot has been successfully initialised.
  volatile bool is_safe_;  ///< True if it is safe to publish commands.

  // Robot state callback
  fnStateCb m_pfnStateCb_; ///> User callback to receive state
  // Robot state async
  rclcpp::Time state_t_; ///< Time when last state was received.
  Vector12d state_q_;    ///< Joint positions (rad)
  Vector12d state_dq_;   ///< Joint velocities (rad/s)
  Vector12d state_ddq_;  ///< Joint accelerations (rad/s²)
  bool state_received_ = false;

  // Scaling factors
  double scaling_glob_; ///< Scaling factor applied on kp, kd, tau when sending commands
  double scaling_gain_; ///< Scaling factor applied on kp, kd when sending commands
  double scaling_ff_;   ///< Scaling factor applied on tau when sending commands

  // Parameter to filter the velocity (as it is quite noisy on the real go2 robot)
  double filter_fq_; ///< Cut-off frequency of the filter

  // Messages
  unitree_go::msg::LowCmd cmd_; ///< Pointer to a pre-filled LowCmd message

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    watchdog_subscription_; ///< Subscription to the "/watchdog/is_safe" topic
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr
    state_subscription_; ///< Subscription to the robot state topic

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr watchdog_publisher_;    ///< Publisher for the "/watchdog/arm" topic
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr command_publisher_; ///< Publisher for the robot command topic

  // clang-format off
  /// @brief Joint names in the order that the user is expecting them by default (URDF alphabetical order).
  static constexpr std::array<std::string_view, 12> default_source_joint_order_ = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  };

  /// @brief Joint names in the order that the robot is expecting them.
  static constexpr std::array<std::string_view, 12> target_joint_order_ = {
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
  };
  // clang-format on

  /// @brief Map indices between source and target joint orderings
  const std::array<uint8_t, 12> idx_source_in_target_;
  const std::array<uint8_t, 12> idx_target_in_source_;
};
