// ros2_control hardware interface for the rover's swerve drivetrain:
// 8x ODESC V4.2 (4 steer + 4 wheel) on one CAN bus, spoken to over SocketCAN
// using ODrive's CANSimple protocol (see odesc_protocol.hpp).
//
// Each ODESC runs its own motor control loop (FOC + position/velocity PID)
// and its own watchdog -- this plugin only ships targets down and encoder
// feedback up.

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rover_hardware/odesc_protocol.hpp"

namespace rover_hardware
{

class OdescSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OdescSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  struct Joint
  {
    // From the URDF <joint> <param>s.
    std::string name;
    uint8_t node_id = 0;
    double gear_ratio = 1.0;
    int direction = 1;
    int32_t input_mode = odesc::kInputModePassthrough;
    bool position_controlled = false;  // true = steer (position cmd), false = wheel (velocity cmd)

    // ros2_control reads/writes these through the exported interface pointers.
    double position = kNaN;  // rad
    double velocity = kNaN;  // rad/s
    double command = kNaN;   // rad (steer) or rad/s (wheel); NaN = nothing to send

    // Latest bus traffic from this node.
    bool have_estimate = false;
    bool have_heartbeat = false;
    odesc::Heartbeat heartbeat;
    Clock::time_point last_estimate;
  };

  bool send(const odesc::Frame & frame);
  void drain_rx();
  bool wait_until(const std::function<bool()> & done, std::chrono::milliseconds timeout);
  void idle_all();
  void close_socket();

  rclcpp::Logger logger_ = rclcpp::get_logger("OdescSystem");
  std::vector<Joint> joints_;
  std::array<int, odesc::kMaxNodeId + 1> joint_index_by_node_;  // -1 = not ours
  std::string can_interface_ = "can0";
  std::chrono::milliseconds timeout_{100};
  int fd_ = -1;
  bool active_ = false;
  Clock::time_point last_tx_warn_;
};

}  // namespace rover_hardware
