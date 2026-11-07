#include "rover_hardware/odesc_system.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rover_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
// Startup waits. These run during activation, not inside the 100 Hz loop, so
// blocking briefly here is fine.
constexpr std::chrono::milliseconds kFirstEstimateTimeout{1000};
constexpr std::chrono::milliseconds kClosedLoopTimeout{2000};

bool has_interface(const std::vector<hardware_interface::InterfaceInfo> & list, const std::string & name)
{
  return std::any_of(list.begin(), list.end(), [&](const auto & i) {return i.name == name;});
}
}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

CallbackReturn OdescSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & hw = info_.hardware_parameters;
  if (hw.count("can_interface")) {
    can_interface_ = hw.at("can_interface");
  }
  if (hw.count("timeout_ms")) {
    timeout_ = std::chrono::milliseconds(std::stoi(hw.at("timeout_ms")));
  }

  joint_index_by_node_.fill(-1);
  joints_.reserve(info_.joints.size());  // never reallocated after this: interfaces point into it

  for (const auto & ji : info_.joints) {
    Joint j;
    j.name = ji.name;

    if (ji.command_interfaces.size() != 1) {
      RCLCPP_FATAL(logger_, "Joint '%s' needs exactly one command interface", ji.name.c_str());
      return CallbackReturn::ERROR;
    }
    const auto & cmd = ji.command_interfaces[0].name;
    if (cmd == hardware_interface::HW_IF_POSITION) {
      j.position_controlled = true;
    } else if (cmd != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        logger_, "Joint '%s': command interface '%s' not supported (use position or velocity)",
        ji.name.c_str(), cmd.c_str());
      return CallbackReturn::ERROR;
    }
    if (!has_interface(ji.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(ji.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(logger_, "Joint '%s' needs position and velocity state interfaces", ji.name.c_str());
      return CallbackReturn::ERROR;
    }

    const auto & p = ji.parameters;
    if (!p.count("node_id") || !p.count("gear_ratio")) {
      RCLCPP_FATAL(logger_, "Joint '%s' is missing <param> node_id or gear_ratio", ji.name.c_str());
      return CallbackReturn::ERROR;
    }
    const int node_id = std::stoi(p.at("node_id"));
    if (node_id < 0 || node_id > odesc::kMaxNodeId) {
      RCLCPP_FATAL(logger_, "Joint '%s': node_id %d out of range 0-63", ji.name.c_str(), node_id);
      return CallbackReturn::ERROR;
    }
    if (joint_index_by_node_[node_id] != -1) {
      RCLCPP_FATAL(
        logger_, "Joints '%s' and '%s' share node_id %d", ji.name.c_str(),
        joints_[joint_index_by_node_[node_id]].name.c_str(), node_id);
      return CallbackReturn::ERROR;
    }
    j.node_id = static_cast<uint8_t>(node_id);

    j.gear_ratio = std::stod(p.at("gear_ratio"));
    if (!(j.gear_ratio > 0.0)) {
      RCLCPP_FATAL(logger_, "Joint '%s': gear_ratio must be > 0", ji.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (p.count("direction")) {
      j.direction = std::stoi(p.at("direction"));
      if (j.direction != 1 && j.direction != -1) {
        RCLCPP_FATAL(logger_, "Joint '%s': direction must be 1 or -1", ji.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    if (p.count("input_mode")) {
      j.input_mode = std::stoi(p.at("input_mode"));
    }

    joint_index_by_node_[node_id] = static_cast<int>(joints_.size());
    joints_.push_back(j);
  }

  RCLCPP_INFO(
    logger_, "Configured %zu ODESC joints on %s (timeout %ld ms)", joints_.size(),
    can_interface_.c_str(), static_cast<long>(timeout_.count()));
  return CallbackReturn::SUCCESS;
}

CallbackReturn OdescSystem::on_configure(const rclcpp_lifecycle::State &)
{
  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger_, "socket(PF_CAN) failed: %s", std::strerror(errno));
    return CallbackReturn::ERROR;
  }

  const unsigned int ifindex = if_nametoindex(can_interface_.c_str());
  if (ifindex == 0) {
    RCLCPP_ERROR(
      logger_, "CAN interface '%s' not found -- is the adapter plugged in and the link up? "
      "(sudo ip link set %s up type can bitrate 1000000)", can_interface_.c_str(),
      can_interface_.c_str());
    close_socket();
    return CallbackReturn::ERROR;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = static_cast<int>(ifindex);
  if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(logger_, "bind(%s) failed: %s", can_interface_.c_str(), std::strerror(errno));
    close_socket();
    return CallbackReturn::ERROR;
  }

  // Non-blocking: read()/write() run inside the controller manager's real-time
  // loop and must never stall waiting on the bus.
  fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);

  RCLCPP_INFO(logger_, "Opened %s", can_interface_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn OdescSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  close_socket();
  return CallbackReturn::SUCCESS;
}

CallbackReturn OdescSystem::on_activate(const rclcpp_lifecycle::State &)
{
  drain_rx();
  for (auto & j : joints_) {
    j.have_estimate = false;
    j.have_heartbeat = false;
    send(odesc::clear_errors(j.node_id));
  }

  // 1. Hear from every ODESC before arming anything, so we know where each
  //    steer motor actually is.
  const bool all_heard = wait_until(
    [this] {
      return std::all_of(joints_.begin(), joints_.end(), [](const Joint & j) {return j.have_estimate;});
    }, kFirstEstimateTimeout);
  if (!all_heard) {
    for (const auto & j : joints_) {
      if (!j.have_estimate) {
        RCLCPP_ERROR(
          logger_, "No encoder estimates from node %u (%s) -- check wiring, node_id, "
          "bitrate, and that encoder_rate_ms is non-zero", j.node_id, j.name.c_str());
      }
    }
    return CallbackReturn::ERROR;
  }

  // 2. Start from "stay where you are": steer targets = current angle, wheel
  //    targets = 0. Without this a steer motor would snap to 0 rad on arming.
  for (auto & j : joints_) {
    j.command = j.position_controlled ? j.position : 0.0;
    send(
      odesc::set_controller_mode(
        j.node_id, j.position_controlled ? odesc::kControlModePosition : odesc::kControlModeVelocity,
        j.input_mode));
    if (j.position_controlled) {
      send(odesc::set_input_pos(j.node_id, odesc::joint_to_motor(j.command, j.gear_ratio, j.direction)));
    } else {
      send(odesc::set_input_vel(j.node_id, 0.0f));
    }
  }

  // 3. Arm. Requires each motor to already be calibrated (pre_calibrated
  //    saved on the ODESC via odrivetool).
  for (auto & j : joints_) {
    j.have_heartbeat = false;
    send(odesc::set_axis_state(j.node_id, odesc::kAxisStateClosedLoop));
  }
  const bool all_closed_loop = wait_until(
    [this] {
      return std::all_of(
        joints_.begin(), joints_.end(), [](const Joint & j) {
          return j.have_heartbeat && j.heartbeat.axis_state == odesc::kAxisStateClosedLoop;
        });
    }, kClosedLoopTimeout);
  if (!all_closed_loop) {
    for (const auto & j : joints_) {
      if (!j.have_heartbeat || j.heartbeat.axis_state != odesc::kAxisStateClosedLoop) {
        RCLCPP_ERROR(
          logger_, "Node %u (%s) didn't enter closed loop (state %u, axis_error 0x%08X)", j.node_id,
          j.name.c_str(), j.heartbeat.axis_state, j.heartbeat.axis_error);
      }
    }
    idle_all();
    return CallbackReturn::ERROR;
  }

  active_ = true;
  RCLCPP_INFO(logger_, "All %zu ODESCs in closed-loop control", joints_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn OdescSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  idle_all();
  return CallbackReturn::SUCCESS;
}

CallbackReturn OdescSystem::on_error(const rclcpp_lifecycle::State &)
{
  active_ = false;
  idle_all();
  close_socket();
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interfaces
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface> OdescSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.position);
    out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.velocity);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> OdescSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(
      j.name, j.position_controlled ? hardware_interface::HW_IF_POSITION :
      hardware_interface::HW_IF_VELOCITY, &j.command);
  }
  return out;
}

// ---------------------------------------------------------------------------
// 100 Hz loop
// ---------------------------------------------------------------------------

return_type OdescSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  drain_rx();

  // ros2_control also calls read() while we're configured-but-inactive.
  // Report whatever feedback arrives, but only treat silence or a dropped
  // axis as fatal once the motors are actually armed.
  if (!active_) {
    return return_type::OK;
  }

  const auto now = Clock::now();
  for (const auto & j : joints_) {
    if (!j.have_estimate || now - j.last_estimate > timeout_) {
      RCLCPP_ERROR(
        logger_, "Lost feedback from node %u (%s) -- no encoder estimates for > %ld ms", j.node_id,
        j.name.c_str(), static_cast<long>(timeout_.count()));
      return return_type::ERROR;
    }
    if (j.have_heartbeat &&
      (j.heartbeat.axis_error != 0 || j.heartbeat.axis_state != odesc::kAxisStateClosedLoop))
    {
      RCLCPP_ERROR(
        logger_, "Node %u (%s) dropped out of closed loop (state %u, axis_error 0x%08X)",
        j.node_id, j.name.c_str(), j.heartbeat.axis_state, j.heartbeat.axis_error);
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

return_type OdescSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return return_type::OK;
  }
  for (const auto & j : joints_) {
    if (std::isnan(j.command)) {
      continue;  // controller hasn't sent anything yet
    }
    const auto motor = static_cast<float>(odesc::joint_to_motor(j.command, j.gear_ratio, j.direction));
    send(j.position_controlled ? odesc::set_input_pos(j.node_id, motor) :
      odesc::set_input_vel(j.node_id, motor));
  }
  return return_type::OK;
}

// ---------------------------------------------------------------------------
// SocketCAN helpers
// ---------------------------------------------------------------------------

bool OdescSystem::send(const odesc::Frame & frame)
{
  if (fd_ < 0) {
    return false;
  }
  can_frame cf{};
  cf.can_id = frame.id;
  cf.can_dlc = frame.len;
  std::memcpy(cf.data, frame.data.data(), frame.len);

  if (::write(fd_, &cf, sizeof(cf)) == static_cast<ssize_t>(sizeof(cf))) {
    return true;
  }
  // ENOBUFS/EAGAIN = kernel TX queue full, usually because nothing on the bus
  // is ACKing (unplugged, wrong bitrate, missing termination). Don't fail the
  // loop over it -- read()'s feedback timeout is what declares the bus dead.
  // Just warn, at most once a second.
  const auto now = Clock::now();
  if (now - last_tx_warn_ > std::chrono::seconds(1)) {
    last_tx_warn_ = now;
    RCLCPP_WARN(logger_, "CAN send failed on %s: %s", can_interface_.c_str(), std::strerror(errno));
  }
  return false;
}

void OdescSystem::drain_rx()
{
  if (fd_ < 0) {
    return;
  }
  can_frame cf{};
  while (::read(fd_, &cf, sizeof(cf)) == static_cast<ssize_t>(sizeof(cf))) {
    if (cf.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) {
      continue;  // CANSimple only uses standard 11-bit data frames
    }
    const int idx = joint_index_by_node_[odesc::node_id_of(cf.can_id)];
    if (idx < 0) {
      continue;  // some other device on the bus (e.g. the STM32)
    }
    odesc::Frame f;
    f.id = cf.can_id;
    f.len = cf.can_dlc;
    std::memcpy(f.data.data(), cf.data, std::min<size_t>(cf.can_dlc, 8));

    Joint & j = joints_[idx];
    switch (odesc::cmd_id_of(cf.can_id)) {
      case odesc::kEncoderEstimates: {
          const auto est = odesc::parse_encoder_estimates(f);
          j.position = odesc::motor_to_joint(est.pos_turns, j.gear_ratio, j.direction);
          j.velocity = odesc::motor_to_joint(est.vel_turns_per_s, j.gear_ratio, j.direction);
          j.have_estimate = true;
          j.last_estimate = Clock::now();
          break;
        }
      case odesc::kHeartbeat:
        j.heartbeat = odesc::parse_heartbeat(f);
        j.have_heartbeat = true;
        break;
      default:
        break;  // our own commands echoed back by other listeners, etc.
    }
  }
}

bool OdescSystem::wait_until(const std::function<bool()> & done, std::chrono::milliseconds timeout)
{
  const auto deadline = Clock::now() + timeout;
  while (true) {
    drain_rx();
    if (done()) {
      return true;
    }
    if (Clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void OdescSystem::idle_all()
{
  for (const auto & j : joints_) {
    if (!j.position_controlled) {
      send(odesc::set_input_vel(j.node_id, 0.0f));
    }
    send(odesc::set_axis_state(j.node_id, odesc::kAxisStateIdle));
  }
}

void OdescSystem::close_socket()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace rover_hardware

PLUGINLIB_EXPORT_CLASS(rover_hardware::OdescSystem, hardware_interface::SystemInterface)
