// CANSimple protocol helpers for ODESC V4.2 (ODrive v3.6 clone, firmware 0.5.6).
//
// Pure C++, no ROS and no sockets -- just "build this CAN frame" and "decode
// that one", plus joint<->motor unit conversion. Kept separate from the
// ros2_control plugin so it can be unit-tested without hardware.
//
// Protocol reference: https://docs.odriverobotics.com/v/0.5.6/can-protocol.html
//   - 11-bit CAN ID = (node_id << 5) | cmd_id
//   - All payload fields little-endian

#pragma once

#include <array>
#include <cstdint>
#include <cstring>

namespace rover_hardware::odesc
{

// Command IDs (lower 5 bits of the CAN ID).
constexpr uint8_t kHeartbeat = 0x001;          // ODESC -> us, cyclic (heartbeat_rate_ms)
constexpr uint8_t kSetAxisState = 0x007;
constexpr uint8_t kEncoderEstimates = 0x009;   // ODESC -> us, cyclic (encoder_rate_ms)
constexpr uint8_t kSetControllerMode = 0x00B;
constexpr uint8_t kSetInputPos = 0x00C;
constexpr uint8_t kSetInputVel = 0x00D;
constexpr uint8_t kClearErrors = 0x018;

// AxisState enum values from ODrive firmware 0.5.x.
constexpr uint32_t kAxisStateIdle = 1;
constexpr uint32_t kAxisStateClosedLoop = 8;

// ControlMode enum values.
constexpr int32_t kControlModeVelocity = 2;
constexpr int32_t kControlModePosition = 3;

// InputMode enum values. PASSTHROUGH applies each command immediately;
// TRAP_TRAJ / VEL_RAMP smooth it using limits configured on the ODESC.
constexpr int32_t kInputModePassthrough = 1;
constexpr int32_t kInputModeVelRamp = 2;
constexpr int32_t kInputModeTrapTraj = 5;

constexpr uint8_t kMaxNodeId = 0x3F;  // 6 bits left over above the 5-bit cmd_id

struct Frame
{
  uint32_t id = 0;
  uint8_t len = 0;
  std::array<uint8_t, 8> data{};
};

struct Heartbeat
{
  uint32_t axis_error = 0;
  uint8_t axis_state = 0;
};

struct EncoderEstimates
{
  float pos_turns = 0.0f;
  float vel_turns_per_s = 0.0f;
};

inline uint32_t make_id(uint8_t node_id, uint8_t cmd_id)
{
  return (static_cast<uint32_t>(node_id & kMaxNodeId) << 5) | (cmd_id & 0x1F);
}
inline uint8_t node_id_of(uint32_t can_id) {return (can_id >> 5) & kMaxNodeId;}
inline uint8_t cmd_id_of(uint32_t can_id) {return can_id & 0x1F;}

// --- little-endian field packing (explicit byte shifts, so this doesn't
// depend on the host's byte order) ---

inline void put_u32(Frame & f, size_t offset, uint32_t v)
{
  for (size_t i = 0; i < 4; ++i) {
    f.data[offset + i] = static_cast<uint8_t>(v >> (8 * i));
  }
}
inline void put_i32(Frame & f, size_t offset, int32_t v) {put_u32(f, offset, static_cast<uint32_t>(v));}
inline void put_f32(Frame & f, size_t offset, float v)
{
  uint32_t bits;
  std::memcpy(&bits, &v, sizeof(bits));
  put_u32(f, offset, bits);
}
inline uint32_t get_u32(const Frame & f, size_t offset)
{
  uint32_t v = 0;
  for (size_t i = 0; i < 4; ++i) {
    v |= static_cast<uint32_t>(f.data[offset + i]) << (8 * i);
  }
  return v;
}
inline float get_f32(const Frame & f, size_t offset)
{
  const uint32_t bits = get_u32(f, offset);
  float v;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

// --- frames we send ---

inline Frame set_axis_state(uint8_t node_id, uint32_t state)
{
  Frame f{make_id(node_id, kSetAxisState), 4, {}};
  put_u32(f, 0, state);
  return f;
}

inline Frame set_controller_mode(uint8_t node_id, int32_t control_mode, int32_t input_mode)
{
  Frame f{make_id(node_id, kSetControllerMode), 8, {}};
  put_i32(f, 0, control_mode);
  put_i32(f, 4, input_mode);
  return f;
}

// Velocity/torque feedforward (bytes 4-7) are left at 0.
inline Frame set_input_pos(uint8_t node_id, float pos_turns)
{
  Frame f{make_id(node_id, kSetInputPos), 8, {}};
  put_f32(f, 0, pos_turns);
  return f;
}

// Torque feedforward (bytes 4-7) is left at 0.
inline Frame set_input_vel(uint8_t node_id, float vel_turns_per_s)
{
  Frame f{make_id(node_id, kSetInputVel), 8, {}};
  put_f32(f, 0, vel_turns_per_s);
  return f;
}

inline Frame clear_errors(uint8_t node_id)
{
  return Frame{make_id(node_id, kClearErrors), 0, {}};
}

// --- frames we receive ---

inline Heartbeat parse_heartbeat(const Frame & f)
{
  return Heartbeat{get_u32(f, 0), f.data[4]};
}

inline EncoderEstimates parse_encoder_estimates(const Frame & f)
{
  return EncoderEstimates{get_f32(f, 0), get_f32(f, 4)};
}

// --- unit conversion ---
//
// ROS (REP-103) wants radians at the *joint*; the ODESC works in turns at the
// *motor*, on the other side of the gearbox. gear_ratio = motor turns per
// joint turn. direction = +1 or -1 for motors mounted flipped. Same formula
// for positions (rad <-> turns) and velocities (rad/s <-> turns/s).

constexpr double kTwoPi = 6.283185307179586;

inline double joint_to_motor(double joint_rad, double gear_ratio, int direction)
{
  return joint_rad * gear_ratio * direction / kTwoPi;
}

inline double motor_to_joint(double motor_turns, double gear_ratio, int direction)
{
  return motor_turns * direction / gear_ratio * kTwoPi;
}

}  // namespace rover_hardware::odesc
