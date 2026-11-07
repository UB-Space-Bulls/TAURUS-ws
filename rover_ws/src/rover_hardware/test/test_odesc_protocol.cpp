#include <gtest/gtest.h>

#include "rover_hardware/odesc_protocol.hpp"

using namespace rover_hardware::odesc;

TEST(CanId, PacksNodeAndCommand)
{
  EXPECT_EQ(make_id(0, kSetInputVel), 0x00Du);
  EXPECT_EQ(make_id(3, kSetInputVel), (3u << 5) | 0x00D);
  EXPECT_EQ(node_id_of(make_id(7, kHeartbeat)), 7);
  EXPECT_EQ(cmd_id_of(make_id(7, kHeartbeat)), kHeartbeat);
}

TEST(Frames, SetAxisStateIsLittleEndianU32)
{
  const auto f = set_axis_state(1, kAxisStateClosedLoop);
  EXPECT_EQ(f.id, make_id(1, kSetAxisState));
  EXPECT_EQ(f.len, 4);
  EXPECT_EQ(f.data[0], 8);
  EXPECT_EQ(f.data[1], 0);
  EXPECT_EQ(f.data[2], 0);
  EXPECT_EQ(f.data[3], 0);
}

TEST(Frames, SetInputVelPacksFloat)
{
  // 1.0f == 0x3F800000 -> little-endian bytes 00 00 80 3F
  const auto f = set_input_vel(2, 1.0f);
  EXPECT_EQ(f.len, 8);
  EXPECT_EQ(f.data[0], 0x00);
  EXPECT_EQ(f.data[1], 0x00);
  EXPECT_EQ(f.data[2], 0x80);
  EXPECT_EQ(f.data[3], 0x3F);
  EXPECT_EQ(get_f32(f, 4), 0.0f);  // torque feedforward left at 0
}

TEST(Frames, SetControllerModePacksBothFields)
{
  const auto f = set_controller_mode(0, kControlModePosition, kInputModeTrapTraj);
  EXPECT_EQ(static_cast<int32_t>(get_u32(f, 0)), kControlModePosition);
  EXPECT_EQ(static_cast<int32_t>(get_u32(f, 4)), kInputModeTrapTraj);
}

TEST(Frames, ClearErrorsHasNoPayload)
{
  EXPECT_EQ(clear_errors(5).len, 0);
}

TEST(Parse, Heartbeat)
{
  Frame f{make_id(4, kHeartbeat), 8, {0x00, 0x08, 0x00, 0x00, 8, 0, 0, 0}};
  const auto hb = parse_heartbeat(f);
  EXPECT_EQ(hb.axis_error, 0x800u);
  EXPECT_EQ(hb.axis_state, kAxisStateClosedLoop);
}

TEST(Parse, EncoderEstimatesRoundTrip)
{
  Frame f{make_id(0, kEncoderEstimates), 8, {}};
  put_f32(f, 0, -2.5f);
  put_f32(f, 4, 10.25f);
  const auto est = parse_encoder_estimates(f);
  EXPECT_FLOAT_EQ(est.pos_turns, -2.5f);
  EXPECT_FLOAT_EQ(est.vel_turns_per_s, 10.25f);
}

TEST(Units, OneJointTurnIsGearRatioMotorTurns)
{
  EXPECT_DOUBLE_EQ(joint_to_motor(kTwoPi, 50.0, 1), 50.0);
  EXPECT_DOUBLE_EQ(joint_to_motor(kTwoPi, 50.0, -1), -50.0);
}

TEST(Units, RoundTrip)
{
  for (double rad : {-3.0, -0.1, 0.0, 0.7, 12.0}) {
    for (int dir : {1, -1}) {
      EXPECT_NEAR(motor_to_joint(joint_to_motor(rad, 18.75, dir), 18.75, dir), rad, 1e-12);
    }
  }
}
