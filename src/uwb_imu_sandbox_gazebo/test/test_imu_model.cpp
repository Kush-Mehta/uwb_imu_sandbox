// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "uwb_imu_sandbox_gazebo/imu_model.hpp"

using namespace uwb_imu_sandbox;

namespace
{
const std::string kFile = std::string(MODEL_DIR) + "/imu_model.bin";

std::vector<ImuSample> runStill(ImuErrorModel & m, double seconds, double dt = 0.001)
{
  std::vector<ImuSample> out;
  ImuTruth tr;
  for (double t = 0; t < seconds; t += dt) {
    ImuSample s;
    if (m.update(t, tr, s)) {out.push_back(s);}
  }
  return out;
}
}  // namespace

TEST(ImuModel, PublishesAtAbout100Hz)
{
  ImuErrorModel m(kFile, {}, 1);
  auto s = runStill(m, 20.0);
  EXPECT_NEAR(s.size() / 20.0, 100.0, 8.0);
}

TEST(ImuModel, StillSensorSeesGravityPlusBias)
{
  ImuErrorModel m(kFile, {}, 3);
  auto s = runStill(m, 30.0);
  double ax = 0, ay = 0, az = 0, gz = 0;
  for (auto & x : s) {ax += x.accel.x; ay += x.accel.y; az += x.accel.z; gz += x.gyro.z;}
  const double n = static_cast<double>(s.size());
  EXPECT_NEAR(ax / n, m.accelBias().x, 0.02);
  EXPECT_NEAR(ay / n, m.accelBias().y, 0.02);
  EXPECT_NEAR(az / n, 9.80665 + m.accelBias().z, 0.02);
  EXPECT_NEAR(gz / n, m.gyroBias().z, 0.002);
  // gravity reading of a real low-cost part: within a few % of 9.81
  EXPECT_NEAR(az / n, 9.81, 0.5);
}

TEST(ImuModel, OutputHasTheSensorResolution)
{
  ImuErrorModel m(kFile, {}, 4);
  auto s = runStill(m, 5.0);
  for (auto & x : s) {
    EXPECT_NEAR(x.accel.x * 256.0, std::round(x.accel.x * 256.0), 1e-6);
    EXPECT_NEAR(x.gyro.z * 512.0, std::round(x.gyro.z * 512.0), 1e-6);
    EXPECT_NEAR(x.orientation.z * 16384.0, std::round(x.orientation.z * 16384.0), 1e-6);
  }
}

TEST(ImuModel, MostMessagesRepeatTheLastInternalSample)
{
  ImuErrorModel m(kFile, {}, 5);
  // move so the values change at every internal sample
  std::vector<ImuSample> out;
  ImuTruth tr;
  for (double t = 0; t < 20.0; t += 0.001) {
    tr.world_lin_vel = {0.5 * std::sin(t), 0.3 * std::cos(0.7 * t), 0};
    tr.body_ang_vel = {0, 0, 0.4 * std::sin(0.5 * t)};
    ImuSample s;
    if (m.update(t, tr, s)) {out.push_back(s);}
  }
  int same = 0;
  for (size_t i = 1; i < out.size(); ++i) {
    if (out[i].gyro.z == out[i - 1].gyro.z && out[i].gyro.x == out[i - 1].gyro.x) {++same;}
  }
  const double frac = static_cast<double>(same) / out.size();
  // the real sensor delivers new gyro data every ~50 ms at a ~100 Hz readout
  EXPECT_GT(frac, 0.6);
  EXPECT_LT(frac, 0.95);
}

TEST(ImuModel, NewRandomBiasEveryStart)
{
  ImuErrorModel a(kFile, {}, 10), b(kFile, {}, 11);
  runStill(a, 0.1);
  runStill(b, 0.1);
  EXPECT_NE(a.accelBias().x, b.accelBias().x);
  EXPECT_NE(a.gyroBias().z, b.gyroBias().z);
  ImuErrorModel c(kFile, {}, 10);
  runStill(c, 0.1);
  EXPECT_DOUBLE_EQ(a.accelBias().x, c.accelBias().x);
}

TEST(ImuModel, BiasIsRedrawnAfterTheReseedPeriod)
{
  ImuOptions o;
  o.bias_reseed_period = 10.0;
  o.bias_transition_time = 1.0;
  ImuErrorModel m(kFile, o, 21);
  runStill(m, 9.0);
  EXPECT_EQ(m.biasDraws(), 1);
  ImuTruth tr;
  ImuSample s;
  const Vec3 before = m.accelBias();
  for (double t = 9.0; t < 15.0; t += 0.001) {m.update(t, tr, s);}
  EXPECT_EQ(m.biasDraws(), 2);
  for (double t = 15.0; t < 25.0; t += 0.001) {m.update(t, tr, s);}
  EXPECT_EQ(m.biasDraws(), 3);
  EXPECT_NE(before.z, m.accelBias().z);
}

TEST(ImuModel, AllEffectsOffGivesTheTruth)
{
  ImuOptions o;
  o.enable_noise = false;
  o.enable_bias = false;
  o.enable_quantisation = false;
  ImuErrorModel m(kFile, o, 1);
  ImuTruth tr;
  tr.body_ang_vel = {0.1, -0.2, 0.3};
  ImuSample s;
  bool got = false;
  for (double t = 0; t < 1.0; t += 0.001) {got = m.update(t, tr, s) || got;}
  ASSERT_TRUE(got);
  EXPECT_NEAR(s.accel.z, 9.80665, 1e-9);
  EXPECT_NEAR(s.gyro.y, -0.2, 1e-12);
  EXPECT_NEAR(s.orientation.w, 1.0, 1e-12);
}

TEST(ImuModel, VibrationGrowsWithSpeed)
{
  auto spread = [&](double speed) {
      ImuOptions o;
      o.enable_bias = false;
      o.enable_quantisation = false;
      ImuErrorModel m(kFile, o, 7);
      ImuTruth tr;
      tr.world_lin_vel = {speed, 0, 0};
      std::vector<double> v;
      ImuSample s;
      for (double t = 0; t < 30.0; t += 0.001) {
        if (m.update(t, tr, s)) {v.push_back(s.accel.x);}
      }
      double mu = 0, var = 0;
      for (double x : v) {mu += x;}
      mu /= v.size();
      for (double x : v) {var += (x - mu) * (x - mu);}
      return std::sqrt(var / v.size());
    };
  EXPECT_LT(spread(0.0), spread(0.5));
  EXPECT_LT(spread(0.5) * 0.9, spread(1.2));
}
