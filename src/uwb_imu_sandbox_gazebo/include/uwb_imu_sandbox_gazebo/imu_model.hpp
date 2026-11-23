// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
//
// IMU error model. Takes the exact motion of a link (from Gazebo) and turns
// it into what a real low-cost IMU reports:
//   * its own internal sample clocks for accelerometer, gyroscope and
//     orientation, read out by a ~100 Hz driver loop (so most published
//     messages repeat the last internal sample, like the real sensor),
//   * a turn-on bias drawn fresh at every start, slowly wandering, and
//     optionally re-drawn after a long run,
//   * motion-dependent jitter (vibration grows with speed) with a measured,
//     non-Gaussian shape and sample-to-sample memory,
//   * the output resolution of the real part.
#pragma once

#include <array>
#include <random>
#include <string>
#include <vector>

#include "uwb_imu_sandbox_gazebo/geometry.hpp"

namespace uwb_imu_sandbox
{

class SealedModel;

struct ImuTruth
{
  Vec3 world_lin_vel;     ///< link velocity in the world frame [m/s]
  Vec3 body_ang_vel;      ///< link angular velocity in the link frame [rad/s]
  Quat orientation;       ///< link orientation (link -> world)
  Vec3 gravity{0, 0, -9.80665};   ///< world gravity [m/s^2]
};

struct ImuSample
{
  double stamp{0};
  Vec3 accel;     ///< specific force in the IMU frame [m/s^2]
  Vec3 gyro;      ///< angular rate in the IMU frame [rad/s]
  Quat orientation;
};

struct ImuOptions
{
  bool enable_noise{true};
  bool enable_bias{true};
  bool enable_quantisation{true};
  bool enable_sample_hold{true};
  double bias_reseed_period{0.0};   ///< re-draw the bias after this long [s], 0 = never
  double bias_transition_time{5.0};  ///< time constant of the move to a re-drawn bias [s]
  double bias_scale{1.0};           ///< scales the turn-on bias spread (0 = mean bias only)
  double noise_scale{1.0};          ///< scales the jitter
  double publish_period{0.01};      ///< period of the driver loop that reads the IMU [s]
};

class ImuErrorModel
{
public:
  ImuErrorModel(const std::string & sealed_file, const ImuOptions & opt, uint64_t seed);
  ImuErrorModel(const SealedModel & m, const ImuOptions & opt, uint64_t seed);

  /// Feed the exact state at time t (call every physics step).  Returns true
  /// when the driver loop publishes a message; `out` then holds it.
  bool update(double t, const ImuTruth & truth, ImuSample & out);

  /// Current (true) bias, for tests and the ground-truth topic.
  const Vec3 & accelBias() const {return bias_a_;}
  const Vec3 & gyroBias() const {return bias_g_;}
  int biasDraws() const {return bias_draws_;}

private:
  void load(const SealedModel & m);
  double drawInterval(const std::vector<double> & q);
  double drawPublishJitter();
  double shape(const std::vector<double> & q, double z) const;
  double stdAt(const std::vector<double> & table, double speed, int axis) const;
  void drawBias(Vec3 & a, Vec3 & g);
  double quantise(double v, double res) const;

  ImuOptions opt_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> n01_{0.0, 1.0};
  std::uniform_real_distribution<double> u01_{0.0, 1.0};

  // tables
  std::vector<double> probs_, t_acc_, t_gyr_, t_quat_, t_pub_, bins_;
  std::vector<double> acc_std_, gyr_std_, ori_std_;
  std::array<std::vector<double>, 3> acc_shape_, gyr_shape_;
  std::array<double, 6> ar_{};
  std::array<double, 12> bias_dist_{};
  std::array<double, 3> res_{};
  std::array<double, 3> walk_{};

  // state
  bool started_{false};
  double t_last_{0}, next_acc_{0}, next_gyr_{0}, next_quat_{0}, next_pub_{0};
  double last_reseed_{0};
  double pub_nominal_{0}, pub_period_{0.01}, pub_jitter_{0}, pub_median_{0.01};
  double t_acc_prev_{0};
  Vec3 v_acc_prev_;
  bool have_v_prev_{false};
  Vec3 bias_a_, bias_g_, target_a_, target_g_;
  double yaw_drift_{0};
  int bias_draws_{0};
  std::array<double, 3> z_acc_{}, z_gyr_{};
  Vec3 hold_acc_, hold_gyr_;
  Quat hold_q_;
  bool have_acc_{false}, have_gyr_{false}, have_q_{false};
};

}  // namespace uwb_imu_sandbox
