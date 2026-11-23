// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
#include "uwb_imu_sandbox_gazebo/imu_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "uwb_imu_sandbox_gazebo/sealed_model.hpp"

namespace uwb_imu_sandbox
{
namespace
{
double normalCdf(double z) {return 0.5 * std::erfc(-z / std::sqrt(2.0));}

Quat mul(const Quat & a, const Quat & b)
{
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

Quat smallRotation(double r, double p, double y)
{
  // roll-pitch-yaw (fixed axes) to quaternion
  const double cr = std::cos(r / 2), sr = std::sin(r / 2);
  const double cp = std::cos(p / 2), sp = std::sin(p / 2);
  const double cy = std::cos(y / 2), sy = std::sin(y / 2);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}
}  // namespace

ImuErrorModel::ImuErrorModel(const std::string & f, const ImuOptions & o, uint64_t seed)
: opt_(o), rng_(seed)
{
  load(SealedModel(f, kImuMagic));
}

ImuErrorModel::ImuErrorModel(const SealedModel & m, const ImuOptions & o, uint64_t seed)
: opt_(o), rng_(seed)
{
  load(m);
}

void ImuErrorModel::load(const SealedModel & m)
{
  probs_ = m.first(tags::PROBS);
  t_acc_ = m.first(tags::IMU_TIMING_ACC);
  t_gyr_ = m.first(tags::IMU_TIMING_GYR);
  t_quat_ = m.first(tags::IMU_TIMING_QUAT);
  t_pub_ = m.first(tags::IMU_TIMING_PUB);
  bins_ = m.first(tags::IMU_SPEED_BINS);
  acc_std_ = m.first(tags::IMU_ACC_STD);
  gyr_std_ = m.first(tags::IMU_GYR_STD);
  ori_std_ = m.first(tags::IMU_ORI_STD);
  const auto & as = m.first(tags::IMU_ACC_SHAPE);
  const auto & gs = m.first(tags::IMU_GYR_SHAPE);
  const size_t K = probs_.size();
  const size_t nb = bins_.size() - 1;
  if (as.size() != 3 * K || gs.size() != 3 * K || acc_std_.size() != 3 * nb ||
    gyr_std_.size() != 3 * nb || ori_std_.size() != 3 * nb || t_acc_.size() != K ||
    t_gyr_.size() != K || t_quat_.size() != K || t_pub_.size() != K)
  {
    throw std::runtime_error("IMU model: bad table layout");
  }
  for (int k = 0; k < 3; ++k) {
    acc_shape_[k].assign(as.begin() + k * K, as.begin() + (k + 1) * K);
    gyr_shape_[k].assign(gs.begin() + k * K, gs.begin() + (k + 1) * K);
  }
  const auto & ar = m.first(tags::IMU_AR);
  const auto & b = m.first(tags::IMU_BIAS);
  const auto & q = m.first(tags::IMU_QUANT);
  const auto & w = m.first(tags::IMU_BIAS_WALK);
  if (ar.size() != 6 || b.size() != 12 || q.size() != 3 || w.size() != 3) {
    throw std::runtime_error("IMU model: bad parameter layout");
  }
  std::copy(ar.begin(), ar.end(), ar_.begin());
  std::copy(b.begin(), b.end(), bias_dist_.begin());
  std::copy(q.begin(), q.end(), res_.begin());
  std::copy(w.begin(), w.end(), walk_.begin());
}

double ImuErrorModel::drawInterval(const std::vector<double> & q)
{
  // inverse-CDF sample from the measured interval distribution
  const double u = u01_(rng_);
  auto it = std::upper_bound(probs_.begin(), probs_.end(), u);
  if (it == probs_.end()) {return q.back();}
  const size_t hi = static_cast<size_t>(it - probs_.begin());
  const size_t lo = hi - 1;
  const double a = (u - probs_[lo]) / (probs_[hi] - probs_[lo]);
  return std::max(1e-3, q[lo] + a * (q[hi] - q[lo]));
}

double ImuErrorModel::drawPublishJitter()
{
  // the driver runs a fixed-period timer; each wake-up is late by a random
  // scheduling delay, so consecutive intervals are period + e_k - e_(k-1)
  return std::max(0.0, (drawInterval(t_pub_) - pub_median_) / std::sqrt(2.0) + 0.002);
}

double ImuErrorModel::shape(const std::vector<double> & q, double z) const
{
  const double u = normalCdf(z);
  auto it = std::upper_bound(probs_.begin(), probs_.end(), u);
  if (it == probs_.end()) {return q.back();}
  const size_t hi = static_cast<size_t>(it - probs_.begin());
  const size_t lo = hi - 1;
  const double a = (u - probs_[lo]) / (probs_[hi] - probs_[lo]);
  return q[lo] + a * (q[hi] - q[lo]);
}

double ImuErrorModel::stdAt(const std::vector<double> & t, double speed, int axis) const
{
  // bin 0 is "at rest"; the others are evaluated at their centres
  const size_t nb = bins_.size() - 1;
  std::vector<double> c(nb);
  c[0] = 0.0;
  for (size_t i = 1; i < nb; ++i) {c[i] = 0.5 * (bins_[i] + bins_[i + 1]);}
  if (speed <= c[0]) {return t[axis];}
  for (size_t i = 1; i < nb; ++i) {
    if (speed <= c[i]) {
      const double a = (speed - c[i - 1]) / (c[i] - c[i - 1]);
      return (1 - a) * t[(i - 1) * 3 + axis] + a * t[i * 3 + axis];
    }
  }
  return t[(nb - 1) * 3 + axis];
}

void ImuErrorModel::drawBias(Vec3 & a, Vec3 & g)
{
  const double s = opt_.bias_scale;
  a = {bias_dist_[0] + s * bias_dist_[3] * n01_(rng_),
    bias_dist_[1] + s * bias_dist_[4] * n01_(rng_),
    bias_dist_[2] + s * bias_dist_[5] * n01_(rng_)};
  g = {bias_dist_[6] + s * bias_dist_[9] * n01_(rng_),
    bias_dist_[7] + s * bias_dist_[10] * n01_(rng_),
    bias_dist_[8] + s * bias_dist_[11] * n01_(rng_)};
  ++bias_draws_;
}

double ImuErrorModel::quantise(double v, double res) const
{
  if (!opt_.enable_quantisation || res <= 0) {return v;}
  return std::round(v / res) * res;
}

bool ImuErrorModel::update(double t, const ImuTruth & tr, ImuSample & out)
{
  if (!started_) {
    started_ = true;
    t_last_ = t;
    last_reseed_ = t;
    if (opt_.enable_bias) {
      drawBias(bias_a_, bias_g_);
    }
    target_a_ = bias_a_;
    target_g_ = bias_g_;
    // random phase of the internal clocks at power-up
    next_acc_ = t + u01_(rng_) * drawInterval(t_acc_);
    next_gyr_ = t + u01_(rng_) * drawInterval(t_gyr_);
    next_quat_ = t + u01_(rng_) * drawInterval(t_quat_);
    // the driver is a fixed-period timer (100 Hz on the real robot)
    pub_period_ = opt_.publish_period;
    pub_median_ = t_pub_[t_pub_.size() / 2];
    pub_nominal_ = t + u01_(rng_) * pub_period_;
    pub_jitter_ = drawPublishJitter();
    next_pub_ = pub_nominal_ + pub_jitter_;
  }
  const double dt = std::max(0.0, t - t_last_);
  t_last_ = t;

  // ---- bias evolution ------------------------------------------------
  if (opt_.enable_bias) {
    if (opt_.bias_reseed_period > 0 && t - last_reseed_ >= opt_.bias_reseed_period) {
      drawBias(target_a_, target_g_);
      last_reseed_ = t;
    }
    const double k = opt_.bias_transition_time > 0 ?
      1.0 - std::exp(-dt / opt_.bias_transition_time) : 1.0;
    bias_a_ = bias_a_ + (target_a_ - bias_a_) * k;
    bias_g_ = bias_g_ + (target_g_ - bias_g_) * k;
    const double sq = std::sqrt(dt);
    const Vec3 wa(n01_(rng_), n01_(rng_), n01_(rng_));
    const Vec3 wg(n01_(rng_), n01_(rng_), n01_(rng_));
    bias_a_ = bias_a_ + wa * (walk_[0] * sq);
    target_a_ = target_a_ + wa * (walk_[0] * sq);
    bias_g_ = bias_g_ + wg * (walk_[1] * sq);
    target_g_ = target_g_ + wg * (walk_[1] * sq);
    yaw_drift_ += walk_[2] * sq * n01_(rng_);
  }

  const double speed = std::hypot(tr.world_lin_vel.x, tr.world_lin_vel.y);
  const bool hold = opt_.enable_sample_hold;

  // ---- accelerometer: its own sample clock, averages accel over the interval
  if (!hold || t >= next_acc_) {
    Vec3 a_world;
    if (have_v_prev_ && t - t_acc_prev_ > 1e-6) {
      a_world = (tr.world_lin_vel - v_acc_prev_) * (1.0 / (t - t_acc_prev_));
    }
    v_acc_prev_ = tr.world_lin_vel;
    t_acc_prev_ = t;
    have_v_prev_ = true;
    Vec3 f = tr.orientation.inverseRotate(a_world - tr.gravity);
    std::array<double, 3> v{f.x, f.y, f.z};
    const std::array<double, 3> b{bias_a_.x, bias_a_.y, bias_a_.z};
    for (int k = 0; k < 3; ++k) {
      if (opt_.enable_noise) {
        z_acc_[k] = ar_[k] * z_acc_[k] + std::sqrt(1 - ar_[k] * ar_[k]) * n01_(rng_);
        v[k] += opt_.noise_scale * stdAt(acc_std_, speed, k) * shape(acc_shape_[k], z_acc_[k]);
      }
      if (opt_.enable_bias) {v[k] += b[k];}
      v[k] = quantise(v[k], res_[0]);
    }
    hold_acc_ = {v[0], v[1], v[2]};
    have_acc_ = true;
    if (hold) {while (next_acc_ <= t) {next_acc_ += drawInterval(t_acc_);}}
  }

  // ---- gyroscope
  if (!hold || t >= next_gyr_) {
    std::array<double, 3> v{tr.body_ang_vel.x, tr.body_ang_vel.y, tr.body_ang_vel.z};
    const std::array<double, 3> b{bias_g_.x, bias_g_.y, bias_g_.z};
    for (int k = 0; k < 3; ++k) {
      if (opt_.enable_noise) {
        z_gyr_[k] = ar_[3 + k] * z_gyr_[k] + std::sqrt(1 - ar_[3 + k] * ar_[3 + k]) * n01_(rng_);
        v[k] += opt_.noise_scale * stdAt(gyr_std_, speed, k) * shape(gyr_shape_[k], z_gyr_[k]);
      }
      if (opt_.enable_bias) {v[k] += b[k];}
      v[k] = quantise(v[k], res_[1]);
    }
    hold_gyr_ = {v[0], v[1], v[2]};
    have_gyr_ = true;
    if (hold) {while (next_gyr_ <= t) {next_gyr_ += drawInterval(t_gyr_);}}
  }

  // ---- orientation (the sensor's own fusion output)
  if (!hold || t >= next_quat_) {
    Quat q = tr.orientation;
    if (opt_.enable_noise) {
      const double s = opt_.noise_scale;
      q = mul(q, smallRotation(s * stdAt(ori_std_, speed, 0) * n01_(rng_),
        s * stdAt(ori_std_, speed, 1) * n01_(rng_),
        s * stdAt(ori_std_, speed, 2) * n01_(rng_)));
    }
    if (opt_.enable_bias) {q = mul(smallRotation(0, 0, yaw_drift_), q);}
    // keep the sign convention of the sensor (w >= 0)
    if (q.w < 0) {q = {-q.w, -q.x, -q.y, -q.z};}
    q = {quantise(q.w, res_[2]), quantise(q.x, res_[2]),
      quantise(q.y, res_[2]), quantise(q.z, res_[2])};
    hold_q_ = q;
    have_q_ = true;
    if (hold) {while (next_quat_ <= t) {next_quat_ += drawInterval(t_quat_);}}
  }

  // ---- driver loop publishes whatever the sensor holds right now
  if (t < next_pub_) {
    return false;
  }
  // one message per timer tick; an overdue tick goes out on the next step
  pub_nominal_ += pub_period_;
  if (pub_nominal_ < t - 0.1) {pub_nominal_ = t;}   // the simulation jumped ahead
  pub_jitter_ = drawPublishJitter();
  next_pub_ = pub_nominal_ + pub_jitter_;
  if (!have_acc_ || !have_gyr_ || !have_q_) {
    return false;
  }
  out.stamp = t;
  out.accel = hold_acc_;
  out.gyro = hold_gyr_;
  out.orientation = hold_q_;
  return true;
}

}  // namespace uwb_imu_sandbox
