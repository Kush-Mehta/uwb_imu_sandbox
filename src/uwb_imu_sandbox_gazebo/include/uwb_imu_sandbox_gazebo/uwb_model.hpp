// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
//
// UWB channel model.  Turns the geometry of one anchor-tag path into one
// ranging result: range [m] plus the four signal fields the DW1000 reports
// (RX power, first-path power, quality, RX-FP delta) on a unitless 0..100
// scale.
//
// Built from measured LOS runs and NLOS runs behind a pillar at several
// lateral offsets.  For every measured condition the model keeps
//   * the full marginal distribution of each field (quantile function),
//   * the dependence between the fields (Gaussian copula),
//   * their sample-to-sample memory (VAR(1) on the copula scores),
//   * the probability that a ranging slot produces nothing.
// A new path is matched to the measured conditions by its geometry (straight
// distance, depth behind the obstacle edge, Fresnel parameter) and the
// matching conditions are blended in quantile space.
#pragma once

#include <array>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "uwb_imu_sandbox_gazebo/geometry.hpp"

namespace uwb_imu_sandbox
{

class SealedModel;

constexpr int kUwbChannels = 5;   // range error, rx, fp, quality, delta

struct UwbMeasurement
{
  double range{0};      ///< measured distance [m]
  double rx{0};         ///< RX power level   (unitless, higher = stronger)
  double fp{0};         ///< first-path level (unitless, higher = stronger)
  double quality{0};    ///< quality index    (unitless, higher = better)
  double delta{0};      ///< RX-FP delta level (unitless, higher = more multipath)
};

/// Per anchor-tag link memory (copula state).
struct UwbLinkState
{
  int anchor_id{0};
  std::array<double, kUwbChannels> z{};
  double delay_offset{0};   ///< this anchor's antenna-delay calibration error [m]
  bool initialised{false};
};

/// Why a slot produced nothing (for diagnostics and the truth topic).
enum class UwbOutcome { kOk, kDropped, kOutOfRange, kTooMuchMaterial };

class UwbChannelModel
{
public:
  explicit UwbChannelModel(const std::string & sealed_file);
  explicit UwbChannelModel(const SealedModel & model);

  /// Draw one ranging result for a link.  Returns kOk and fills `out`, or
  /// one of the failure outcomes.
  UwbOutcome sample(
    const LinkGeometry & g, UwbLinkState & state, std::mt19937_64 & rng,
    UwbMeasurement & out) const;

  /// Probability that a slot with this geometry produces nothing.
  double dropProbability(const LinkGeometry & g, int anchor_id = 0) const;

  /// NLOS multipath is spot-specific: the floor is split into cells of this
  /// size and each cell (per anchor) keeps one measured NLOS behaviour, so
  /// the same spot always behaves the same way and neighbouring spots can
  /// differ, as in the measurements.  0 = blend the nearest conditions instead.
  void setSpatialCell(double s) {cell_ = s;}
  double spatialCell() const {return cell_;}
  /// Every anchor has its own antenna-delay calibration error: a constant
  /// range offset drawn per anchor (normal, this standard deviation [m]) the
  /// first time the tag ranges to it.
  void setDelaySpread(double s) {delay_spread_ = s;}
  /// Constant offset added to every range [m], to match your own hardware's
  /// antenna-delay calibration (0 = the calibration of the measured tag).
  void setDelayOffset(double o) {delay_offset_ = o;}
  /// Seed of that spot-to-behaviour map (a property of the room, not of the run).
  void setMultipathSeed(uint64_t s) {field_seed_ = s;}

  /// Share of the NLOS model used for this geometry (0 = pure LOS, 1 = pure NLOS).
  double nlosWeight(const LinkGeometry & g) const;

  /// Time from one ranging slot to the next, for the slot `index` of a
  /// cycle over `n_anchors` anchors (index n_anchors-1 is followed by the
  /// tag's idle gap).
  double slotGap(int n_anchors, int index) const;

  /// Standard deviation of the random jitter on each slot time [s].
  double timingJitter() const {return timing_jitter_;}

  double maxRange() const {return max_range_;}
  void setMaxRange(double r) {max_range_ = r;}
  double maxMaterial() const {return max_material_;}
  void setMaxMaterial(double m) {max_material_ = m;}
  double wavelength() const {return wavelength_;}

private:
  struct Record
  {
    bool nlos{false};
    double D{0}, h{0}, nu{0}, p_drop{0};
    double Xo{0}, Lo{0};   ///< NLOS: lateral offset and distance from the obstacle of the measurement
    std::array<std::vector<double>, kUwbChannels> q;   // quantiles per channel
    std::array<double, 25> A{};                         // VAR(1) matrix (row major)
    std::array<double, 25> L{};                         // innovation Cholesky factor
  };
  void load(const SealedModel & m);
  void weights(
    const LinkGeometry & g, int anchor_id,
    std::vector<std::pair<int, double>> & w) const;
  double quantile(const Record & r, int ch, double u) const;

  std::vector<double> probs_;
  std::vector<Record> recs_;
  std::vector<int> los_sorted_;
  std::vector<int> nlos_;
  std::vector<std::vector<double>> timing_;   // per anchor count: the gaps
  double nu_clear_{-0.7};
  double h_min_{0.15};
  double h_max_{1.0};
  double D_min_nlos_{0}, D_max_nlos_{0};
  double X_max_{1.1}, L_min_{0}, L_max_{10};
  // geometry of the NLOS measurements (to map any path onto them)
  double exp_edge_{1.0}, exp_front_{1.96}, exp_mid_{1.71}, exp_half_{0.25};
  double bw_h_{0.25}, bw_D_{0.7};   // bandwidths in lateral offset and distance [m]
  int k_nlos_{4};
  double shepard_p_{0.0};   // >0: inverse-distance weights with this power
  double timing_jitter_{0.0004};
  double extra_drop_multi_{0.25};
  double max_range_{12.0};
  double max_material_{1.0};
  double wavelength_{299792458.0 / 6.4896e9};
  double cell_{0.25};
  double delay_spread_{0.03};
  double delay_offset_{0.0};
  uint64_t field_seed_{0x9E3779B97F4A7C15ULL};
};

}  // namespace uwb_imu_sandbox
