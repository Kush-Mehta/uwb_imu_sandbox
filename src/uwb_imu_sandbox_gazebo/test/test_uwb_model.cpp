// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "uwb_imu_sandbox_gazebo/uwb_model.hpp"

using namespace uwb_imu_sandbox;

namespace
{
const std::string kFile = std::string(MODEL_DIR) + "/uwb_model.bin";

LinkGeometry los(double d)
{
  LinkGeometry g;
  g.distance = d;
  return g;
}

LinkGeometry behindPillar(double d, double h)
{
  LinkGeometry g;
  g.distance = d;
  g.blocked = true;
  g.n_obstacles = 1;
  g.thickness = 0.5;
  g.clearance = h;
  g.d1 = 1.7;
  g.d2 = d - 1.7;
  g.nu = fresnelNu(h, g.d1, g.d2, 0.0462);
  return g;
}

struct Stats
{
  double mean_err{0}, std_err{0}, mean_q{0}, mean_rx{0}, drop{0};
  double p95_abs_err{0};
};

Stats run(const UwbChannelModel & m, const LinkGeometry & g, int n, uint64_t seed)
{
  std::mt19937_64 rng(seed);
  UwbLinkState st;
  std::vector<double> e, q, rx;
  int dropped = 0;
  for (int i = 0; i < n; ++i) {
    UwbMeasurement r;
    auto o = m.sample(g, st, rng, r);
    if (o != UwbOutcome::kOk) {++dropped; continue;}
    EXPECT_TRUE(std::isfinite(r.range) && std::isfinite(r.rx) && std::isfinite(r.fp) &&
      std::isfinite(r.quality) && std::isfinite(r.delta));
    e.push_back(r.range - g.distance);
    q.push_back(r.quality);
    rx.push_back(r.rx);
  }
  Stats s;
  s.mean_err = std::accumulate(e.begin(), e.end(), 0.0) / e.size();
  double v = 0;
  for (double x : e) {v += (x - s.mean_err) * (x - s.mean_err);}
  s.std_err = std::sqrt(v / e.size());
  s.mean_q = std::accumulate(q.begin(), q.end(), 0.0) / q.size();
  s.mean_rx = std::accumulate(rx.begin(), rx.end(), 0.0) / rx.size();
  s.drop = static_cast<double>(dropped) / n;
  std::vector<double> a(e.size());
  std::transform(e.begin(), e.end(), a.begin(), [](double x) {return std::abs(x);});
  std::sort(a.begin(), a.end());
  s.p95_abs_err = a[static_cast<size_t>(0.95 * a.size())];
  return s;
}
}  // namespace

TEST(UwbModel, LoadsShippedModel)
{
  EXPECT_NO_THROW(UwbChannelModel m(kFile));
}

TEST(UwbModel, LosIsTightAndBiasedLikeADw1000)
{
  UwbChannelModel m(kFile);
  for (double d : {1.0, 3.0, 5.0, 8.0, 11.0}) {
    Stats s = run(m, los(d), 4000, 11);
    // a few cm of spread, a positive offset of 0..30 cm, few drops
    EXPECT_LT(s.std_err, 0.06) << d;
    EXPECT_GT(s.mean_err, -0.10) << d;
    EXPECT_LT(s.mean_err, 0.30) << d;
    EXPECT_LT(s.drop, 0.15) << d;
  }
}

TEST(UwbModel, NlosIsWorseThanLos)
{
  UwbChannelModel m(kFile);
  Stats l = run(m, los(6.0), 6000, 3);
  Stats n = run(m, behindPillar(6.0, 1.0), 6000, 3);
  EXPECT_GT(n.p95_abs_err, l.p95_abs_err);
  EXPECT_LT(n.mean_q, l.mean_q);
  EXPECT_LT(n.mean_rx, l.mean_rx);
  EXPECT_DOUBLE_EQ(m.nlosWeight(behindPillar(6.0, 1.0)), 1.0);
  EXPECT_DOUBLE_EQ(m.nlosWeight(los(6.0)), 0.0);
}

TEST(UwbModel, TransitionIsGradualNearTheEdge)
{
  UwbChannelModel m(kFile);
  // a clear path 1 cm from an edge is partly shadowed, 1 m away it is LOS
  LinkGeometry near = los(6.0);
  near.clearance = -0.01;
  near.d1 = 2.0;
  near.d2 = 4.0;
  near.nu = fresnelNu(-0.01, 2.0, 4.0, 0.0462);
  LinkGeometry far = near;
  far.clearance = -1.0;
  far.nu = fresnelNu(-1.0, 2.0, 4.0, 0.0462);
  const double wn = m.nlosWeight(near);
  EXPECT_GT(wn, 0.0);
  EXPECT_LT(wn, 1.0);
  EXPECT_DOUBLE_EQ(m.nlosWeight(far), 0.0);
}

TEST(UwbModel, RangeLimitAndMaterial)
{
  UwbChannelModel m(kFile);
  std::mt19937_64 rng(1);
  UwbLinkState st;
  UwbMeasurement r;
  EXPECT_EQ(m.sample(los(12.5), st, rng, r), UwbOutcome::kOutOfRange);
  m.setMaxRange(20.0);
  EXPECT_NE(m.sample(los(12.5), st, rng, r), UwbOutcome::kOutOfRange);
  LinkGeometry thick = behindPillar(6.0, 1.0);
  thick.thickness = 2.0;
  EXPECT_EQ(m.sample(thick, st, rng, r), UwbOutcome::kTooMuchMaterial);
}

TEST(UwbModel, SeedMakesItRepeatable)
{
  UwbChannelModel m(kFile);
  auto draw = [&](uint64_t seed) {
      std::mt19937_64 rng(seed);
      UwbLinkState st;
      std::vector<double> v;
      for (int i = 0; i < 50; ++i) {
        UwbMeasurement r;
        if (m.sample(behindPillar(5.0, 0.6), st, rng, r) == UwbOutcome::kOk) {v.push_back(r.range);}
      }
      return v;
    };
  EXPECT_EQ(draw(5), draw(5));
  EXPECT_NE(draw(5), draw(6));
}

TEST(UwbModel, ScheduleMatchesTheRealTag)
{
  UwbChannelModel m(kFile);
  auto cycle = [&](int n) {
      double s = 0;
      for (int i = 0; i < n; ++i) {s += m.slotGap(n, i);}
      return s;
    };
  // the real tag: ~0.10 s with one anchor, ~0.165 s with four (about 6 Hz each)
  EXPECT_NEAR(cycle(1), 0.102, 0.01);
  EXPECT_NEAR(cycle(4), 0.165, 0.01);
  EXPECT_GT(cycle(6), cycle(4));
  EXPECT_GT(m.slotGap(4, 3), m.slotGap(4, 1));   // idle gap after the last anchor
}

TEST(UwbModel, SignalFieldsAreNormalised)
{
  UwbChannelModel m(kFile);
  std::mt19937_64 rng(2);
  UwbLinkState st;
  for (double d : {1.0, 6.0, 11.0}) {
    for (auto g : {los(d), behindPillar(d + 2.0, 0.8)}) {
      for (int i = 0; i < 300; ++i) {
        UwbMeasurement r;
        if (m.sample(g, st, rng, r) != UwbOutcome::kOk) {continue;}
        for (double v : {r.rx, r.fp, r.quality, r.delta}) {
          EXPECT_GT(v, -15.0);
          EXPECT_LT(v, 120.0);
        }
      }
    }
  }
}

TEST(UwbModel, SpotSpecificMultipathIsStable)
{
  UwbChannelModel m(kFile);
  m.setDelaySpread(0.0);   // compare the multipath only, not the per-run anchor calibration
  auto median_err = [&](double x, uint64_t seed) {
      LinkGeometry g = behindPillar(7.0, 0.9);
      g.tag_pos = {x, 3.0, 0.5};
      std::mt19937_64 rng(seed);
      UwbLinkState st;
      st.anchor_id = 3;
      std::vector<double> e;
      for (int i = 0; i < 400; ++i) {
        UwbMeasurement r;
        if (m.sample(g, st, rng, r) == UwbOutcome::kOk) {e.push_back(r.range - g.distance);}
      }
      std::sort(e.begin(), e.end());
      return e[e.size() / 2];
    };
  // the same spot behaves the same in every run (different random seeds)
  EXPECT_NEAR(median_err(1.0, 1), median_err(1.0, 99), 0.05);
  // and not every spot behaves the same
  std::vector<double> spots;
  for (int i = 0; i < 12; ++i) {spots.push_back(median_err(1.0 + 0.5 * i, 7));}
  const double lo = *std::min_element(spots.begin(), spots.end());
  const double hi = *std::max_element(spots.begin(), spots.end());
  EXPECT_GT(hi - lo, 0.02);
}

TEST(UwbModel, EachAnchorHasItsOwnCalibrationOffset)
{
  UwbChannelModel m(kFile);
  m.setDelaySpread(0.05);
  std::mt19937_64 rng(3);
  std::vector<double> offsets;
  for (int a = 1; a <= 6; ++a) {
    UwbLinkState st;
    st.anchor_id = a;
    double sum = 0;
    int n = 0;
    for (int i = 0; i < 300; ++i) {
      UwbMeasurement r;
      if (m.sample(los(5.0), st, rng, r) == UwbOutcome::kOk) {sum += r.range - 5.0; ++n;}
    }
    offsets.push_back(sum / n);
    EXPECT_NE(st.delay_offset, 0.0);
  }
  const double lo = *std::min_element(offsets.begin(), offsets.end());
  const double hi = *std::max_element(offsets.begin(), offsets.end());
  EXPECT_GT(hi - lo, 0.02);
}
