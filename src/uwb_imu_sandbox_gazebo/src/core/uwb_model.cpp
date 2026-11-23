// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
#include "uwb_imu_sandbox_gazebo/uwb_model.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

#include "uwb_imu_sandbox_gazebo/sealed_model.hpp"

namespace uwb_imu_sandbox
{
namespace
{
double normalCdf(double z) {return 0.5 * std::erfc(-z / std::sqrt(2.0));}

uint64_t mix(uint64_t x)
{
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}
}  // namespace

UwbChannelModel::UwbChannelModel(const std::string & sealed_file)
{
  load(SealedModel(sealed_file, kUwbMagic));
}

UwbChannelModel::UwbChannelModel(const SealedModel & model)
{
  load(model);
}

void UwbChannelModel::load(const SealedModel & m)
{
  probs_ = m.first(tags::PROBS);
  const size_t K = probs_.size();
  if (K < 3 || probs_.front() != 0.0 || probs_.back() != 1.0) {
    throw std::runtime_error("UWB model: bad quantile grid");
  }
  const auto & hdrs = m.all(tags::REC_HDR);
  const auto & qs = m.all(tags::REC_Q);
  const auto & As = m.all(tags::REC_A);
  const auto & Ls = m.all(tags::REC_LCHOL);
  const size_t n = static_cast<size_t>(m.first(tags::NREC).at(0));
  if (hdrs.size() != n || qs.size() != n || As.size() != n || Ls.size() != n || n == 0) {
    throw std::runtime_error("UWB model: inconsistent record count");
  }
  recs_.resize(n);
  for (size_t i = 0; i < n; ++i) {
    auto & r = recs_[i];
    if (hdrs[i].size() < 8 || qs[i].size() != kUwbChannels * K ||
      As[i].size() != 25 || Ls[i].size() != 25)
    {
      throw std::runtime_error("UWB model: bad record layout");
    }
    r.nlos = hdrs[i][0] > 0.5;
    r.D = hdrs[i][1];
    r.h = hdrs[i][2];
    r.nu = hdrs[i][3];
    r.p_drop = hdrs[i][4];
    r.Xo = hdrs[i][6];
    r.Lo = hdrs[i][7];
    for (int c = 0; c < kUwbChannels; ++c) {
      r.q[c].assign(qs[i].begin() + c * K, qs[i].begin() + (c + 1) * K);
    }
    std::copy(As[i].begin(), As[i].end(), r.A.begin());
    std::copy(Ls[i].begin(), Ls[i].end(), r.L.begin());
    (r.nlos ? nlos_ : los_sorted_).push_back(static_cast<int>(i));
  }
  if (los_sorted_.empty() || nlos_.empty()) {
    throw std::runtime_error("UWB model: needs both LOS and NLOS records");
  }
  std::sort(los_sorted_.begin(), los_sorted_.end(),
    [this](int a, int b) {return recs_[a].D < recs_[b].D;});
  h_max_ = 0.0;
  D_min_nlos_ = 1e9;
  D_max_nlos_ = 0.0;
  for (int i : nlos_) {
    h_max_ = std::max(h_max_, recs_[i].h);
    D_min_nlos_ = std::min(D_min_nlos_, recs_[i].D);
    D_max_nlos_ = std::max(D_max_nlos_, recs_[i].D);
  }
  X_max_ = 0.0;
  L_min_ = 1e9;
  L_max_ = 0.0;
  for (int i : nlos_) {
    X_max_ = std::max(X_max_, recs_[i].Xo);
    L_min_ = std::min(L_min_, recs_[i].Lo);
    L_max_ = std::max(L_max_, recs_[i].Lo);
  }

  // ranging schedule: [N, gap_1..gap_N, N, ...]
  const auto & tm = m.first(tags::TIMING);
  for (size_t i = 0; i < tm.size(); ) {
    const int N = static_cast<int>(tm[i]);
    if (N <= 0 || i + 1 + N > tm.size()) {throw std::runtime_error("UWB model: bad timing table");}
    if (static_cast<int>(timing_.size()) < N + 1) {timing_.resize(N + 1);}
    timing_[N].assign(tm.begin() + i + 1, tm.begin() + i + 1 + N);
    i += 1 + N;
  }
  const auto & tr = m.first(tags::TRANSITION);
  nu_clear_ = tr.at(0);
  h_min_ = tr.at(1);
  if (tr.size() >= 6) {
    exp_edge_ = tr[2];
    exp_front_ = tr[3];
    exp_mid_ = tr[4];
    exp_half_ = tr[5];
  }
  const auto & kn = m.first(tags::KERNEL);
  bw_h_ = kn.at(0);
  bw_D_ = kn.at(1);
  k_nlos_ = std::max(1, static_cast<int>(kn.at(2)));
  shepard_p_ = kn.size() > 5 ? kn[5] : 0.0;
  const auto & misc = m.first(tags::MISC);
  timing_jitter_ = misc.at(0);
  extra_drop_multi_ = misc.at(1);
}

double UwbChannelModel::nlosWeight(const LinkGeometry & g) const
{
  if (g.blocked && g.clearance >= h_min_) {
    return 1.0;
  }
  // Fresnel parameter at which the measured NLOS data starts for this path
  const double nu_data = fresnelNu(h_min_, std::max(g.d1, 0.05), std::max(g.d2, 0.05), wavelength_);
  if (g.nu <= nu_clear_) {return 0.0;}
  return std::clamp((g.nu - nu_clear_) / (nu_data - nu_clear_), 0.0, 1.0);
}

void UwbChannelModel::weights(
  const LinkGeometry & g, int anchor_id, std::vector<std::pair<int, double>> & w) const
{
  w.clear();
  const double beta = nlosWeight(g);
  if (beta < 1.0) {
    // LOS: linear blend of the two measured distances around d
    const double d = g.distance;
    auto it = std::lower_bound(los_sorted_.begin(), los_sorted_.end(), d,
        [this](int i, double v) {return recs_[i].D < v;});
    if (it == los_sorted_.begin()) {
      w.emplace_back(*it, 1.0 - beta);
    } else if (it == los_sorted_.end()) {
      w.emplace_back(los_sorted_.back(), 1.0 - beta);
    } else {
      const int hi = *it, lo = *(it - 1);
      const double span = recs_[hi].D - recs_[lo].D;
      const double a = span > 1e-9 ? (d - recs_[lo].D) / span : 0.5;
      w.emplace_back(lo, (1.0 - beta) * (1.0 - a));
      w.emplace_back(hi, (1.0 - beta) * a);
    }
  }
  if (beta > 0.0) {
    // map the path onto the measurement set-up: the distance of the tag from
    // the obstacle, and the lateral offset that would give the same depth
    // behind the obstacle edge in that set-up
    const double h = std::clamp(g.blocked ? g.clearance : h_min_, h_min_, h_max_);
    const double L = std::clamp(std::max(0.0, g.d2 - exp_half_), L_min_, L_max_);
    const double X = std::clamp((exp_edge_ - h) * (L + exp_front_) / exp_mid_, 0.0, X_max_);
    std::vector<std::pair<double, int>> k;
    k.reserve(nlos_.size());
    for (int i : nlos_) {
      const double a = (X - recs_[i].Xo) / bw_h_;
      const double b = (L - recs_[i].Lo) / bw_D_;
      k.emplace_back(-0.5 * (a * a + b * b), i);
    }
    const int K = std::min<int>(k_nlos_, static_cast<int>(k.size()));
    std::partial_sort(k.begin(), k.begin() + K, k.end(),
      [](const auto & x, const auto & y) {return x.first > y.first;});
    // weights of the K closest measured positions: Gaussian kernel, or
    // inverse-distance (Shepard) weights, which reproduce a measured position
    // exactly when the tag stands on it and blend smoothly in between
    std::vector<double> wk(K);
    const double top = k[0].first;
    double sum = 0.0;
    for (int j = 0; j < K; ++j) {
      if (shepard_p_ > 0.0) {
        const double dist = std::sqrt(std::max(0.0, -2.0 * k[j].first));
        wk[j] = 1.0 / (std::pow(dist, shepard_p_) + 1e-6);
      } else {
        wk[j] = std::exp(k[j].first - top);
      }
      sum += wk[j];
    }
    if (cell_ > 0.0) {
      // spot-specific multipath: this cell keeps one of the matching measured
      // behaviours, picked with probability proportional to its match
      const auto ix = static_cast<int64_t>(std::floor(g.tag_pos.x / cell_ + 0.5));
      const auto iy = static_cast<int64_t>(std::floor(g.tag_pos.y / cell_ + 0.5));
      const auto iz = static_cast<int64_t>(std::floor(g.tag_pos.z / cell_ + 0.5));
      uint64_t hsh = mix(field_seed_ ^ mix(static_cast<uint64_t>(anchor_id) + 0x51ED27ULL));
      hsh = mix(hsh ^ static_cast<uint64_t>(ix));
      hsh = mix(hsh ^ static_cast<uint64_t>(iy));
      hsh = mix(hsh ^ static_cast<uint64_t>(iz));
      const double u = static_cast<double>(hsh >> 11) * (1.0 / 9007199254740992.0);
      double acc = 0.0;
      int pick = k[K - 1].second;
      for (int j = 0; j < K; ++j) {
        acc += wk[j] / sum;
        if (u < acc) {pick = k[j].second; break;}
      }
      w.emplace_back(pick, beta);
    } else {
      for (int j = 0; j < K; ++j) {
        w.emplace_back(k[j].second, beta * wk[j] / sum);
      }
    }
  }
}

double UwbChannelModel::quantile(const Record & r, int ch, double u) const
{
  const auto & q = r.q[ch];
  u = std::clamp(u, 0.0, 1.0);
  auto it = std::upper_bound(probs_.begin(), probs_.end(), u);
  if (it == probs_.end()) {return q.back();}
  const size_t hi = static_cast<size_t>(it - probs_.begin());
  const size_t lo = hi - 1;
  const double a = (u - probs_[lo]) / (probs_[hi] - probs_[lo]);
  return q[lo] + a * (q[hi] - q[lo]);
}

double UwbChannelModel::dropProbability(const LinkGeometry & g, int anchor_id) const
{
  std::vector<std::pair<int, double>> w;
  weights(g, anchor_id, w);
  double p = 0.0;
  for (const auto & [i, wi] : w) {p += wi * recs_[i].p_drop;}
  if (g.n_obstacles >= 2) {p = p + (1.0 - p) * extra_drop_multi_;}
  return std::clamp(p, 0.0, 0.98);
}

UwbOutcome UwbChannelModel::sample(
  const LinkGeometry & g, UwbLinkState & st, std::mt19937_64 & rng, UwbMeasurement & out) const
{
  if (g.distance > max_range_) {return UwbOutcome::kOutOfRange;}
  if (g.thickness > max_material_) {return UwbOutcome::kTooMuchMaterial;}

  std::vector<std::pair<int, double>> w;
  weights(g, st.anchor_id, w);
  std::uniform_real_distribution<double> U(0.0, 1.0);
  std::normal_distribution<double> N(0.0, 1.0);

  // dependence and memory come from the condition with the largest weight
  const auto dom = std::max_element(w.begin(), w.end(),
      [](const auto & a, const auto & b) {return a.second < b.second;});
  const Record & rd = recs_[dom->first];

  // advance the copula state (also in slots that end up dropped, as the
  // channel keeps evolving)
  std::array<double, kUwbChannels> e{};
  for (auto & v : e) {v = N(rng);}
  std::array<double, kUwbChannels> zn{};
  if (!st.initialised) {
    st.delay_offset = delay_offset_ + delay_spread_ * N(rng);
  }
  const int warm = st.initialised ? 1 : 30;
  for (int step = 0; step < warm; ++step) {
    for (int i = 0; i < kUwbChannels; ++i) {
      double s = 0.0;
      for (int j = 0; j < kUwbChannels; ++j) {
        s += rd.A[i * 5 + j] * st.z[j] + rd.L[i * 5 + j] * e[j];
      }
      zn[i] = s;
    }
    st.z = zn;
    if (step + 1 < warm) {for (auto & v : e) {v = N(rng);}}
  }
  st.initialised = true;

  double p = 0.0;
  for (const auto & [i, wi] : w) {p += wi * recs_[i].p_drop;}
  if (g.n_obstacles >= 2) {p = p + (1.0 - p) * extra_drop_multi_;}
  if (U(rng) < std::clamp(p, 0.0, 0.98)) {return UwbOutcome::kDropped;}

  std::array<double, kUwbChannels> v{};
  for (int c = 0; c < kUwbChannels; ++c) {
    const double u = normalCdf(st.z[c]);
    double s = 0.0;
    for (const auto & [i, wi] : w) {s += wi * quantile(recs_[i], c, u);}
    v[c] = s;
  }
  out.range = std::max(0.0, g.distance + v[0] + st.delay_offset);
  out.rx = v[1];
  out.fp = v[2];
  out.quality = std::max(0.0, v[3]);
  out.delta = v[4];
  return UwbOutcome::kOk;
}

double UwbChannelModel::slotGap(int n, int index) const
{
  if (n <= 0) {return 0.1;}
  if (n < static_cast<int>(timing_.size()) && !timing_[n].empty()) {
    return timing_[n].at(static_cast<size_t>(index % n));
  }
  // more anchors than measured: keep the measured short slot gap and the
  // tag's idle gap from the largest measured network
  const auto & big = timing_.back();
  const int nb = static_cast<int>(big.size());
  if (index % n == n - 1) {return big.back();}
  return big.at(static_cast<size_t>(std::min(index % n, nb - 2)));
}

}  // namespace uwb_imu_sandbox
