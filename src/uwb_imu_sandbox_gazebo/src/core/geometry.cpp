// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
#include "uwb_imu_sandbox_gazebo/geometry.hpp"

#include <algorithm>
#include <limits>

namespace uwb_imu_sandbox
{

Vec3 Quat::rotate(const Vec3 & v) const
{
  const Vec3 u(x, y, z);
  const Vec3 t = u.cross(v) * 2.0;
  return v + t * w + u.cross(t);
}

Vec3 Quat::inverseRotate(const Vec3 & v) const
{
  const Vec3 u(-x, -y, -z);
  const Vec3 t = u.cross(v) * 2.0;
  return v + t * w + u.cross(t);
}

double Obstacle::boundingRadius() const
{
  switch (type) {
    case ShapeType::kBox: return 0.5 * size.norm();
    case ShapeType::kCylinder: return std::sqrt(radius * radius + 0.25 * length * length);
    case ShapeType::kSphere: return radius;
  }
  return 0.0;
}

namespace
{
bool slab(double a, double d, double half, double & t0, double & t1)
{
  if (std::abs(d) < 1e-12) {
    return std::abs(a) <= half;
  }
  double te = (-half - a) / d;
  double tx = (half - a) / d;
  if (te > tx) {std::swap(te, tx);}
  t0 = std::max(t0, te);
  t1 = std::min(t1, tx);
  return t0 <= t1;
}

bool quadratic(double A, double B, double C, double & r0, double & r1)
{
  if (std::abs(A) < 1e-14) {
    if (C > 0) {return false;}
    r0 = -std::numeric_limits<double>::infinity();
    r1 = std::numeric_limits<double>::infinity();
    return true;
  }
  const double disc = B * B - 4 * A * C;
  if (disc < 0) {return false;}
  const double s = std::sqrt(disc);
  r0 = (-B - s) / (2 * A);
  r1 = (-B + s) / (2 * A);
  return true;
}
}  // namespace

bool Obstacle::chord(const Vec3 & aw, const Vec3 & bw, double & t0, double & t1) const
{
  const Vec3 a = pose.toLocal(aw);
  const Vec3 d = pose.toLocal(bw) - a;
  t0 = 0.0;
  t1 = 1.0;
  switch (type) {
    case ShapeType::kBox:
      return slab(a.x, d.x, 0.5 * size.x, t0, t1) &&
             slab(a.y, d.y, 0.5 * size.y, t0, t1) &&
             slab(a.z, d.z, 0.5 * size.z, t0, t1);
    case ShapeType::kCylinder: {
        if (!slab(a.z, d.z, 0.5 * length, t0, t1)) {return false;}
        double r0, r1;
        if (!quadratic(d.x * d.x + d.y * d.y, 2 * (a.x * d.x + a.y * d.y),
          a.x * a.x + a.y * a.y - radius * radius, r0, r1)) {return false;}
        t0 = std::max(t0, r0);
        t1 = std::min(t1, r1);
        return t0 <= t1;
      }
    case ShapeType::kSphere: {
        double r0, r1;
        if (!quadratic(d.dot(d), 2 * a.dot(d), a.dot(a) - radius * radius, r0, r1)) {return false;}
        t0 = std::max(t0, r0);
        t1 = std::min(t1, r1);
        return t0 <= t1;
      }
  }
  return false;
}

double Obstacle::distance(const Vec3 & pw) const
{
  const Vec3 q = pose.toLocal(pw);
  switch (type) {
    case ShapeType::kBox: {
        const double dx = std::max(std::abs(q.x) - 0.5 * size.x, 0.0);
        const double dy = std::max(std::abs(q.y) - 0.5 * size.y, 0.0);
        const double dz = std::max(std::abs(q.z) - 0.5 * size.z, 0.0);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
      }
    case ShapeType::kCylinder: {
        const double dr = std::max(std::hypot(q.x, q.y) - radius, 0.0);
        const double dz = std::max(std::abs(q.z) - 0.5 * length, 0.0);
        return std::hypot(dr, dz);
      }
    case ShapeType::kSphere:
      return std::max(q.norm() - radius, 0.0);
  }
  return 0.0;
}

bool Obstacle::contains(const Vec3 & p, double margin) const
{
  return distance(p) <= margin;
}

double fresnelNu(double h, double d1, double d2, double wavelength)
{
  d1 = std::max(d1, 1e-3);
  d2 = std::max(d2, 1e-3);
  return h * std::sqrt(2.0 * (d1 + d2) / (wavelength * d1 * d2));
}

namespace
{
constexpr double kMinChord = 1e-3;

bool cuts(const Obstacle & o, const Vec3 & a, const Vec3 & b, double len)
{
  double t0, t1;
  return o.chord(a, b, t0, t1) && (t1 - t0) * len > kMinChord;
}

double pointSegmentDistance(const Vec3 & p, const Vec3 & a, const Vec3 & b)
{
  const Vec3 ab = b - a;
  const double L2 = ab.dot(ab);
  double t = L2 > 0 ? (p - a).dot(ab) / L2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  return (a + ab * t - p).norm();
}
}  // namespace

LinkGeometry analyseLink(
  const Vec3 & anchor, const Vec3 & tag,
  const std::vector<Obstacle> & obstacles,
  double wavelength, double near_range, double floor_z)
{
  LinkGeometry g;
  g.anchor_pos = anchor;
  g.tag_pos = tag;
  const Vec3 ab = tag - anchor;
  g.distance = ab.norm();
  if (g.distance < 1e-6) {
    return g;
  }
  const Vec3 u = ab * (1.0 / g.distance);
  // two unit vectors perpendicular to the path
  Vec3 ref = std::abs(u.z) < 0.9 ? Vec3(0, 0, 1) : Vec3(1, 0, 0);
  const Vec3 e1 = u.cross(ref).normalized();
  const Vec3 e2 = u.cross(e1).normalized();

  double best_blocked_nu = -std::numeric_limits<double>::infinity();
  double best_clear_nu = -std::numeric_limits<double>::infinity();
  LinkGeometry clear_best;

  for (const auto & o : obstacles) {
    // devices mounted on (or inside) an obstacle do not see it as a blocker
    if (o.contains(anchor, 0.02) || o.contains(tag, 0.02)) {
      continue;
    }
    const double rb = o.boundingRadius();
    if (pointSegmentDistance(o.pose.p, anchor, tag) > rb + near_range) {
      continue;
    }
    double t0, t1;
    if (o.chord(anchor, tag, t0, t1) && (t1 - t0) * g.distance > kMinChord) {
      g.blocked = true;
      g.n_obstacles += 1;
      g.thickness += (t1 - t0) * g.distance;
      const Vec3 m = anchor + ab * (0.5 * (t0 + t1));
      const double d1 = (m - anchor).norm();
      const double d2 = (tag - m).norm();
      // smallest sideways shift of the path that clears this obstacle
      const double smax = 2.0 * rb + 0.1;
      double h = std::numeric_limits<double>::infinity();
      constexpr int kDirs = 24;
      for (int k = 0; k < kDirs; ++k) {
        const double ang = 2.0 * M_PI * k / kDirs;
        const Vec3 e = e1 * std::cos(ang) + e2 * std::sin(ang);
        // a shifted path that dips under the floor does not count as clear
        auto blockedAt = [&](double s) {
            const Vec3 a2 = anchor + e * s, b2 = tag + e * s;
            return std::min(a2.z, b2.z) < floor_z || cuts(o, a2, b2, g.distance);
          };
        if (blockedAt(smax)) {
          continue;
        }
        double lo = 0.0, hi = smax;
        for (int it = 0; it < 30; ++it) {
          const double mid = 0.5 * (lo + hi);
          if (blockedAt(mid)) {lo = mid;} else {hi = mid;}
        }
        h = std::min(h, hi);
      }
      if (!std::isfinite(h)) {h = smax;}
      const double nu = fresnelNu(h, d1, d2, wavelength);
      if (nu > best_blocked_nu) {
        best_blocked_nu = nu;
        g.clearance = h;
        g.d1 = d1;
        g.d2 = d2;
        g.nu = nu;
      }
    } else {
      // clear path: distance to the shape along the segment is convex in t
      double lo = 0.0, hi = 1.0;
      for (int it = 0; it < 80; ++it) {
        const double m1 = lo + (hi - lo) / 3.0, m2 = hi - (hi - lo) / 3.0;
        if (o.distance(anchor + ab * m1) < o.distance(anchor + ab * m2)) {hi = m2;} else {lo = m1;}
      }
      const double t = 0.5 * (lo + hi);
      const Vec3 c = anchor + ab * t;
      const double dist = o.distance(c);
      if (dist < near_range) {
        const double d1 = std::max((c - anchor).norm(), 0.05);
        const double d2 = std::max((tag - c).norm(), 0.05);
        const double nu = fresnelNu(-dist, d1, d2, wavelength);
        if (nu > best_clear_nu) {
          best_clear_nu = nu;
          clear_best.clearance = -dist;
          clear_best.d1 = d1;
          clear_best.d2 = d2;
          clear_best.nu = nu;
        }
      }
    }
  }
  if (!g.blocked && std::isfinite(best_clear_nu)) {
    g.clearance = clear_best.clearance;
    g.d1 = clear_best.d1;
    g.d2 = clear_best.d2;
    g.nu = clear_best.nu;
  }
  return g;
}

}  // namespace uwb_imu_sandbox
