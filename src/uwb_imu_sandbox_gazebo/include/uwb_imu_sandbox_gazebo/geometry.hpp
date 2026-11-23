// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Radio-path geometry between a UWB anchor and a tag: which obstacles block
// the straight path, how deep the path sits behind the obstacle edge, and the
// Fresnel-Kirchhoff diffraction parameter of the worst obstacle.
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace uwb_imu_sandbox
{

struct Vec3
{
  double x{0}, y{0}, z{0};
  Vec3() = default;
  Vec3(double a, double b, double c)
  : x(a), y(b), z(c) {}
  Vec3 operator+(const Vec3 & o) const {return {x + o.x, y + o.y, z + o.z};}
  Vec3 operator-(const Vec3 & o) const {return {x - o.x, y - o.y, z - o.z};}
  Vec3 operator*(double s) const {return {x * s, y * s, z * s};}
  double dot(const Vec3 & o) const {return x * o.x + y * o.y + z * o.z;}
  Vec3 cross(const Vec3 & o) const
  {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
  double norm() const {return std::sqrt(dot(*this));}
  Vec3 normalized() const {double n = norm(); return n > 0 ? (*this) * (1.0 / n) : Vec3();}
};

/// Unit quaternion (w, x, y, z).
struct Quat
{
  double w{1}, x{0}, y{0}, z{0};
  Vec3 rotate(const Vec3 & v) const;
  Vec3 inverseRotate(const Vec3 & v) const;
};

struct Pose
{
  Vec3 p;
  Quat q;
  Vec3 toLocal(const Vec3 & w) const {return q.inverseRotate(w - p);}
};

enum class ShapeType { kBox, kCylinder, kSphere };

/// A convex collision shape in the world. Box: size = full edge lengths.
/// Cylinder: radius, length along local z. Sphere: radius.
struct Obstacle
{
  ShapeType type{ShapeType::kBox};
  Vec3 size;
  double radius{0};
  double length{0};
  Pose pose;
  uint64_t owner{0};   ///< top-level model id (used to skip the devices' own models)

  /// Radius of a sphere around pose.p that contains the whole shape.
  double boundingRadius() const;
  /// Chord of the segment a->b inside the shape as fractions t0 <= t1 of the
  /// segment. Returns false if the segment misses the shape.
  bool chord(const Vec3 & a, const Vec3 & b, double & t0, double & t1) const;
  /// Distance from a point to the shape surface (0 inside).
  double distance(const Vec3 & p) const;
  bool contains(const Vec3 & p, double margin = 0.0) const;
};

/// Everything the UWB channel model needs about one anchor-tag path.
struct LinkGeometry
{
  double distance{0};       ///< straight-line distance [m]
  bool blocked{false};      ///< at least one obstacle cuts the straight path
  int n_obstacles{0};       ///< number of obstacles cutting the path
  double thickness{0};      ///< total length of the path inside obstacles [m]
  double clearance{0};      ///< worst obstacle: depth behind its edge [m] (>0 blocked, <0 clear by)
  double d1{0}, d2{0};      ///< anchor->obstacle and obstacle->tag distances for that obstacle [m]
  double nu{-10};           ///< Fresnel-Kirchhoff diffraction parameter of that obstacle
  Vec3 anchor_pos, tag_pos;  ///< the two antennas (world frame)
};

/// Analyse the straight path from anchor to tag.
/// @param wavelength  carrier wavelength [m] (DW1000 channel 5: 0.0462 m)
/// @param near_range  obstacles closer than this to a clear path still count
///                    for the LOS->NLOS transition [m]
/// @param floor_z     height of the floor [m]: a radio path cannot get round an
///                    obstacle by passing under the floor
LinkGeometry analyseLink(
  const Vec3 & anchor, const Vec3 & tag,
  const std::vector<Obstacle> & obstacles,
  double wavelength, double near_range = 1.5, double floor_z = 0.0);

/// Fresnel-Kirchhoff parameter for an edge at clearance h with distances d1, d2.
double fresnelNu(double h, double d1, double d2, double wavelength);

}  // namespace uwb_imu_sandbox
