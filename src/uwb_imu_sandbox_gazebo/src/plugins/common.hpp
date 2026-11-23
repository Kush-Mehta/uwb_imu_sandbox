// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Small helpers shared by the Gazebo plugins.
#pragma once

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>

#include "uwb_imu_sandbox_gazebo/geometry.hpp"

namespace uwb_imu_sandbox
{

inline Vec3 toVec(const gz::math::Vector3d & v) {return {v.X(), v.Y(), v.Z()};}
inline Quat toQuat(const gz::math::Quaterniond & q) {return {q.W(), q.X(), q.Y(), q.Z()};}
inline Pose toPose(const gz::math::Pose3d & p) {return {toVec(p.Pos()), toQuat(p.Rot())};}

template<typename T>
T sdfGet(const std::shared_ptr<const sdf::Element> & sdf, const std::string & key, const T & def)
{
  if (!sdf || !sdf->HasElement(key)) {return def;}
  return sdf->Get<T>(key, def).first;
}

/// Resolve a model file given as an absolute path, a path relative to the
/// world file, or a model:// URI searched in GZ_SIM_RESOURCE_PATH.
inline std::string resolveModelFile(
  const std::string & spec, const std::shared_ptr<const sdf::Element> & sdf)
{
  namespace fs = std::filesystem;
  std::vector<std::string> candidates;
  std::string rel = spec;
  const std::string prefix = "model://";
  const bool is_uri = spec.rfind(prefix, 0) == 0;
  if (is_uri) {
    rel = spec.substr(prefix.size());
  } else if (fs::path(spec).is_absolute()) {
    candidates.push_back(spec);
  } else if (sdf && !sdf->FilePath().empty()) {
    candidates.push_back((fs::path(sdf->FilePath()).parent_path() / spec).string());
  }
  for (const char * var : {"GZ_SIM_RESOURCE_PATH", "SDF_PATH"}) {
    const char * env = std::getenv(var);
    if (!env) {continue;}
    std::stringstream ss(env);
    std::string dir;
    while (std::getline(ss, dir, ':')) {
      if (!dir.empty()) {candidates.push_back((fs::path(dir) / rel).string());}
    }
  }
  for (const auto & c : candidates) {
    std::error_code ec;
    if (fs::is_regular_file(c, ec)) {return c;}
  }
  return {};
}

inline uint64_t seedFrom(int64_t seed)
{
  if (seed >= 0) {return static_cast<uint64_t>(seed);}
  std::random_device rd;
  return (static_cast<uint64_t>(rd()) << 32) ^ rd();
}

}  // namespace uwb_imu_sandbox
