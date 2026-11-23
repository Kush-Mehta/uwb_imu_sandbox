// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
//
// uwb_replay: run the UWB tag model offline along a given tag trajectory in a
// given world, without starting Gazebo.  Anchors and obstacles are read from
// the world SDF exactly as the plugin sees them (static box/cylinder/sphere
// collisions, and models carrying a UwbDevice anchor plugin).
//
//   uwb_replay <world.sdf> <uwb_model.bin> <trajectory.csv> [seed] [max_range] [nlos_cell_size] [antenna_delay_spread]
//
// trajectory.csv: header line, then rows "t,x,y,z" (tag antenna, world frame,
// time in seconds, increasing).  Output (stdout) is one CSV row per slot.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/ParserConfig.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>
#include <sdf/World.hh>

#include "uwb_imu_sandbox_gazebo/uwb_model.hpp"

using namespace uwb_imu_sandbox;

namespace
{
Vec3 V(const gz::math::Vector3d & v) {return {v.X(), v.Y(), v.Z()};}
Pose P(const gz::math::Pose3d & p)
{
  return {V(p.Pos()), {p.Rot().W(), p.Rot().X(), p.Rot().Y(), p.Rot().Z()}};
}
gz::math::Pose3d resolve(const sdf::SemanticPose & sp)
{
  gz::math::Pose3d p;
  auto errs = sp.Resolve(p);
  return p;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "usage: uwb_replay <world.sdf> <uwb_model.bin> <trajectory.csv> [seed] [max_range] [nlos_cell_size] [antenna_delay_spread]\n";
    return 2;
  }
  // resolve model:// URIs the way Gazebo does (GZ_SIM_RESOURCE_PATH)
  sdf::ParserConfig config;
  config.SetFindCallback([](const std::string & uri) -> std::string {
      std::string rel = uri;
      const std::string prefix = "model://";
      if (rel.rfind(prefix, 0) == 0) {rel = rel.substr(prefix.size());}
      const char * env = std::getenv("GZ_SIM_RESOURCE_PATH");
      std::stringstream ss(env ? env : "");
      std::string dir;
      while (std::getline(ss, dir, ':')) {
        const auto p = std::filesystem::path(dir) / rel;
        if (!dir.empty() && std::filesystem::exists(p)) {return p.string();}
      }
      return uri;
    });
  sdf::Root root;
  auto errors = root.Load(argv[1], config);
  if (!errors.empty() || root.WorldCount() == 0) {
    for (auto & e : errors) {std::cerr << e << "\n";}
    std::cerr << "cannot load world " << argv[1] << "\n";
    return 1;
  }
  const sdf::World * world = root.WorldByIndex(0);
  std::vector<Obstacle> obstacles;
  std::map<int, Vec3> anchors;
  for (uint64_t mi = 0; mi < world->ModelCount(); ++mi) {
    const sdf::Model * m = world->ModelByIndex(mi);
    const gz::math::Pose3d mp = resolve(m->SemanticPose());
    // anchor plugins
    for (auto el = m->Element()->FindElement("plugin"); el; el = el->GetNextElement("plugin")) {
      const std::string fn = el->GetAttribute("filename")->GetAsString();
      if (fn.find("UwbDevice") == std::string::npos) {continue;}
      if (!el->HasElement("role") || el->Get<std::string>("role") != "anchor") {continue;}
      const int id = el->Get<int>("id");
      gz::math::Pose3d off;
      if (el->HasElement("antenna_offset")) {off = el->Get<gz::math::Pose3d>("antenna_offset");}
      const sdf::Link * l = m->LinkByIndex(0);
      const gz::math::Pose3d lp = l ? resolve(l->SemanticPose()) : gz::math::Pose3d::Zero;
      anchors[id] = V((mp * lp * off).Pos());
    }
    for (uint64_t li = 0; li < m->LinkCount(); ++li) {
      const sdf::Link * l = m->LinkByIndex(li);
      const gz::math::Pose3d lp = resolve(l->SemanticPose());
      for (uint64_t ci = 0; ci < l->CollisionCount(); ++ci) {
        const sdf::Collision * c = l->CollisionByIndex(ci);
        const sdf::Geometry * g = c->Geom();
        Obstacle o;
        if (g->Type() == sdf::GeometryType::BOX) {
          o.type = ShapeType::kBox;
          o.size = V(g->BoxShape()->Size());
        } else if (g->Type() == sdf::GeometryType::CYLINDER) {
          o.type = ShapeType::kCylinder;
          o.radius = g->CylinderShape()->Radius();
          o.length = g->CylinderShape()->Length();
        } else if (g->Type() == sdf::GeometryType::SPHERE) {
          o.type = ShapeType::kSphere;
          o.radius = g->SphereShape()->Radius();
        } else {
          continue;
        }
        o.pose = P(mp * lp * resolve(c->SemanticPose()));
        o.owner = mi + 1;
        // the robot carrying the tag is not part of the static scene
        if (!m->Static()) {continue;}
        obstacles.push_back(o);
      }
    }
  }
  if (anchors.empty()) {
    std::cerr << "no UwbDevice anchors found in the world\n";
    return 1;
  }

  // trajectory
  std::ifstream tf(argv[3]);
  std::string line;
  std::getline(tf, line);
  std::vector<double> T;
  std::vector<Vec3> X;
  while (std::getline(tf, line)) {
    std::stringstream ss(line);
    std::string a, b, c, d;
    if (!std::getline(ss, a, ',') || !std::getline(ss, b, ',') ||
      !std::getline(ss, c, ',') || !std::getline(ss, d, ',')) {continue;}
    T.push_back(std::stod(a));
    X.push_back({std::stod(b), std::stod(c), std::stod(d)});
  }
  if (T.size() < 2) {
    std::cerr << "trajectory needs at least two rows\n";
    return 1;
  }
  UwbChannelModel model(argv[2]);
  if (argc > 5) {model.setMaxRange(std::stod(argv[5]));}
  if (argc > 6) {model.setSpatialCell(std::stod(argv[6]));}
  if (argc > 7) {model.setDelaySpread(std::stod(argv[7]));}
  std::mt19937_64 rng(argc > 4 ? std::stoull(argv[4]) : 1);
  std::normal_distribution<double> jit(0.0, model.timingJitter());
  std::map<int, UwbLinkState> state;

  std::printf("t,anchor,net,true_range,outcome,range,rx,fp,quality,delta,blocked,obstacles,clearance,nu,nlos_weight\n");
  double t = T.front();
  size_t k = 0;
  int slot = 0;
  std::vector<int> net;
  static const char * names[] = {"ok", "dropped", "out_of_range", "too_much_material"};
  while (t <= T.back()) {
    while (k + 1 < T.size() && T[k + 1] < t) {++k;}
    const double a = (k + 1 < T.size()) ? std::clamp((t - T[k]) / (T[k + 1] - T[k]), 0.0, 1.0) : 0.0;
    const Vec3 tag = X[k] + (X[std::min(k + 1, X.size() - 1)] - X[k]) * a;
    if (slot == 0) {
      net.clear();
      for (const auto & [id, p] : anchors) {
        if ((p - tag).norm() <= model.maxRange()) {net.push_back(id);}
      }
      if (net.empty()) {t += model.slotGap(1, 0); continue;}
    }
    const int id = net[slot];
    const LinkGeometry g = analyseLink(anchors[id], tag, obstacles, model.wavelength());
    UwbMeasurement m;
    state[id].anchor_id = id;
    const UwbOutcome o = model.sample(g, state[id], rng, m);
    std::printf("%.4f,%d,%zu,%.5f,%s,%.5f,%.4f,%.4f,%.4f,%.4f,%d,%d,%.4f,%.3f,%.3f\n",
      t, id, net.size(), g.distance, names[static_cast<int>(o)],
      o == UwbOutcome::kOk ? m.range : -1.0, m.rx, m.fp, m.quality, m.delta,
      g.blocked ? 1 : 0, g.n_obstacles, g.clearance, g.nu, model.nlosWeight(g));
    t += std::max(0.002, model.slotGap(static_cast<int>(net.size()), slot) + jit(rng));
    slot = (slot + 1) % static_cast<int>(net.size());
  }
  return 0;
}
