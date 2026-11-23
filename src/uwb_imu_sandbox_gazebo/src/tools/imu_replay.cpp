// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
//
// imu_replay: run the IMU error model offline on an exact motion profile.
//
//   imu_replay <imu_model.bin> <motion.csv> [seed] [reseed_period]
//
// motion.csv: header, then rows at the physics rate (e.g. 1 kHz):
//   t,vx,vy,vz,wx,wy,wz,qw,qx,qy,qz
// (world-frame linear velocity, body-frame angular velocity, body->world
// orientation).  Output: one CSV row per published IMU message.

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "uwb_imu_sandbox_gazebo/imu_model.hpp"

using namespace uwb_imu_sandbox;

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "usage: imu_replay <imu_model.bin> <motion.csv> [seed] [reseed_period]\n";
    return 2;
  }
  ImuOptions opt;
  if (argc > 4) {opt.bias_reseed_period = std::stod(argv[4]);}
  ImuErrorModel model(argv[1], opt, argc > 3 ? std::stoull(argv[3]) : 1);
  std::ifstream f(argv[2]);
  std::string line;
  std::getline(f, line);
  std::printf("t,ax,ay,az,gx,gy,gz,qw,qx,qy,qz,bax,bay,baz,bgx,bgy,bgz\n");
  while (std::getline(f, line)) {
    std::stringstream ss(line);
    std::vector<double> v;
    std::string c;
    while (std::getline(ss, c, ',')) {v.push_back(std::stod(c));}
    if (v.size() < 11) {continue;}
    ImuTruth tr;
    tr.world_lin_vel = {v[1], v[2], v[3]};
    tr.body_ang_vel = {v[4], v[5], v[6]};
    tr.orientation = {v[7], v[8], v[9], v[10]};
    ImuSample s;
    if (model.update(v[0], tr, s)) {
      const auto & ba = model.accelBias();
      const auto & bg = model.gyroBias();
      std::printf("%.4f,%.8f,%.8f,%.8f,%.9f,%.9f,%.9f,%.8f,%.8f,%.8f,%.8f,%.6f,%.6f,%.6f,%.8f,%.8f,%.8f\n",
        s.stamp, s.accel.x, s.accel.y, s.accel.z, s.gyro.x, s.gyro.y, s.gyro.z,
        s.orientation.w, s.orientation.x, s.orientation.y, s.orientation.z,
        ba.x, ba.y, ba.z, bg.x, bg.y, bg.z);
    }
  }
  return 0;
}
