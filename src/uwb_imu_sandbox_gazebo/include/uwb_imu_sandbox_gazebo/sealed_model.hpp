// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Reader for the sealed sensor model files (uwb_model.bin, imu_model.bin).
//
// A sealed file holds the fitted sensor tables as a list of tagged float64
// arrays.  The tables were produced by a private builder from real
// measurements: the signal fields are already in a secret, unitless scale and
// every table comes from a secret bootstrap resample, so the file does not
// contain the measured data or its exact statistics.
#pragma once

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace uwb_imu_sandbox
{

class SealedModel
{
public:
  /// Load and unmask a sealed file. Throws std::runtime_error on any problem
  /// (missing file, wrong magic, version mismatch, failed checksum).
  SealedModel(const std::string & path, const std::string & expected_magic);

  /// Build from an in-memory buffer (used by the unit tests).
  SealedModel(const std::vector<uint8_t> & bytes, const std::string & expected_magic);

  /// All arrays stored under a tag, in file order.
  const std::vector<std::vector<double>> & all(uint32_t tag) const;

  /// The first array stored under a tag. Throws if the tag is missing.
  const std::vector<double> & first(uint32_t tag) const;

  bool has(uint32_t tag) const {return records_.count(tag) > 0;}

  /// Helper for tests and the private builder: produce a sealed buffer.
  static std::vector<uint8_t> seal(
    const std::string & magic, uint64_t salt,
    const std::vector<std::pair<uint32_t, std::vector<double>>> & records);

private:
  void parse(const std::vector<uint8_t> & bytes, const std::string & expected_magic);
  std::map<uint32_t, std::vector<std::vector<double>>> records_;
};

namespace tags
{
// shared
constexpr uint32_t PROBS = 1;
// UWB
constexpr uint32_t NREC = 2;
constexpr uint32_t REC_HDR = 3;
constexpr uint32_t REC_Q = 4;
constexpr uint32_t REC_A = 5;
constexpr uint32_t REC_LCHOL = 6;
constexpr uint32_t TIMING = 7;
constexpr uint32_t TRANSITION = 8;
constexpr uint32_t KERNEL = 9;
constexpr uint32_t MISC = 10;
// IMU
constexpr uint32_t IMU_TIMING_ACC = 20;
constexpr uint32_t IMU_TIMING_GYR = 21;
constexpr uint32_t IMU_TIMING_QUAT = 22;
constexpr uint32_t IMU_TIMING_PUB = 23;
constexpr uint32_t IMU_SPEED_BINS = 24;
constexpr uint32_t IMU_ACC_STD = 25;
constexpr uint32_t IMU_GYR_STD = 26;
constexpr uint32_t IMU_ORI_STD = 27;
constexpr uint32_t IMU_ACC_SHAPE = 28;
constexpr uint32_t IMU_GYR_SHAPE = 29;
constexpr uint32_t IMU_AR = 30;
constexpr uint32_t IMU_BIAS = 31;
constexpr uint32_t IMU_QUANT = 32;
constexpr uint32_t IMU_BIAS_WALK = 33;
}  // namespace tags

constexpr const char * kUwbMagic = "UISBUWB2";
constexpr const char * kImuMagic = "UISBIMU2";
constexpr uint32_t kFormatVersion = 2;

}  // namespace uwb_imu_sandbox
