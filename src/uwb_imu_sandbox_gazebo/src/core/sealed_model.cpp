// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
#include "uwb_imu_sandbox_gazebo/sealed_model.hpp"

#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>

namespace uwb_imu_sandbox
{
namespace
{
// Compiled-in part of the mask seed. Together with the per-file salt it
// drives the mask stream. This is obfuscation of the tables, not encryption.
constexpr uint64_t kMaskConstant = 0x6A09E667F3BCC909ULL;
constexpr size_t kMagicLen = 8;
constexpr size_t kHeaderLen = kMagicLen + 4 + 8 + 8 + 8;

uint64_t splitmix64(uint64_t & state)
{
  state += 0x9E3779B97F4A7C15ULL;
  uint64_t z = state;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

uint64_t fnv1a64(const uint8_t * data, size_t n)
{
  uint64_t h = 0xCBF29CE484222325ULL;
  for (size_t i = 0; i < n; ++i) {
    h ^= data[i];
    h *= 0x100000001B3ULL;
  }
  return h;
}

template<typename T>
T readLE(const uint8_t * p)
{
  T v;
  std::memcpy(&v, p, sizeof(T));   // all supported targets are little endian
  return v;
}

template<typename T>
void writeLE(std::vector<uint8_t> & out, T v)
{
  const auto * p = reinterpret_cast<const uint8_t *>(&v);
  out.insert(out.end(), p, p + sizeof(T));
}
}  // namespace

SealedModel::SealedModel(const std::string & path, const std::string & expected_magic)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("cannot open sensor model file: " + path);
  }
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  parse(bytes, expected_magic);
}

SealedModel::SealedModel(const std::vector<uint8_t> & bytes, const std::string & expected_magic)
{
  parse(bytes, expected_magic);
}

void SealedModel::parse(const std::vector<uint8_t> & bytes, const std::string & expected_magic)
{
  if (bytes.size() < kHeaderLen) {
    throw std::runtime_error("sensor model file is too short");
  }
  if (std::string(bytes.begin(), bytes.begin() + kMagicLen) != expected_magic) {
    throw std::runtime_error(
            "sensor model file has the wrong type (expected " + expected_magic + ")");
  }
  const uint8_t * p = bytes.data() + kMagicLen;
  const auto version = readLE<uint32_t>(p);
  const auto salt = readLE<uint64_t>(p + 4);
  const auto check = readLE<uint64_t>(p + 12);
  const auto plen = readLE<uint64_t>(p + 20);
  if (version != kFormatVersion) {
    std::ostringstream ss;
    ss << "sensor model format " << version << " is not supported (need " << kFormatVersion << ")";
    throw std::runtime_error(ss.str());
  }
  if (plen % 8 != 0 || bytes.size() != kHeaderLen + plen) {
    throw std::runtime_error("sensor model file is truncated or corrupt");
  }
  std::vector<uint8_t> payload(bytes.begin() + kHeaderLen, bytes.end());
  uint64_t state = salt ^ kMaskConstant;
  for (size_t i = 0; i < payload.size(); i += 8) {
    uint64_t w = readLE<uint64_t>(payload.data() + i) ^ splitmix64(state);
    std::memcpy(payload.data() + i, &w, 8);
  }
  // the checksum covers the unpadded payload; find its true length by walking the records
  size_t pos = 0;
  std::map<uint32_t, std::vector<std::vector<double>>> recs;
  while (pos + 8 <= payload.size()) {
    const auto tag = readLE<uint32_t>(payload.data() + pos);
    const auto n = readLE<uint32_t>(payload.data() + pos + 4);
    if (tag == 0 && n == 0) {
      break;   // padding
    }
    if (pos + 8 + static_cast<size_t>(n) * 8 > payload.size()) {
      throw std::runtime_error("sensor model file is corrupt (record overruns file)");
    }
    std::vector<double> v(n);
    std::memcpy(v.data(), payload.data() + pos + 8, static_cast<size_t>(n) * 8);
    recs[tag].push_back(std::move(v));
    pos += 8 + static_cast<size_t>(n) * 8;
  }
  if (fnv1a64(payload.data(), pos) != check) {
    throw std::runtime_error("sensor model file failed its integrity check");
  }
  records_ = std::move(recs);
}

const std::vector<std::vector<double>> & SealedModel::all(uint32_t tag) const
{
  auto it = records_.find(tag);
  if (it == records_.end()) {
    throw std::runtime_error("sensor model file is missing table " + std::to_string(tag));
  }
  return it->second;
}

const std::vector<double> & SealedModel::first(uint32_t tag) const
{
  return all(tag).front();
}

std::vector<uint8_t> SealedModel::seal(
  const std::string & magic, uint64_t salt,
  const std::vector<std::pair<uint32_t, std::vector<double>>> & records)
{
  std::vector<uint8_t> payload;
  for (const auto & r : records) {
    writeLE<uint32_t>(payload, r.first);
    writeLE<uint32_t>(payload, static_cast<uint32_t>(r.second.size()));
    for (double d : r.second) {
      writeLE<double>(payload, d);
    }
  }
  const uint64_t check = fnv1a64(payload.data(), payload.size());
  while (payload.size() % 8) {
    payload.push_back(0);
  }
  uint64_t state = salt ^ kMaskConstant;
  for (size_t i = 0; i < payload.size(); i += 8) {
    uint64_t w = readLE<uint64_t>(payload.data() + i) ^ splitmix64(state);
    std::memcpy(payload.data() + i, &w, 8);
  }
  std::vector<uint8_t> out(magic.begin(), magic.end());
  writeLE<uint32_t>(out, kFormatVersion);
  writeLE<uint64_t>(out, salt);
  writeLE<uint64_t>(out, check);
  writeLE<uint64_t>(out, static_cast<uint64_t>(payload.size()));
  out.insert(out.end(), payload.begin(), payload.end());
  return out;
}

}  // namespace uwb_imu_sandbox
