// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "uwb_imu_sandbox_gazebo/sealed_model.hpp"

using namespace uwb_imu_sandbox;

TEST(SealedModel, RoundTrip)
{
  auto bytes = SealedModel::seal("TESTMAG1", 1234, {{1, {1.0, 2.0, 3.0}}, {5, {-4.5}}, {5, {7.0, 8.0}}});
  SealedModel m(bytes, "TESTMAG1");
  EXPECT_EQ(m.first(1).size(), 3u);
  EXPECT_DOUBLE_EQ(m.first(1)[2], 3.0);
  ASSERT_EQ(m.all(5).size(), 2u);
  EXPECT_DOUBLE_EQ(m.all(5)[1][1], 8.0);
  EXPECT_FALSE(m.has(9));
  EXPECT_THROW(m.first(9), std::runtime_error);
}

TEST(SealedModel, PayloadIsMasked)
{
  auto bytes = SealedModel::seal("TESTMAG1", 99, {{1, {1.0, 1.0, 1.0, 1.0}}});
  // the plain value 1.0 (0x3FF0000000000000) must not appear in the file
  const uint8_t pattern[8] = {0, 0, 0, 0, 0, 0, 0xF0, 0x3F};
  bool found = false;
  for (size_t i = 0; i + 8 <= bytes.size(); ++i) {
    if (std::equal(pattern, pattern + 8, bytes.begin() + i)) {found = true;}
  }
  EXPECT_FALSE(found);
}

TEST(SealedModel, RejectsCorruption)
{
  auto bytes = SealedModel::seal("TESTMAG1", 7, {{1, {1.0, 2.0}}});
  auto bad = bytes;
  bad.back() ^= 0x01;
  EXPECT_THROW(SealedModel(bad, "TESTMAG1"), std::runtime_error);
  EXPECT_THROW(SealedModel(bytes, "OTHERMAG"), std::runtime_error);
  bytes.resize(bytes.size() - 8);
  EXPECT_THROW(SealedModel(bytes, "TESTMAG1"), std::runtime_error);
  EXPECT_THROW(SealedModel(std::string("/nonexistent/file.bin"), "TESTMAG1"), std::runtime_error);
}

TEST(SealedModel, ShippedFilesLoad)
{
  EXPECT_NO_THROW(SealedModel(std::string(MODEL_DIR) + "/uwb_model.bin", kUwbMagic));
  EXPECT_NO_THROW(SealedModel(std::string(MODEL_DIR) + "/imu_model.bin", kImuMagic));
}
