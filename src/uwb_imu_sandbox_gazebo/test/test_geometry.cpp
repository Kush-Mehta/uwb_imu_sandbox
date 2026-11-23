// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>

#include "uwb_imu_sandbox_gazebo/geometry.hpp"

using namespace uwb_imu_sandbox;

namespace
{
Obstacle box(Vec3 c, Vec3 size)
{
  Obstacle o;
  o.type = ShapeType::kBox;
  o.size = size;
  o.pose.p = c;
  return o;
}
}  // namespace

TEST(Geometry, QuaternionRotate)
{
  // 90 deg about z
  Quat q{std::cos(M_PI / 4), 0, 0, std::sin(M_PI / 4)};
  Vec3 v = q.rotate({1, 0, 0});
  EXPECT_NEAR(v.x, 0, 1e-12);
  EXPECT_NEAR(v.y, 1, 1e-12);
  Vec3 w = q.inverseRotate(v);
  EXPECT_NEAR(w.x, 1, 1e-12);
  EXPECT_NEAR(w.y, 0, 1e-12);
}

TEST(Geometry, BoxChord)
{
  Obstacle o = box({0, 0, 0}, {2, 2, 2});
  double t0, t1;
  ASSERT_TRUE(o.chord({-2, 0, 0}, {2, 0, 0}, t0, t1));
  EXPECT_NEAR(t0, 0.25, 1e-12);
  EXPECT_NEAR(t1, 0.75, 1e-12);
  EXPECT_FALSE(o.chord({-2, 1.5, 0}, {2, 1.5, 0}, t0, t1));
  EXPECT_FALSE(o.chord({-3, 0, 0}, {-2, 0, 0}, t0, t1));
}

TEST(Geometry, RotatedBoxChord)
{
  Obstacle o = box({0, 0, 0}, {4, 0.2, 2});
  // rotate 90 deg about z: the thin side now faces x
  o.pose.q = {std::cos(M_PI / 4), 0, 0, std::sin(M_PI / 4)};
  double t0, t1;
  ASSERT_TRUE(o.chord({-1, 0, 0}, {1, 0, 0}, t0, t1));
  EXPECT_NEAR((t1 - t0) * 2.0, 0.2, 1e-9);
  ASSERT_TRUE(o.chord({0, -3, 0}, {0, 3, 0}, t0, t1));
  EXPECT_NEAR((t1 - t0) * 6.0, 4.0, 1e-9);
}

TEST(Geometry, CylinderAndSphere)
{
  Obstacle c;
  c.type = ShapeType::kCylinder;
  c.radius = 0.5;
  c.length = 2.0;
  double t0, t1;
  ASSERT_TRUE(c.chord({-1, 0, 0}, {1, 0, 0}, t0, t1));
  EXPECT_NEAR((t1 - t0) * 2.0, 1.0, 1e-9);
  EXPECT_FALSE(c.chord({-1, 0, 1.5}, {1, 0, 1.5}, t0, t1));
  EXPECT_NEAR(c.distance({2, 0, 0}), 1.5, 1e-12);
  Obstacle s;
  s.type = ShapeType::kSphere;
  s.radius = 1.0;
  ASSERT_TRUE(s.chord({0, -2, 0}, {0, 2, 0}, t0, t1));
  EXPECT_NEAR((t1 - t0) * 4.0, 2.0, 1e-9);
  EXPECT_NEAR(s.distance({0, 3, 0}), 2.0, 1e-12);
}

TEST(Geometry, PillarClearanceMatchesEdgeDistance)
{
  // pillar 2.5 m wide (x), 0.5 m thick (y), floor to ceiling; anchor 1.5 m
  // behind it, 1 m from its left end
  Obstacle p = box({0.25, 1.75, 1.5}, {2.5, 0.5, 5.0});
  const Vec3 anchor{0, 0, 1.0};
  const Vec3 tag{0, 6, 1.0};
  LinkGeometry g = analyseLink(anchor, tag, {p}, 0.0462);
  ASSERT_TRUE(g.blocked);
  EXPECT_EQ(g.n_obstacles, 1);
  EXPECT_NEAR(g.thickness, 0.5, 1e-6);
  // the path is 1.0 m from the left edge and 1.5 m from the right edge
  EXPECT_NEAR(g.clearance, 1.0, 2e-3);
  EXPECT_NEAR(g.d1, 1.75, 1e-6);
  EXPECT_NEAR(g.d2, 4.25, 1e-6);
  EXPECT_GT(g.nu, 3.0);
}

TEST(Geometry, ClearPathNearEdgeHasNegativeClearance)
{
  Obstacle p = box({0, 2, 1}, {1, 0.5, 2});
  const Vec3 anchor{0.8, 0, 1};
  const Vec3 tag{0.8, 4, 1};
  LinkGeometry g = analyseLink(anchor, tag, {p}, 0.0462);
  EXPECT_FALSE(g.blocked);
  EXPECT_NEAR(g.clearance, -0.3, 1e-3);
  EXPECT_LT(g.nu, 0.0);
  // far from everything: no transition at all
  LinkGeometry far = analyseLink({10, 0, 1}, {10, 4, 1}, {p}, 0.0462);
  EXPECT_FALSE(far.blocked);
  EXPECT_LE(far.nu, -10.0);
}

TEST(Geometry, MountedDeviceIgnoresItsWall)
{
  // anchor sitting on a wall surface must not be blocked by that wall
  Obstacle wall = box({0, -0.1, 1.5}, {6, 0.2, 3});
  LinkGeometry g = analyseLink({0, 0.0, 1.0}, {0, 5, 1}, {wall}, 0.0462);
  EXPECT_FALSE(g.blocked);
}

TEST(Geometry, TwoObstaclesAddUp)
{
  Obstacle a = box({0, 2, 1}, {2, 0.3, 2});
  Obstacle b = box({0, 4, 1}, {2, 0.4, 2});
  LinkGeometry g = analyseLink({0, 0, 1}, {0, 6, 1}, {a, b}, 0.0462);
  EXPECT_EQ(g.n_obstacles, 2);
  EXPECT_NEAR(g.thickness, 0.7, 1e-6);
}

TEST(Geometry, PathCannotSneakUnderTheFloor)
{
  // floor-standing pillar, path 0.5 m above the floor, nearest side edge 1.6 m away
  Obstacle p = box({0.0, 2.0, 1.5}, {3.2, 0.5, 3.0});
  LinkGeometry g = analyseLink({0, 0, 0.5}, {0, 5, 0.5}, {p}, 0.0462);
  ASSERT_TRUE(g.blocked);
  EXPECT_NEAR(g.clearance, 1.6, 2e-3);
  // with the floor rule switched off, going under (0.5 m) would look shorter
  LinkGeometry h = analyseLink({0, 0, 0.5}, {0, 5, 0.5}, {p}, 0.0462, 1.5, -1e9);
  EXPECT_NEAR(h.clearance, 0.5, 2e-3);
}
