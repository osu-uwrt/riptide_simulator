#include "c_simulator/collisionBox.h"
#include <gtest/gtest.h>

TEST(CollisionBox, PreservesSubmeterGeometryAndContainment) {
  collisionBox box("robot", .35, .83, .55, v3d::Zero());
  EXPECT_DOUBLE_EQ(box.getLength(), .35);
  EXPECT_DOUBLE_EQ(box.getWidth(), .83);
  EXPECT_DOUBLE_EQ(box.getHeight(), .55);
  EXPECT_TRUE(box.isInBox({.17, .41, .27}));
  EXPECT_FALSE(box.isInBox({.18, 0, 0}));
  EXPECT_FALSE(box.isInBox({0, -.42, 0}));
  EXPECT_FALSE(box.isInBox({0, 0, .28}));
}
TEST(CollisionBox, OffsetUsesParentRotationAndOrientationUsesBothRotations) {
  const quat parent(Eigen::AngleAxisd(M_PI / 2, v3d::UnitZ()));
  const quat local(Eigen::AngleAxisd(.4, v3d::UnitY()));
  collisionBox box("offset", .2, .4, .6, {1, 2, 3}, {.3, 0, 0}, parent, local);
  EXPECT_TRUE(box.getCenter().isApprox(v3d(1, 2.3, 3), 1e-12));
  EXPECT_TRUE(box.getOrientation().isApprox(parent * local, 1e-12));
  EXPECT_NEAR(box.maxProjection(box.getAxis(0)) -
                  box.minProjection(box.getAxis(0)),
              .2, 1e-12);
}
