#include <gtest/gtest.h>

#include "c_simulator/MarineDynamics.h"

using c_simulator::MarineDynamics;
using c_simulator::Matrix6d;
using c_simulator::Vector6d;

TEST(MarineDynamics, CoriolisIsSkewSymmetric) {
  Matrix6d mass = Matrix6d::Random();
  mass = mass.transpose() * mass + Matrix6d::Identity();
  const Matrix6d coriolis = MarineDynamics::coriolis(mass, Vector6d::Random());
  EXPECT_TRUE(coriolis.isApprox(-coriolis.transpose(), 1e-12));
}

TEST(MarineDynamics, CoriolisDoesNoWork) {
  Matrix6d mass = Matrix6d::Random();
  mass = mass.transpose() * mass + Matrix6d::Identity();
  const Vector6d velocity = Vector6d::Random();
  EXPECT_NEAR(velocity.dot(MarineDynamics::coriolis(mass, velocity) * velocity),
              0.0, 1e-12);
}

TEST(MarineDynamics, AddedMassReducesAcceleration) {
  MarineDynamics without_added_mass;
  MarineDynamics with_added_mass;
  const Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
  Matrix6d added_mass = Matrix6d::Zero();
  added_mass(0, 0) = 10.0;
  without_added_mass.configure(10.0, inertia, Matrix6d::Zero());
  with_added_mass.configure(10.0, inertia, added_mass);

  Vector6d wrench = Vector6d::Zero();
  wrench.x() = 20.0;
  EXPECT_NEAR(without_added_mass
                  .acceleration(Vector6d::Zero(), Vector6d::Zero(), wrench)
                  .x(),
              2.0, 1e-12);
  EXPECT_NEAR(
      with_added_mass.acceleration(Vector6d::Zero(), Vector6d::Zero(), wrench)
          .x(),
      1.0, 1e-12);
}

TEST(MarineDynamics, RejectsNonPhysicalMass) {
  MarineDynamics dynamics;
  Matrix6d added_mass = Matrix6d::Zero();
  added_mass(0, 0) = -11.0;
  EXPECT_THROW(
      dynamics.configure(10.0, Eigen::Matrix3d::Identity(), added_mass),
      std::invalid_argument);
}

#include "c_simulator/ThrusterDynamics.h"
#include <limits>
using c_simulator::State13d;
using c_simulator::ThrusterDynamics;
using c_simulator::ThrusterParameters;

namespace {
State13d restingState() {
  State13d x = State13d::Zero();
  x[2] = -10;
  x[3] = 1;
  return x;
}
MarineDynamics neutralModel() {
  MarineDynamics d;
  d.configure(10., Eigen::Matrix3d::Identity(), Matrix6d::Identity() * 2.);
  d.configureHydrostatics(1000, .01, Eigen::Vector3d::Zero(), {.2, .4, .2});
  return d;
}
} // namespace
TEST(MarineDynamics, CoordinateCurrentDerivativeHasNoRigidBodyInertiaTerm) {
  MarineDynamics d;
  d.configure(10., Eigen::Matrix3d::Identity(), Matrix6d::Zero());
  const Vector6d v = Vector6d::Random(), r = Vector6d::Random(),
                 dc = Vector6d::Random();
  EXPECT_TRUE(d.acceleration(v, r, Vector6d::Zero(), dc)
                  .isApprox(d.acceleration(v, r, Vector6d::Zero()), 1e-12));
}
TEST(MarineDynamics, AddedMassCurrentDerivativeCoefficient) {
  MarineDynamics d;
  Matrix6d added = Matrix6d::Zero();
  added(0, 0) = 5;
  d.configure(10., Eigen::Matrix3d::Identity(), added);
  Vector6d dc = Vector6d::Zero();
  dc[0] = 3;
  EXPECT_NEAR(d.acceleration(Vector6d::Zero(), Vector6d::Zero(),
                             Vector6d::Zero(), dc)[0],
              1., 1e-12);
}
TEST(MarineDynamics, CoriolisMatchesIndependentMomentumCrossProducts) {
  Matrix6d a = Matrix6d::Random();
  a = (a.transpose() * a).eval();
  Vector6d v = Vector6d::Random(), p = a * v, expected;
  expected.head<3>() = v.tail<3>().cross(p.head<3>());
  expected.tail<3>() =
      v.head<3>().cross(p.head<3>()) + v.tail<3>().cross(p.tail<3>());
  EXPECT_TRUE((MarineDynamics::coriolis(a, v) * v).isApprox(expected, 1e-12));
}
TEST(MarineDynamics, NeutralBodyFollowsUniformAcceleratingFluid) {
  auto d = neutralModel();
  const Eigen::Vector3d acceleration(.3, -.2, .1);
  const auto dx = d.derivative(restingState(), Vector6d::Zero(),
                               Eigen::Vector3d::Zero(), acceleration);
  EXPECT_TRUE(dx.segment<3>(7).isApprox(acceleration, 1e-12));
  EXPECT_LT(dx.tail<3>().norm(), 1e-12);
}
TEST(MarineDynamics,
     RotatingBodyDriftingWithUniformWaterHasNoSpuriousInertialForce) {
  auto d = neutralModel();
  auto x = restingState();
  x[7] = 1;
  x[12] = .5;
  const auto dx = d.derivative(x, Vector6d::Zero(), {1, 0, 0});
  Eigen::Vector3d inertial =
      dx.segment<3>(7) + x.tail<3>().cross(x.segment<3>(7));
  EXPECT_LT(inertial.norm(), 1e-12);
}
TEST(MarineDynamics, DampingAtOffsetCenterCannotAddEnergy) {
  auto d = neutralModel();
  Matrix6d a = Matrix6d::Random();
  a = (a.transpose() * a).eval();
  d.configureDamping(a, Vector6d::Ones() * 3., {.2, -.1, .3});
  for (int i = 0; i < 1000; ++i) {
    Vector6d v = Vector6d::Random();
    EXPECT_LE(v.dot(d.dampingWrench(v)), 1e-12);
  }
  EXPECT_DOUBLE_EQ(d.dampingWrench(Vector6d::Zero()).norm(), 0.);
}
TEST(MarineDynamics, BuoyancyIncludesOrientationAndMovingWetCentroid) {
  auto d = neutralModel();
  d.configureHydrostatics(1000, .01, {0, 0, .05}, {.2, .4, .2});
  Eigen::Quaterniond roll(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX()));
  EXPECT_LT(d.restoringWrench({0, 0, -2}, roll)[3], 0);
  EXPECT_NEAR(d.restoringWrench({0, 0, -2}, Eigen::Quaterniond::Identity())
                  .head<3>()
                  .norm(),
              0, 1e-12);
  EXPECT_NEAR(d.submergedFraction({0, 0, -.05}, Eigen::Quaterniond::Identity()),
              .5, 1e-12);
  EXPECT_DOUBLE_EQ(d.submergedFraction({0, 0, 2}, roll), 0.);
  EXPECT_NEAR(d.restoringWrench({0, 0, 2}, Eigen::Quaterniond::Identity())[2],
              -98.0665, 1e-10);
  Eigen::Vector3d wet;
  d.submergedFraction({0, 0, -.05}, Eigen::Quaterniond::Identity(), &wet);
  EXPECT_NEAR(wet.z(), .05 - 3 * .2 / 8., 1e-12); // half-ellipsoid centroid
  EXPECT_NEAR(
      d.submergedFraction({0, 0, 0}, Eigen::Quaterniond(Eigen::AngleAxisd(
                                         M_PI / 2, Eigen::Vector3d::UnitY()))),
      .5, 1e-12);
}
TEST(MarineDynamics, UnforcedCoupledMotionConservesEnergy) {
  auto d = neutralModel();
  Matrix6d added = Matrix6d::Random();
  added = (added.transpose() * added).eval();
  d.configure(10., Eigen::Vector3d(1, 1.3, 1.5).asDiagonal(), added);
  auto x = restingState();
  x.tail<6>() << .4, -.2, .1, .2, .15, -.1;
  const double energy = .5 * x.tail<6>().dot(d.mass() * x.tail<6>());
  for (int i = 0; i < 10000; ++i)
    x = d.step(x, Vector6d::Zero(), .002);
  EXPECT_NEAR(.5 * x.tail<6>().dot(d.mass() * x.tail<6>()), energy, 1e-9);
  EXPECT_NEAR(x.segment<4>(3).norm(), 1., 1e-12);
}
TEST(MarineDynamics, DragDecaysEnergyAndStepRefinementConverges) {
  auto d = neutralModel();
  d.configureDamping(Matrix6d::Identity(), Vector6d::Ones() * 8);
  auto x = restingState();
  x.tail<6>() << .5, .2, -.1, .2, .1, .3;
  double energy = .5 * x.tail<6>().dot(d.mass() * x.tail<6>());
  auto coarse = x, fine = x;
  for (int i = 0; i < 500; ++i) {
    coarse = d.step(coarse, Vector6d::Zero(), .004);
    fine = d.step(d.step(fine, Vector6d::Zero(), .002), Vector6d::Zero(), .002);
    const double next = .5 * coarse.tail<6>().dot(d.mass() * coarse.tail<6>());
    EXPECT_LE(next, energy + 1e-12);
    energy = next;
  }
  EXPECT_LT((coarse - fine).norm(), 1e-7);
}
TEST(MarineDynamics, RejectsNaNsNegativeAddedMassAndImpossibleInertia) {
  MarineDynamics d;
  Matrix6d added = Matrix6d::Zero();
  added(0, 0) = -.01;
  EXPECT_THROW(d.configure(10., Eigen::Matrix3d::Identity(), added),
               std::invalid_argument);
  EXPECT_THROW(
      d.configure(10., Eigen::Vector3d(1, 1, 3).asDiagonal(), Matrix6d::Zero()),
      std::invalid_argument);
  EXPECT_THROW(d.configure(std::numeric_limits<double>::quiet_NaN(),
                           Eigen::Matrix3d::Identity(), Matrix6d::Zero()),
               std::invalid_argument);
  EXPECT_THROW(d.configureDamping(Matrix6d::Identity(), -Vector6d::Ones()),
               std::invalid_argument);
}
TEST(Thrusters, ExactDelayAndLagIndependentOfStepPartition) {
  ThrusterParameters p;
  p.delay = .1;
  p.rise = .2;
  p.slew = 0;
  ThrusterDynamics a, b;
  a.configure({p}, 0);
  b.configure({p}, 0);
  Eigen::VectorXd cmd = Eigen::VectorXd::Constant(1, 10);
  a.command(cmd);
  b.command(cmd);
  a.advance(.1);
  EXPECT_NEAR(a.forces()[0], 0., 1e-12);
  a.advance(.1);
  for (int i = 0; i < 200; ++i)
    b.advance(.001);
  EXPECT_NEAR(a.forces()[0], 10 * (1 - exp(-.1 / .2)), 1e-12);
  EXPECT_NEAR(a.forces()[0], b.forces()[0], 1e-11);
}
TEST(Thrusters, AsymmetricLimitsFailuresSlewAndWatchdog) {
  ThrusterParameters p;
  p.delay = 0;
  p.rise = 0;
  p.fall = 0;
  p.slew = 10;
  p.forwardLimit = 20;
  p.reverseLimit = 10;
  p.efficiency = .5;
  ThrusterDynamics a;
  a.configure({p}, .2);
  a.command(Eigen::VectorXd::Constant(1, 100));
  a.advance(.1);
  EXPECT_NEAR(a.forces()[0], 1, 1e-12);
  a.advance(.2);
  EXPECT_NEAR(a.forces()[0], 1, 1e-12); // watchdog at .2, coast to zero
  a.advance(.2);
  EXPECT_NEAR(a.forces()[0], 0, 1e-12);
  p.slew = 0;
  a.configure({p}, 0);
  a.command(Eigen::VectorXd::Constant(1, -100));
  a.advance(.01);
  EXPECT_NEAR(a.forces()[0], -5, 1e-12);
  p.efficiency = 0;
  a.configure({p}, 0);
  a.command(Eigen::VectorXd::Constant(1, 100));
  a.advance(.1);
  EXPECT_DOUBLE_EQ(a.forces()[0], 0);
}
TEST(Thrusters, KillClearsDelayedCommandsAndRejectsNonfiniteInput) {
  ThrusterDynamics a;
  a.configure({ThrusterParameters{}}, 0);
  a.command(Eigen::VectorXd::Constant(1, 20));
  a.stop();
  a.advance(1);
  EXPECT_DOUBLE_EQ(a.forces()[0], 0);
  EXPECT_THROW(a.command(Eigen::VectorXd::Constant(1, NAN)),
               std::invalid_argument);
}
