#include "c_simulator/TaskContacts.h"
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
using V = Eigen::Vector3d;
using Q = Eigen::Quaterniond;
using Pose = TaskContacts::Pose;
namespace {
struct Fixture {
  YAML::Node cfg =
      YAML::LoadFile(std::string(TASK_SOURCE) + "/config/talos_tasks.yaml");
  std::unique_ptr<TaskContacts> world;
  std::vector<V> vertices;
  c_simulator::Matrix6d inverseMass = c_simulator::Matrix6d::Identity();
  Fixture() {
    cfg["claw"]["pose"] = std::vector<double>{0, 0, 0, 0, 0, 0};
    auto frame = [](std::string name) {
      Pose t = Pose::Identity();
      if (name != "table")
        t.translate(V(5, 5, 0));
      return t;
    };
    world = std::make_unique<TaskContacts>(
        cfg, std::string(TASK_SOURCE) + "/collision_files/tasks", V::Zero(),
        frame);
    inverseMass.diagonal() << 1. / 40, 1. / 40, 1. / 50, 1., 1., 1.;
    std::ifstream in(std::string(TASK_SOURCE) +
                     "/collision_files/tasks/claw_pad.obj");
    std::string line;
    while (std::getline(in, line)) {
      std::istringstream s(line);
      std::string kind;
      s >> kind;
      if (kind == "v") {
        V v;
        s >> v.x() >> v.y() >> v.z();
        vertices.push_back(v);
      }
    }
  }
  Eigen::VectorXd state(double z) {
    Eigen::VectorXd s = Eigen::VectorXd::Zero(13);
    s[2] = z;
    s[3] = 1;
    return s;
  }
  double bottom(const Eigen::VectorXd &s, double jaw) {
    Q q(s[3], s[4], s[5], s[6]);
    double low = 100;
    for (auto v : vertices) {
      v.y() += jaw;
      low = std::min(low, (s.head<3>() + q * v).z());
    }
    return low;
  }
  void step(Eigen::VectorXd &s) {
    Q q(s[3], s[4], s[5], s[6]);
    V w = s.segment<3>(10);
    s.head<3>() += q * s.segment<3>(7) * .002;
    if (w.norm() > 1e-10)
      q = q * Q(Eigen::AngleAxisd(w.norm() * .002, w.normalized()));
    q.normalize();
    s[3] = q.w();
    s.segment<3>(4) = q.vec();
    s = world->resolve(s, inverseMass);
  }
};
} // namespace
TEST(TaskContacts, OpenAndClosedClawStopsVehicleOnSolidTable) {
  for (double jaw : {0., .0694}) {
    Fixture f;
    f.world->setJaws(jaw, jaw);
    auto s = f.state(.08);
    s[9] = -.3;
    for (int i = 0; i < 1200; ++i) {
      s.segment<3>(7) +=
          Q(s[3], s[4], s[5], s[6]).conjugate() * V(0, 0, -1.) * .002;
      f.step(s);
      ASSERT_TRUE(s.allFinite());
      ASSERT_GT(f.bottom(s, jaw), -.002);
      ASSERT_LT(s.segment<6>(7).norm(), 2.);
    }
    EXPECT_LT(f.bottom(s, jaw), .004);
    s.segment<3>(7) = Q(s[3], s[4], s[5], s[6]).conjugate() * V(0, 0, .3);
    s.segment<3>(10).setZero();
    for (int i = 0; i < 150; ++i)
      f.step(s);
    EXPECT_GT(f.bottom(s, jaw), .05);
  }
}
TEST(TaskContacts, HeldObjectTransmitsContactToVehicle) {
  Fixture f;
  Pose carried = Pose::Identity();
  carried.translate(V(0, 0, -.04));
  f.world->setProp("pill", carried, true);
  auto s = f.state(.15);
  s[9] = -.2;
  for (int i = 0; i < 600; ++i) {
    s[9] -= .001;
    f.step(s);
    ASSERT_GT(s[2], .085);
  }
  EXPECT_GT(s[2], .089);
  EXPECT_LT(s[2], .10);
}
TEST(TaskContacts, SupportedPropCannotBePressedThroughTable) {
  Fixture f;
  f.world->setJaws(.0694, .0694); // Put the open pad above the offset prop.
  Pose prop = Pose::Identity();
  prop.translate(V(0, .085, .0508));
  f.world->setProp("pill", prop, false);
  auto s = f.state(.12);
  s[9] = -.2;
  for (int i = 0; i < 700; ++i) {
    s[9] -= .001;
    f.step(s);
    ASSERT_TRUE(s.allFinite());
  }
  EXPECT_GT(f.bottom(s, .0694), .035);
}
