#pragma once
#include "c_simulator/MarineDynamics.h"
#include <Eigen/Geometry>
#include <btBulletCollisionCommon.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

// Collision detection runs inside each Fossen step; no delayed force feedback.
class TaskContacts {
  public:
    using Pose = Eigen::Isometry3d;
    TaskContacts(const YAML::Node &, const std::string &, const Eigen::Vector3d &,
                 const std::function<Pose(std::string)> &);
    ~TaskContacts();
    void addBox(const std::string &, const Eigen::Vector3d &, const Pose &, bool vehicle = false);
    void setJaws(double left, double right);
    void setProp(const std::string &, const Pose &, bool attached);
    Eigen::VectorXd resolve(Eigen::VectorXd, const c_simulator::Matrix6d &, double friction = .4);

  private:
    struct Entry {
        std::string name;
        int kind;
        bool attached = false, supported = false;
        Pose local = Pose::Identity();
        std::unique_ptr<btTriangleMesh> triangles;
        std::unique_ptr<btCollisionShape> shape;
        btCollisionObject object;
    };
    struct Contact {
        Eigen::Vector3d point, normal;
        double depth;
    };
    btDefaultCollisionConfiguration config;
    btCollisionDispatcher dispatcher{&config};
    btDbvtBroadphase broadphase;
    btCollisionWorld world{&dispatcher, &broadphase, &config};
    std::vector<std::unique_ptr<Entry>> entries;
    Pose mount = Pose::Identity();
    double travel = 0;
    void addMesh(const std::string &, const std::string &, const Pose &, int);
    void insert(std::unique_ptr<Entry>);
    std::vector<Contact> contacts(const Eigen::VectorXd &);
};
