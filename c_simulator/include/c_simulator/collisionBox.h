#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "c_simulator/settings.h"

using std::string, std::cout, std::endl;
typedef Eigen::Vector3d v3d;
typedef Eigen::Vector4d v4d;
typedef Eigen::VectorXd vXd;
typedef Eigen::Matrix3d m3d;
typedef Eigen::MatrixXd mXd;
typedef Eigen::Quaterniond quat;

class collisionBox
{
public:
    collisionBox(std::string name_,
                 double length_,
                 double width_,
                 double height_,
                 v3d baseCenter,
                 v3d baseOffset_ = v3d(0, 0, 0),
                 quat baseOrientation = quat(1, 0, 0, 0),
                 quat baseOrientationOffset_ = quat(1, 0, 0, 0));

    v3d getCenter();
    double getWidth();
    double getHeight();
    double getLength();
    m3d rotationMatrix();
    quat getOrientation();
    std::string getName();
    v3d getAxis(int index);
    void setCenter(v3d center_);
    v3d maxVertex(const v3d &axis);
    v3d minVertex(const v3d &axis);
    bool isInBox(const v3d &point);
    v3d moveInBox(const v3d &point);
    void updateLocation(const vXd &state);
    double maxProjection(const v3d &axis);
    double minProjection(const v3d &axis);
    void setOrientation(quat orientation_);

private:
    m3d rotM;
    v3d center;
    mXd vertices;
    double width;
    double height;
    double length;
    v3d baseOffset;
    std::string name;
    quat orientation;
    void setVertices();
    quat baseOrientationOffset;
};

struct collisionResult
{
    double depth;
    bool collided;
    v3d unitDirection;
    v3d collisionPoint;
    collisionResult();
};