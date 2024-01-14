#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <bits/stdc++.h>
#include <chrono>

using std::cout, std::endl;

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
                 v3d baseOffset_,
                 quat baseOrientation,
                 quat baseOrientationOffset_);

    void updateLocation(const vXd &state);
    v3d getCenter();
    std::string getName();
    v3d getAxis(int index);
    m3d rotationMatrix();
    mXd getVertices();
    void setCenter(v3d center_);
    void setOrientation(quat orientation_);
    double maxProjection(const v3d &axis);
    double minProjection(const v3d &axis);
    v3d maxVertex(const v3d &axis);
    v3d minVertex(const v3d &axis);
    bool isInBox(const v3d &point);
    v3d moveInBox(const v3d &point);

private:
    std::string name;
    double length;
    double width;
    double height;
    v3d center;
    v3d baseOffset;
    quat orientation;
    quat baseOrientationOffset;
    mXd vertices;

    void setVertices();
};

struct collisionResult
{
    double depth;
    bool collided;
    v3d unitDirection;
    v3d collisionPoint;

    collisionResult();
};