#include "c_simulator/collisionBox.h"

collisionBox::collisionBox(std::string name_,
                           double length_,
                           double width_,
                           double height_,
                           v3d baseCenter,
                           v3d baseOffset_,
                           quat baseOrientation,
                           quat baseOrientationOffset_)
{
    name = name_;
    length = abs(length_);
    width = abs(width_);
    height = abs(height_);
    baseOffset = baseOffset_;
    baseOrientationOffset = baseOrientationOffset_.normalized();
    orientation = (baseOrientation.normalized() * baseOrientationOffset).normalized();
    center = baseCenter + orientation * baseOffset;
    setVertices();
}

void collisionBox::updateLocation(const vXd &state)
{
    // Get quaternion from state
    quat q;
    q.w() = state[3];
    q.vec() = state.segment(4, 3);
    q.normalize();

    // Update position and orientation
    center = state.segment(0, 3) + q * baseOffset;
    orientation = (q * baseOrientationOffset).normalized();
    // Also update vertice location
    setVertices();
}
v3d collisionBox::getCenter()
{
    return center;
}
v3d collisionBox::getAxis(int index)
{
    return orientation.toRotationMatrix().col(index);
}
std::string collisionBox::getName()
{
    return name;
}
m3d collisionBox::rotationMatrix()
{
    return orientation.toRotationMatrix();
}
double collisionBox::maxProjection(const v3d &axis)
{
    return (vertices.transpose() * axis).maxCoeff();
}
double collisionBox::minProjection(const v3d &axis)
{
    return (vertices.transpose() * axis).minCoeff();
}

v3d collisionBox::maxVertex(const v3d &axis)
{
    // No vertices in box, pick the furthest on the axis
    int maxIndex;
    (vertices.transpose() * axis).maxCoeff(&maxIndex);
    return vertices.col(maxIndex);
}
v3d collisionBox::minVertex(const v3d &axis)
{
    // No vertices in box, pick the shortest on the axis
    int minIndex;
    (vertices.transpose() * axis).minCoeff(&minIndex);
    return vertices.col(minIndex);
}

bool collisionBox::isInBox(const v3d &point)
{
    // Transform from world cordinates to box cordinates
    v3d point_body = orientation.conjugate() * (point - center);
    // Return if it is inside or not
    return abs(point_body[0]) <= length / 2 &&
           abs(point_body[1]) <= width / 2 &&
           abs(point_body[2]) <= height / 2;
}

v3d collisionBox::moveInBox(const v3d &point)
{
    // Transform from world cordinates to box cordinates
    v3d point_body = orientation.conjugate() * (point - center);
    // Move vector to within box bounds
    point_body(0) = std::max(-length / 2, std::min(length / 2, point_body(0)));
    point_body(1) = std::max(-width / 2, std::min(width / 2, point_body(1)));
    point_body(2) = std::max(-height / 2, std::min(height / 2, point_body(2)));

    // Transform back to world cordinates
    return orientation * point_body + center;
}
void collisionBox::setVertices()
{
    vertices.resize(3, 8);

    // Store vertex point relative to box axes
    vertices.col(0) = v3d(-length / 2, -width / 2, -height / 2);
    vertices.col(1) = v3d(-length / 2, -width / 2, height / 2);
    vertices.col(2) = v3d(-length / 2, width / 2, -height / 2);
    vertices.col(3) = v3d(-length / 2, width / 2, height / 2);
    vertices.col(4) = v3d(length / 2, -width / 2, -height / 2);
    vertices.col(5) = v3d(length / 2, -width / 2, height / 2);
    vertices.col(6) = v3d(length / 2, width / 2, -height / 2);
    vertices.col(7) = v3d(length / 2, width / 2, height / 2);

    // Convert from box cordinates to world cordinates
    vertices = orientation.toRotationMatrix() * vertices + center.replicate(1, 8);
}

collisionResult::collisionResult()
{
    depth = 0;
    collided = false;
    unitDirection = v3d(1, 0, 0);
    collisionPoint = v3d(0, 0, 0);
};
