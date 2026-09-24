#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>

namespace pool {
// RPY uses Rz(yaw) * Ry(pitch) * Rx(roll), matching the RViz command fields.
inline glm::mat4 rpyPose(const glm::vec3 &position, const glm::vec3 &angles) {
    return glm::translate(glm::mat4(1), position) * glm::mat4_cast(glm::quat(angles));
}
inline glm::vec3 rpyAxis(const glm::vec3 &angles, int axis) {
    const auto yaw = glm::rotate(glm::mat4(1), angles.z, glm::vec3(0, 0, 1));
    if (axis == 0)
        return glm::vec3(yaw * glm::rotate(glm::mat4(1), angles.y, glm::vec3(0, 1, 0)) * glm::vec4(1, 0, 0, 0));
    if (axis == 1)
        return glm::vec3(yaw * glm::vec4(0, 1, 0, 0));
    return {0, 0, 1};
}
// An in-plane reference perpendicular to each RPY rotation axis.
inline glm::vec3 rpyReference(const glm::vec3 &angles, int axis) {
    const auto rotation = glm::mat3_cast(glm::quat(angles));
    if (axis == 0)
        return rotation[1];
    if (axis == 1)
        return rotation[0];
    return {std::cos(angles.z), std::sin(angles.z), 0};
}
struct Ray {
    glm::vec3 origin, direction;
};
inline float segmentDistance(const glm::vec2 &point, const glm::vec2 &a, const glm::vec2 &b, float &fraction) {
    const auto delta = b - a;
    const auto squared = glm::dot(delta, delta);
    fraction = squared > 1e-6f ? glm::clamp(glm::dot(point - a, delta) / squared, 0.f, 1.f) : 0.f;
    return glm::length(point - (a + fraction * delta));
}
inline Ray screenRay(const glm::mat4 &viewProjection, const glm::vec2 &pixel, const glm::vec2 &size) {
    const auto inverse = glm::inverse(viewProjection);
    const glm::vec2 ndc(pixel.x / size.x * 2 - 1, 1 - pixel.y / size.y * 2);
    auto near = inverse * glm::vec4(ndc, -1, 1), far = inverse * glm::vec4(ndc, 1, 1);
    return {glm::vec3(near) / near.w, glm::normalize(glm::vec3(far) / far.w - glm::vec3(near) / near.w)};
}
inline bool planeHit(const Ray &ray, const glm::vec3 &origin, const glm::vec3 &normal, glm::vec3 &point) {
    const float denominator = glm::dot(ray.direction, normal);
    if (std::abs(denominator) < 1e-4f)
        return false;
    const float distance = glm::dot(origin - ray.origin, normal) / denominator;
    if (distance < 0 || !std::isfinite(distance))
        return false;
    point = ray.origin + ray.direction * distance;
    return true;
}
inline bool axisHit(const Ray &ray, const glm::vec3 &origin, const glm::vec3 &axis, float &position) {
    const float dot = glm::dot(ray.direction, axis), denominator = 1 - dot * dot;
    if (denominator < 1e-4f)
        return false;
    const auto offset = ray.origin - origin;
    position = (glm::dot(offset, axis) - dot * glm::dot(offset, ray.direction)) / denominator;
    return std::isfinite(position);
}
} // namespace pool
