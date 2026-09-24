#pragma once
#include <glm/glm.hpp>
#include <cmath>
namespace pool {
// Camera-plane grab pan: a scene point at the focal depth follows the cursor.
inline glm::vec3 orbitPan(const glm::mat4 &view, const glm::mat4 &projection, float distance, float height,
                          glm::vec2 delta) {
    const auto camera = glm::inverse(view);
    const float metresPerPixel = 2.f * distance / (height * projection[1][1]);
    return (-glm::vec3(camera[0]) * delta.x + glm::vec3(camera[1]) * delta.y) * metresPerPixel;
}
inline bool depthPoint(const glm::mat4 &viewProjection, glm::vec2 uv, float depth, glm::vec3 &point) {
    if (!std::isfinite(depth) || depth < 0 || depth >= 1 || uv.x < 0 || uv.x > 1 || uv.y < 0 || uv.y > 1)
        return false;
    const auto world = glm::inverse(viewProjection) * glm::vec4(uv.x * 2 - 1, 1 - uv.y * 2, depth * 2 - 1, 1);
    if (std::abs(world.w) < 1e-6f)
        return false;
    point = glm::vec3(world) / world.w;
    return std::isfinite(point.x + point.y + point.z);
}
// Empty background has no depth sample. Use the plane through the current
// orbit target, facing the camera, so F remains useful anywhere in the viewport.
inline bool focusPlanePoint(const glm::mat4 &vp, glm::vec3 eye, glm::vec3 target, glm::vec2 uv, glm::vec3 &point) {
    glm::vec3 nearPoint;
    if (!depthPoint(vp, uv, 0, nearPoint))
        return false;
    const auto ray = nearPoint - eye, normal = target - eye;
    const float denominator = glm::dot(ray, normal);
    if (std::abs(denominator) < 1e-7f)
        return false;
    const float t = glm::dot(normal, normal) / denominator;
    if (t <= 0)
        return false;
    point = eye + ray * t;
    return std::isfinite(point.x + point.y + point.z);
}
// Hit-testing for the visible screen-space TF/marker overlays. A hit focuses
// the overlay's semantic origin, not an unrelated background depth sample.
class OverlayFocusPicker {
    glm::mat4 vp;
    glm::vec2 size, mouse;
    float best = 8.f;
    bool found = false;
    glm::vec3 selected{};
    bool project(glm::vec3 p, glm::vec2 &out) const {
        const auto clip = vp * glm::vec4(p, 1);
        if (clip.w <= 0 || std::abs(clip.z) > clip.w)
            return false;
        out = {(clip.x / clip.w * .5f + .5f) * size.x, (.5f - clip.y / clip.w * .5f) * size.y};
        return true;
    }

  public:
    OverlayFocusPicker(glm::mat4 projectionView, glm::vec2 viewportSize, glm::vec2 cursor)
        : vp(projectionView), size(viewportSize), mouse(cursor) {}
    void segment(glm::vec3 a, glm::vec3 b, glm::vec3 origin) {
        glm::vec2 x, y;
        if (!project(a, x) || !project(b, y))
            return;
        const auto d = y - x;
        const float t = glm::dot(d, d) > 1e-6f ? glm::clamp(glm::dot(mouse - x, d) / glm::dot(d, d), 0.f, 1.f) : 0;
        const float distance = glm::length(mouse - x - t * d);
        if (distance < best) {
            best = distance;
            found = true;
            selected = origin;
        }
    }
    void quad(const glm::mat4 &pose, glm::vec2 halfSize) {
        glm::vec2 p[4];
        const glm::vec2 corner[] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
        for (int i = 0; i < 4; ++i) {
            auto c = corner[i] * halfSize;
            if (!project(glm::vec3(pose * glm::vec4(c, 0, 1)), p[i]))
                return;
        }
        bool positive = false, negative = false;
        float area = 0;
        for (int i = 0; i < 4; ++i) {
            auto a = p[(i + 1) % 4] - p[i], b = mouse - p[i];
            float cross = a.x * b.y - a.y * b.x;
            positive |= cross > 0;
            negative |= cross < 0;
            area += p[i].x * p[(i + 1) % 4].y - p[i].y * p[(i + 1) % 4].x;
        }
        if (!(positive && negative) && std::abs(area) > 1) {
            found = true;
            best = 0;
            selected = glm::vec3(pose[3]);
        }
    }
    bool result(glm::vec3 &point) const {
        if (found)
            point = selected;
        return found;
    }
};
} // namespace pool
