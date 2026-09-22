#pragma once
#include <array>
#include <glm/glm.hpp>
#include <limits>

namespace pool {
struct Bounds {
    glm::vec3 min{std::numeric_limits<float>::max()};
    glm::vec3 max{std::numeric_limits<float>::lowest()};

    void include(const glm::vec3 &p) {
        min = glm::min(min, p);
        max = glm::max(max, p);
    }
};

// OpenGL clip volume (-w <= x,y,z <= w), expressed in mesh coordinates.
// Extract once per object from projection * view * model: cached local bounds
// then work for moving, rotated, scaled and reflected objects without inverses.
class Frustum {
  public:
    explicit Frustum(const glm::mat4 &clipFromLocal) {
        const auto rows = glm::transpose(clipFromLocal);
        for (int axis = 0; axis < 3; ++axis) {
            planes[2 * axis] = rows[3] + rows[axis];
            planes[2 * axis + 1] = rows[3] - rows[axis];
        }
    }

    bool intersects(const Bounds &bounds) const {
        if (glm::any(glm::greaterThan(bounds.min, bounds.max)))
            return false;
        for (const auto &plane : planes) {
            const glm::vec3 normal(plane);
            // The corner furthest inside this plane must be outside to reject the
            // entire mesh. Intersecting/enclosing bounds must remain visible.
            const glm::vec3 support(normal.x >= 0 ? bounds.max.x : bounds.min.x,
                                    normal.y >= 0 ? bounds.max.y : bounds.min.y,
                                    normal.z >= 0 ? bounds.max.z : bounds.min.z);
            const float tolerance = 1e-5f * (glm::dot(glm::abs(normal), glm::abs(support)) + glm::abs(plane.w) + 1.f);
            if (glm::dot(normal, support) + plane.w < -tolerance)
                return false;
        }
        return true;
    }

  private:
    std::array<glm::vec4, 6> planes;
};
} // namespace pool
