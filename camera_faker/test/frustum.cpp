#include "pool_viewer/frustum.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <random>
#include <stdexcept>

namespace {
void check(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}
pool::Bounds box(glm::vec3 center, glm::vec3 half = glm::vec3(.1f)) {
  pool::Bounds b;
  b.include(center - half);
  b.include(center + half);
  return b;
}
// Independent reference: transform all eight corners and test OpenGL's six
// homogeneous clipping inequalities, without dividing by w (which may be <= 0).
bool reference(const glm::mat4 &matrix, const pool::Bounds &b) {
  bool outside[6] = {true, true, true, true, true, true};
  for (int corner = 0; corner < 8; ++corner) {
    glm::vec4 p(1);
    for (int axis = 0; axis < 3; ++axis)
      p[axis] = (corner & (1 << axis)) ? b.max[axis] : b.min[axis];
    const auto clip = matrix * p;
    for (int axis = 0; axis < 3; ++axis) {
      outside[2 * axis] &= clip[axis] < -clip.w;
      outside[2 * axis + 1] &= clip[axis] > clip.w;
    }
  }
  for (bool rejected : outside)
    if (rejected)
      return false;
  return true;
}
} // namespace

int main() {
  try {
    const auto projection = glm::perspective(glm::radians(90.f), 1.f, 1.f, 10.f);
    const pool::Frustum f(projection);
    check(f.intersects(box({0, 0, -3})), "Object in view");
    for (const auto center : {glm::vec3(-5, 0, -3), glm::vec3(5, 0, -3),
                              glm::vec3(0, -5, -3), glm::vec3(0, 5, -3),
                              glm::vec3(0, 0, -.5f), glm::vec3(0, 0, -11),
                              glm::vec3(0, 0, 3)})
      check(!f.intersects(box(center)), "Reject outside each plane/behind camera");
    check(f.intersects(box({3, 0, -3})), "Keep partially visible mesh");
    check(f.intersects(box({0, 0, -1})), "Keep mesh crossing near plane");
    check(f.intersects(box({0, 0, -5}, glm::vec3(20))), "Keep enclosing pool geometry");
    check(f.intersects(box({0, 0, -1}, glm::vec3(0))), "Keep point on plane");
    check(!f.intersects(pool::Bounds{}), "Empty mesh is invisible");
    const auto local = box({0, 0, 0});
    check(pool::Frustum(projection * glm::translate(glm::mat4(1), {0, 0, -3}))
              .intersects(local), "Moving instance enters view");
    check(!pool::Frustum(projection * glm::translate(glm::mat4(1), {8, 0, -3}))
               .intersects(local), "Same instance leaves view");
    const auto mirror = glm::scale(glm::mat4(1), glm::vec3(1, 1, -1));
    check(pool::Frustum(projection * mirror).intersects(box({0, 0, 3})),
          "Reflections use their own view");
    const auto light = glm::ortho(-8.f, 8.f, -8.f, 8.f, .1f, 20.f);
    check(pool::Frustum(light).intersects(box({5, 0, -3})),
          "Off-camera shadow caster remains in light volume");
    check(!pool::Frustum(light).intersects(box({9, 0, -3})), "Outside light volume");

    std::mt19937 random(42);
    std::uniform_real_distribution<float> position(-20, 20), angle(-3, 3), scale(.1f, 4);
    const auto offAxis = glm::frustum(-.3f, .7f, -.2f, .5f, .5f, 40.f);
    for (int i = 0; i < 10000; ++i) {
      const auto model = glm::scale(glm::rotate(
          glm::translate(glm::mat4(1), {position(random), position(random), position(random)}),
          angle(random), glm::normalize(glm::vec3(1, 2, 3))),
          {scale(random), scale(random), i % 2 ? -scale(random) : scale(random)});
      const auto matrix = (i % 3 == 0 ? light : i % 3 == 1 ? offAxis : projection) * model;
      const auto bounds = box({.2f, -.3f, .4f}, {.4f, .7f, .2f});
      // Conservative tolerance may retain a borderline box but must never
      // reject one accepted by the independent clip-space reference.
      check(!reference(matrix, bounds) || pool::Frustum(matrix).intersects(bounds),
            "Transformed bounds incorrectly culled");
    }
    std::cout << "Frustum regression passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
