#include "pool_viewer/viewer_input.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <cassert>
#include <limits>

namespace {
glm::vec3 screen(const glm::mat4 &vp, glm::vec3 point) {
    auto clip = vp * glm::vec4(point, 1);
    auto ndc = glm::vec3(clip) / clip.w;
    return {(ndc.x + 1) * .5f, (1 - ndc.y) * .5f, (ndc.z + 1) * .5f};
}
} // namespace
int main() {
    const glm::vec3 target(2, -3, -.8), eye(5, -7, 3), up(0, 0, 1);
    const auto view = glm::lookAt(eye, target, up);
    // Unequal dimensions catch accidentally using horizontal FOV for vertical motion.
    const float width = 940, height = 700;
    const auto projection = glm::perspective(glm::radians(53.f), width / height, .05f, 100.f);
    const glm::vec2 drag(34, -21);
    auto translation = pool::orbitPan(view, projection, glm::distance(eye, target), height, drag);
    auto moved = glm::lookAt(eye + translation, target + translation, up);
    auto before = screen(projection * view, target), after = screen(projection * moved, target);
    assert(glm::length(glm::vec2(after - before) * glm::vec2(width, height) - drag) < .001f);
    // Pan scales with distance, maintaining the same grab behavior after zooming.
    assert(glm::length(pool::orbitPan(view, projection, glm::distance(eye, target) * 2, height, drag) -
                       translation * 2.f) < .0001f);
    // F can focus an off-center rendered surface, including under a rotated camera.
    const glm::vec3 surface(2.2, -2.6, -1.4);
    auto pixel = screen(projection * view, surface);
    glm::vec3 picked;
    assert(pool::depthPoint(projection * view, pixel, pixel.z, picked));
    assert(glm::distance(surface, picked) < .0002f);
    assert(!pool::depthPoint(projection * view, {.5, .5}, 1, picked));
    assert(!pool::depthPoint(projection * view, {1.1, .5}, .5, picked));
    assert(!pool::depthPoint(projection * view, {.5, .5}, -.1, picked));
    assert(!pool::depthPoint(projection * view, {.5, .5}, std::numeric_limits<float>::quiet_NaN(), picked));
    const glm::mat4 overlayVP = glm::perspective(glm::radians(53.f), 4.f / 3.f, .05f, 100.f) *
                                glm::lookAt(glm::vec3(0, 0, 5), glm::vec3(0), glm::vec3(0, 1, 0));
    auto axisPixel = screen(overlayVP, {.5, 0, 0});
    pool::OverlayFocusPicker axisPick(overlayVP, {800, 600}, glm::vec2(axisPixel) * glm::vec2(800, 600));
    axisPick.segment({0, 0, 0}, {1, 0, 0}, {0, 0, 0});
    assert(axisPick.result(picked) && glm::length(picked) < .001f);
    pool::OverlayFocusPicker markerPick(overlayVP, {800, 600}, {440, 280});
    markerPick.quad(glm::mat4(1), {1, 1});
    assert(markerPick.result(picked) && glm::length(picked) < .001f);
    pool::OverlayFocusPicker miss(overlayVP, {800, 600}, {780, 580});
    miss.segment({0, 0, 0}, {1, 0, 0}, {0, 0, 0});
    miss.quad(glm::mat4(1), {1, 1});
    assert(!miss.result(picked));
    pool::OverlayFocusPicker behind(overlayVP, {800, 600}, {400, 300});
    behind.quad(glm::translate(glm::mat4(1), glm::vec3(0, 0, 10)), {1, 1});
    assert(!behind.result(picked));
    glm::vec3 emptyFocus;
    assert(pool::focusPlanePoint(projection * view, eye, target, {.8f, .7f}, emptyFocus));
    assert(std::abs(glm::dot(emptyFocus - target, glm::normalize(target - eye))) < .001f);
    const auto emptyPixel = screen(projection * view, emptyFocus);
    assert(glm::length(glm::vec2(emptyPixel) - glm::vec2(.8f, .7f)) < .001f);
    assert(!pool::focusPlanePoint(projection * view, eye, target, {1.1f, .5f}, emptyFocus));
}
