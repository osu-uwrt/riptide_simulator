#include "pool_viewer/status_lights.hpp"
#include <cassert>
#include <limits>

using pool::LightMode;
static bool equal(glm::vec3 a, glm::vec3 b) { return glm::length(a - b) < 1e-5f; }

int main(int argc, char **argv) {
    assert(argc == 2);
    pool::StatusLights absent;
    assert(absent.lights.empty() && absent.input.empty() && absent.topic.empty());
    pool::StatusLights lights(argv[1]);
    assert(lights.lights.size() == 3);
    assert(lights.topic == "command/led");
    for (const auto &light : lights.lights) {
        assert(light.mount[3].y > 0 && light.mount[3].z > 0); // port hull, above origin
        assert(equal(light.state.color(0), glm::vec3(0)));
    }
    auto color = [&](double t) { return lights.lights.front().state.color(t); };
    lights.command({1, 0, 1}, LightMode::Solid, 0, 0);
    assert(equal(color(0), glm::vec3(0))); // TARGET_NONE
    lights.command({1, 0, 1}, LightMode::Solid, 1, 0);
    assert(equal(color(0), glm::vec3(0))); // CCB must not change ALU lights
    lights.command({1, 0, 1}, LightMode::Solid, 2, 0);
    assert(equal(color(0), {1, 0, 1}));
    lights.command({0, 1, 0}, LightMode::SlowFlash, 3, 0);
    assert(equal(color(.5), glm::vec3(0)));
    assert(equal(color(1.5), {0, 1, 0}));
    lights.command({0, 1, 0}, LightMode::FastFlash, 3, 0);
    assert(equal(color(.1), glm::vec3(0)));
    assert(equal(color(.3), {0, 1, 0}));
    lights.command({1, 0, 0}, LightMode::Breath, 3, 0);
    assert(equal(color(0), {.5f, 0, 0}));
    assert(equal(color(.75), {1, 0, 0}));
    assert(equal(color(2.25), glm::vec3(0)));
    // A detection pulse temporarily overlays, then restores the status command.
    lights.command({0, 0, 1}, LightMode::Solid, 3, 0);
    lights.command({1, 1, 1}, LightMode::Flash, 2, 10);
    assert(equal(color(10.05), glm::vec3(1)));
    assert(equal(color(10.2), {0, 0, 1}));
    assert(equal(color(0), {0, 0, 1})); // reset clock cannot extend old pulses
    lights.command({std::numeric_limits<float>::quiet_NaN(), 0, 0}, LightMode::Solid, 3, 11);
    assert(equal(color(11), {0, 0, 1}));
    lights.command({-1, 2, .5f}, LightMode::Solid, UINT32_MAX, 11);
    assert(equal(color(11), {0, 1, .5f}));
    lights.command({0, 0, 0}, LightMode::Solid, 3, 12);
    for (const auto &light : lights.lights)
        assert(equal(light.state.color(12), glm::vec3(0)));
    // A robot can independently route commands to multiple groups of emitters.
    lights.lights.front().targets = 1;
    lights.command({1, 0, 0}, LightMode::Solid, 1, 13);
    lights.command({0, 1, 0}, LightMode::Solid, 2, 13);
    assert(equal(color(13), {1, 0, 0}));
    assert(equal(lights.lights.back().state.color(13), {0, 1, 0}));
}
