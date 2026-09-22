#include "pool_viewer/thruster_visuals.hpp"
#include <cassert>
#include <limits>

static bool near(double a, double b) { return std::abs(a - b) < 1e-6; }

int main(int argc, char **argv) {
    assert(argc == 2);
    pool::ThrusterVisuals absent;
    assert(absent.rotors.empty() && absent.topic.empty());
    pool::ThrusterVisuals visuals(argv[1], 8);
    assert(visuals.rotors.size() == 8);
    // Recorded expectations from Talos's signed forward/reverse calibration.
    assert(near(visuals.rpm(4), 1370.783617011576));
    assert(near(visuals.rpm(-4), -1284.8405445584433));
    assert(near(visuals.rpm(24), 3170.953217372712));
    assert(near(visuals.rpm(-24), -2932.0274207755842));
    assert(visuals.rpm(0) == 0 && visuals.rpm(.011) >= 0 && visuals.rpm(-.011) <= 0);
    assert(visuals.receive(std::vector<float>(8, 4), 0));
    visuals.advance(.01);
    assert(near(visuals.rotors[1].angle, 1370.783617011576 * glm::pi<double>() / 30. * .01));
    visuals.advance(.18); // Preserve full turns between render frames, without slowing or clamping.
    assert(near(visuals.rotors[1].angle,
                std::remainder(1370.783617011576 * glm::pi<double>() / 30. * .18, 2 * glm::pi<double>())));
    visuals.advance(-1); // Reset before testing timing with an independent linear fixture.
    visuals.forwardRpm = visuals.reverseRpm = {0, 30. / glm::pi<double>(), 0, 0};
    std::vector<float> forces{2, 2, -3, 0, 0, 0, 0, 0};
    assert(visuals.receive(forces, 0));
    visuals.advance(.1);
    assert(near(visuals.rotors[0].angle, -.2));
    assert(near(visuals.rotors[1].angle, .2));
    assert(near(visuals.rotors[2].angle, -.3));
    for (size_t i = 3; i < 8; ++i)
        assert(near(visuals.rotors[i].angle, 0));
    visuals.advance(.1); // Pausing the ROS clock freezes the animation.
    assert(near(visuals.rotors[0].angle, -.2));
    for (auto &f : forces)
        f = -f;
    assert(visuals.receive(forces, .1));
    visuals.advance(.2);
    for (const auto &rotor : visuals.rotors)
        assert(near(rotor.angle, 0));
    assert(visuals.receive(std::vector<float>(8, 0), .2));
    visuals.advance(.4);
    assert(near(visuals.rotors[0].angle, 0));

    forces.assign(8, 2);
    assert(visuals.receive(forces, .4));
    assert(!visuals.receive({1, 2}, .6));
    forces[0] = std::numeric_limits<float>::quiet_NaN();
    assert(!visuals.receive(forces, .7));
    visuals.advance(1.0); // Invalid packets must not extend the .5 s timeout.
    assert(near(visuals.rotors[0].angle, -1.0));
    visuals.advance(2.0);
    assert(near(visuals.rotors[0].angle, -1.0));
    assert(visuals.receive(std::vector<float>(8, .005f), 2));
    visuals.advance(2.1);
    assert(near(visuals.rotors[0].angle, -1.0)); // Negligible residual force is idle.
    visuals.advance(0); // A simulator clock reset clears old forces and phase.
    visuals.advance(.2);
    for (const auto &rotor : visuals.rotors)
        assert(near(rotor.angle, 0));
    assert(visuals.receive(std::vector<float>(8, 100), .2));
    visuals.advance(.201);
    assert(near(visuals.rotors[0].angle, -.1)); // No artificial display speed cap.

    for (auto &rotor : visuals.rotors) {
        rotor.angle = .7;
        auto matrix = rotor.transform();
        glm::vec3 pivot = matrix * glm::vec4(rotor.pivot, 1);
        glm::vec3 shaft = rotor.pivot + .02f * rotor.axis;
        assert(glm::length(pivot - rotor.pivot) < 1e-6f);
        assert(glm::length(glm::vec3(matrix * glm::vec4(shaft, 1)) - shaft) < 1e-6f);
        // An off-axis blade moves without changing its distance to the shaft.
        glm::vec3 radial = glm::cross(rotor.axis, glm::vec3(0, 0, 1));
        glm::vec3 point = rotor.pivot + .02f * radial;
        glm::vec3 rotated = matrix * glm::vec4(point, 1);
        assert(glm::length(rotated - point) > .001f);
        assert(near(glm::length(rotated - rotor.pivot), glm::length(point - rotor.pivot)));
    }
    bool rejected = false;
    try { pool::ThrusterVisuals wrongVehicle(argv[1], 3); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
}
