#pragma once
#include "pool_viewer/camera.hpp"

namespace pool {
// Actuator TF origins are aiming references, not loaded projectile centers.
// Compose the measured seating offsets with the robot's actuator joints.
inline std::vector<glm::mat4> payloadMounts(const YAML::Node &vehicle, const YAML::Node &task,
                                            const std::string &kind) {
    const auto actuator = vehicle[kind == "torpedo" ? "torpedoes" : "droppers"];
    const auto cfg = task[kind];
    const int count = cfg["count"].as<int>();
    const auto mount = baseToOrigin(vehicle) * yamlPose(actuator["pose"]);
    const auto slots = cfg["slot_offsets"];
    if (count <= 0 || !slots || slots.size() != size_t(count))
        throw std::runtime_error("Each payload needs a seating offset in its actuator frame");
    std::vector<glm::mat4> result;
    if (kind == "torpedo" && actuator["baseline"]) {
        const float baseline = actuator["baseline"].as<float>();
        if (count != 2 || !std::isfinite(baseline) || baseline < 0)
            throw std::runtime_error("Torpedoes require two slots and a finite nonnegative baseline");
        for (int i = 0; i < count; ++i)
            result.push_back(mount * pose({0, (i == 0 ? -1.f : 1.f) * baseline / 2, 0}) * yamlPose(slots[i]));
    } else {
        for (const auto &slot : slots)
            result.push_back(mount * yamlPose(slot));
    }
    for (const auto &m : result)
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                if (!std::isfinite(m[c][r]))
                    throw std::runtime_error("Payload mounts must be finite");
    return result;
}
} // namespace pool
