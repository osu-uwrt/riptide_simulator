#pragma once
#include "pool_viewer/camera.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace pool {
// Transport-independent display state. ROS message translation stays in the node.
enum class LightMode { Solid, SlowFlash, FastFlash, Breath, Flash };

struct LightState {
    glm::vec3 steady{0}, pulse{0};
    LightMode mode = LightMode::Solid;
    double pulseStart = 0, pulseEnd = 0;

    void command(glm::vec3 rgb, LightMode next, double now, double duration) {
        if (next == LightMode::Flash) {
            pulse = rgb;
            pulseStart = now;
            pulseEnd = now + duration;
        } else {
            steady = rgb;
            mode = next;
        }
    }
    glm::vec3 color(double now) const {
        if (now >= pulseStart && now < pulseEnd)
            return pulse;
        float brightness = 1;
        // Match RViz's ROS-clock phase and periods, including paused sim time.
        switch (mode) {
        case LightMode::SlowFlash: brightness = std::fmod(now, 2.) < 1. ? 0.f : 1.f; break;
        case LightMode::FastFlash: brightness = std::fmod(now, .5) < .25 ? 0.f : 1.f; break;
        case LightMode::Breath: brightness = (std::sin(now * 2. * 3.141592653589793 / 3.) + 1.) * .5; break;
        default: break;
        }
        return steady * brightness;
    }
};

struct StatusLight {
    std::string id;
    uint32_t targets = UINT32_MAX;
    glm::mat4 mount{1};
    glm::vec3 size{.01f};
    float radiance = 200;
    LightState state;
};

struct StatusLights {
    std::string input, topic;
    double flashDuration = .15;
    std::vector<StatusLight> lights;

    explicit StatusLights(const std::string &path = "") {
        if (path.empty())
            return;
        const auto config = YAML::LoadFile(path);
        input = config["input"]["type"].as<std::string>();
        topic = config["input"]["topic"].as<std::string>();
        if (input != "riptide_msgs2/msg/LedCommand" && input != "std_msgs/msg/ColorRGBA")
            throw std::invalid_argument("Unsupported status light input type: " + input);
        if (topic.empty())
            throw std::invalid_argument("Status light topic must not be empty");
        flashDuration = config["flash_duration"].as<double>(.15);
        if (!std::isfinite(flashDuration) || flashDuration <= 0 || flashDuration > 10)
            throw std::invalid_argument("Status light flash_duration must be in (0,10]");
        const auto entries = config["lights"];
        if (!entries.IsSequence() || entries.size() == 0 || entries.size() > 256)
            throw std::invalid_argument("Status lights must contain 1 to 256 emitters");
        std::set<std::string> ids;
        for (const auto &entry : entries) {
            StatusLight light;
            light.id = entry["id"].as<std::string>();
            if (light.id.empty() || !ids.insert(light.id).second)
                throw std::invalid_argument("Status light IDs must be nonempty and unique");
            light.targets = entry["target_mask"].as<uint32_t>(UINT32_MAX);
            if (!light.targets)
                throw std::invalid_argument("Status light target_mask must not be zero");
            for (const auto &key : {"pose", "size"}) {
                auto v = entry[key];
                if (!v.IsSequence() || v.size() != (std::string(key) == "pose" ? 6 : 3))
                    throw std::invalid_argument("Invalid status light " + std::string(key));
                for (const auto &value : v)
                    if (!std::isfinite(value.as<double>()))
                        throw std::invalid_argument("Status light geometry must be finite");
            }
            light.mount = yamlPose(entry["pose"]);
            light.size = vector3(entry["size"]);
            for (int axis = 0; axis < 3; ++axis)
                if (light.size[axis] <= 0)
                    throw std::invalid_argument("Status light sizes must be positive");
            light.radiance = entry["radiance"].as<float>(200.f);
            if (!std::isfinite(light.radiance) || light.radiance <= 0 || light.radiance > 1000)
                throw std::invalid_argument("Status light radiance must be in (0,1000]");
            lights.push_back(light);
        }
    }
    void command(glm::vec3 rgb, LightMode mode, uint32_t targets, double now) {
        if (!std::isfinite(now))
            return;
        for (int axis = 0; axis < 3; ++axis)
            if (!std::isfinite(rgb[axis]))
                return;
        rgb = glm::clamp(rgb, glm::vec3(0), glm::vec3(1));
        for (auto &light : lights)
            if (targets & light.targets)
                light.state.command(rgb, mode, now, flashDuration);
    }
};
} // namespace pool
