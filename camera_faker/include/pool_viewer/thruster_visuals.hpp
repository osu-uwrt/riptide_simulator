#pragma once
#include "pool_viewer/camera.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <set>

namespace pool {
struct ThrusterRotor {
    std::string id, mesh;
    size_t inputIndex = 0;
    glm::vec3 pivot{0}, axis{1, 0, 0};
    double direction = 1, angle = 0;

    glm::mat4 transform() const {
        // Mesh vertices and the rotation axis share the robot's model frame.
        return glm::translate(glm::mat4(1), pivot) *
               glm::rotate(glm::mat4(1), float(angle), axis) *
               glm::translate(glm::mat4(1), -pivot);
    }
};

// Optional visual state driven by realized forces. No robot name, motor count,
// ROS message type, or robot-specific calibration is embedded in the renderer.
struct ThrusterVisuals {
    std::string topic;
    std::vector<ThrusterRotor> rotors;
    double timeout = .5, deadband = .01, speedScale = 1;
    std::array<double, 4> forwardRpm{}, reverseRpm{};

    explicit ThrusterVisuals(const std::string &path = "", size_t inputCount = 0) {
        if (path.empty())
            return;
        const auto config = YAML::LoadFile(path);
        topic = config["topic"].as<std::string>();
        if (topic.empty() || inputCount == 0)
            throw std::invalid_argument("Thruster visuals need a topic and vehicle thrusters");
        timeout = config["timeout"].as<double>(.5);
        deadband = config["force_deadband"].as<double>(.01);
        speedScale = config["speed_scale"].as<double>(1.);
        for (double value : {timeout, speedScale})
            if (!std::isfinite(value) || value <= 0)
                throw std::invalid_argument("Thruster visual timing/speed settings must be positive and finite");
        if (!std::isfinite(deadband) || deadband < 0)
            throw std::invalid_argument("Thruster visual deadband must be finite and nonnegative");
        for (const auto &key : {"forward", "reverse"}) {
            const auto coefficients = config["force_to_rpm"][key];
            if (!coefficients.IsSequence() || coefficients.size() != 4)
                throw std::invalid_argument("Thruster force_to_rpm curves need four coefficients per direction");
            auto &curve = std::string(key) == "forward" ? forwardRpm : reverseRpm;
            for (size_t i = 0; i < curve.size(); ++i) {
                curve[i] = coefficients[i].as<double>();
                if (!std::isfinite(curve[i]))
                    throw std::invalid_argument("Thruster RPM coefficients must be finite");
            }
        }
        const auto entries = config["rotors"];
        if (!entries.IsSequence() || entries.size() == 0 || entries.size() > 256)
            throw std::invalid_argument("Thruster visuals must contain 1 to 256 rotors");
        std::set<std::string> ids;
        for (const auto &entry : entries) {
            ThrusterRotor rotor;
            rotor.id = entry["id"].as<std::string>();
            if (rotor.id.empty() || !ids.insert(rotor.id).second)
                throw std::invalid_argument("Thruster rotor IDs must be nonempty and unique");
            const int index = entry["input_index"].as<int>();
            if (index < 0 || size_t(index) >= inputCount)
                throw std::invalid_argument("Thruster rotor input_index exceeds vehicle thruster count");
            rotor.inputIndex = size_t(index);
            for (const auto &key : {"pivot", "axis"}) {
                const auto vector = entry[key];
                if (!vector.IsSequence() || vector.size() != 3)
                    throw std::invalid_argument("Thruster rotor pivot/axis must contain three numbers");
                for (const auto &value : vector)
                    if (!std::isfinite(value.as<float>()))
                        throw std::invalid_argument("Thruster rotor geometry must be finite");
            }
            rotor.pivot = vector3(entry["pivot"]);
            rotor.axis = vector3(entry["axis"]);
            if (glm::length(rotor.axis) < 1e-6f)
                throw std::invalid_argument("Thruster rotor axis must be nonzero");
            rotor.axis = glm::normalize(rotor.axis);
            rotor.direction = entry["direction"].as<double>(1.);
            if (rotor.direction != 1 && rotor.direction != -1)
                throw std::invalid_argument("Thruster rotor direction must be 1 or -1");
            const auto mesh = entry["mesh"].as<std::string>();
            auto asset = std::filesystem::path(mesh);
            if (asset.is_relative())
                asset = std::filesystem::path(path).parent_path() / asset;
            if (mesh.empty() || !std::filesystem::is_regular_file(asset))
                throw std::invalid_argument("Missing thruster rotor asset: " + asset.string());
            rotor.mesh = std::filesystem::absolute(asset).string();
            rotors.push_back(rotor);
        }
        forces.assign(inputCount, 0.f);
    }

    double rpm(double force) const {
        if (!std::isfinite(force) || std::abs(force) <= deadband)
            return 0;
        const auto &c = force > 0 ? forwardRpm : reverseRpm;
        const double value = c[0] + c[1] * force + c[2] * std::tanh(force) +
                             c[3] * std::pow(std::abs(force), .25);
        // Empirical fits can cross zero near idle; never reverse due to that intercept.
        return force > 0 ? std::max(0., value) : std::min(0., value);
    }

    bool receive(const std::vector<float> &values, double now) {
        if (values.size() != forces.size() || !std::isfinite(now))
            return false;
        for (float force : values)
            if (!std::isfinite(force))
                return false;
        // Integrate the previous sample only up to its replacement time.
        advance(now);
        forces = values;
        receivedAt = now;
        return true;
    }

    void advance(double now) {
        if (!std::isfinite(now))
            return;
        if (!std::isfinite(lastStep) || now < lastStep) {
            if (std::isfinite(lastStep)) {
                for (auto &rotor : rotors)
                    rotor.angle = 0;
                std::fill(forces.begin(), forces.end(), 0.f);
                receivedAt = std::numeric_limits<double>::quiet_NaN();
            }
            lastStep = now;
            return;
        }
        if (std::isfinite(receivedAt)) {
            const double dt = std::max(0., std::min(now, receivedAt + timeout) - std::max(lastStep, receivedAt));
            for (auto &rotor : rotors) {
                const double force = forces[rotor.inputIndex];
                const double speed = rpm(force) * (2. * glm::pi<double>() / 60.) * speedScale;
                rotor.angle = std::remainder(rotor.angle + rotor.direction * speed * dt, 2. * glm::pi<double>());
            }
            if (now >= receivedAt + timeout)
                std::fill(forces.begin(), forces.end(), 0.f);
        }
        lastStep = now;
    }

  private:
    std::vector<float> forces;
    double receivedAt = std::numeric_limits<double>::quiet_NaN();
    double lastStep = std::numeric_limits<double>::quiet_NaN();
};
} // namespace pool
