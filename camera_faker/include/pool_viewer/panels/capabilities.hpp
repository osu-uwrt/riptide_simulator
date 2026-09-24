#pragma once
#include <glm/glm.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace pool::panels {
using Pose = glm::mat4;
enum class Kind { Motion, Autonomy, Mapping, Actuators, Run, Simulation };
enum class Mode { Disabled, Position, Feedforward };
struct Provider {
    virtual ~Provider() = default;
    virtual Kind kind() const = 0;
    virtual void touch() {} // presentation liveness, never an enable request
};
struct MotionState {
    Pose actual{1}, commanded{1};
    bool fresh = false, enabled = false, pending = false, blocked = false, competing = false;
    bool hasCommand = false, supportsFeedforward = false;
    std::optional<bool> observedKilled;
    Mode mode = Mode::Disabled;
    uint64_t revision = 0;
    std::string frame, message = "Waiting for pose";
};
struct Motion : Provider {
    Kind kind() const final {
        return Kind::Motion;
    }
    virtual MotionState state() = 0;
    virtual void enable() = 0;
    virtual void kill() = 0;
    virtual void activate(Mode mode, const Pose &target) = 0;
    virtual void drag(const Pose &target) = 0;
    virtual void block(bool active) = 0;
};
struct MissionState {
    bool connected = false, refreshing = false, busy = false, pending = false, stackStale = false;
    bool failed = false;
    std::string message = "Waiting for autonomy", activeTree;
    std::vector<std::string> trees, stack;
};
struct Autonomy : Provider {
    Kind kind() const final {
        return Kind::Autonomy;
    }
    virtual MissionState state() = 0;
    virtual void refresh() = 0;
    virtual void start(const std::string &tree) = 0;
    virtual void stop() = 0;
};
struct MappingState {
    bool calibrationReady = false, resetReady = false, targetReady = false, fresh = false;
    bool calibrating = false, canceling = false, resetting = false, settingTarget = false;
    bool locked = false;
    unsigned samples = 0;
    std::string target, calibrationMessage = "Waiting for calibration server", resetMessage, targetMessage;
};
struct Mapping : Provider {
    Kind kind() const final {
        return Kind::Mapping;
    }
    virtual MappingState state() = 0;
    virtual void calibrate(const std::string &parent, const std::string &child, unsigned samples) = 0;
    virtual void cancelCalibration() = 0;
    virtual void reset() = 0;
    virtual void setTarget(const std::string &, bool locked) = 0;
};
struct ActuatorAction {
    std::string id, label;
    bool available = false;
};
struct ActuatorState {
    bool fresh = false, armed = false;
    std::vector<ActuatorAction> actions;
    std::vector<std::pair<std::string, std::string>> readings;
    std::string message = "Waiting for actuator status";
};
struct Actuators : Provider {
    Kind kind() const final {
        return Kind::Actuators;
    }
    virtual ActuatorState state() = 0;
    virtual void command(const std::string &id) = 0;
};
// Run schemas and snapshots are structured documents: competition-specific fields
// remain in the profile and score publisher, never in the viewer or ROS UI layer.
struct RunState {
    bool fresh = false;
    YAML::Node score;
    std::string message = "Waiting for run tracking";
    std::string taskSummary;
    std::vector<std::pair<std::string, bool>> magnetTargets;
    std::vector<std::pair<std::string, std::string>> simulationReadings;
    std::vector<std::string> events;
};
struct Run : Provider {
    Kind kind() const final {
        return Kind::Run;
    }
    virtual RunState state() = 0;
    virtual void command(const YAML::Node &) = 0;
    virtual void reset() = 0;
};
struct SimulationState {
    bool connected = false, pending = false;
    double rate = 1, resumeRate = 1, maxRate = 10;
    bool syncReady = false, resetReady = false, operationPending = false;
    std::string operationMessage;
    std::string message = "Waiting for simulator";
};
struct Simulation : Provider {
    Kind kind() const final {
        return Kind::Simulation;
    }
    virtual SimulationState state() = 0;
    virtual void setRate(double) = 0;
    virtual void setPaused(bool) = 0;
    virtual void sync() = 0;
    virtual void reset() = 0;
};
} // namespace pool::panels
