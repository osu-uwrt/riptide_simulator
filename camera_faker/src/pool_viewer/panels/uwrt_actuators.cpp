#include "ros_runtime.hpp"
#include <riptide_msgs2/msg/actuator_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <set>

namespace pool::panels {
namespace {
class UwrtActuators final : public Actuators {
    struct Command {
        std::string id, label, armedLabel;
        bool requiresArmed = false;
        std::function<void(bool)> publish;
    };

  public:
    UwrtActuators(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx)
        : runtime(runtime), timeout(cfg["status_timeout"].as<double>(1)) {
        auto node = runtime->node;
        for (const auto &entry : cfg["commands"]) {
            Command command;
            command.id = entry["id"].as<std::string>();
            command.label = entry["label"].as<std::string>();
            command.armedLabel = entry["armed_label"].as<std::string>(command.label);
            command.requiresArmed = entry["requires_armed"].as<bool>(false);
            auto topic = expand(entry["topic"].as<std::string>(), ctx);
            auto type = entry["type"].as<std::string>();
            if (type == "bool") {
                auto pub = node->create_publisher<std_msgs::msg::Bool>(topic, 10);
                bool toggle = entry["toggle_armed"].as<bool>(false), value = entry["value"].as<bool>(false);
                command.publish = [pub, toggle, value](bool armed) {
                    std_msgs::msg::Bool msg;
                    msg.data = toggle ? !armed : value;
                    pub->publish(msg);
                };
            } else if (type == "float32") {
                auto pub = node->create_publisher<std_msgs::msg::Float32>(topic, 10);
                float value = entry["value"].as<float>();
                command.publish = [pub, value](bool) {
                    std_msgs::msg::Float32 msg;
                    msg.data = value;
                    pub->publish(msg);
                };
            } else {
                auto pub = node->create_publisher<std_msgs::msg::Empty>(topic, 10);
                command.publish = [pub](bool) { pub->publish(std_msgs::msg::Empty{}); };
            }
            commands.push_back(std::move(command));
        }
        status = node->create_subscription<riptide_msgs2::msg::ActuatorStatus>(
            expand(cfg["status_topic"].as<std::string>(), ctx), rclcpp::SensorDataQoS(),
            [this](const riptide_msgs2::msg::ActuatorStatus &msg) {
                std::lock_guard<std::mutex> lock(mutex);
                armed = msg.actuators_armed;
                lastStatus = Steady::now();
                readings["Torpedoes remaining"] = std::to_string(msg.torpedo_available_count);
                readings["Markers remaining"] = std::to_string(msg.dropper_available_count);
            });
    }
    ActuatorState state() override {
        std::lock_guard<std::mutex> lock(mutex);
        ActuatorState value;
        value.fresh = fresh();
        value.armed = armed;
        value.message =
            value.fresh ? (armed ? "Actuators armed" : "Actuators disarmed") : "Actuator status unavailable / stale";
        for (const auto &command : commands)
            value.actions.push_back({command.id, armed ? command.armedLabel : command.label,
                                     value.fresh && (!command.requiresArmed || armed)});
        value.readings.assign(readings.begin(), readings.end());
        return value;
    }
    void command(const std::string &id) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!fresh())
            return;
        for (const auto &command : commands)
            if (command.id == id && (!command.requiresArmed || armed)) {
                command.publish(armed);
                return;
            }
    }

  private:
    bool fresh() const {
        return std::chrono::duration<double>(Steady::now() - lastStatus).count() < timeout;
    }
    std::shared_ptr<RosRuntime> runtime;
    std::mutex mutex;
    double timeout;
    bool armed = false;
    Steady::time_point lastStatus{};
    std::vector<Command> commands;
    std::map<std::string, std::string> readings;
    rclcpp::Subscription<riptide_msgs2::msg::ActuatorStatus>::SharedPtr status;
};
} // namespace
void registerUwrtActuators(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "uwrt.actuators",
        ProviderFactory{Kind::Actuators,
                        [](const YAML::Node &cfg) {
                            keys(cfg, {"status_topic", "commands", "status_timeout"}, "uwrt.actuators");
                            required(cfg, {"status_topic"});
                            positive(cfg, "status_timeout", 1);
                            if (!cfg["commands"].IsSequence())
                                throw std::invalid_argument("commands must be a sequence");
                            std::set<std::string> ids;
                            for (const auto &entry : cfg["commands"]) {
                                keys(entry,
                                     {"id", "label", "armed_label", "topic", "type", "value", "requires_armed",
                                      "toggle_armed"},
                                     "actuator command");
                                required(entry, {"id", "label", "topic", "type"});
                                if (!ids.insert(entry["id"].as<std::string>()).second)
                                    throw std::invalid_argument("duplicate actuator command ID");
                                auto type = entry["type"].as<std::string>();
                                if (type != "bool" && type != "float32" && type != "empty")
                                    throw std::invalid_argument("command type must be bool, float32 or empty");
                                if (type == "float32" && !std::isfinite(entry["value"].as<float>()))
                                    throw std::invalid_argument("nonfinite actuator command");
                                if (type == "bool")
                                    (void)entry["value"].as<bool>(false);
                                (void)entry["requires_armed"].as<bool>(false);
                                (void)entry["toggle_armed"].as<bool>(false);
                            }
                        },
                        [runtime](const YAML::Node &cfg, const Context &ctx) {
                            return std::make_shared<UwrtActuators>(runtime(ctx), cfg, ctx);
                        }});
}
} // namespace pool::panels
