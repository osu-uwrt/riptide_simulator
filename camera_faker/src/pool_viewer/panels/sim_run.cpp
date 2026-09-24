#include "ros_runtime.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <set>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pool::panels {
namespace {
class SimRun final : public Run {
  public:
    SimRun(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx)
        : runtime(runtime), timeout(cfg["status_timeout"].as<double>(1)) {
        if (cfg["profile"] && ctx.documents.count(cfg["profile"].as<std::string>())) {
            const auto document = ctx.documents.at(cfg["profile"].as<std::string>());
            enabled = document["scoring_enabled"].as<bool>(true);
            if (document["ui"])
                schema = YAML::Clone(document["ui"]);
        }
        auto node = runtime->node;
        if (cfg["task_score_topic"])
            taskScoreSub = node->create_subscription<std_msgs::msg::String>(
                expand(cfg["task_score_topic"].as<std::string>(), ctx), 10, [this](const std_msgs::msg::String &msg) {
                    std::lock_guard<std::mutex> lock(mutex);
                    taskScore = msg.data;
                });
        if (cfg["events_topic"])
            events = node->create_subscription<std_msgs::msg::String>(
                expand(cfg["events_topic"].as<std::string>(), ctx), 10, [this](const std_msgs::msg::String &msg) {
                    std::lock_guard<std::mutex> lock(mutex);
                    std::string text = msg.data;
                    try {
                        auto e = YAML::Load(msg.data);
                        auto kind = e["kind"].as<std::string>(), result = e["result"].as<std::string>();
                        if (kind == "tasks" && result == "reset")
                            history.clear();
                        text = kind + " / " + result + " / " + e["target"].as<std::string>();
                    } catch (const YAML::Exception &) {
                    }
                    history.insert(history.begin(), text);
                    if (history.size() > 5)
                        history.pop_back();
                });
        if (cfg["lights_topic"])
            lights = node->create_subscription<visualization_msgs::msg::MarkerArray>(
                expand(cfg["lights_topic"].as<std::string>(), ctx), 10,
                [this](const visualization_msgs::msg::MarkerArray &msg) {
                    std::lock_guard<std::mutex> lock(mutex);
                    for (const auto &m : msg.markers) {
                        if (m.action == visualization_msgs::msg::Marker::ADD)
                            magnetTargets[m.ns] = m.color.g > m.color.r;
                        else if (m.action == visualization_msgs::msg::Marker::DELETE)
                            magnetTargets.erase(m.ns);
                        else if (m.action == visualization_msgs::msg::Marker::DELETEALL)
                            magnetTargets.clear();
                    }
                });
        float minGap = 0;
        if (cfg["profile"] && ctx.documents.count(cfg["profile"].as<std::string>())) {
            const auto document = ctx.documents.at(cfg["profile"].as<std::string>());
            if (document["claw"])
                minGap = document["claw"]["min_gap"].as<float>(0);
        }
        if (cfg["joints_topic"])
            joints = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                expand(cfg["joints_topic"].as<std::string>(), ctx), 10,
                [this, minGap](const std_msgs::msg::Float64MultiArray &msg) {
                    if (msg.data.size() != 2 || !std::isfinite(msg.data[0]) || !std::isfinite(msg.data[1]))
                        return;
                    std::lock_guard<std::mutex> lock(mutex);
                    readings["Jaw gap"] =
                        std::to_string(std::lround(1000 * (minGap + msg.data[0] + msg.data[1]))) + " mm";
                });
        commandPub = runtime->node->create_publisher<std_msgs::msg::String>(
            expand(cfg["command_topic"].as<std::string>(), ctx), 10);
        resetPub = runtime->node->create_publisher<std_msgs::msg::Empty>(
            expand(cfg["reset_topic"].as<std::string>(), ctx), 10);
        subscription = runtime->node->create_subscription<std_msgs::msg::String>(
            expand(cfg["score_topic"].as<std::string>(), ctx), 10, [this](const std_msgs::msg::String &msg) {
                std::lock_guard<std::mutex> lock(mutex);
                try {
                    auto document = YAML::Load(msg.data);
                    if (!document.IsMap())
                        throw std::invalid_argument("score must be a mapping");
                    required(document, {"running", "elapsed", "total"});
                    for (auto field : {"total", "elapsed", "adjustment"})
                        if (document[field] && !std::isfinite(document[field].as<double>()))
                            throw std::invalid_argument("nonfinite score");
                    (void)document["running"].as<bool>(false);
                    if (document["rows"] && !document["rows"].IsSequence())
                        throw std::invalid_argument("rows must be a sequence");
                    for (const auto &row : document["rows"]) {
                        (void)row["label"].as<std::string>();
                        (void)row["points"].as<double>();
                    }
                    // Rebind: YAML assignment retains prior snapshots through merged memory pools.
                    score.reset(document);
                    lastScore = Steady::now();
                    message.clear();
                } catch (const std::exception &e) {
                    message = std::string("Invalid run score: ") + e.what();
                    lastScore = {};
                }
            });
    }
    RunState state() override {
        std::lock_guard<std::mutex> lock(mutex);
        RunState value;
        value.fresh = fresh();
        value.score.reset(YAML::Clone(score));
        value.message = !enabled           ? "Run tracking disabled by profile"
                        : !message.empty() ? message
                        : fresh()          ? ""
                                           : "Run tracking unavailable / stale";
        value.taskSummary = taskScore;
        value.magnetTargets.assign(magnetTargets.begin(), magnetTargets.end());
        value.simulationReadings.assign(readings.begin(), readings.end());
        value.events = history;
        return value;
    }
    void command(const YAML::Node &command) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!fresh() || !command.IsMap())
            return;
        auto action = command["action"].as<std::string>("");
        bool running = score["running"].as<bool>(false);
        if (action.empty() || (action == "start" && running) || (action == "stop" && !running))
            return;
        YAML::Emitter json;
        json << YAML::Flow << YAML::BeginMap;
        for (const auto &entry : command) {
            auto key = entry.first.as<std::string>();
            bool number = key == "points", boolean = false;
            for (const auto &option : schema["run_options"])
                if (option["key"].as<std::string>() == key) {
                    number = option["type"].as<std::string>() == "number";
                    boolean = option["type"].as<std::string>() == "bool";
                }
            json << YAML::Key << YAML::DoubleQuoted << key << YAML::Value;
            if (number) {
                double value = entry.second.as<double>();
                if (!std::isfinite(value))
                    return;
                json << value;
            } else if (boolean || (entry.second.Tag() != "!" && (entry.second.as<std::string>() == "true" ||
                                                                 entry.second.as<std::string>() == "false")))
                json << entry.second.as<bool>();
            else
                json << YAML::DoubleQuoted << entry.second.as<std::string>();
        }
        json << YAML::EndMap;
        std_msgs::msg::String msg;
        msg.data = json.c_str();
        commandPub->publish(msg);
    }
    void reset() override {
        std::lock_guard<std::mutex> lock(mutex);
        if (fresh())
            resetPub->publish(std_msgs::msg::Empty{});
    }

  private:
    bool fresh() const {
        return enabled && std::chrono::duration<double>(Steady::now() - lastScore).count() < timeout;
    }
    std::shared_ptr<RosRuntime> runtime;
    std::mutex mutex;
    double timeout;
    bool enabled = true;
    YAML::Node score, schema;
    std::string taskScore;
    std::map<std::string, bool> magnetTargets;
    std::map<std::string, std::string> readings;
    std::vector<std::string> history;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr taskScoreSub, events;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joints;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr lights;
    std::string message;
    Steady::time_point lastScore{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commandPub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr resetPub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};
} // namespace
void registerSimRun(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "sim.run", ProviderFactory{Kind::Run,
                                   [](const YAML::Node &cfg) {
                                       keys(cfg,
                                            {"command_topic", "reset_topic", "score_topic", "status_timeout", "profile",
                                             "task_score_topic", "events_topic", "lights_topic", "joints_topic"},
                                            "sim.run");
                                       required(cfg, {"command_topic", "reset_topic", "score_topic"});
                                       positive(cfg, "status_timeout", 1);
                                   },
                                   [runtime](const YAML::Node &cfg, const Context &ctx) {
                                       return std::make_shared<SimRun>(runtime(ctx), cfg, ctx);
                                   }});
}
} // namespace pool::panels
