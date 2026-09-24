#include "ros_runtime.hpp"
#include <riptide_msgs2/msg/controller_command.hpp>
#include <riptide_msgs2/msg/kill_switch_report.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <unistd.h>
#include <atomic>

namespace pool::panels {
namespace {
using Command = riptide_msgs2::msg::ControllerCommand;
using Kill = riptide_msgs2::msg::KillSwitchReport;
class UwrtMotion final : public RosMotion {
  public:
    UwrtMotion(std::shared_ptr<RosRuntime> rt, const YAML::Node &cfg, const Context &ctx) : RosMotion(rt, cfg, ctx) {
        auto node = runtime->node;
        const auto endpoint = [&](const char *key) { return expand(cfg[key].as<std::string>(), ctx); };
        linear = node->create_publisher<Command>(endpoint("linear_topic"), rclcpp::SystemDefaultsQoS());
        angular = node->create_publisher<Command>(endpoint("angular_topic"), rclcpp::SystemDefaultsQoS());
        killPub = node->create_publisher<Kill>(endpoint("kill_topic"), rclcpp::SystemDefaultsQoS());
        modeClient = node->create_client<std_srvs::srv::SetBool>(endpoint("mode_service"));
        switchId = cfg["kill_switch_id"].as<int>();
        static std::atomic<unsigned> instances{0};
        char host[256]{};
        gethostname(host, sizeof(host) - 1);
        sender = cfg["sender_prefix"].as<std::string>("viewer") + "_" + host + "_" + std::to_string(getpid()) + "_" +
                 std::to_string(++instances);
        value.supportsFeedforward = true;
        killSub = node->create_subscription<Kill>(endpoint("kill_topic"), rclcpp::SystemDefaultsQoS(),
                                                  [this](const Kill &msg) {
                                                      if (msg.kill_switch_id != switchId || msg.sender_id == sender)
                                                          return;
                                                      std::lock_guard<std::mutex> lock(mutex);
                                                      competitor = Steady::now();
                                                      value.competing = true;
                                                      if (value.enabled)
                                                          killLocked("Another operator owns the enable switch");
                                                  });
        stateSub = node->create_subscription<std_msgs::msg::Bool>(endpoint("kill_state_topic"), rclcpp::SensorDataQoS(),
                                                                  [this](const std_msgs::msg::Bool &msg) {
                                                                      std::lock_guard<std::mutex> lock(mutex);
                                                                      value.observedKilled = msg.data;
                                                                      observedSince = Steady::now();
                                                                  });
        // Observe commands from autonomy too; this is telemetry, never ownership.
        linearSub = node->create_subscription<Command>(endpoint("linear_topic"), rclcpp::SystemDefaultsQoS(),
                                                       [this](const Command &msg) { observe(msg, true); });
        angularSub = node->create_subscription<Command>(endpoint("angular_topic"), rclcpp::SystemDefaultsQoS(),
                                                        [this](const Command &msg) { observe(msg, false); });
        startTimer();
    }
    ~UwrtMotion() override {
        if (session && rclcpp::ok())
            kill();
    }

  private:
    void observe(const Command &msg, bool lin) {
        if (msg.mode != Command::POSITION)
            return;
        std::lock_guard<std::mutex> lock(mutex);
        auto world = commandFromFixed * value.commanded;
        if (lin)
            world[3] = glm::vec4(msg.setpoint_vect.x, msg.setpoint_vect.y, msg.setpoint_vect.z, 1);
        else {
            const auto &q = msg.setpoint_quat;
            if (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z < 1e-8)
                return;
            auto rotation = glm::mat4_cast(glm::normalize(glm::quat(q.w, q.x, q.y, q.z)));
            rotation[3] = world[3];
            world = rotation;
        }
        value.commanded = glm::inverse(commandFromFixed) * world;
        value.hasCommand = true;
        ++value.revision;
    }
    void send(const Pose &pose, Mode mode) override {
        Command lin, ang;
        lin.mode = ang.mode = mode == Mode::Position      ? Command::POSITION
                              : mode == Mode::Feedforward ? Command::FEEDFORWARD
                                                          : Command::DISABLED;
        const auto world = commandFromFixed * pose;
        const auto q = glm::normalize(glm::quat_cast(world));
        lin.setpoint_vect.x = world[3].x;
        lin.setpoint_vect.y = world[3].y;
        lin.setpoint_vect.z = world[3].z;
        ang.setpoint_quat.w = q.w;
        ang.setpoint_quat.x = q.x;
        ang.setpoint_quat.y = q.y;
        ang.setpoint_quat.z = q.z;
        linear->publish(lin);
        angular->publish(ang);
    }
    void report() override {
        Kill msg;
        msg.kill_switch_id = switchId;
        msg.sender_id = sender;
        msg.switch_asserting_kill = !value.enabled;
        msg.switch_needs_update = true;
        killPub->publish(msg);
    }
    bool modeReady() override {
        return modeClient->service_is_ready();
    }
    void requestMode(uint64_t epoch, Mode mode, const Pose &pose) override {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;
        requestId = modeClient
                        ->async_send_request(
                            request,
                            [this, epoch, mode, pose](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture reply) {
                                complete(epoch, mode, pose, reply.get()->success, reply.get()->message);
                            })
                        .request_id;
    }
    void cancelRequest() override {
        if (requestId)
            modeClient->remove_pending_request(requestId);
        requestId = 0;
    }
    int switchId;
    std::string sender;
    int64_t requestId = 0;
    rclcpp::Publisher<Command>::SharedPtr linear, angular;
    rclcpp::Publisher<Kill>::SharedPtr killPub;
    rclcpp::Subscription<Kill>::SharedPtr killSub;
    rclcpp::Subscription<Command>::SharedPtr linearSub, angularSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stateSub;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr modeClient;
};
} // namespace
void registerUwrtMotion(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "uwrt.motion",
        ProviderFactory{Kind::Motion,
                        [](const YAML::Node &cfg) {
                            keys(cfg,
                                 {"base_frame", "command_frame", "setpoint_frame", "linear_topic", "angular_topic",
                                  "kill_topic", "kill_state_topic", "mode_service", "kill_switch_id", "sender_prefix",
                                  "pose_timeout", "ui_timeout", "request_timeout", "heartbeat_period"},
                                 "uwrt.motion");
                            required(cfg, {"base_frame", "command_frame", "linear_topic", "angular_topic", "kill_topic",
                                           "kill_state_topic", "mode_service", "kill_switch_id"});
                            const auto id = cfg["kill_switch_id"].as<int>();
                            if (id < 1 || id > 255)
                                throw std::invalid_argument("kill_switch_id must be 1..255");
                            if (cfg["setpoint_frame"])
                                required(cfg, {"setpoint_frame"});
                            positive(cfg, "pose_timeout", 1);
                            positive(cfg, "ui_timeout", .75);
                            positive(cfg, "request_timeout", 3);
                            positive(cfg, "heartbeat_period", .05, 1);
                        },
                        [runtime](const YAML::Node &cfg, const Context &ctx) {
                            return std::make_shared<UwrtMotion>(runtime(ctx), cfg, ctx);
                        }});
}
} // namespace pool::panels
