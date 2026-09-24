#include "ros_runtime.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace pool::panels {
namespace {
class StandardMotion final : public RosMotion {
  public:
    StandardMotion(std::shared_ptr<RosRuntime> rt, const YAML::Node &cfg, const Context &ctx)
        : RosMotion(rt, cfg, ctx) {
        publisher = rt->node->create_publisher<geometry_msgs::msg::PoseStamped>(
            expand(cfg["pose_topic"].as<std::string>(), ctx), 10);
        client = rt->node->create_client<std_srvs::srv::SetBool>(expand(cfg["enable_service"].as<std::string>(), ctx));
        feedback = rt->node->create_subscription<std_msgs::msg::Bool>(
            expand(cfg["enabled_topic"].as<std::string>(), ctx), rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Bool &msg) {
                std::lock_guard<std::mutex> lock(mutex);
                value.observedKilled = !msg.data;
                observedSince = Steady::now();
            });
        startTimer();
    }
    ~StandardMotion() override {
        if (session && rclcpp::ok())
            kill();
    }
    void enable() override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!ready() || value.pending || !client->service_is_ready())
            return;
        session = true;
        value.pending = true;
        pendingSince = Steady::now();
        const auto epoch = ++generation;
        value.message = "Requesting enable...";
        disableSent = false;
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        requestId = client
                        ->async_send_request(request,
                                             [this, epoch](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture reply) {
                                                 std::lock_guard<std::mutex> lock(mutex);
                                                 if (epoch != generation)
                                                     return;
                                                 value.pending = false;
                                                 value.enabled = reply.get()->success && ready();
                                                 value.message = value.enabled
                                                                     ? "Enabled"
                                                                     : "Enable failed: " + reply.get()->message;
                                                 if (!value.enabled)
                                                     report();
                                             })
                        .request_id;
    }

  private:
    void send(const Pose &pose, Mode mode) override {
        if (mode != Mode::Position)
            return;
        const auto world = commandFromFixed * pose;
        const auto q = glm::normalize(glm::quat_cast(world));
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = commandFrame;
        msg.header.stamp = runtime->node->now();
        msg.pose.position.x = world[3].x;
        msg.pose.position.y = world[3].y;
        msg.pose.position.z = world[3].z;
        msg.pose.orientation.w = q.w;
        msg.pose.orientation.x = q.x;
        msg.pose.orientation.y = q.y;
        msg.pose.orientation.z = q.z;
        publisher->publish(msg);
    }
    void report() override {
        if (value.enabled || value.pending || disableSent || !client->service_is_ready())
            return;
        disableSent = true;
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;
        requestId = client
                        ->async_send_request(request,
                                             [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture reply) {
                                                 std::lock_guard<std::mutex> lock(mutex);
                                                 if (!reply.get()->success)
                                                     value.message = "Disable rejected: " + reply.get()->message;
                                             })
                        .request_id;
    }
    bool modeReady() override {
        return client->service_is_ready();
    }
    void requestMode(uint64_t epoch, Mode mode, const Pose &pose) override {
        modeTimer = runtime->node->create_wall_timer(std::chrono::milliseconds(1), [this, epoch, mode, pose] {
            modeTimer->cancel();
            complete(epoch, mode, pose, true, "");
        });
    }
    void cancelRequest() override {
        if (requestId)
            client->remove_pending_request(requestId);
        requestId = 0;
        if (modeTimer)
            modeTimer->cancel();
        disableSent = false;
    }
    bool disableSent = false;
    int64_t requestId = 0;
    rclcpp::TimerBase::SharedPtr modeTimer;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr feedback;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client;
};
} // namespace
void registerStandardMotion(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "ros.pose",
        ProviderFactory{Kind::Motion,
                        [](const YAML::Node &cfg) {
                            keys(cfg,
                                 {"base_frame", "command_frame", "setpoint_frame", "pose_topic", "enable_service",
                                  "enabled_topic", "pose_timeout", "ui_timeout", "request_timeout", "heartbeat_period"},
                                 "ros.pose");
                            required(cfg,
                                     {"base_frame", "command_frame", "pose_topic", "enable_service", "enabled_topic"});
                            if (cfg["setpoint_frame"])
                                required(cfg, {"setpoint_frame"});
                            positive(cfg, "pose_timeout", 1);
                            positive(cfg, "ui_timeout", .75);
                            positive(cfg, "request_timeout", 3);
                            positive(cfg, "heartbeat_period", .05, 1);
                        },
                        [runtime](const YAML::Node &cfg, const Context &ctx) {
                            return std::make_shared<StandardMotion>(runtime(ctx), cfg, ctx);
                        }});
}
} // namespace pool::panels
