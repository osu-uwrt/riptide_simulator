#include "ros_runtime.hpp"
#include <chameleon_tf_msgs/action/model_frame.hpp>
#include <riptide_msgs2/msg/mapping_target_info.hpp>
#include <riptide_msgs2/srv/mapping_target.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace pool::panels {
namespace {
using Cal = chameleon_tf_msgs::action::ModelFrame;
using Goal = rclcpp_action::ClientGoalHandle<Cal>;
using Target = riptide_msgs2::srv::MappingTarget;
using Reset = std_srvs::srv::Trigger;
class UwrtMapping final : public Mapping {
  public:
    UwrtMapping(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx)
        : runtime(runtime), timeout(cfg["request_timeout"].as<double>(3)),
          calibrationTimeout(cfg["calibration_timeout"].as<double>(60)),
          statusTimeout(cfg["status_timeout"].as<double>(2)) {
        cal =
            rclcpp_action::create_client<Cal>(runtime->node, expand(cfg["calibration_action"].as<std::string>(), ctx));
        resetClient = runtime->node->create_client<Reset>(expand(cfg["reset_service"].as<std::string>(), ctx));
        targetClient = runtime->node->create_client<Target>(expand(cfg["target_service"].as<std::string>(), ctx));
        status = runtime->node->create_subscription<riptide_msgs2::msg::MappingTargetInfo>(
            expand(cfg["status_topic"].as<std::string>(), ctx), 10,
            [this](const riptide_msgs2::msg::MappingTargetInfo &msg) {
                std::lock_guard<std::mutex> lock(mutex);
                value.target = msg.target_object;
                value.locked = msg.lock_map;
                lastStatus = Steady::now();
            });
        timer = runtime->node->create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
    }
    ~UwrtMapping() override {
        // The host stops the executor before destroying providers. Cancel only
        // our accepted goal, with no callback that could outlive this instance.
        if (goal && rclcpp::ok())
            cal->async_cancel_goal(goal);
    }
    MappingState state() override {
        std::lock_guard<std::mutex> lock(mutex);
        return value;
    }
    void calibrate(const std::string &parent, const std::string &child, unsigned samples) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (value.calibrating || !cal->action_server_is_ready() || parent.empty() || child.empty() || parent == child ||
            samples == 0 || samples > 65535)
            return;
        value.calibrating = true;
        value.canceling = false;
        value.samples = 0;
        value.calibrationMessage = "Starting tag calibration...";
        calSince = Steady::now();
        awaiting = true;
        Cal::Goal request;
        request.monitor_parent = parent;
        request.monitor_child = child;
        request.samples = samples;
        auto options = rclcpp_action::Client<Cal>::SendGoalOptions();
        options.goal_response_callback = [this](Goal::SharedPtr accepted) {
            std::lock_guard<std::mutex> lock(mutex);
            awaiting = false;
            goal = accepted;
            if (!goal) {
                value.calibrating = value.canceling = false;
                value.calibrationMessage = "Calibration rejected";
                return;
            }
            if (value.canceling)
                cancelLocked();
            else
                value.calibrationMessage = "Collecting tag samples";
        };
        options.feedback_callback = [this](Goal::SharedPtr, Cal::Feedback::ConstSharedPtr feedback) {
            std::lock_guard<std::mutex> lock(mutex);
            value.samples = feedback->sample_count;
        };
        options.result_callback = [this](const Goal::WrappedResult &reply) {
            std::lock_guard<std::mutex> lock(mutex);
            goal.reset();
            awaiting = false;
            value.calibrating = value.canceling = false;
            if (reply.code == rclcpp_action::ResultCode::CANCELED)
                value.calibrationMessage = "Calibration canceled";
            else if (reply.code == rclcpp_action::ResultCode::SUCCEEDED && reply.result && reply.result->success)
                value.calibrationMessage = "Tag calibration complete";
            else
                value.calibrationMessage = "Calibration failed" + (reply.result ? ": " + reply.result->err_msg : "");
        };
        cal->async_send_goal(request, options);
    }
    void cancelCalibration() override {
        std::lock_guard<std::mutex> lock(mutex);
        if (value.calibrating) {
            value.canceling = true;
            cancelLocked();
        }
    }
    void reset() override {
        std::lock_guard<std::mutex> lock(mutex);
        if (value.resetting || !resetClient->service_is_ready())
            return;
        value.resetting = true;
        resetSince = Steady::now();
        value.resetMessage = "Resetting mapping...";
        auto epoch = ++resetEpoch;
        resetId = resetClient
                      ->async_send_request(std::make_shared<Reset::Request>(),
                                           [this, epoch](rclcpp::Client<Reset>::SharedFuture reply) {
                                               std::lock_guard<std::mutex> lock(mutex);
                                               if (epoch != resetEpoch)
                                                   return;
                                               value.resetting = false;
                                               auto response = reply.get();
                                               value.resetMessage =
                                                   response->success ? "Mapping reset" : "Mapping reset failed";
                                               if (!response->message.empty())
                                                   value.resetMessage += ": " + response->message;
                                           })
                      .request_id;
    }
    void setTarget(const std::string &target, bool locked) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (value.settingTarget || !targetClient->service_is_ready())
            return;
        auto request = std::make_shared<Target::Request>();
        request->target_info.target_object = target;
        request->target_info.lock_map = locked;
        value.settingTarget = true;
        targetSince = Steady::now();
        value.targetMessage = "Setting mapping target...";
        auto epoch = ++targetEpoch;
        targetId = targetClient
                       ->async_send_request(request,
                                            [this, epoch](rclcpp::Client<Target>::SharedFuture reply) {
                                                std::lock_guard<std::mutex> lock(mutex);
                                                if (epoch != targetEpoch)
                                                    return;
                                                reply.get();
                                                value.settingTarget = false;
                                                value.targetMessage =
                                                    "Target request accepted; observed state shown above";
                                            })
                       .request_id;
    }

  private:
    void cancelLocked() {
        value.calibrationMessage = goal ? "Canceling calibration..." : "Cancel requested; awaiting goal response";
        if (goal)
            cal->async_cancel_goal(goal, [this](rclcpp_action::Client<Cal>::CancelResponse::SharedPtr reply) {
                std::lock_guard<std::mutex> lock(mutex);
                if (value.calibrating && reply->return_code != 0)
                    value.calibrationMessage = "Cancel not accepted; calibration still active";
            });
    }
    void tick() {
        std::lock_guard<std::mutex> lock(mutex);
        const auto now = Steady::now();
        auto elapsed = [&](auto since) { return std::chrono::duration<double>(now - since).count(); };
        value.calibrationReady = cal->action_server_is_ready();
        value.resetReady = resetClient->service_is_ready();
        value.targetReady = targetClient->service_is_ready();
        value.fresh = elapsed(lastStatus) < statusTimeout;
        if (value.calibrating && !value.canceling && elapsed(calSince) > (awaiting ? timeout : calibrationTimeout)) {
            value.canceling = true;
            cancelLocked();
            value.calibrationMessage = "Calibration timed out; cancellation requested";
        }
        if (value.resetting && elapsed(resetSince) > timeout) {
            resetClient->remove_pending_request(resetId);
            ++resetEpoch;
            value.resetting = false;
            value.resetMessage = "Reset timed out; result unknown";
        }
        if (value.settingTarget && elapsed(targetSince) > timeout) {
            targetClient->remove_pending_request(targetId);
            ++targetEpoch;
            value.settingTarget = false;
            value.targetMessage = "Target request timed out; result unknown";
        }
    }
    std::shared_ptr<RosRuntime> runtime;
    std::mutex mutex;
    MappingState value;
    double timeout, calibrationTimeout, statusTimeout;
    bool awaiting = false;
    uint64_t resetEpoch = 0, targetEpoch = 0;
    int64_t resetId = 0, targetId = 0;
    Steady::time_point lastStatus{}, calSince{}, resetSince{}, targetSince{};
    Goal::SharedPtr goal;
    rclcpp_action::Client<Cal>::SharedPtr cal;
    rclcpp::Client<Reset>::SharedPtr resetClient;
    rclcpp::Client<Target>::SharedPtr targetClient;
    rclcpp::Subscription<riptide_msgs2::msg::MappingTargetInfo>::SharedPtr status;
    rclcpp::TimerBase::SharedPtr timer;
};
} // namespace
void registerUwrtMapping(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "uwrt.mapping",
        ProviderFactory{Kind::Mapping,
                        [](const YAML::Node &cfg) {
                            keys(cfg,
                                 {"calibration_action", "reset_service", "target_service", "status_topic",
                                  "request_timeout", "calibration_timeout", "status_timeout"},
                                 "uwrt.mapping");
                            required(cfg, {"calibration_action", "reset_service", "target_service", "status_topic"});
                            positive(cfg, "request_timeout", 3);
                            positive(cfg, "calibration_timeout", 60, 600);
                            positive(cfg, "status_timeout", 2);
                        },
                        [runtime](const YAML::Node &cfg, const Context &ctx) {
                            return std::make_shared<UwrtMapping>(runtime(ctx), cfg, ctx);
                        }});
}
} // namespace pool::panels
