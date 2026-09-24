#include "ros_runtime.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <riptide_msgs2/action/execute_tree.hpp>
#include <riptide_msgs2/srv/list_trees.hpp>
#include <riptide_msgs2/msg/tree_stack.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <algorithm>

namespace pool::panels {
namespace {
using Execute = riptide_msgs2::action::ExecuteTree;
using Goal = rclcpp_action::ClientGoalHandle<Execute>;
using List = riptide_msgs2::srv::ListTrees;
class UwrtAutonomy final : public Autonomy {
  public:
    UwrtAutonomy(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx)
        : runtime(runtime), timeout(cfg["request_timeout"].as<double>(3)),
          stackTimeout(cfg["stack_timeout"].as<double>(5)) {
        const auto action = expand(cfg["action"].as<std::string>(), ctx);
        client = rclcpp_action::create_client<Execute>(runtime->node, action);
        list = runtime->node->create_client<List>(expand(cfg["list_service"].as<std::string>(), ctx));
        stack = runtime->node->create_subscription<riptide_msgs2::msg::TreeStack>(
            expand(cfg["stack_topic"].as<std::string>(), ctx), rclcpp::SystemDefaultsQoS(),
            [this](const riptide_msgs2::msg::TreeStack &msg) {
                std::lock_guard<std::mutex> lock(mutex);
                value.stack = msg.stack;
                stackReceived = Steady::now();
            });
        status = runtime->node->create_subscription<action_msgs::msg::GoalStatusArray>(
            action + "/_action/status", rclcpp::QoS(1).reliable().transient_local(),
            [this](const action_msgs::msg::GoalStatusArray &msg) {
                std::lock_guard<std::mutex> lock(mutex);
                externalBusy = std::any_of(msg.status_list.begin(), msg.status_list.end(),
                                           [](const auto &item) { return item.status >= 1 && item.status <= 3; });
                if (externalBusy && !value.busy && !goal && !awaitingGoal) {
                    externalExecution = true;
                    value.activeTree.clear();
                    value.failed = false;
                }
                value.busy = externalBusy || awaitingGoal || bool(goal);
                if (!value.busy && externalExecution) {
                    externalExecution = false;
                    value.message = "External tree ended";
                }
                if (externalBusy && !goal && !awaitingGoal)
                    value.message = "Tree running (external)";
                if (!value.busy && stopping) {
                    stopping = false;
                    value.pending = false;
                    value.message = "Stopped";
                }
            });
        timer = runtime->node->create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
    }
    ~UwrtAutonomy() override {
        // Closing an observer must not cancel a tree started by another client.
        if ((goal || awaitingGoal) && rclcpp::ok())
            stop();
    }
    MissionState state() override {
        std::lock_guard<std::mutex> lock(mutex);
        return value;
    }
    void refresh() override {
        std::lock_guard<std::mutex> lock(mutex);
        refreshLocked();
    }
    void start(const std::string &tree) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (value.busy || !client->action_server_is_ready() ||
            std::find(value.trees.begin(), value.trees.end(), tree) == value.trees.end())
            return;
        stopping = false;
        externalExecution = false;
        awaitingGoal = true;
        value.pending = true;
        value.busy = true;
        value.failed = false;
        value.activeTree = tree;
        value.stack.clear();
        value.message = "Starting...";
        operationSince = Steady::now();
        Execute::Goal request;
        request.tree = tree;
        auto options = rclcpp_action::Client<Execute>::SendGoalOptions();
        options.goal_response_callback = [this](Goal::SharedPtr accepted) {
            std::lock_guard<std::mutex> lock(mutex);
            awaitingGoal = false;
            value.pending = false;
            if (!accepted) {
                value.busy = externalBusy;
                value.failed = true;
                value.message = "Tree rejected";
                return;
            }
            goal = accepted;
            value.busy = true;
            value.message = "Running";
            if (stopping) {
                client->async_cancel_goal(goal);
                value.pending = true;
                value.message = "Stopping...";
            }
        };
        options.feedback_callback = [this](Goal::SharedPtr, Execute::Feedback::ConstSharedPtr feedback) {
            std::lock_guard<std::mutex> lock(mutex);
            value.stack = feedback->stack.stack;
            stackReceived = Steady::now();
        };
        options.result_callback = [this](const Goal::WrappedResult &result) {
            std::lock_guard<std::mutex> lock(mutex);
            goal.reset();
            awaitingGoal = false;
            value.pending = false;
            value.busy = externalBusy;
            stopping = false;
            const bool canceled = result.code == rclcpp_action::ResultCode::CANCELED;
            value.failed = !canceled && (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
                                         result.result->error || result.result->returncode != 2);
            value.message = canceled ? "Stopped" : value.failed ? "Tree failed" : "Completed";
            if (result.result)
                value.message += " (code " + std::to_string(result.result->returncode) + ")";
        };
        client->async_send_goal(request, options);
    }
    void stop() override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!client->action_server_is_ready()) {
            value.message = "Autonomy unavailable; stop unconfirmed";
            return;
        }
        stopping = true;
        value.pending = true;
        operationSince = Steady::now();
        value.message = "Stopping...";
        // Like RViz, Stop also handles a tree started by another client of this
        // configured action server. A late local goal acceptance is canceled too.
        client->async_cancel_all_goals([this](rclcpp_action::Client<Execute>::CancelResponse::SharedPtr reply) {
            std::lock_guard<std::mutex> lock(mutex);
            if (reply->return_code != 0) {
                value.pending = false;
                value.message = "Stop not accepted; retry or Kill";
            } else if (!awaitingGoal && !goal && !externalBusy) {
                value.pending = false;
                value.busy = false;
                stopping = false;
                value.message = "Stopped";
            }
        });
    }

  private:
    void refreshLocked() {
        if (value.refreshing)
            return;
        lastRefresh = Steady::now();
        if (!list->service_is_ready()) {
            value.message = "Tree list unavailable";
            return;
        }
        value.refreshing = true;
        const auto epoch = ++refreshEpoch;
        requestId = list->async_send_request(std::make_shared<List::Request>(),
                                             [this, epoch](rclcpp::Client<List>::SharedFuture reply) {
                                                 std::lock_guard<std::mutex> lock(mutex);
                                                 if (epoch != refreshEpoch)
                                                     return;
                                                 value.refreshing = false;
                                                 value.trees = reply.get()->trees;
                                                 std::sort(value.trees.begin(), value.trees.end());
                                                 value.trees.erase(std::unique(value.trees.begin(), value.trees.end()),
                                                                   value.trees.end());
                                                 if (!value.busy)
                                                     value.message = "Ready";
                                             })
                        .request_id;
    }
    void tick() {
        std::lock_guard<std::mutex> lock(mutex);
        const auto now = Steady::now();
        value.connected = client->action_server_is_ready();
        value.stackStale = value.busy && std::chrono::duration<double>(now - stackReceived).count() > stackTimeout;
        if (value.refreshing && std::chrono::duration<double>(now - lastRefresh).count() > timeout) {
            list->remove_pending_request(requestId);
            ++refreshEpoch;
            value.refreshing = false;
            value.message = "Tree list request timed out";
        }
        if (value.connected && value.trees.empty() && !value.refreshing && now - lastRefresh > std::chrono::seconds(3))
            refreshLocked();
        if (value.pending && std::chrono::duration<double>(now - operationSince).count() > timeout) {
            value.pending = false;
            value.failed = true;
            value.message = "Autonomy request timed out; stop unconfirmed";
            stopping = true; // late goal acceptance must not start an unattended tree
            if (value.connected)
                client->async_cancel_all_goals();
        }
        if (!value.connected && value.busy)
            value.message = "Autonomy disconnected; execution unknown";
    }
    std::shared_ptr<RosRuntime> runtime;
    std::mutex mutex;
    MissionState value;
    bool externalBusy = false, awaitingGoal = false, stopping = false, externalExecution = false;
    double timeout, stackTimeout;
    uint64_t refreshEpoch = 0;
    int64_t requestId = 0;
    Steady::time_point lastRefresh{}, operationSince{}, stackReceived{};
    Goal::SharedPtr goal;
    rclcpp_action::Client<Execute>::SharedPtr client;
    rclcpp::Client<List>::SharedPtr list;
    rclcpp::Subscription<riptide_msgs2::msg::TreeStack>::SharedPtr stack;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status;
    rclcpp::TimerBase::SharedPtr timer;
};
} // namespace
void registerUwrtAutonomy(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "uwrt.autonomy",
        ProviderFactory{Kind::Autonomy,
                        [](const YAML::Node &cfg) {
                            keys(cfg, {"action", "list_service", "stack_topic", "request_timeout", "stack_timeout"},
                                 "uwrt.autonomy");
                            required(cfg, {"action", "list_service", "stack_topic"});
                            positive(cfg, "request_timeout", 3);
                            positive(cfg, "stack_timeout", 5);
                        },
                        [runtime](const YAML::Node &cfg, const Context &ctx) {
                            return std::make_shared<UwrtAutonomy>(runtime(ctx), cfg, ctx);
                        }});
}
} // namespace pool::panels
