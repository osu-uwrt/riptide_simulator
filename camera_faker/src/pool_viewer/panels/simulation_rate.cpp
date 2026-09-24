#include "ros_runtime.hpp"
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace pool::panels {
namespace {
using Get = rcl_interfaces::srv::GetParameters;
using Set = rcl_interfaces::srv::SetParameters;
using Trigger = std_srvs::srv::Trigger;
class SimulationRate final : public Simulation {
  public:
    SimulationRate(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx)
        : runtime(runtime), parameter(cfg["parameter"].as<std::string>()),
          timeout(cfg["request_timeout"].as<double>(3)) {
        const auto remote = expand(cfg["node"].as<std::string>(), ctx);
        get = runtime->node->create_client<Get>(remote + "/get_parameters");
        set = runtime->node->create_client<Set>(remote + "/set_parameters");
        value.maxRate = cfg["max_rate"].as<double>(10);
        value.resumeRate = std::min(1.0, value.maxRate);
        if (cfg["sync_service"])
            syncClient = runtime->node->create_client<Trigger>(expand(cfg["sync_service"].as<std::string>(), ctx));
        if (cfg["reset_service"])
            resetClient = runtime->node->create_client<Trigger>(expand(cfg["reset_service"].as<std::string>(), ctx));
        timer = runtime->node->create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
    }
    SimulationState state() override {
        std::lock_guard<std::mutex> lock(mutex);
        return value;
    }
    void setRate(double rate) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!std::isfinite(rate) || rate < 0 || rate > value.maxRate)
            return;
        setRateLocked(rate);
    }
    void setPaused(bool paused) override {
        std::lock_guard<std::mutex> lock(mutex);
        setRateLocked(paused ? 0 : value.resumeRate);
    }
    void sync() override {
        trigger(syncClient, "Sync");
    }
    void reset() override {
        trigger(resetClient, "Reset");
    }

  private:
    void trigger(const rclcpp::Client<Trigger>::SharedPtr &client, const std::string &name) {
        std::lock_guard<std::mutex> lock(mutex);
        if (value.operationPending || !client || !client->service_is_ready())
            return;
        value.operationPending = true;
        value.operationMessage = name + ": requested";
        operationSince = Steady::now();
        operationClient = client;
        const auto epoch = ++operationEpoch;
        operationId = client
                          ->async_send_request(std::make_shared<Trigger::Request>(),
                                               [this, epoch, name](rclcpp::Client<Trigger>::SharedFuture future) {
                                                   std::lock_guard<std::mutex> lock(mutex);
                                                   if (epoch != operationEpoch)
                                                       return;
                                                   value.operationPending = false;
                                                   const auto reply = future.get();
                                                   value.operationMessage =
                                                       name + (reply->success ? ": " : " failed: ") + reply->message;
                                               })
                          .request_id;
    }
    void setRateLocked(double rate) {
        if (!value.connected || value.pending || !set->service_is_ready())
            return;
        // A read started before this write must not overwrite its confirmation.
        if (reading) {
            get->remove_pending_request(getId);
            reading = false;
            ++readEpoch;
        }
        auto request = std::make_shared<Set::Request>();
        rcl_interfaces::msg::Parameter item;
        item.name = parameter;
        item.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        item.value.double_value = rate;
        request->parameters.push_back(item);
        value.pending = true;
        value.message = "Applying simulation speed...";
        setSince = Steady::now();
        auto epoch = ++writeEpoch;
        setId = set->async_send_request(request,
                                        [this, epoch, rate](rclcpp::Client<Set>::SharedFuture reply) {
                                            std::lock_guard<std::mutex> lock(mutex);
                                            if (epoch != writeEpoch)
                                                return;
                                            value.pending = false;
                                            const auto response = reply.get();
                                            if (response->results.size() != 1 || !response->results[0].successful) {
                                                error = response->results.empty()
                                                            ? "Speed request failed"
                                                            : "Speed rejected: " + response->results[0].reason;
                                                value.message = error;
                                            } else {
                                                error.clear();
                                                if (rate > 0)
                                                    value.resumeRate = rate;
                                                value.message = "Speed accepted; reading simulator state";
                                            }
                                            lastPoll = {};
                                        })
                    .request_id;
    }

    void tick() {
        std::lock_guard<std::mutex> lock(mutex);
        const auto now = Steady::now();
        auto elapsed = [&](auto t) { return std::chrono::duration<double>(now - t).count(); };
        value.syncReady = syncClient && syncClient->service_is_ready();
        value.resetReady = resetClient && resetClient->service_is_ready();
        if (value.operationPending && elapsed(operationSince) > timeout) {
            operationClient->remove_pending_request(operationId);
            ++operationEpoch;
            value.operationPending = false;
            value.operationMessage = "Simulation request timed out; result unknown";
        }
        value.connected = get->service_is_ready() && set->service_is_ready() && elapsed(lastRead) < timeout;
        if (value.pending && elapsed(setSince) > timeout) {
            set->remove_pending_request(setId);
            ++writeEpoch;
            value.pending = false;
            value.message = error = "Speed request timed out; result unknown";
        }
        if (reading && elapsed(lastPoll) > timeout) {
            get->remove_pending_request(getId);
            ++readEpoch;
            reading = false;
            value.connected = false;
            value.message = "Simulator speed read timed out";
        }
        if (!get->service_is_ready() || !set->service_is_ready()) {
            value.message = "Simulator unavailable";
            return;
        }
        if (reading || value.pending || elapsed(lastPoll) < .5)
            return;
        reading = true;
        lastPoll = now;
        const auto epoch = ++readEpoch;
        auto request = std::make_shared<Get::Request>();
        request->names = {parameter};
        getId =
            get->async_send_request(
                   request,
                   [this, epoch](rclcpp::Client<Get>::SharedFuture reply) {
                       std::lock_guard<std::mutex> lock(mutex);
                       if (epoch != readEpoch)
                           return;
                       reading = false;
                       const auto response = reply.get();
                       if (response->values.size() != 1 ||
                           response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE ||
                           !std::isfinite(response->values[0].double_value) || response->values[0].double_value < 0) {
                           value.connected = false;
                           lastRead = {};
                           value.message = "Simulator speed parameter unavailable / invalid";
                           return;
                       }
                       value.rate = response->values[0].double_value;
                       if (value.rate > 0)
                           value.resumeRate = value.rate;
                       lastRead = Steady::now();
                       value.connected = true;
                       value.message =
                           error.empty() ? (value.rate == 0 ? "Simulation paused" : "Simulation running") : error;
                   })
                .request_id;
    }
    std::shared_ptr<RosRuntime> runtime;
    std::mutex mutex;
    SimulationState value;
    std::string parameter, error;
    double timeout;
    bool reading = false;
    uint64_t readEpoch = 0, writeEpoch = 0;
    int64_t getId = 0, setId = 0;
    Steady::time_point lastPoll{}, lastRead{}, setSince{};
    rclcpp::Client<Trigger>::SharedPtr syncClient, resetClient, operationClient;
    uint64_t operationEpoch = 0;
    int64_t operationId = 0;
    Steady::time_point operationSince{};
    rclcpp::Client<Get>::SharedPtr get;
    rclcpp::Client<Set>::SharedPtr set;
    rclcpp::TimerBase::SharedPtr timer;
};
} // namespace
void registerSimulationRate(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "ros.simulation_rate",
        ProviderFactory{Kind::Simulation,
                        [](const YAML::Node &cfg) {
                            keys(cfg,
                                 {"node", "parameter", "max_rate", "request_timeout", "sync_service", "reset_service"},
                                 "ros.simulation_rate");
                            required(cfg, {"node", "parameter"});
                            positive(cfg, "request_timeout", 3);
                            positive(cfg, "max_rate", 10, 1000);
                        },
                        [runtime](const YAML::Node &cfg, const Context &ctx) {
                            return std::make_shared<SimulationRate>(runtime(ctx), cfg, ctx);
                        }});
}
} // namespace pool::panels
