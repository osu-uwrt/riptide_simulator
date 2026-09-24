#include "ros_runtime.hpp"
#include "pool_viewer/panels/ros_providers.hpp"
#include <glm/gtc/matrix_transform.hpp>

namespace pool::panels {
RosRuntime::RosRuntime(const Context &ctx)
    : node(std::make_shared<rclcpp::Node>("viewer_panels", "/" + ctx.robotNamespace,
                                          rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides(
                                              {rclcpp::Parameter("use_sim_time", ctx.useSimTime)}))),
      tf(node->get_clock()), listener(tf, node, false) {
    executor.add_node(node);
}
RosRuntime::~RosRuntime() {
    stop();
}
void RosRuntime::start() {
    if (!worker.joinable())
        worker = std::thread([this] { executor.spin(); });
}
void RosRuntime::stop() {
    executor.cancel();
    if (worker.joinable())
        worker.join();
}
struct RosProviders::Impl {
    std::shared_ptr<RosRuntime> runtime;
    std::shared_ptr<RosRuntime> get(const Context &ctx) {
        if (!runtime)
            runtime = std::make_shared<RosRuntime>(ctx);
        return runtime;
    }
};
RosProviders::RosProviders() : impl(std::make_shared<Impl>()) {}
RosProviders::~RosProviders() {
    stop();
}
void RosProviders::start() {
    if (impl->runtime)
        impl->runtime->start();
}
void RosProviders::stop() {
    if (impl->runtime)
        impl->runtime->stop();
}
void RosProviders::registerFactories(Registry &registry) {
    RuntimeFactory factory = [owner = impl](const Context &ctx) { return owner->get(ctx); };
    registerUwrtMotion(registry, factory);
    registerUwrtAutonomy(registry, factory);
    registerStandardMotion(registry, factory);
    registerSimRun(registry, factory);
    registerSimulationRate(registry, factory);
    registerUwrtActuators(registry, factory);
    registerUwrtMapping(registry, factory);
}
Pose poseMatrix(const geometry_msgs::msg::Transform &t) {
    auto &q = t.rotation;
    return glm::translate(Pose(1), glm::vec3(t.translation.x, t.translation.y, t.translation.z)) *
           glm::mat4_cast(glm::normalize(glm::quat(q.w, q.x, q.y, q.z)));
}
bool finitePose(const Pose &pose) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            if (!std::isfinite(pose[c][r]))
                return false;
    return true;
}
RosMotion::RosMotion(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx)
    : runtime(std::move(runtime)), config(cfg), context(ctx),
      baseFrame(expand(cfg["base_frame"].as<std::string>(), ctx)),
      commandFrame(expand(cfg["command_frame"].as<std::string>(), ctx)), poseTimeout(cfg["pose_timeout"].as<double>(1)),
      uiTimeout(cfg["ui_timeout"].as<double>(.75)), requestTimeout(cfg["request_timeout"].as<double>(3)) {
    value.frame = ctx.fixedFrame;
}
MotionState RosMotion::state() {
    std::lock_guard<std::mutex> lock(mutex);
    return value;
}
void RosMotion::touch() {
    std::lock_guard<std::mutex> lock(mutex);
    lastUi = Steady::now();
}
bool RosMotion::ready() const {
    return value.fresh && !value.blocked && !value.competing &&
           std::chrono::duration<double>(Steady::now() - lastUi).count() < uiTimeout;
}
void RosMotion::enable() {
    std::lock_guard<std::mutex> lock(mutex);
    if (!ready() || value.pending)
        return;
    killLocked("Enable requested");
    value.enabled = true;
    report();
}
void RosMotion::kill() {
    std::lock_guard<std::mutex> lock(mutex);
    killLocked("Kill requested");
}
void RosMotion::killLocked(const std::string &message) {
    session = true;
    value.enabled = false;
    value.pending = false;
    value.mode = Mode::Disabled;
    ++generation;
    cancelRequest();
    value.message = message;
    report();
    send(value.commanded, Mode::Disabled);
}
void RosMotion::block(bool active) {
    std::lock_guard<std::mutex> lock(mutex);
    if (active && !value.blocked) {
        ++generation;
        cancelRequest();
        value.pending = false;
        value.mode = Mode::Disabled;
        value.message = "Autonomy owns motion";
    }
    if (!active && value.blocked)
        value.message = value.enabled ? "Autonomy ended / Command to resume" : "Autonomy ended / enable to control";
    value.blocked = active;
}
void RosMotion::activate(Mode mode, const Pose &pose) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!value.enabled || !ready() || value.pending || !finitePose(pose) || mode == Mode::Disabled ||
        (mode == Mode::Feedforward && !value.supportsFeedforward))
        return;
    if (!modeReady()) {
        value.message = "Control mode service unavailable";
        return;
    }
    value.pending = true;
    pendingSince = Steady::now();
    value.message = "Changing control mode...";
    requestMode(++generation, mode, pose);
}
void RosMotion::complete(uint64_t epoch, Mode mode, const Pose &pose, bool success, const std::string &message) {
    std::lock_guard<std::mutex> lock(mutex);
    if (epoch != generation)
        return;
    value.pending = false;
    if (!success) {
        killLocked("Mode request failed: " + message);
        return;
    }
    if (!value.enabled || !ready())
        return;
    value.mode = mode;
    value.commanded = pose;
    value.hasCommand = true;
    ++value.revision;
    value.message = mode == Mode::Position ? "Position control" : "Feedforward control";
    send(pose, mode);
}
void RosMotion::drag(const Pose &pose) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!value.enabled || !ready() || value.pending || value.mode != Mode::Position || !finitePose(pose))
        return;
    value.commanded = pose;
    value.hasCommand = true;
    ++value.revision;
    send(pose, Mode::Position);
}
void RosMotion::startTimer() {
    timer =
        runtime->node->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                             std::chrono::duration<double>(config["heartbeat_period"].as<double>(.05))),
                                         [this] { tick(); });
}
void RosMotion::tick() {
    std::lock_guard<std::mutex> lock(mutex);
    const auto now = Steady::now();
    value.competing = now - competitor < std::chrono::seconds(1);
    if (now - observedSince > std::chrono::seconds(2))
        value.observedKilled.reset();
    try {
        const auto pose = runtime->tf.lookupTransform(context.fixedFrame, baseFrame, tf2::TimePointZero);
        value.actual = poseMatrix(pose.transform);
        commandFromFixed =
            poseMatrix(runtime->tf.lookupTransform(commandFrame, context.fixedFrame, tf2::TimePointZero).transform);
        const auto stamp = rclcpp::Time(pose.header.stamp).nanoseconds();
        if (stamp != poseStamp) {
            poseStamp = stamp;
            lastPose = now;
        }
        value.fresh = std::chrono::duration<double>(now - lastPose).count() < poseTimeout && finitePose(value.actual) &&
                      finitePose(commandFromFixed);
    } catch (const tf2::TransformException &) {
        value.fresh = false;
    }
    if (!session)
        value.message = value.fresh ? "Ready / enable to control" : "Waiting for fresh pose";
    if (value.enabled && std::chrono::duration<double>(now - lastUi).count() >= uiTimeout)
        killLocked("Viewer unresponsive; enable again");
    else if (value.enabled && !value.fresh)
        killLocked("Pose stale; enable again");
    else if (value.pending && std::chrono::duration<double>(now - pendingSince).count() > requestTimeout)
        killLocked("Control request timed out");
    if (session)
        report();
}
} // namespace pool::panels
