#pragma once
#include "pool_viewer/panels/composition.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <glm/gtc/quaternion.hpp>
#include <mutex>
#include <thread>

namespace pool::panels {
using Steady = std::chrono::steady_clock;
struct RosRuntime {
    explicit RosRuntime(const Context &ctx);
    ~RosRuntime();
    void start();
    void stop();
    std::shared_ptr<rclcpp::Node> node;
    tf2_ros::Buffer tf;
    tf2_ros::TransformListener listener;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::thread worker;
};
using RuntimeFactory = std::function<std::shared_ptr<RosRuntime>(const Context &)>;
void registerUwrtMotion(Registry &, const RuntimeFactory &);
void registerUwrtAutonomy(Registry &, const RuntimeFactory &);
void registerUwrtMapping(Registry &, const RuntimeFactory &);
void registerUwrtActuators(Registry &, const RuntimeFactory &);
void registerSimulationRate(Registry &, const RuntimeFactory &);
void registerSimRun(Registry &, const RuntimeFactory &);
void registerStandardMotion(Registry &, const RuntimeFactory &);
Pose poseMatrix(const geometry_msgs::msg::Transform &);
bool finitePose(const Pose &);

// Shared ROS mechanics; protocol-specific types and endpoints stay in adapters.
class RosMotion : public Motion {
  public:
    RosMotion(std::shared_ptr<RosRuntime>, const YAML::Node &, const Context &);
    MotionState state() override;
    void touch() override;
    void enable() override;
    void kill() override;
    void activate(Mode, const Pose &) override;
    void drag(const Pose &) override;
    void block(bool) override;

  protected:
    void startTimer();
    void tick();
    bool ready() const;
    void killLocked(const std::string &);
    void complete(uint64_t epoch, Mode, const Pose &, bool success, const std::string &message);
    virtual void send(const Pose &, Mode) = 0;
    virtual void report() = 0;
    virtual bool modeReady() = 0;
    virtual void requestMode(uint64_t, Mode, const Pose &) = 0;
    virtual void cancelRequest() {}
    std::shared_ptr<RosRuntime> runtime;
    YAML::Node config;
    Context context;
    std::mutex mutex;
    MotionState value;
    Pose commandFromFixed{1};
    std::string baseFrame, commandFrame;
    bool session = false;
    uint64_t generation = 0;
    int64_t poseStamp = -1;
    double poseTimeout, uiTimeout, requestTimeout;
    Steady::time_point lastUi = Steady::now(), lastPose{}, pendingSince{}, competitor{}, observedSince{};
    rclcpp::TimerBase::SharedPtr timer;
};
} // namespace pool::panels
