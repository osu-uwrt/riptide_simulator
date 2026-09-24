#include "pool_viewer/panels/composition.hpp"
#include "pool_viewer/panels/ros_providers.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <riptide_msgs2/msg/controller_command.hpp>
#include <riptide_msgs2/msg/kill_switch_report.hpp>
#include <riptide_msgs2/action/execute_tree.hpp>
#include <riptide_msgs2/srv/list_trees.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
#include "pool_viewer/panels/pose_math.hpp"
using namespace pool::panels;
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cassert>
#include <iostream>
#include <limits>
#include <unistd.h>

using namespace std::chrono_literals;
using Command = riptide_msgs2::msg::ControllerCommand;
using Kill = riptide_msgs2::msg::KillSwitchReport;

int main(int argc, char **argv) {
    // This test publishes commands only in a dedicated ROS domain and namespace.
    setenv("ROS_DOMAIN_ID", "183", 1);
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
    rclcpp::init(argc, argv);
    const std::string robot = "operator_test_" + std::to_string(getpid());
    auto node = std::make_shared<rclcpp::Node>("operator_test", "/" + robot);
    tf2_ros::Buffer targetTf(node->get_clock());
    tf2_ros::TransformListener targetListener(targetTf, node, false);
    tf2_ros::TransformBroadcaster tf(node);
    tf2_ros::StaticTransformBroadcaster staticTf(node);
    geometry_msgs::msg::TransformStamped world;
    world.header.frame_id = "world";
    world.child_frame_id = "map";
    world.transform.translation.x = 10;
    world.transform.rotation.w = world.transform.rotation.z = std::sqrt(.5);
    staticTf.sendTransform(world);
    geometry_msgs::msg::TransformStamped pose;
    pose.header.frame_id = "map";
    pose.child_frame_id = robot + "/base_link";
    pose.transform.translation.x = 1;
    pose.transform.translation.y = 2;
    pose.transform.translation.z = -1;
    pose.transform.rotation.w = 1;
    auto freshPose = [&] {
        pose.header.stamp = node->now();
        tf.sendTransform(pose);
    };
    freshPose();
    std::vector<Kill> reports;
    std::vector<Command> lin, ang;
    auto ks =
        node->create_subscription<Kill>("command/software_kill", 10, [&](const Kill &m) { reports.push_back(m); });
    auto ls = node->create_subscription<Command>("controller/linear", 10, [&](const Command &m) { lin.push_back(m); });
    auto as = node->create_subscription<Command>("controller/angular", 10, [&](const Command &m) { ang.push_back(m); });
    bool serviceSuccess = true;
    std::shared_ptr<rmw_request_id_t> delayed;
    bool delay = false;
    auto service = node->create_service<std_srvs::srv::SetBool>(
        "setTeleop", [&](const std::shared_ptr<rmw_request_id_t> header,
                         const std::shared_ptr<std_srvs::srv::SetBool::Request> req) {
            assert(!req->data);
            delayed = header;
        });
    // Send deferred replies from the test loop, after the callback has returned.
    Registry registry;
    registerPanels(registry);
    RosProviders ros;
    ros.registerFactories(registry);
    auto cfg = YAML::LoadFile(argv[1]);
    auto composition = std::make_unique<Composition>(cfg, Context{robot, "map", false, false}, registry);
    auto control = std::dynamic_pointer_cast<Motion>(composition->providers().at("motion"));
    auto mission = std::dynamic_pointer_cast<Autonomy>(composition->providers().at("mission"));
    using Execute = riptide_msgs2::action::ExecuteTree;
    using Goal = rclcpp_action::ServerGoalHandle<Execute>;
    std::shared_ptr<Goal> running;
    bool reject = false;
    auto action = rclcpp_action::create_server<Execute>(
        node, "autonomy/run_tree",
        [&](const auto &, auto) {
            return reject ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](auto) { return rclcpp_action::CancelResponse::ACCEPT; }, [&](auto accepted) { running = accepted; });
    auto list = node->create_service<riptide_msgs2::srv::ListTrees>(
        "autonomy/list_trees",
        [](riptide_msgs2::srv::ListTrees::Request::SharedPtr,
           riptide_msgs2::srv::ListTrees::Response::SharedPtr reply) { reply->trees = {"/trees/test.xml"}; });
    ros.start();
    auto spin = [&](double seconds, bool touch = true, bool fresh = true) {
        auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < end) {
            if (control && touch)
                composition->touch();
            if (fresh)
                freshPose();
            rclcpp::spin_some(node);
            if (delayed && !delay) {
                std_srvs::srv::SetBool::Response reply;
                reply.success = serviceSuccess;
                service->send_response(*delayed, reply);
                delayed.reset();
            }
            if (running && running->is_canceling()) {
                auto result = std::make_shared<Execute::Result>();
                running->canceled(result);
                running.reset();
            }
            std::this_thread::sleep_for(5ms);
        }
    };
    spin(1);
    assert(control->state().fresh);
    assert(reports.empty() && lin.empty() && ang.empty()); // passive until clicked
    control->enable();
    spin(.2);
    assert(control->state().enabled && !(control->state().mode == Mode::Position));
    assert(!reports.back().switch_asserting_kill && reports.back().switch_needs_update);
    assert(lin.back().mode == Command::DISABLED);
    control->activate(Mode::Position, control->state().actual);
    spin(.3);
    assert((control->state().mode == Mode::Position) && lin.back().mode == Command::POSITION);
    const auto setpoint = targetTf.lookupTransform("world", "ghost/base_link", tf2::TimePointZero);
    assert(std::abs(setpoint.transform.translation.x - 8) < 1e-4);
    assert(std::abs(setpoint.transform.translation.y - 1) < 1e-4);
    assert(std::abs(setpoint.transform.rotation.z - std::sqrt(.5)) < 1e-4);
    // map (1,2,-1) -> world (8,1,-1), including the world/map yaw.
    assert(std::abs(lin.back().setpoint_vect.x - 8) < 1e-4);
    assert(std::abs(lin.back().setpoint_vect.y - 1) < 1e-4);
    assert(std::abs(ang.back().setpoint_quat.z - std::sqrt(.5)) < 1e-4);
    const auto targetAngles = glm::radians(glm::vec3(20, -15, 30));
    auto target = pool::rpyPose({4, 5, -2}, targetAngles);
    control->drag(target);
    spin(.1);
    assert(std::abs(lin.back().setpoint_vect.x - 5) < 1e-4);
    assert(std::abs(lin.back().setpoint_vect.y - 4) < 1e-4);
    assert(std::abs(lin.back().setpoint_vect.z + 2) < 1e-4);
    const auto expectedQuat = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0, 0, 1)) * glm::quat(targetAngles);
    const auto &q = ang.back().setpoint_quat;
    assert(std::abs(glm::dot(expectedQuat, glm::quat(q.w, q.x, q.y, q.z))) > 1 - 1e-5);
    // Each rotation handle changes its named Euler angle, preserving the others.
    for (int i = 0; i < 3; ++i) {
        auto edited = targetAngles;
        edited[i] += .2f;
        const auto actual = glm::mat4_cast(glm::quat(edited));
        const auto expected =
            glm::rotate(glm::mat4(1), .2f, pool::rpyAxis(targetAngles, i)) * glm::mat4_cast(glm::quat(targetAngles));
        for (int c = 0; c < 4; ++c)
            assert(glm::length(actual[c] - expected[c]) < 1e-5);
    }
    // NaNs cannot enter the controller.
    const auto count = lin.size();
    target[3].x = std::numeric_limits<float>::quiet_NaN();
    control->drag(target);
    spin(.1);
    assert(lin.size() == count);
    spin(.9, false); // render hang, while ROS and TF continue
    assert(!control->state().enabled && reports.back().switch_asserting_kill);
    assert(lin.back().mode == Command::DISABLED && ang.back().mode == Command::DISABLED);
    spin(.1);
    assert(!control->state().enabled); // no automatic resume
    control->enable();
    spin(.1);
    spin(1.15, true, false); // estimator stops, UI remains responsive
    assert(!control->state().enabled && !control->state().fresh);
    spin(.1);
    control->enable();
    delay = true;
    control->activate(Mode::Position, control->state().actual);
    spin(.1);
    assert(delayed && control->state().pending);
    control->kill();
    const auto killedCount = lin.size();
    delay = false;
    spin(.2);
    assert(!(control->state().mode == Mode::Position) && !control->state().enabled);
    for (size_t i = killedCount; i < lin.size(); ++i)
        assert(lin[i].mode == Command::DISABLED);
    control->enable();
    serviceSuccess = false;
    control->activate(Mode::Position, control->state().actual);
    spin(.2);
    assert(!control->state().enabled);
    serviceSuccess = true;
    control->enable();
    delay = true;
    control->activate(Mode::Position, control->state().actual);
    spin(3.2);
    assert(!control->state().enabled && !control->state().pending);
    assert(control->state().message == "Control request timed out");
    delay = false;
    spin(.1);
    assert(!(control->state().mode == Mode::Position));
    control->enable();
    control->activate(Mode::Position, control->state().actual);
    spin(.2);
    assert(mission->state().connected && mission->state().trees.size() == 1);
    mission->start("/trees/test.xml");
    spin(.4);
    assert(running && mission->state().busy && control->state().blocked);
    const auto beforeAutonomyTf = targetTf.lookupTransform("map", "ghost/base_link", tf2::TimePointZero).header.stamp;
    spin(.15);
    assert(targetTf.lookupTransform("map", "ghost/base_link", tf2::TimePointZero).header.stamp != beforeAutonomyTf);
    assert(control->state().mode == Mode::Disabled);
    auto feedback = std::make_shared<Execute::Feedback>();
    feedback->stack.stack = {"Root", "Dive"};
    running->publish_feedback(feedback);
    spin(.2);
    assert(mission->state().stack == feedback->stack.stack);
    const auto before = lin.size();
    control->drag(glm::mat4(1));
    spin(.1);
    assert(lin.size() == before);
    mission->stop();
    spin(.5);
    assert(!mission->state().busy && !control->state().blocked && control->state().mode == Mode::Disabled);
    assert(mission->state().stack.size() == 2); // no forged empty stack
    reject = true;
    mission->start("/trees/test.xml");
    spin(.3);
    assert(mission->state().failed && !mission->state().busy);
    reject = false;
    mission->start("/trees/test.xml");
    spin(.3);
    assert(running);
    auto result = std::make_shared<Execute::Result>();
    result->returncode = 3;
    running->succeed(result);
    running.reset();
    spin(.3);
    assert(mission->state().failed && !mission->state().busy);
    mission->start("/trees/test.xml");
    spin(.3);
    result->returncode = 2;
    running->succeed(result);
    running.reset();
    spin(.3);
    assert(!mission->state().failed && !mission->state().busy);
    // A late goal response after timeout is canceled, never left executing.
    mission->start("/trees/test.xml");
    auto untilLate = std::chrono::steady_clock::now() + 3200ms;
    while (std::chrono::steady_clock::now() < untilLate) {
        composition->touch();
        freshPose();
        std::this_thread::sleep_for(5ms);
    }
    assert(mission->state().failed && mission->state().busy && control->state().blocked);
    spin(.6);
    assert(!mission->state().busy && !running);
    // Discover a goal submitted by another client, including after local results.
    auto external = rclcpp_action::create_client<Execute>(node, "autonomy/run_tree");
    spin(.3);
    Execute::Goal externalRequest;
    externalRequest.tree = "/trees/test.xml";
    external->async_send_goal(externalRequest);
    spin(.4);
    assert(mission->state().busy && control->state().blocked);
    mission->stop();
    spin(.5);
    assert(!mission->state().busy);
    control->activate(Mode::Feedforward, control->state().actual);
    spin(.3);
    assert(lin.back().mode == Command::FEEDFORWARD && ang.back().mode == Command::FEEDFORWARD);
    auto ffCount = lin.size();
    control->drag(glm::mat4(1));
    spin(.1);
    assert(lin.size() == ffCount);
    auto competitor = node->create_publisher<Kill>("command/software_kill", 10);
    spin(.3);
    Kill other;
    other.kill_switch_id = Kill::KILL_SWITCH_RQT_CONTROLLER;
    other.sender_id = "rviz_test";
    competitor->publish(other);
    spin(.1);
    assert(control->state().competing && !control->state().enabled);
    control->enable();
    assert(!control->state().enabled);
    spin(1.1);
    control->enable();
    spin(.1);
    assert(control->state().enabled);
    ros.stop();
    control.reset();
    mission.reset();
    composition.reset();
    spin(.2);
    assert(reports.back().switch_asserting_kill && lin.back().mode == Command::DISABLED);

    // A second protocol reuses the exact same motion capability/panel.
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    auto poseSub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "control/pose", 10, [&](const geometry_msgs::msg::PoseStamped &m) { poses.push_back(m); });
    std::vector<bool> enables;
    auto enabler = node->create_service<std_srvs::srv::SetBool>(
        "control/enable",
        [&](std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr reply) {
            enables.push_back(req->data);
            reply->success = true;
        });
    RosProviders standardRos;
    Registry standardRegistry;
    registerPanels(standardRegistry);
    standardRos.registerFactories(standardRegistry);
    auto standard =
        std::make_unique<Composition>(YAML::LoadFile(argv[2]), Context{robot, "map", false, false}, standardRegistry);
    auto generic = std::dynamic_pointer_cast<Motion>(standard->providers().at("vehicle"));
    standardRos.start();
    auto waitStandard = [&](double seconds) {
        auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < until) {
            standard->touch();
            freshPose();
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(5ms);
        }
    };
    waitStandard(.6);
    assert(enables.empty() && poses.empty());
    generic->enable();
    waitStandard(.3);
    assert(generic->state().enabled && !generic->state().supportsFeedforward);
    generic->activate(Mode::Position, pool::rpyPose({2, 3, -1}, {.2, -.1, .5}));
    waitStandard(.3);
    assert(!poses.empty() && poses.back().header.frame_id == "map" &&
           std::abs(poses.back().pose.position.x - 2) < 1e-4);
    generic->kill();
    waitStandard(.2);
    assert(!enables.back());
    standardRos.stop();
    generic.reset();
    standard.reset();

    // Pointer geometry: perspective rays, XY/depth axes, parallel rejection.
    const auto view = glm::lookAt(glm::vec3(3, -4, 5), glm::vec3(0), glm::vec3(0, 0, 1));
    const auto projection = glm::perspective(glm::radians(55.f), 1.f, .1f, 100.f);
    auto pixel = [&](glm::vec3 p) {
        auto clip = projection * view * glm::vec4(p, 1);
        return glm::vec2((clip.x / clip.w + 1) * 400, (1 - clip.y / clip.w) * 400);
    };
    auto ray = pool::screenRay(projection * view, pixel({1, 2, 0}), {800, 800});
    glm::vec3 point;
    assert(pool::planeHit(ray, {0, 0, 0}, {0, 0, 1}, point));
    assert(glm::length(point - glm::vec3(1, 2, 0)) < 1e-4);
    for (const auto axis : {glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1)}) {
        ray = pool::screenRay(projection * view, pixel(axis * 1.3f), {800, 800});
        float along;
        assert(pool::axisHit(ray, {0, 0, 0}, axis, along));
        assert(std::abs(along - 1.3f) < 1e-4);
    }
    float along;
    assert(!pool::axisHit({{0, 0, 0}, {0, 0, 1}}, {0, 0, 0}, {0, 0, 1}, along));
    assert(!pool::planeHit({{0, 0, 1}, {1, 0, 0}}, {0, 0, 0}, {0, 0, 1}, point));
    std::cout << "PASS: operator commands, frames, watchdogs, ownership, async cancellation, shutdown and dragging\n";
    rclcpp::shutdown();
}
