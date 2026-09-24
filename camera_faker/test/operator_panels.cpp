#include "pool_viewer/panels/composition.hpp"
#include "pool_viewer/panels/ros_providers.hpp"
#include <chameleon_tf_msgs/action/model_frame.hpp>
#include <riptide_msgs2/msg/mapping_target_info.hpp>
#include <riptide_msgs2/msg/actuator_status.hpp>
#include <riptide_msgs2/srv/mapping_target.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <imgui.h>
#include <imgui_internal.h>
#include <cassert>
#include <thread>
#include <unistd.h>
#include <iostream>
using namespace pool::panels;
using namespace std::chrono_literals;
using Cal = chameleon_tf_msgs::action::ModelFrame;
using Goal = rclcpp_action::ServerGoalHandle<Cal>;
using Target = riptide_msgs2::srv::MappingTarget;
using Reset = std_srvs::srv::Trigger;
int main(int argc, char **argv) {
    setenv("ROS_DOMAIN_ID", "184", 1);
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
    rclcpp::init(argc, argv);
    auto ns = "new_panels_" + std::to_string(getpid());
    auto node = std::make_shared<rclcpp::Node>("mock", "/" + ns);
    Registry registry;
    registerPanels(registry);
    RosProviders ros;
    ros.registerFactories(registry);
    auto config = YAML::LoadFile(argv[1]);
    const auto toolsConfig = YAML::LoadFile(argv[2]);
    for (const auto &entry : toolsConfig["providers"])
        config["providers"][entry.first.as<std::string>()] = YAML::Clone(entry.second);
    config["tools"] = YAML::Clone(toolsConfig["tools"]);
    config["providers"]["simulation"]["options"]["node"] = "mock";
    config["providers"]["simulation"]["options"]["request_timeout"] = .75;
    node->declare_parameter<double>("real_time_factor", 1.0);
    int syncRequests = 0, resetSimRequests = 0;
    auto syncService = node->create_service<Reset>("sync_sim_to_estimate",
                                                   [&](Reset::Request::SharedPtr, Reset::Response::SharedPtr reply) {
                                                       ++syncRequests;
                                                       reply->success = true;
                                                       reply->message = "Aligned to estimate";
                                                   });
    std::shared_ptr<rmw_request_id_t> resetSimHeader;
    auto resetSimService = node->create_service<Reset>(
        "reset_sim_to_start", [&](std::shared_ptr<rmw_request_id_t> header, Reset::Request::SharedPtr) {
            ++resetSimRequests;
            resetSimHeader = header;
        });
    int speedRequests = 0;
    auto parameterCallback =
        node->add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult response;
            response.successful = true;
            for (const auto &parameter : parameters)
                if (parameter.get_name() == "real_time_factor") {
                    ++speedRequests;
                    response.successful = parameter.as_double() != 3;
                    response.reason = response.successful ? "" : "test rejection";
                }
            return response;
        });
    // Only the providers exercised here; remap all mapping endpoints to prove configurability.
    config["providers"].remove("motion");
    config["providers"].remove("mission");
    config.remove("overlays");
    config.remove("ownership");
    config["panels"] = YAML::Load(R"([
      {id: mapping, type: mapping, provider: mapping, options: {parent_frame: test_world, tag_frame: test_tag}},
      {id: actuators, type: actuators, provider: actuators}])");
    auto mapCfg = config["providers"]["mapping"]["options"];
    mapCfg["calibration_action"] = "calibration";
    mapCfg["reset_service"] = "reset";
    mapCfg["target_service"] = "target";
    mapCfg["status_topic"] = "mapping_state";
    mapCfg["request_timeout"] = .25;
    mapCfg["status_timeout"] = .3;
    config["providers"]["actuators"]["options"]["status_timeout"] = .3;
    config["providers"]["run"]["options"]["status_timeout"] = .3;
    Context ctx{ns, "test_world", false, false};
    ctx.documents["task"] = YAML::Load(R"(
ui:
  title: Test scorecard
  run_options:
    - {key: role, label: Role, type: choice, default: repair, choices: [{value: repair, label: Repair}]}
    - {key: coin, label: Coin, type: bool, default: true}
    - {key: amount, label: Amount, type: number, default: 1}
  manual_adjustment: true
)");
    ctx.initialWindows = {"run"};
    auto composition = std::make_unique<Composition>(config, ctx, registry);
    auto mapping = std::dynamic_pointer_cast<Mapping>(composition->providers().at("mapping"));
    auto actuators = std::dynamic_pointer_cast<Actuators>(composition->providers().at("actuators"));
    auto run = std::dynamic_pointer_cast<Run>(composition->providers().at("run"));
    auto simulation = std::dynamic_pointer_cast<Simulation>(composition->providers().at("simulation"));
    auto mappingPub = node->create_publisher<riptide_msgs2::msg::MappingTargetInfo>("mapping_state", 10);
    auto actuatorPub =
        node->create_publisher<riptide_msgs2::msg::ActuatorStatus>("state/actuator/status", rclcpp::SensorDataQoS());
    auto scorePub = node->create_publisher<std_msgs::msg::String>("simulator/run_score", 10);
    auto lightPub = node->create_publisher<visualization_msgs::msg::MarkerArray>("simulator/magnet_lights", 10);
    auto eventPub = node->create_publisher<std_msgs::msg::String>("simulator/task_events", 10);
    auto taskScorePub = node->create_publisher<std_msgs::msg::String>("simulator/task_score", 10);
    auto jointsPub = node->create_publisher<std_msgs::msg::Float64MultiArray>("simulator/claw_joints", 10);
    riptide_msgs2::msg::MappingTargetInfo mappingMsg;
    mappingMsg.target_object = "observed";
    mappingMsg.lock_map = true;
    riptide_msgs2::msg::ActuatorStatus actuatorMsg;
    actuatorMsg.torpedo_available_count = 2;
    std_msgs::msg::String scoreMsg;
    scoreMsg.data = R"({"running":false,"elapsed":2.5,"total":12,"rows":[{"label":"Award","points":12}]})";
    int armCount = 0, fireCount = 0, resetCount = 0;
    bool lastArm = false;
    std::vector<std::string> runCommands;
    auto armSub =
        node->create_subscription<std_msgs::msg::Bool>("command/actuator/arm", 10, [&](const std_msgs::msg::Bool &m) {
            ++armCount;
            lastArm = m.data;
        });
    auto fireSub = node->create_subscription<std_msgs::msg::Empty>("command/actuator/torpedo", 10,
                                                                   [&](const std_msgs::msg::Empty &) { ++fireCount; });
    auto resetSub = node->create_subscription<std_msgs::msg::Empty>(
        "simulator/reset_tasks", 10, [&](const std_msgs::msg::Empty &) { ++resetCount; });
    auto runSub = node->create_subscription<std_msgs::msg::String>(
        "simulator/run_command", 10, [&](const std_msgs::msg::String &m) { runCommands.push_back(m.data); });
    Target::Request targetRequest;
    int targetCount = 0;
    auto targetService =
        node->create_service<Target>("target", [&](Target::Request::SharedPtr req, Target::Response::SharedPtr) {
            targetRequest = *req;
            ++targetCount;
        });
    std::shared_ptr<rmw_request_id_t> resetHeader;
    auto resetService = node->create_service<Reset>(
        "reset", [&](std::shared_ptr<rmw_request_id_t> header, Reset::Request::SharedPtr) { resetHeader = header; });
    std::shared_ptr<Goal> goal;
    int calCount = 0;
    bool reject = false;
    auto server = rclcpp_action::create_server<Cal>(
        node, "calibration",
        [&](const auto &, auto) {
            return reject ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
        [&](auto accepted) {
            goal = accepted;
            ++calCount;
        });
    ros.start();
    auto spin = [&](double seconds, bool publish = true) {
        auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < end) {
            if (publish) {
                mappingPub->publish(mappingMsg);
                actuatorPub->publish(actuatorMsg);
                scorePub->publish(scoreMsg);
            }
            rclcpp::spin_some(node);
            if (goal && goal->is_canceling()) {
                goal->canceled(std::make_shared<Cal::Result>());
                goal.reset();
            }
            std::this_thread::sleep_for(5ms);
        }
    };
    spin(1.2);
    assert(mapping->state().fresh && mapping->state().calibrationReady);
    assert(run->state().fresh && actuators->state().fresh);
    assert(calCount == 0 && armCount == 0 && fireCount == 0 && resetCount == 0 && runCommands.empty());
    assert(simulation->state().connected && simulation->state().rate == 1 && speedRequests == 0);
    simulation->setRate(0);
    spin(.7);
    assert(simulation->state().rate == 0 && node->get_parameter("real_time_factor").as_double() == 0);
    simulation->setRate(2);
    spin(.7);
    assert(simulation->state().rate == 2 && speedRequests == 2);
    simulation->setRate(3);
    spin(.7);
    assert(simulation->state().rate == 2 && simulation->state().message.find("test rejection") != std::string::npos);
    simulation->setRate(-1);
    simulation->setRate(100);
    spin(.1);
    assert(speedRequests == 3);
    node->set_parameter(rclcpp::Parameter("real_time_factor", .5));
    spin(.7);
    assert(simulation->state().rate == .5); // observe external speed changes as well
    simulation->setPaused(true);
    spin(.7);
    assert(simulation->state().rate == 0 && simulation->state().resumeRate == .5);
    simulation->sync();
    spin(.15);
    assert(syncRequests == 1 && !simulation->state().operationPending &&
           simulation->state().operationMessage == "Sync: Aligned to estimate");
    simulation->setPaused(false);
    spin(.7);
    assert(simulation->state().rate == .5); // resume the last nonzero speed
    simulation->reset();
    simulation->reset();
    spin(.1);
    assert(resetSimRequests == 1 && simulation->state().operationPending);
    Reset::Response simResetReply;
    simResetReply.success = false;
    simResetReply.message = "not ready";
    resetSimService->send_response(*resetSimHeader, simResetReply);
    spin(.15);
    assert(simulation->state().operationMessage == "Reset failed: not ready");
    simulation->reset();
    spin(.95);
    assert(!simulation->state().operationPending &&
           simulation->state().operationMessage.find("timed out") != std::string::npos);
    simResetReply.success = true;
    resetSimService->send_response(*resetSimHeader, simResetReply);
    spin(.1);
    assert(simulation->state().operationMessage.find("timed out") != std::string::npos);
    // Only Run tracking subscribes to simulation feedback; Actuators does not.
    assert(lightPub->get_subscription_count() == 1 && eventPub->get_subscription_count() == 1 &&
           jointsPub->get_subscription_count() == 1);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker light;
    light.ns = "test_target";
    light.color.g = 1;
    markers.markers.push_back(light);
    lightPub->publish(markers);
    std_msgs::msg::Float64MultiArray jaws;
    jaws.data = {.01, .02};
    jointsPub->publish(jaws);
    std_msgs::msg::String summary;
    summary.data = "Task completed";
    taskScorePub->publish(summary);
    for (int i = 0; i < 7; ++i) {
        std_msgs::msg::String event;
        event.data = "event " + std::to_string(i);
        eventPub->publish(event);
        spin(.02);
    }
    spin(.1);
    assert(run->state().magnetTargets.size() == 1 && run->state().magnetTargets[0].second);
    assert(run->state().taskSummary == "Task completed" && run->state().events.size() == 5 &&
           run->state().events.front() == "event 6");
    assert(run->state().simulationReadings[0].second == "30 mm");
    for (const auto &reading : actuators->state().readings)
        assert(reading.first != "test_target" && reading.first != "Jaw gap");
    for (const auto &action : actuators->state().actions)
        assert(action.id != "reset");
    std_msgs::msg::String resetEvent;
    resetEvent.data = "{kind: tasks, result: reset, target: all}";
    eventPub->publish(resetEvent);
    spin(.1);
    assert(run->state().events.size() == 1);
    mapping->setTarget("new_target", false);
    spin(.2);
    assert(targetCount == 1 && targetRequest.target_info.target_object == "new_target" &&
           !targetRequest.target_info.lock_map);
    assert(mapping->state().target == "observed" && mapping->state().locked); // request never fabricates observed state
    mapping->calibrate("test_world", "test_tag", 17);
    spin(.15);
    assert(goal && goal->get_goal()->samples == 17 && goal->get_goal()->monitor_child == "test_tag");
    auto feedback = std::make_shared<Cal::Feedback>();
    feedback->sample_count = 4;
    goal->publish_feedback(feedback);
    spin(.1);
    assert(mapping->state().samples == 4);
    auto result = std::make_shared<Cal::Result>();
    result->success = true;
    goal->succeed(result);
    goal.reset();
    spin(.15);
    assert(!mapping->state().calibrating && mapping->state().calibrationMessage == "Tag calibration complete");
    mapping->calibrate("test_world", "test_tag", 10);
    mapping->cancelCalibration();
    spin(.4);
    assert(!goal && !mapping->state().calibrating); // cancel before goal response
    reject = true;
    mapping->calibrate("test_world", "test_tag", 10);
    spin(.2);
    assert(!mapping->state().calibrating && mapping->state().calibrationMessage == "Calibration rejected");
    reject = false;
    mapping->reset();
    spin(.1);
    assert(resetHeader);
    Reset::Response resetReply;
    resetReply.success = false;
    resetReply.message = "blocked";
    resetService->send_response(*resetHeader, resetReply);
    resetHeader.reset();
    spin(.15);
    assert(mapping->state().resetMessage.find("failed: blocked") != std::string::npos);
    mapping->reset();
    spin(.45);
    assert(!mapping->state().resetting && mapping->state().resetMessage.find("timed out") != std::string::npos);
    resetReply.success = true;
    resetService->send_response(*resetHeader, resetReply);
    resetHeader.reset();
    spin(.1);
    assert(mapping->state().resetMessage.find("timed out") != std::string::npos);
    actuators->command("torpedo");
    actuators->command("arm");
    spin(.15);
    assert(fireCount == 0 && armCount == 1 && lastArm);
    actuatorMsg.actuators_armed = true;
    spin(.1);
    actuators->command("torpedo");
    actuators->command("arm");
    spin(.15);
    assert(fireCount == 1 && armCount == 2 && !lastArm);
    run->command(YAML::Load("{action: start, role: 'quoted \" role', coin: true, amount: 2.5}"));
    run->reset();
    spin(.15);
    assert(runCommands.size() == 1 && resetCount == 1);
    auto command = YAML::Load(runCommands.back());
    assert(command["coin"].as<bool>() && command["amount"].as<double>() == 2.5 &&
           command["role"].as<std::string>() == "quoted \" role");
    assert(runCommands.back().find("\"coin\": true") != std::string::npos &&
           runCommands.back().find("\"amount\": 2.5") != std::string::npos);
    scoreMsg.data = R"({"running":true,"elapsed":3,"total":12})";
    spin(.1);
    run->command(YAML::Load("{action: start}"));
    run->command(YAML::Load("{action: stop}"));
    spin(.1);
    assert(runCommands.size() == 2);
    spin(.5, false);
    assert(!mapping->state().fresh && !actuators->state().fresh && !run->state().fresh);
    actuators->command("torpedo");
    run->command(YAML::Load("{action: stop}"));
    run->reset();
    spin(.1, false);
    assert(fireCount == 1 && runCommands.size() == 2 && resetCount == 1);
    scoreMsg.data = "{running: true}";
    spin(.1);
    assert(!run->state().fresh && run->state().message.find("Invalid run score") != std::string::npos);
    // Draw all panels (including empty/preview snapshots) at minimum sidebar width.
    ImGui::CreateContext();
    auto &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = {1000, 900};
    io.DeltaTime = 1.f / 30;
    unsigned char *pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
    ImVec2 simulationButtonMin, simulationButtonMax;
    auto draw = [&](Composition &c) {
        ImGui::NewFrame();
        ImGui::SetNextWindowSize({320, 850});
        ImGui::Begin("test");
        c.setWidth(300);
        c.drawToolsToolbar("settings");
        simulationButtonMin = ImGui::GetItemRectMin();
        simulationButtonMax = ImGui::GetItemRectMax();
        c.drawToolsToolbar();
        c.drawSidebar(800);
        c.drawWindows();
        ImGui::End();
        ImGui::Render();
    };
    for (int frame = 0; frame < 4; ++frame)
        draw(*composition);
    const auto anchor = simulationButtonMin;
    const auto bottom = simulationButtonMax.y + ImGui::GetStyle().ItemSpacing.y;
    io.AddMousePosEvent(anchor.x + 10, anchor.y + 10);
    draw(*composition);
    io.AddMouseButtonEvent(0, true);
    draw(*composition);
    io.AddMouseButtonEvent(0, false);
    bool sawPopup = false;
    for (int frame = 0; frame < 4; ++frame) {
        draw(*composition);
        for (const auto *window : ImGui::GetCurrentContext()->Windows)
            if (window->Active && !window->Hidden && (window->Flags & ImGuiWindowFlags_Popup)) {
                sawPopup = true;
                // Check the very first visible popup frame as well as settled frames.
                assert(std::abs(window->Pos.x - anchor.x) < 1);
                assert(std::abs(window->Pos.y - bottom) < 1);
                assert(std::abs(window->Size.x - 340) < 1);
                assert(window->Pos.y + window->Size.y <= io.DisplaySize.y - 7);
            }
    }
    assert(sawPopup);
    io.AddKeyEvent(ImGuiKey_Escape, true);
    draw(*composition);
    io.AddKeyEvent(ImGuiKey_Escape, false);
    auto checkScorecard = [&] {
        bool found = false;
        for (const auto *window : ImGui::GetCurrentContext()->Windows)
            if (window->Active && !window->Hidden &&
                std::string(window->Name).find("###scorecard_") != std::string::npos) {
                found = true;
                assert(window->Pos.x >= 11 && window->Pos.y >= 11);
                assert(window->Pos.x + window->Size.x <= io.DisplaySize.x - 11);
                assert(window->Pos.y + window->Size.y <= io.DisplaySize.y - 11);
            }
        assert(found);
    };
    draw(*composition);
    checkScorecard();
    io.DisplaySize = {500, 350};
    for (int frame = 0; frame < 3; ++frame)
        draw(*composition);
    checkScorecard();
    io.DisplaySize = {1000, 900};
    ctx.preview = true;
    Composition preview(config, ctx, registry);
    assert(preview.providers().empty());
    draw(preview);
    ctx.documents.clear();
    Composition noProfile(config, ctx, registry);
    draw(noProfile);
    ImGui::DestroyContext();
    ros.stop();
    composition.reset();
    mapping.reset();
    actuators.reset();
    run.reset();
    simulation.reset();
    rclcpp::shutdown();
    std::cout << "Mapping, actuator and run panel checks passed\n";
}
