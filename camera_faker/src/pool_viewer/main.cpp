#include "pool_viewer/depth_noise.hpp"
#include "pool_viewer/detection_pose.hpp"
#include "pool_viewer/renderer.hpp"
#include "pool_viewer/payload_mounts.hpp"
#include "pool_viewer/tf_tree.hpp"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <array>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <deque>
#include <future>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>
#include <imgui_internal.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <riptide_msgs2/msg/actuator_status.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <set>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

using Clock = std::chrono::steady_clock;
namespace {
glm::mat4 matrix(const geometry_msgs::msg::Transform &t) {
  const auto &q = t.rotation;
  return glm::translate(
             glm::mat4(1),
             glm::vec3(t.translation.x, t.translation.y, t.translation.z)) *
         glm::mat4_cast(glm::normalize(glm::quat(q.w, q.x, q.y, q.z)));
}
geometry_msgs::msg::Transform transform(const glm::mat4 &m) {
  geometry_msgs::msg::Transform t;
  auto q = glm::quat_cast(m);
  t.translation.x = m[3].x;
  t.translation.y = m[3].y;
  t.translation.z = m[3].z;
  t.rotation.w = q.w;
  t.rotation.x = q.x;
  t.rotation.y = q.y;
  t.rotation.z = q.z;
  return t;
}
std::string fixed(double x, int decimals = 1) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(decimals) << x;
  return s.str();
}
ImVec4 cyan(.32f, .86f, .82f, 1), muted(.47f, .57f, .64f, 1),
    white(.87f, .92f, .95f, 1);
ImU32 color(ImVec4 c) { return ImGui::ColorConvertFloat4ToU32(c); }
ImTextureID textureID(GLuint t) { return static_cast<ImTextureID>(t); }
struct CameraOutput {
  cv::Mat preview;
  pool::View view;
  double processingMs = 0;
  // Overlay copy of the published cloud, in the camera link frame, with the
  // true camera pose it was rendered from.
  std::vector<float> cloud;
  glm::mat4 world{1};
  bool hasCloud = false;
};
struct Camera {
  std::string name, frame;
  pool::Intrinsics k;
  pool::Frame image;
  glm::mat4 mount{1}, world{1};
  // Overlays must use the pose/projection of the displayed texture, including
  // when a camera skips an acquisition or its depth worker finishes later.
  pool::View imageView, depthView;
  // Exact optical pose of each published acquisition in the simulated world.
  std::deque<std::pair<int64_t, glm::mat4>> renders;
  double next = 0, nextCloud = 0, lastStamp = -1, measuredHz = 0;
  double renderMs = 0, outputMs = 0, processingMs = 0;
  std::future<CameraOutput> pending;
  Clock::time_point lastRender{};
  bool ready = false, depthPreview = false;
  GLuint depthTexture = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb, left, depth;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed,
      leftCompressed;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info, leftInfo,
      depthInfo;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud;
  std::mt19937 random{7};
  ~Camera() {
    if (pending.valid()) pending.wait();
    if (depthTexture)
      glDeleteTextures(1, &depthTexture);
  }
};
} // namespace
class PoolViewer : public rclcpp::Node {
public:
  PoolViewer() : Node("pool_viewer") {
    robot = declare_parameter<std::string>("robot", "talos");
    mapFrame = declare_parameter<std::string>("fixed_frame", "map");
    showTf = declare_parameter<bool>("show_tf", false);
    demo = declare_parameter<bool>("demo", false);
    hidden = declare_parameter<bool>("headless", false);
    scorecardOpen = declare_parameter<bool>("show_scorecard", false);
    exitFrames = declare_parameter<int>("exit_after_frames", 0);
    screenshot = declare_parameter<std::string>("screenshot_path", "");
    detections = declare_parameter<bool>("detections", false);
    renderRate = declare_parameter<double>("render_rate", 30.);
    previewWidth = declare_parameter<int>("camera_preview_width", 480);
    profile = declare_parameter<bool>("profile", false);
    if (previewWidth < 16 || previewWidth > 4096)
      throw std::invalid_argument("camera_preview_width must be in [16,4096]");
    cloudEnabled = declare_parameter<bool>("point_cloud.enabled", true);
    cloudRate = declare_parameter<double>("point_cloud.rate", 5.);
    cloudStride = declare_parameter<int>("point_cloud.stride", 8);
    cloudOverlay = std::clamp(int(declare_parameter<int>("point_cloud.overlay", 0)), 0, 3);
    depthModel.rangeSigma = declare_parameter<double>("depth_noise", .0015);
    depthModel.enabled = declare_parameter<bool>("depth_model.enabled", true);
    depthModel.baseSigma =
        declare_parameter<double>("depth_model.base_sigma", .002);
    depthModel.exponent =
        declare_parameter<double>("depth_model.range_exponent", 2.);
    depthModel.minRange =
        declare_parameter<double>("depth_model.min_range", .15);
    depthModel.maxRange =
        declare_parameter<double>("depth_model.max_range", 8.);
    depthModel.bias = declare_parameter<double>("depth_model.bias", 0.);
    depthModel.dropout = declare_parameter<double>("depth_model.dropout", .005);
    depthModel.rangeDropout =
        declare_parameter<double>("depth_model.range_dropout", .10);
    depthModel.edgeDropout =
        declare_parameter<double>("depth_model.edge_dropout", .20);
    depthModel.outliers =
        declare_parameter<double>("depth_model.outliers", .002);
    depthModel.correlation =
        declare_parameter<double>("depth_model.correlation", .5);
    depthModel.patchSize = declare_parameter<int>("depth_model.patch_size", 8);
    depthModel.validate();
    auto waterVector = [this](const char *name, glm::vec3 value) {
      const auto values = declare_parameter<std::vector<double>>(name, {value.x, value.y, value.z});
      if (values.size() != 3) throw std::runtime_error("Water colors require three components");
      return glm::vec3(values[0], values[1], values[2]);
    };
    look.water.tint = waterVector("water.tint", look.water.tint);
    look.water.absorption = waterVector("water.absorption", look.water.absorption);
    look.water.scattering = declare_parameter<double>("water.scattering", .10);
    look.water.distanceScale = declare_parameter<double>("water.distance_scale", 1.);
    look.water.distancePower = declare_parameter<double>("water.distance_power", 1.);
    look.water.clearDistance = declare_parameter<double>("water.clear_distance", 0.);
    look.water.validate();
    depthSettingsCallback = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          auto candidate = depthModel;
          auto water = look.water;
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          try {
            for (const auto &p : parameters) {
              const auto &n = p.get_name();
              if (n == "water.tint" || n == "water.absorption") {
                const auto values = p.as_double_array();
                if (values.size() != 3) throw std::invalid_argument("Water colors require three components");
                auto &value = n == "water.tint" ? water.tint : water.absorption;
                value = glm::vec3(values[0], values[1], values[2]);
              } else if (n == "water.scattering") water.scattering = p.as_double();
              else if (n == "water.distance_scale") water.distanceScale = p.as_double();
              else if (n == "water.distance_power") water.distancePower = p.as_double();
              else if (n == "water.clear_distance") water.clearDistance = p.as_double();
              else if (n == "depth_model.enabled")
                candidate.enabled = p.as_bool();
              else if (n == "depth_noise")
                candidate.rangeSigma = p.as_double();
              else if (n == "depth_model.base_sigma")
                candidate.baseSigma = p.as_double();
              else if (n == "depth_model.range_exponent")
                candidate.exponent = p.as_double();
              else if (n == "depth_model.min_range")
                candidate.minRange = p.as_double();
              else if (n == "depth_model.max_range")
                candidate.maxRange = p.as_double();
              else if (n == "depth_model.bias")
                candidate.bias = p.as_double();
              else if (n == "depth_model.dropout")
                candidate.dropout = p.as_double();
              else if (n == "depth_model.range_dropout")
                candidate.rangeDropout = p.as_double();
              else if (n == "depth_model.edge_dropout")
                candidate.edgeDropout = p.as_double();
              else if (n == "depth_model.outliers")
                candidate.outliers = p.as_double();
              else if (n == "depth_model.correlation")
                candidate.correlation = p.as_double();
              else if (n == "depth_model.patch_size")
                candidate.patchSize = p.as_int();
            }
            candidate.validate();
            water.validate();
            depthModel = candidate;
            look.water = water;
          } catch (const std::exception &e) {
            result.successful = false;
            result.reason = e.what();
          }
          return result;
        });
    const bool depthPreview = declare_parameter<bool>("depth_preview", false);
    rcl_interfaces::msg::ParameterDescriptor scaleDescriptor;
    scaleDescriptor.read_only = true;
    scaleDescriptor.description = "Startup-only camera resolution scale; restart to change";
    const double scale = declare_parameter<double>("camera_scale", 1.0, scaleDescriptor);
    if (!std::isfinite(scale) || scale <= 0 || scale > 1)
      throw std::invalid_argument("camera_scale must be in (0,1]");
    if (!std::isfinite(renderRate) || !std::isfinite(cloudRate) ||
        renderRate <= 0 || renderRate > 120 || cloudRate <= 0 ||
        cloudRate > 120 || cloudStride < 1)
      throw std::runtime_error(
          "Invalid render, point cloud, or noise parameters");
    const auto vehicle =
        YAML::LoadFile(declare_parameter<std::string>("vehicle_config", ""));
    const auto base = pool::vector3(vehicle["base_link"]);
    modelOffset = pool::baseToOrigin(vehicle);
    previewFrames[robot + "/base_link"] = glm::mat4(1);
    previewFrames[robot + "/origin"] = modelOffset;
    for (const auto &name : {"torpedoes", "droppers"})
      if (vehicle[name])
        previewFrames[robot + (std::string(name) == "torpedoes" ? "/torpedo_link" : "/droppers_link")] =
            pool::pose(-base) * pool::yamlPose(vehicle[name]["pose"]);
    if (vehicle["torpedoes"])
      for (int i = 0; i < 2; ++i)
        previewFrames[robot + "/torpedo_" + std::to_string(i) + "_link"] =
            previewFrames.at(robot + "/torpedo_link") *
            pool::pose({0, (i == 0 ? -.5f : .5f) * vehicle["torpedoes"]["baseline"].as<float>(), 0});
    if (vehicle["claw"])
      clawMount = glm::translate(glm::mat4(1), -base) * pool::yamlPose(vehicle["claw"]["pose"]);
    if (vehicle["magnet"]) {
      magnetMount = pool::pose(-base) * pool::yamlPose(vehicle["magnet"]["pose"]);
      previewFrames[robot+"/magnet_link"]=magnetMount;
    }
    buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener = std::make_unique<tf2_ros::TransformListener>(*buffer);
    broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    const bool publishCameraOpticalTf =
        declare_parameter<bool>("publish_camera_optical_tf", true);
    int index = 0;
    for (const std::string name : {"ffc", "dfc"}) {
      auto &c = cameras[index++];
      c.name = name;
      c.depthPreview = depthPreview;
      c.frame = robot + "/" + name + "_left_camera_optical_frame";
      bool found = false;
      for (const auto &item : vehicle["cameras"])
        if (item["name"].as<std::string>() == name) {
          c.mount = glm::translate(glm::mat4(1), -base) *
                    pool::yamlPose(item["pose"]);
          found = true;
        }
      if (!found)
        throw std::runtime_error("Vehicle has no " + name + " camera mount");
      const double cameraScale = declare_parameter<double>(
          name + ".resolution_scale", 1., scaleDescriptor);
      if (!std::isfinite(cameraScale) || cameraScale <= 0 || cameraScale > 1)
        throw std::invalid_argument(name + ".resolution_scale must be in (0,1]");
      c.k = pool::loadCamera(
          declare_parameter<std::string>(name + ".config", ""),
          declare_parameter<std::string>(name + ".calibration_file", ""),
          scale * cameraScale);
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
      std::string root = name + "/zed_node/";
      c.rgb = create_publisher<sensor_msgs::msg::Image>(
          root + "rgb/image_rect_color", qos);
      c.left = create_publisher<sensor_msgs::msg::Image>(
          root + "left/image_rect_color", qos);
      c.depth = create_publisher<sensor_msgs::msg::Image>(
          root + "depth/depth_registered", qos);
      c.compressed = create_publisher<sensor_msgs::msg::CompressedImage>(
          root + "rgb/image_rect_color/compressed", qos);
      c.leftCompressed = create_publisher<sensor_msgs::msg::CompressedImage>(
          root + "left/image_rect_color/compressed", qos);
      c.info = create_publisher<sensor_msgs::msg::CameraInfo>(
          root + "rgb/camera_info", qos);
      c.leftInfo = create_publisher<sensor_msgs::msg::CameraInfo>(
          root + "left/camera_info", qos);
      c.depthInfo = create_publisher<sensor_msgs::msg::CameraInfo>(
          root + "depth/camera_info", qos);
      c.cloud = create_publisher<sensor_msgs::msg::PointCloud2>(
          root + "point_cloud/cloud_registered", qos);
      if (c.k.calibration.find("Approximate") == 0)
        RCLCPP_WARN(get_logger(),
                    "%s: no readable wet calibration; using approximate ZED X "
                    "Mini 2.2 mm intrinsics",
                    name.c_str());
      RCLCPP_INFO(get_logger(),
                  "%s ZED X Mini: %dx%d at %.1f Hz, calibration: %s",
                  name.c_str(), c.k.width, c.k.height, c.k.rate,
                  c.k.calibration.c_str());
      if (name == "dfc") {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = "simulator/" + robot + "/base_link";
        t.child_frame_id = "simulator/" + robot + "/dfc_camera_link";
        t.transform = transform(c.mount);
        transforms.push_back(t);
        t.header.frame_id = t.child_frame_id;
        t.child_frame_id = "simulator/" + c.frame;
        t.transform = transform(pool::pose(
            {}, {-glm::half_pi<float>(), 0, -glm::half_pi<float>()}));
        transforms.push_back(t);
      }
      // Sensor messages use the estimated robot's camera tree. Ground-truth
      // camera frames are strictly simulator-prefixed and only drive rendering.
      // Supply the optical joints when navigation has no ZED description.
      if (publishCameraOpticalTf) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = robot + "/" + name + "_camera_link";
        t.child_frame_id = c.frame;
        t.transform = transform(pool::pose(
            {}, {-glm::half_pi<float>(), 0, -glm::half_pi<float>()}));
        transforms.push_back(t);
      }
    }
    // Expose the CAD origin independently of the ROS estimator branch, so the
    // two base links and their local origin offsets can be inspected in TF.
    geometry_msgs::msg::TransformStamped origin;
    origin.header.stamp = now();
    origin.header.frame_id = "simulator/" + robot + "/base_link";
    origin.child_frame_id = "simulator/" + robot + "/origin";
    origin.transform = transform(modelOffset);
    transforms.push_back(origin);
    // Physics owns truth FFC TF; the viewer supplies truth DFC/CAD TF and,
    // when needed, the estimated cameras' optical joints.
    if (!demo)
      broadcaster->sendTransform(transforms);
    const auto lighting =
        declare_parameter<std::string>("lighting.profile", "indoor");
    if (lighting != "indoor" && lighting != "outdoor")
      throw std::runtime_error("lighting.profile must be indoor or outdoor");
    look.outdoor = lighting == "outdoor";
    look.directLight = declare_parameter<double>("lighting.brightness", 1.);
    look.ambientLight = declare_parameter<double>("lighting.ambient", .8);
    look.sunAzimuth = declare_parameter<double>("lighting.sun_azimuth", 225.);
    look.sunElevation =
        declare_parameter<double>("lighting.sun_elevation", 55.);
    look.glare = declare_parameter<double>("lighting.glare", .5);
    for (float value : {look.directLight, look.ambientLight, look.sunAzimuth,
                        look.sunElevation, look.glare})
      if (!std::isfinite(value))
        throw std::runtime_error("Lighting parameters must be finite");
    if (look.directLight < 0 || look.ambientLight < 0 || look.glare < 0 ||
        look.sunElevation < 0 || look.sunElevation > 90)
      throw std::runtime_error("Invalid lighting intensity or sun elevation");
    initializeWindow();
    renderer = std::make_unique<pool::Renderer>(
        declare_parameter<std::string>("shader_folder", ""),
        declare_parameter<std::string>("riptide_mesh_folder", ""),
        declare_parameter<std::string>("texture_folder", ""),
        declare_parameter<std::string>("mapping_config", ""),
        declare_parameter<std::string>("marker_config", ""),
        declare_parameter<std::string>("scene_config", ""), robot,
        declare_parameter<std::string>("robot_model", ""),
        declare_parameter<std::string>("task_config", ""),
        declare_parameter<std::string>("payload_model", ""),
        declare_parameter<std::string>("launcher_model", ""),
        declare_parameter<std::string>("claw_model", ""));
    if (!get_parameter("task_config").as_string().empty()) {
      const auto task = YAML::LoadFile(get_parameter("task_config").as_string());
      if (task["magnet_lights"]) {
        magnetMount *= pool::pose(pool::vector3(task["magnet_lights"]["robot_tip_offset"]));
        for (const auto &light : task["magnet_lights"]["targets"])
          magnetStates[light.first.as<std::string>()]=light.second.as<std::string>()=="green";
      }
      if (task["claw"]) {
        if (task["claw"]["pose"])
          clawMount=glm::translate(glm::mat4(1),-base)*pool::yamlPose(task["claw"]["pose"]);
        clawMinGap = task["claw"]["min_gap"].as<float>();
        clawJoints.fill(0.f);
      }
      for (const std::string kind : {"torpedo", "dropper"}) {
        const auto cfg = task[kind];
        int index = 0;
        for (const auto &mount : pool::payloadMounts(vehicle, task, kind)) {
          payloadMounts[{kind + "_loaded", index++}] = mount * glm::scale(glm::mat4(1),
                glm::vec3(cfg["length"].as<float>(), 2*cfg["radius"].as<float>(), 2*cfg["radius"].as<float>()));
        }
      }
      payloadFocus = glm::vec3(0);
      for (const auto &entry : payloadMounts) payloadFocus += glm::vec3(entry.second[3]);
      if (!payloadMounts.empty()) payloadFocus /= float(payloadMounts.size());
    }
    taskArm = create_publisher<std_msgs::msg::Bool>("command/actuator/arm", 10);
    magnetSubscription = create_subscription<visualization_msgs::msg::MarkerArray>(
        "simulator/magnet_lights",10,[this](const visualization_msgs::msg::MarkerArray &msg) {
          if (demo) return;
          for (const auto &m : msg.markers)
            if (m.action==visualization_msgs::msg::Marker::ADD && magnetStates.count(m.ns)) {
              const bool green=m.color.g>m.color.r;
              magnetStates[m.ns]=green;
              renderer->magnetLight(m.ns,green);
            }
        });
    taskClaw = create_publisher<std_msgs::msg::Bool>("command/actuator/claw", 10);
    taskClawTimed = create_publisher<std_msgs::msg::Float32>("command/actuator/claw_move_s", 10);
    clawSubscription = create_subscription<std_msgs::msg::Float64MultiArray>(
        "simulator/claw_joints",10,[this](const std_msgs::msg::Float64MultiArray &msg) {
          if (!demo && msg.data.size()==2 && std::isfinite(msg.data[0]) && std::isfinite(msg.data[1]))
            clawJoints={float(msg.data[0]),float(msg.data[1])};
        });
    objectSubscription = create_subscription<visualization_msgs::msg::MarkerArray>(
        "simulator/task_objects",10,[this](const visualization_msgs::msg::MarkerArray &msg) {
          if (demo) return;
          for (const auto &m : msg.markers) {
            if (m.action!=visualization_msgs::msg::Marker::ADD) continue;
            const bool attached=m.header.frame_id==robot+"/base_link";
            if (!attached && m.header.frame_id!=mapFrame) continue;
            geometry_msgs::msg::Transform t;
            t.rotation=m.pose.orientation;
            t.translation.x=m.pose.position.x; t.translation.y=m.pose.position.y; t.translation.z=m.pose.position.z;
            taskObjects[m.ns+"_frame"]={matrix(t),attached};
          }
        });
    taskTorpedo =
        create_publisher<std_msgs::msg::Empty>("command/actuator/torpedo", 10);
    taskDropper =
        create_publisher<std_msgs::msg::Empty>("command/actuator/dropper", 10);
    taskReload = create_publisher<std_msgs::msg::Empty>(
        "command/actuator/notify_reload", 10);
    taskReset = create_publisher<std_msgs::msg::Empty>("simulator/reset_tasks", 10);
    syncClient = create_client<std_srvs::srv::Trigger>("sync_sim_to_estimate");
    resetStartClient = create_client<std_srvs::srv::Trigger>("reset_sim_to_start");
    // Same marker array RViz shows, resolved at each marker's acquisition time.
    detectionSub = create_subscription<visualization_msgs::msg::MarkerArray>(
        "yolo_orientation/visualization_marker_array", 10,
        [this](const visualization_msgs::msg::MarkerArray &msg) { receiveDetections(msg); });
    runCommand = create_publisher<std_msgs::msg::String>("simulator/run_command", 10);
    runSubscription = create_subscription<std_msgs::msg::String>(
        "simulator/run_score", 10, [this](const std_msgs::msg::String &msg) {
          try { runScore = YAML::Load(msg.data); lastRunScore = Clock::now(); }
          catch (const YAML::Exception &e) { RCLCPP_WARN(get_logger(), "Invalid run score: %s", e.what()); }
        });
    payloadSubscription =
        create_subscription<visualization_msgs::msg::MarkerArray>(
            "simulator/projectiles", 10,
            [this](const visualization_msgs::msg::MarkerArray &msg) {
              if (demo)
                return;
              // Each message is a complete snapshot. Keep occupancy and released
              // poses together so a fired round cannot also remain in its mount.
              loadedPayloads.clear();
              releasedPayloads.clear();
              for (const auto &m : msg.markers) {
                if (m.action != visualization_msgs::msg::Marker::ADD ||
                    m.header.frame_id != mapFrame)
                  continue;
                const PayloadSlot slot{m.ns, m.id};
                if (payloadMounts.count(slot)) {
                  loadedPayloads.insert(slot);
                  continue;
                }
                geometry_msgs::msg::Transform t;
                t.rotation = m.pose.orientation;
                t.translation.x = m.pose.position.x;
                t.translation.y = m.pose.position.y;
                t.translation.z = m.pose.position.z;
                releasedPayloads.push_back(
                    glm::scale(matrix(t), glm::vec3(m.scale.x, m.scale.y, m.scale.z)));
              }
            });
    actuatorSubscription =
        create_subscription<riptide_msgs2::msg::ActuatorStatus>(
            "state/actuator/status", 10,
            [this](const riptide_msgs2::msg::ActuatorStatus &msg) {
              actuatorStatus = msg;
              lastActuator = Clock::now();
            });
    eventSubscription = create_subscription<std_msgs::msg::String>(
        "simulator/task_events", 10, [this](const std_msgs::msg::String &msg) {
          try {
            auto event = YAML::Load(msg.data);
            if (event["kind"].as<std::string>() == "tasks" &&
                event["result"].as<std::string>() == "reset")
              taskEvents.clear();
            taskEvents.push_front(event["kind"].as<std::string>() + " / " +
                                  event["result"].as<std::string>() + " / " +
                                  event["target"].as<std::string>());
          } catch (const std::exception &) {
            taskEvents.push_front(msg.data);
          }
          if (taskEvents.size() > 5)
            taskEvents.pop_back();
        });
    scoreSubscription = create_subscription<std_msgs::msg::String>(
        "simulator/task_score", 10,
        [this](const std_msgs::msg::String &msg) { taskScore = msg.data; });
    const auto scene =
        YAML::LoadFile(get_parameter("scene_config").as_string());
    // The mesh is authored at the robot's CAD origin. A second scene offset
    // would disagree with the URDF and move the mesh around base_link in yaw.
    if (scene["april_tag"])
      look.tag = scene["april_tag"]["visible"].as<bool>(true);
    // An explicit preview pose, never published to ROS and never sent to
    // physics.
    body = pool::pose({3.0f, -2.0f, -.75f}, {0, 0, -.14f});
    selectedFocus = 0;
    const auto previewTask =
        declare_parameter<std::string>("demo_task", "gate");
    if (demo)
      previewPose(previewTask);
    focus(declare_parameter<std::string>("initial_focus", "Vehicle"));
    status = demo ? "SCENE PREVIEW" : "WAITING FOR PHYSICS";
  }
  ~PoolViewer() override {
    // All GL objects must be released while the context still exists.
    for (auto &c : cameras) {
      if (c.pending.valid()) c.pending.wait();
      c.image.opaque.release();
      c.image.composite.release();
      c.image.final.release();
      if (c.depthTexture) {
        glDeleteTextures(1, &c.depthTexture);
        c.depthTexture = 0;
      }
    }
    overview.opaque.release();
    overview.composite.release();
    overview.final.release();
    renderer.reset();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (window)
      glfwDestroyWindow(window);
    glfwTerminate();
  }
  void run() {
    auto start = Clock::now(), previous = start;
    int frames = 0;
    while (rclcpp::ok() && !glfwWindowShouldClose(window)) {
      const auto frameStart = Clock::now();
      double t = std::chrono::duration<double>(frameStart - start).count();
      double dt = std::chrono::duration<double>(frameStart - previous).count();
      previous = frameStart;
      frameMs = frameMs * .92 + dt * 1000 * .08;
      rclcpp::spin_some(shared_from_this());
      const auto callbacksEnd = Clock::now();
      callbackMs = .92 * callbackMs + .08 *
          std::chrono::duration<double, std::milli>(callbacksEnd-frameStart).count();
      glfwPollEvents();
      updatePose();
      captureTf();
      captureDetections();
      renderer->robotPose(body * modelOffset);
      renderer->magnetPose(body * magnetMount);
      renderer->clawPose(body * clawMount, clawJoints[0], clawJoints[1]);
      for (const auto &entry : taskObjects)
        renderer->objectPose(entry.first, entry.second.second ? body*entry.second.first : entry.second.first);
      // Loaded rounds must use the same frame's robot pose as the launcher,
      // not the independently sampled world poses in the task markers.
      std::vector<glm::mat4> poses = releasedPayloads;
      for (const auto &entry : payloadMounts)
        if (demo || loadedPayloads.count(entry.first))
          poses.push_back(body * entry.second);
      renderer->payloadPoses(poses);
      renderer->shadows(look);
      for (auto &c : cameras) {
        collectCamera(c);
        c.world = body * c.mount;
        if (c.ready && c.name == "ffc" && !demo) {
          try {
            c.world = matrix(
                buffer
                    ->lookupTransform(mapFrame,
                                      "simulator/" + robot + "/ffc_camera_link",
                                      tf2::TimePointZero)
                    .transform);
          } catch (const tf2::TransformException &) {
            c.ready = false;
          }
        }
        const auto demand = cameraDemand(c);
        const bool visible = !hidden || exitFrames > 0;
        const bool save = !screenshot.empty() || capture;
        const bool cloudDue = cloudEnabled && t >= c.nextCloud;
        // The overlay samples the full-resolution render like a subscriber.
        const bool overlayCloud = cloudDue && overlayWants(c);
        if (!overlayWants(c)) renderer->clearPointCloud(int(&c - &cameras[0]));
        const bool full = demand.pixels() || save || overlayCloud ||
            (visible && mode >= 2 && &c == &cameras[mode-2]);
        const bool render = visible || save || demand.pixels() || overlayCloud;
        const bool wanted = render || demand.metadata();
        // A subscribed camera's next acquisition waits for its worker without
        // moving the deadline forward. Resume immediately when it is ready.
        if (t >= c.next && wanted &&
            (!demand.pixels() || !c.pending.valid() || save)) {
          // Missed camera deadlines are skipped, never caught up in a burst.
          c.next = t + 1. / c.k.rate;
          const auto renderStart = Clock::now();
          if (render) {
            auto intrinsics = c.k;
            if (!full && intrinsics.width > previewWidth)
              intrinsics.resize(previewWidth, std::max(16, int(std::lround(
                  double(previewWidth) * c.k.height / c.k.width))));
            if (profile && (c.image.opaque.width != intrinsics.width ||
                            c.image.opaque.height != intrinsics.height))
              RCLCPP_INFO(get_logger(), "%s render: %dx%d (%s)", c.name.c_str(),
                          intrinsics.width, intrinsics.height,
                          full ? "sensor/primary" : "side preview");
            c.image.resize(intrinsics.width, intrinsics.height);
            c.imageView = pool::cameraView(c.world, intrinsics);
            renderer->render(c.image, c.imageView, look, t,
                             true, false);
          }
          const auto renderEnd = Clock::now();
          c.renderMs = std::chrono::duration<double, std::milli>(renderEnd-renderStart).count();
          if (c.lastRender.time_since_epoch().count())
            c.measuredHz =
                1. / std::chrono::duration<double>(frameStart - c.lastRender)
                         .count();
          c.lastRender = frameStart;
          publish(c, demand, cloudDue, render, overlayCloud);
          c.outputMs = std::chrono::duration<double, std::milli>(Clock::now()-renderEnd).count();
          if (cloudDue)
            c.nextCloud = t + 1. / cloudRate;
        }
      }
      if (profile && t >= nextProfile) {
        nextProfile = t + 3.;
        RCLCPP_INFO(get_logger(),
            "Viewer %.1f fps; callbacks %.1f ms; FFC submit/readback/worker %.1f/%.1f/%.1f ms; DFC %.1f/%.1f/%.1f ms",
            1000./std::max(frameMs, 1.), callbackMs,
            cameras[0].renderMs, cameras[0].outputMs, cameras[0].processingMs,
            cameras[1].renderMs, cameras[1].outputMs, cameras[1].processingMs);
      }
      if (hidden && screenshot.empty() && exitFrames == 0) {
        std::this_thread::sleep_until(
            frameStart + std::chrono::duration_cast<Clock::duration>(
                             std::chrono::duration<double>(1. / renderRate)));
        continue;
      }
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();
      drawInterface(t, float(std::min(dt, .1)));
      if (largeMap) {
        ImGui::SetNextWindowSize({1000, 620}, ImGuiCond_FirstUseEver);
        if (focusMap) {
          ImGui::SetNextWindowFocus();
          ImGui::SetNextWindowCollapsed(false);
          focusMap = false;
        }
        if (ImGui::Begin("Course map", &largeMap)) {
          ImGui::TextDisabled(
              "SCROLL zoom / DRAG pan / CLICK a task to focus the pool view");
          ImGui::SameLine();
          if (ImGui::SmallButton("Fit pool")) {
            mapZoom = 1;
            mapPan = {0, 0};
          }
          auto space = ImGui::GetContentRegionAvail();
          mapCanvas(space.x, std::max(100.f, space.y), true);
        }
        ImGui::End();
      }
      ImGui::Render();
      if (exitFrames > 0 &&
          ImGui::GetCurrentContext()->ErrorCountCurrentFrame > 0)
        throw std::runtime_error("ImGui validation failed during capture run");
      int w, h;
      glfwGetFramebufferSize(window, &w, &h);
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      glViewport(0, 0, w, h);
      glClearColor(.028, .043, .057, 1);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      ++frames;
      if ((exitFrames > 0 && frames == exitFrames) || capture) {
        saveScreenshot();
        capture = false;
      }
      glfwSwapBuffers(window);
      if (exitFrames > 0 && frames >= exitFrames)
        break;
      std::this_thread::sleep_until(
          frameStart + std::chrono::duration_cast<Clock::duration>(
                           std::chrono::duration<double>(1. / renderRate)));
    }
  }

private:
  GLFWwindow *window = nullptr;
  std::string robot, mapFrame, status, screenshot, lastCapture;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr syncClient, resetStartClient;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr detectionSub;
  // Latest vision observations keyed by ns/id. Each one is placed once at its
  // header stamp and stays fixed in the map frame for the detector's lifetime.
  struct DetectionEntry {
    visualization_msgs::msg::Marker marker;
    Clock::time_point received;
    pool::DetectionPose placement{};
  };
  std::map<std::pair<std::string, int>, DetectionEntry> detectionMarkers;
  bool detections = false; // place camera observations at their simulator acquisition pose
  int cloudOverlay = 0;  // published point clouds in the pool view: 0 off, 1 ffc, 2 dfc, 3 both
  float cloudPointSize = 3.f, cloudHighlight = .5f;
  bool overlayWants(const Camera &c) const {
    const int slot = int(&c - &cameras[0]);
    return !demo && (cloudOverlay == 3 || cloudOverlay == slot + 1);
  }
  struct PlacedDetection { glm::mat4 pose; visualization_msgs::msg::Marker marker; };
  std::vector<PlacedDetection> placedDetections;
  std::string detectionStatus;
  std::string syncStatus;
  std::unique_ptr<pool::Renderer> renderer;
  std::unique_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
  std::array<Camera, 2> cameras;
  pool::Frame overview;
  pool::Look look;
  using PayloadSlot = std::pair<std::string, int>;
  std::map<PayloadSlot, glm::mat4> payloadMounts;
  std::set<PayloadSlot> loadedPayloads;
  std::vector<glm::mat4> releasedPayloads;
  glm::vec3 payloadFocus{0};
  std::map<std::string, glm::mat4> previewFrames;
  std::map<std::string, glm::mat4> displayedFrames;
  std::string tfDifference;
  bool showTf = false, tfNames = true, tfTreeOpen = false;
  float tfAxisLength = .12f;
  pool::TfTree tfTree;
  int tfResolved = 0, tfMissing = 0;
  glm::mat4 clawMount{1};
  glm::mat4 magnetMount{1};
  std::map<std::string,bool> magnetStates;
  float clawMinGap=.008f;
  std::array<float,2> clawJoints{0.f,0.f};
  std::map<std::string,std::pair<glm::mat4,bool>> taskObjects;
  glm::mat4 body{1}, modelOffset{1};
  glm::vec3 target{10, 4, -.8f}, freeEye{-2, -5, 2};
  float yaw = -2.45f, pitch = .57f, distance = 19, freeYaw = .5f,
        freePitch = -.2f;
  bool mouseCaptured = false;
  double lastMouseX = 0, lastMouseY = 0;
  bool demo = false, hidden = false, cloudEnabled = true, capture = false,
       labels = true, follow = false;
  int mode = 0, exitFrames = 0, cloudStride = 8, selectedFocus = 0,
      selectedDemo = 0;
  double renderRate = 30, cloudRate = 5, frameMs = 33.3;
  int previewWidth = 480;
  bool profile = false;
  double callbackMs = 0, nextProfile = 0;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr taskArm, taskClaw;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr taskClawTimed;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr clawSubscription;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr objectSubscription;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr magnetSubscription;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr taskTorpedo, taskDropper,
      taskReload, taskReset;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      payloadSubscription;
  rclcpp::Subscription<riptide_msgs2::msg::ActuatorStatus>::SharedPtr
      actuatorSubscription;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr eventSubscription,
      scoreSubscription;
  riptide_msgs2::msg::ActuatorStatus actuatorStatus;
  Clock::time_point lastActuator{};
  std::deque<std::string> taskEvents;
  std::string taskScore = "No task results yet";
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr runCommand;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr runSubscription;
  YAML::Node runScore;
  Clock::time_point lastRunScore{};
  int intendedRole = 0;
  bool headingCoin = true, roleCoin = true, scorecardOpen = false;
  float manualPoints = 0.f;
  pool::DepthNoise depthModel;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      depthSettingsCallback;
  bool largeMap = false;
  bool focusMap = false, focusScorecard = false;
  float mapZoom = 1;
  ImVec2 mapPan{0, 0};
  ImFont *normalFont = nullptr, *smallFont = nullptr, *titleFont = nullptr,
         *numberFont = nullptr;
  std::deque<glm::vec3> trail;
  Clock::time_point lastPoseWall{};
  double lastPoseStamp = -1;
  void initializeWindow() {
    glfwSetErrorCallback(
        [](int, const char *e) { std::cerr << "GLFW: " << e << '\n'; });
    if (!glfwInit())
      throw std::runtime_error("GLFW initialization failed. OpenGL 3.3 and an "
                               "X/Wayland display are required (no CUDA).");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_VISIBLE, hidden ? GL_FALSE : GL_TRUE);
    window =
        glfwCreateWindow(1480, 940, "Riptide | RoboSub Pool", nullptr, nullptr);
    if (!window)
      throw std::runtime_error("Cannot create an OpenGL 3.3 window");
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
      throw std::runtime_error("OpenGL loading failed");
    RCLCPP_INFO(get_logger(), "Renderer: %s / %s", glGetString(GL_RENDERER),
                glGetString(GL_VERSION));
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    auto &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    const std::string font = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";
    const std::string bold =
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf";
    if (std::filesystem::exists(font)) {
      normalFont = io.Fonts->AddFontFromFileTTF(font.c_str(), 15);
      smallFont = io.Fonts->AddFontFromFileTTF(font.c_str(), 12);
      titleFont = io.Fonts->AddFontFromFileTTF(
          std::filesystem::exists(bold) ? bold.c_str() : font.c_str(), 21);
      numberFont = io.Fonts->AddFontFromFileTTF(font.c_str(), 25);
    } else
      normalFont = smallFont = titleFont = numberFont =
          io.Fonts->AddFontDefault();
    ImGui::StyleColorsDark();
    auto &s = ImGui::GetStyle();
    s.WindowPadding = {18, 16};
    s.FramePadding = {10, 7};
    s.ItemSpacing = {10, 9};
    s.WindowRounding = 9;
    s.ChildRounding = 8;
    s.FrameRounding = 5;
    s.WindowBorderSize = 0;
    s.ChildBorderSize = 1;
    s.PopupRounding = 6;
    s.GrabRounding = 5;
    s.Colors[ImGuiCol_WindowBg] = {.035, .052, .066, 1};
    s.Colors[ImGuiCol_ChildBg] = {.052, .074, .091, 1};
    s.Colors[ImGuiCol_Border] = {.12, .18, .21, 1};
    s.Colors[ImGuiCol_Text] = white;
    s.Colors[ImGuiCol_TextDisabled] = muted;
    s.Colors[ImGuiCol_FrameBg] = {.083, .12, .145, 1};
    s.Colors[ImGuiCol_Button] = {.085, .14, .17, 1};
    s.Colors[ImGuiCol_ButtonHovered] = {.13, .27, .29, 1};
    s.Colors[ImGuiCol_ButtonActive] = {.11, .36, .35, 1};
    s.Colors[ImGuiCol_CheckMark] = cyan;
    s.Colors[ImGuiCol_SliderGrab] = cyan;
    s.Colors[ImGuiCol_SliderGrabActive] = {.5, .96, .89, 1};
    s.Colors[ImGuiCol_Header] = {.09, .25, .27, 1};
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
  }
  void updatePose() {
    if (demo) {
      for (auto &c : cameras)
        c.ready = true;
      return;
    }
    try {
      auto t = buffer->lookupTransform(
          mapFrame, "simulator/" + robot + "/base_link", tf2::TimePointZero);
      double stamp = rclcpp::Time(t.header.stamp).seconds();
      if (stamp != lastPoseStamp) {
        const bool firstPose = lastPoseStamp < 0;
        lastPoseWall = Clock::now();
        lastPoseStamp = stamp;
        body = matrix(t.transform);
        if (firstPose && (selectedFocus == 1 || selectedFocus == 6 || selectedFocus == 7))
          focus(selectedFocus == 1 ? "Vehicle" : selectedFocus == 6 ? "Payloads" : "Claw");
        glm::vec3 p(body[3]);
        if (trail.empty() || glm::distance(trail.back(), p) > .06f) {
          trail.push_back(p);
          if (trail.size() > 1200)
            trail.pop_front();
        }
      }
      const bool fresh =
          std::chrono::duration<double>(Clock::now() - lastPoseWall).count() <
          1.;
      for (auto &c : cameras)
        c.ready = fresh;
      status = fresh ? "PHYSICS CONNECTED" : "POSE STALE";
    } catch (const tf2::TransformException &) {
      status = "WAITING FOR PHYSICS";
      for (auto &c : cameras)
        c.ready = false;
    }
  }
  template <class T> bool subscribed(const T &pub) const {
    return pub->get_subscription_count() > 0;
  }
  struct CameraDemand {
    bool rgb, left, depth, compressed, leftCompressed, cloud;
    bool info, leftInfo, depthInfo;
    bool pixels() const {
      return rgb || left || depth || compressed || leftCompressed || cloud;
    }
    bool metadata() const { return info || leftInfo || depthInfo; }
  };
  CameraDemand cameraDemand(const Camera &c) const {
    if (!c.ready || demo)
      return {};
    return {subscribed(c.rgb),
            subscribed(c.left),
            subscribed(c.depth),
            subscribed(c.compressed),
            subscribed(c.leftCompressed),
            cloudEnabled && subscribed(c.cloud),
            subscribed(c.info),
            subscribed(c.leftInfo),
            subscribed(c.depthInfo)};
  }
  void collectCamera(Camera &c) {
    if (!c.pending.valid() || c.pending.wait_for(std::chrono::seconds(0)) !=
                                  std::future_status::ready)
      return;
    auto output = c.pending.get();
    c.processingMs = output.processingMs;
    if (output.hasCloud) {
      const int slot = int(&c - &cameras[0]);
      renderer->pointCloud(slot, output.cloud, output.world,
                           slot == 0 ? glm::vec3(1.f, .55f, .15f) : glm::vec3(.2f, .9f, 1.f));
    }
    if (output.preview.empty())
      return;
    const auto &preview = output.preview;
    c.depthView = output.view;
    if (!c.depthTexture)
      glGenTextures(1, &c.depthTexture);
    glBindTexture(GL_TEXTURE_2D, c.depthTexture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, preview.cols, preview.rows, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, preview.data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  }
  void publish(Camera &c, const CameraDemand &demand, bool cloudDue,
               bool rendered, bool drawCloud = false) {
    // One output job per camera, with no queue. Drop acquisitions while busy
    // instead of blocking the GL/UI thread or accumulating stale observations.
    if (c.pending.valid())
      return;
    const bool live = c.ready && !demo;
    const bool point = cloudDue && (demand.cloud || drawCloud);
    const bool wantDepth =
        (rendered && c.depthPreview) || demand.depth || point;
    const bool wantRgb = demand.rgb || demand.left || demand.compressed ||
                         demand.leftCompressed || point;
    const auto stamp = now();
    // ROS-time pause: do not emit multiple observations with identical
    // acquisition stamps.
    if (live && stamp.seconds() == c.lastStamp && !c.depthPreview)
      return;
    cv::Mat rgb, depth;
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    if (wantRgb) {
      rgb.create(c.image.opaque.height, c.image.opaque.width, CV_8UC3);
      glBindFramebuffer(GL_FRAMEBUFFER, c.image.final.fbo);
      glReadPixels(0, 0, rgb.cols, rgb.rows, GL_RGB, GL_UNSIGNED_BYTE,
                   rgb.data);
      cv::flip(rgb, rgb, 0);
    }
    if (wantDepth) {
      depth.create(c.image.opaque.height, c.image.opaque.width, CV_32FC1);
      glBindFramebuffer(GL_FRAMEBUFFER, c.image.opaque.fbo);
      glReadPixels(0, 0, depth.cols, depth.rows, GL_DEPTH_COMPONENT, GL_FLOAT,
                   depth.data);
      cv::flip(depth, depth, 0);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    const bool emit = live && stamp.seconds() != c.lastStamp;
    if (emit) {
      c.lastStamp = stamp.seconds();
      if (!c.renders.empty() && stamp.nanoseconds() < c.renders.back().first)
        c.renders.clear();
      c.renders.emplace_back(stamp.nanoseconds(), c.world * pool::opticalToLink());
      while (c.renders.size() > 600) c.renders.pop_front();
    }
    if (!wantRgb && !wantDepth && !demand.metadata())
      return;
    // All OpenGL calls stay on this thread. The worker uses owned image buffers
    // and immutable configuration snapshots; ROS publishers are thread-safe.
    c.pending = std::async(
        std::launch::async,
        [&c, demand, point, drawCloud, wantRgb, wantDepth, emit, stamp,
         world = c.world, view = c.imageView, depthPreview = c.depthPreview, depthModel = depthModel,
         cloudStride = cloudStride, rgb = std::move(rgb),
         depth = std::move(depth)]() mutable {
          const auto start = Clock::now();
          CameraOutput output;
          output.view = view;
          if (wantDepth) {
            for (int y = 0; y < depth.rows; ++y)
              for (int x = 0; x < depth.cols; ++x) {
                float &z = depth.at<float>(y, x);
                z = z >= .999999f
                        ? NAN
                        : pool::linearDepth(z, c.k.nearPlane, c.k.farPlane);
              }
            depthModel.apply(depth, c.random);
            if (depthPreview) {
              cv::Mat gray;
              auto &preview = output.preview;
              depth.convertTo(gray, CV_8U, 255 / depthModel.maxRange);
              cv::applyColorMap(gray, preview, cv::COLORMAP_TURBO);
              for (int y = 0; y < depth.rows; ++y)
                for (int x = 0; x < depth.cols; ++x)
                  if (!std::isfinite(depth.at<float>(y, x)))
                    preview.at<cv::Vec3b>(y, x) = {16, 22, 27};
              cv::cvtColor(preview, preview, cv::COLOR_BGR2RGB);
              cv::flip(preview, preview, 0);
            }
          }
          if (emit) {
            std_msgs::msg::Header header;
            header.stamp = stamp;
            header.frame_id = c.frame;
            sensor_msgs::msg::CameraInfo info;
            info.header = header;
            info.width = c.k.width;
            info.height = c.k.height;
            info.distortion_model = "plumb_bob";
            info.d.assign(5, 0);
            info.k = {c.k.fx, 0, c.k.cx, 0, c.k.fy, c.k.cy, 0, 0, 1};
            info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            info.p = {c.k.fx, 0, c.k.cx, 0, 0, c.k.fy, c.k.cy, 0, 0, 0, 1, 0};
            if (demand.info)
              c.info->publish(info);
            if (demand.leftInfo)
              c.leftInfo->publish(info);
            if (demand.depthInfo)
              c.depthInfo->publish(info);
            if (wantRgb) {
              if (demand.rgb || demand.left) {
                sensor_msgs::msg::Image m;
                m.header = header;
                m.width = c.k.width;
                m.height = c.k.height;
                m.encoding = "rgb8";
                m.step = c.k.width * 3;
                m.data.assign(rgb.data, rgb.data + rgb.total() * 3);
                if (demand.rgb)
                  c.rgb->publish(m);
                if (demand.left)
                  c.left->publish(m);
              }
              if (demand.compressed || demand.leftCompressed) {
                sensor_msgs::msg::CompressedImage m;
                m.header = header;
                m.format = "rgb8; jpeg compressed bgr8";
                cv::Mat bgr;
                cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
                cv::imencode(".jpg", bgr, m.data,
                             {cv::IMWRITE_JPEG_QUALITY, 93});
                if (demand.compressed)
                  c.compressed->publish(m);
                if (demand.leftCompressed)
                  c.leftCompressed->publish(m);
              }
            }
            if (demand.depth) {
              sensor_msgs::msg::Image m;
              m.header = header;
              m.width = c.k.width;
              m.height = c.k.height;
              m.encoding = "32FC1";
              m.step = c.k.width * 4;
              m.data.resize(depth.total() * 4);
              std::memcpy(m.data.data(), depth.data, m.data.size());
              c.depth->publish(m);
            }
            if (point && demand.cloud) {
              sensor_msgs::msg::PointCloud2 m;
              m.header = header;
              m.is_dense = false;
              sensor_msgs::PointCloud2Modifier modifier(m);
              modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
              const int w = (c.k.width + cloudStride - 1) / cloudStride,
                        h = (c.k.height + cloudStride - 1) / cloudStride;
              modifier.resize(w * h);
              m.width = w;
              m.height = h;
              m.row_step = m.point_step * w;
              sensor_msgs::PointCloud2Iterator<float> x(m, "x"), y(m, "y"),
                  z(m, "z");
              sensor_msgs::PointCloud2Iterator<uint8_t> r(m, "r"), g(m, "g"),
                  b(m, "b");
              for (int v = 0; v < c.k.height; v += cloudStride)
                for (int u = 0; u < c.k.width;
                     u += cloudStride, ++x, ++y, ++z, ++r, ++g, ++b) {
                  const float d = depth.at<float>(v, u);
                  *x = (u - c.k.cx) * d / c.k.fx;
                  *y = (v - c.k.cy) * d / c.k.fy;
                  *z = d;
                  auto pixel = rgb.at<cv::Vec3b>(v, u);
                  *r = pixel[0];
                  *g = pixel[1];
                  *b = pixel[2];
                }
              c.cloud->publish(m);
            }
          }
          // Same samples as the published cloud (stride, depth noise, RGB),
          // rotated from the optical frame into the camera link frame so the
          // render-time camera pose places them where they were measured.
          if (point && drawCloud && depth.rows == c.k.height &&
              depth.cols == c.k.width && rgb.rows == c.k.height &&
              rgb.cols == c.k.width) {
            output.hasCloud = true;
            output.world = world;
            output.cloud.reserve(size_t(c.k.width / cloudStride + 1) *
                                 size_t(c.k.height / cloudStride + 1) * 6);
            for (int v = 0; v < c.k.height; v += cloudStride)
              for (int u = 0; u < c.k.width; u += cloudStride) {
                const float d = depth.at<float>(v, u);
                if (!std::isfinite(d)) continue;
                const float X = (u - c.k.cx) * d / c.k.fx,
                            Y = (v - c.k.cy) * d / c.k.fy;
                const auto pixel = rgb.at<cv::Vec3b>(v, u);
                output.cloud.insert(output.cloud.end(),
                                    {d, -X, -Y, pixel[0] / 255.f, pixel[1] / 255.f,
                                     pixel[2] / 255.f});
              }
          }
          output.processingMs =
              std::chrono::duration<double, std::milli>(Clock::now() - start)
                  .count();
          return output;
        });
  }

  void previewPose(const std::string &name) {
    const char *names[] = {"gate", "torpedo", "bin", "table"};
    for (int i = 0; i < 4; ++i)
      if (name == names[i])
        selectedDemo = i;
    auto it = renderer->landmarks.find(name);
    if (it == renderer->landmarks.end())
      throw std::runtime_error("Unknown demo_task: " + name);
    const glm::vec3 at(it->second[3]), normal(it->second[0]);
    float heading = std::atan2(-normal.y, -normal.x);
    glm::vec3 position = at + normal * 2.f;
    if (name == "gate")
      position.z = -.75f;
    if (name == "torpedo")
      position.z = at.z;
    if (name == "bin" || name == "table") {
      position = at;
      position.z = -.55f;
      // Centre the downward camera above the task using its configured mount.
      position -= glm::vec3(pool::pose({}, {0, 0, heading}) *
                            glm::vec4(glm::vec3(cameras[1].mount[3]), 0));
    }
    body = pool::pose(position, {0, 0, heading});
    for (auto &c : cameras)
      c.next = 0;
  }
  void focus(const std::string &name) {
    if (name == "Course") {
      target = {10, 4, -.6f};
      distance = 19;
      pitch = .64f;
      yaw = -2.5f;
    } else if (name == "Vehicle") {
      target = glm::vec3(body[3]);
      distance = 1.9f;
      pitch = .32f;
      yaw = std::atan2(body[0].y, body[0].x);
    } else if (name == "Claw") {
      target = glm::vec3(body * clawMount * glm::vec4(0,0,.06f,1));
      distance = .65f; pitch = .1f;
      yaw = std::atan2(body[0].y, body[0].x) + .5f;
    } else if (name == "Payloads") {
      target = glm::vec3(body * glm::vec4(payloadFocus, 1));
      distance = .48f;
      pitch = -.18f;
      yaw = std::atan2(body[0].y, body[0].x) + .06f;
    } else if (renderer->landmarks.count(name)) {
      auto p = renderer->landmarks.at(name);
      target = glm::vec3(p[3]);
      distance = name == "table" ? 3.7f : 3.4f;
      pitch = .28f;
      if (name.rfind("magnet_target",0)==0) { distance=.32f;pitch=.6f; }
      glm::vec3 facing(p[0]);
      yaw = std::atan2(facing.y, facing.x);
    }
    const char *names[] = {"Course",  "Vehicle", "gate",
                           "torpedo", "bin",     "table", "Payloads", "Claw",
                           "magnet_target1", "magnet_target2", "octagon"};
    for (int i = 0; i < 11; ++i)
      if (name == names[i])
        selectedFocus = i;
    mode = 0;
    follow = name == "Vehicle";
  }
  pool::View overviewView(float aspect, float dt, bool hovered) {
    auto &io = ImGui::GetIO();
    if (mouseCaptured && (mode != 1 || ImGui::IsKeyPressed(ImGuiKey_Escape) ||
                          !glfwGetWindowAttrib(window, GLFW_FOCUSED))) {
      mouseCaptured = false;
      glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
    if (mode == 1 && hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
        !mouseCaptured) {
      mouseCaptured = true;
      glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
      if (glfwRawMouseMotionSupported())
        glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
      glfwGetCursorPos(window, &lastMouseX, &lastMouseY);
    }
    if (mode == 1 && mouseCaptured) {
      double x, y;
      glfwGetCursorPos(window, &x, &y);
      freeYaw -= float(x - lastMouseX) * .003f;
      freePitch =
          glm::clamp(freePitch - float(y - lastMouseY) * .003f, -1.55f, 1.55f);
      lastMouseX = x;
      lastMouseY = y;
      glm::vec3 forward(cos(freeYaw), sin(freeYaw), 0),
          left(-forward.y, forward.x, 0), motion(0);
      if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        motion += forward;
      if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        motion -= forward;
      if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        motion += left;
      if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        motion -= left;
      if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        motion.z += 1;
      if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        motion.z -= 1;
      if (glm::length(motion) > 0)
        freeEye += glm::normalize(motion) * dt * (io.KeyCtrl ? 8.f : 2.5f);
    }
    if (mode == 0 && hovered) {
      if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        yaw -= io.MouseDelta.x * .004f;
        pitch = glm::clamp(pitch + io.MouseDelta.y * .004f, -1.3f, 1.5f);
      }
      distance =
          glm::clamp(distance * std::exp(-io.MouseWheel * .1f), .15f, 75.f);
      if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
        target += glm::vec3(-sin(yaw), cos(yaw), 0) * io.MouseDelta.x *
                  distance * .0015f;
        target += glm::vec3(cos(yaw), sin(yaw), 0) * io.MouseDelta.y *
                  distance * .0015f;
      }
    }
    if (follow)
      target = selectedFocus == 6 ? glm::vec3(body * glm::vec4(payloadFocus, 1))
                                  : selectedFocus == 7 ? glm::vec3(body * clawMount * glm::vec4(0,0,.06f,1)) : glm::vec3(body[3]);
    if (mode >= 2) {
      const auto &c = cameras[mode - 2];
      return c.depthPreview && c.depthTexture ? c.depthView : c.imageView;
    }
    glm::vec3 eye, at;
    if (mode == 1) {
      eye = freeEye;
      at = eye + glm::vec3(std::cos(freeYaw) * std::cos(freePitch),
                           std::sin(freeYaw) * std::cos(freePitch),
                           std::sin(freePitch));
    } else {
      eye = target + distance * glm::vec3(std::cos(yaw) * std::cos(pitch),
                                          std::sin(yaw) * std::cos(pitch),
                                          std::sin(pitch));
      at = target;
    }
    return {eye, glm::lookAt(eye, at, glm::vec3(0, 0, 1)),
            glm::perspective(glm::radians(53.f), aspect, .05f, 100.f)};
  }
  void pill(const std::string &text, ImVec4 tint) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(tint.x * .15f, tint.y * .15f,
                                                  tint.z * .15f, 1));
    ImGui::PushStyleColor(ImGuiCol_Text, tint);
    ImGui::Button(text.c_str());
    ImGui::PopStyleColor(2);
  }
  void heading(const char *text) {
    ImGui::PushFont(smallFont);
    ImGui::TextColored(muted, "%s", text);
    ImGui::PopFont();
  }
  void cameraCard(Camera &c, float width, float maxHeight) {
    ImGui::PushID(c.name.c_str());
    ImGui::BeginChild("camera", {width, 0},
                      ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
    ImGui::TextColored(cyan, "%s",
                       c.name == "ffc" ? "01  FORWARD CAMERA"
                                       : "02  DOWNWARD CAMERA");
    ImGui::SameLine(ImGui::GetContentRegionAvail().x - 48);
    if (ImGui::SmallButton(c.depthPreview ? "DEPTH" : "RGB"))
      c.depthPreview = !c.depthPreview;
    ImGui::PushFont(smallFont);
    ImGui::TextColored(muted, "ZED X MINI  /  %d x %d", c.k.width, c.k.height);
    ImGui::TextDisabled("Preview: %d x %d", c.image.opaque.width, c.image.opaque.height);
    ImGui::PopFont();
    float available = ImGui::GetContentRegionAvail().x;
    float w = std::min(available, maxHeight * float(c.k.width) / c.k.height);
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (available - w) / 2);
    ImVec2 pos = ImGui::GetCursorScreenPos();
    const GLuint tex =
        c.depthPreview && c.depthTexture ? c.depthTexture : c.image.final.color;
    ImGui::Image(textureID(tex), {w, w * c.k.height / c.k.width}, {0, 1},
                 {1, 0});
    if (!c.ready) {
      auto *draw = ImGui::GetWindowDrawList();
      draw->AddRectFilled(pos, {pos.x + w, pos.y + w * c.k.height / c.k.width},
                          IM_COL32(4, 13, 19, 175));
      draw->AddText({pos.x + 18, pos.y + 18}, color(muted),
                    "Awaiting vehicle pose");
    }
    ImGui::PushFont(smallFont);
    ImGui::TextColored(c.ready ? cyan : muted, "%s",
                       demo      ? "PREVIEW ONLY"
                       : c.ready ? "CONNECTED"
                                 : "NO SENSOR OUTPUT");
    ImGui::SameLine();
    ImGui::TextDisabled("  %.1f Hz  |  %s", c.measuredHz,
                        c.depthPreview ? "METRES" : "RECTIFIED RGB");
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip(
          "Calibration: %s\nDepth is rendered geometry with configurable "
          "noise; not the ZED neural stereo pipeline.",
          c.k.calibration.c_str());
    ImGui::PopFont();
    ImGui::EndChild();
    ImGui::PopID();
  }
  void drawWaterControls() {
    auto edited = look.water;
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10, 3));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 3));
    ImGui::SetNextItemWidth(240);
    bool changed = ImGui::ColorEdit3("Water tint", &edited.tint.x);
    ImGui::SameLine();
    if (ImGui::SmallButton("Clear blue")) {
      edited = pool::WaterOptics{};
      edited.tint = {.015f, .16f, .24f};
      edited.absorption = {.075f, .02f, .012f};
      edited.scattering = .045f;
      changed = true;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Pool")) { edited = pool::WaterOptics{}; changed = true; }
    ImGui::SameLine();
    if (ImGui::SmallButton("Green / murky")) {
      edited = pool::WaterOptics{};
      edited.tint = {.07f, .22f, .10f};
      edited.absorption = {.20f, .06f, .12f};
      edited.scattering = .25f;
      changed = true;
    }
    ImGui::Columns(4, "water controls", false);
    auto slider = [&](const char *label, float &v, float lo, float hi, const char *format) {
      ImGui::TextUnformatted(label);
      ImGui::SetNextItemWidth(-10);
      changed |= ImGui::SliderFloat((std::string("##water ")+label).c_str(), &v, lo, hi, format);
      ImGui::NextColumn();
    };
    slider("Haze / scattering", edited.scattering, 0, 1, "%.3f /m");
    slider("Distance strength", edited.distanceScale, 0, 5, "%.2f x");
    slider("Distance exponent", edited.distancePower, .25f, 3, "%.2f");
    slider("Clear distance", edited.clearDistance, 0, 10, "%.2f m");
    slider("Red absorption", edited.absorption.r, 0, 1, "%.3f /m");
    slider("Green absorption", edited.absorption.g, 0, 1, "%.3f /m");
    slider("Blue absorption", edited.absorption.b, 0, 1, "%.3f /m");
    ImGui::TextDisabled("More red absorption\nmakes distant objects\nlook bluer.");
    ImGui::Columns(1);
    ImGui::TextDisabled("Tint and haze accumulate along the underwater sightline. Both robot RGB cameras use these settings.");
    ImGui::TextDisabled("Exponent 1 / clear distance 0: exponential attenuation. Depth geometry is unchanged.");
    if (changed)
      set_parameters_atomically({
          rclcpp::Parameter("water.tint", std::vector<double>{edited.tint.r, edited.tint.g, edited.tint.b}),
          rclcpp::Parameter("water.absorption", std::vector<double>{edited.absorption.r, edited.absorption.g, edited.absorption.b}),
          rclcpp::Parameter("water.scattering", double(edited.scattering)),
          rclcpp::Parameter("water.distance_scale", double(edited.distanceScale)),
          rclcpp::Parameter("water.distance_power", double(edited.distancePower)),
          rclcpp::Parameter("water.clear_distance", double(edited.clearDistance))});
    ImGui::PopStyleVar(2);
  }
  void drawDepthControls(float width) {
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10, 3));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 3));
    auto edited = depthModel;
    bool changed = ImGui::Checkbox("Noise enabled", &edited.enabled);
    ImGui::SameLine();
    if (ImGui::SmallButton("Show both depth maps"))
      for (auto &c : cameras)
        c.depthPreview = true;
    ImGui::SameLine();
    if (ImGui::SmallButton("Reset noise")) {
      edited = pool::DepthNoise{};
      changed = true;
    }
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Point cloud in pool view");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90);
    ImGui::Combo("##cloudoverlay", &cloudOverlay, "Off\0FFC\0DFC\0Both\0");
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Draws the same samples the cloud_registered topics carry (stride,\n"
                        "rate and depth noise included) at the true camera pose in the\n"
                        "pool view. Sensor images never contain them.");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110);
    ImGui::SliderFloat("##cloudsize", &cloudPointSize, 1, 8, "%.0f px");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(150);
    ImGui::SliderFloat("##cloudtint", &cloudHighlight, 0, 1, "tint %.2f");
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("0 = sensor RGB as rviz shows it; 1 = orange (FFC) / cyan (DFC).");
    ImGui::Columns(4, "depth controls", false);
    auto slider = [&](const char *label, double &value, float lo, float hi,
                      const char *format) {
      ImGui::TextUnformatted(label);
      ImGui::SetNextItemWidth(-10);
      float v = value;
      const std::string id = std::string("##") + label;
      if (ImGui::SliderFloat(id.c_str(), &v, lo, hi, format)) {
        value = v;
        changed = true;
      }
      ImGui::NextColumn();
    };
    slider("Base sigma", edited.baseSigma, 0, .03, "%.3f m");
    slider("Range coefficient", edited.rangeSigma, 0, .015, "%.4f");
    slider("Range exponent", edited.exponent, 0, 4, "%.2f");
    slider("Bias", edited.bias, -.10, .10, "%.3f m");
    slider("Minimum range", edited.minRange, .05, 1, "%.2f m");
    slider("Maximum range", edited.maxRange, 1.1, 20, "%.1f m");
    slider("Missing pixels", edited.dropout, 0, .5, "%.3f");
    slider("Range dropout", edited.rangeDropout, 0, 1, "%.2f");
    slider("Edge dropout", edited.edgeDropout, 0, 1, "%.2f");
    slider("Outliers", edited.outliers, 0, .10, "%.3f");
    slider("Spatial correlation", edited.correlation, 0, 1, "%.2f");
    ImGui::TextUnformatted("Patch size");
    ImGui::SetNextItemWidth(-10);
    changed |= ImGui::SliderInt("##patch", &edited.patchSize, 1, 32, "%d px");
    ImGui::Columns(1);
    ImGui::TextDisabled("Sigma: %.1f mm at 1 m / %.1f mm at 3 m / %.1f mm at 6 "
                        "m. Dark pixels are invalid.",
                        edited.sigma(1) * 1000, edited.sigma(3) * 1000,
                        edited.sigma(6) * 1000);
    if (changed)
      set_parameters_atomically(
          {rclcpp::Parameter("depth_model.enabled", edited.enabled),
           rclcpp::Parameter("depth_noise", edited.rangeSigma),
           rclcpp::Parameter("depth_model.base_sigma", edited.baseSigma),
           rclcpp::Parameter("depth_model.range_exponent", edited.exponent),
           rclcpp::Parameter("depth_model.min_range", edited.minRange),
           rclcpp::Parameter("depth_model.max_range", edited.maxRange),
           rclcpp::Parameter("depth_model.bias", edited.bias),
           rclcpp::Parameter("depth_model.dropout", edited.dropout),
           rclcpp::Parameter("depth_model.range_dropout", edited.rangeDropout),
           rclcpp::Parameter("depth_model.edge_dropout", edited.edgeDropout),
           rclcpp::Parameter("depth_model.outliers", edited.outliers),
           rclcpp::Parameter("depth_model.correlation", edited.correlation),
           rclcpp::Parameter("depth_model.patch_size", edited.patchSize)});
    ImGui::PopStyleVar(2);
    (void)width;
  }
  void mapCanvas(float width, float height, bool interactive) {
    const ImVec2 a = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("course canvas", {width, height});
    const bool hovered = ImGui::IsItemHovered();
    auto &io = ImGui::GetIO();
    if (interactive && hovered) {
      mapZoom = glm::clamp(mapZoom * std::exp(io.MouseWheel * .15f), 1.f, 8.f);
      if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        mapPan.x += io.MouseDelta.x;
        mapPan.y += io.MouseDelta.y;
      }
    }
    const float scale = std::min((width - 36) / 50, (height - 36) / 22.86f) *
                        (interactive ? mapZoom : 1.f);
    const ImVec2 center(a.x + width / 2 + (interactive ? mapPan.x : 0),
                        a.y + height / 2 + (interactive ? mapPan.y : 0));
    auto poolXY = [&](glm::vec2 p) {
      return ImVec2(center.x + (p.x - 25) * scale,
                    center.y - (p.y - 11.43f) * scale);
    };
    auto xy = [&](glm::vec3 p) {
      return poolXY(glm::vec2(renderer->mapToPool * glm::vec4(p, 1)));
    };
    auto *d = ImGui::GetWindowDrawList();
    d->AddRectFilled(a, {a.x + width, a.y + height}, IM_COL32(9, 24, 32, 255),
                     5);
    d->PushClipRect(a, {a.x + width, a.y + height}, true);
    d->AddRectFilled(poolXY({0, 22.86f}), poolXY({50, 0}),
                     IM_COL32(13, 40, 50, 255));
    for (int i = 0; i <= 50; i += 5)
      d->AddLine(poolXY({float(i), 0}), poolXY({float(i), 22.86f}),
                 IM_COL32(35, 64, 74, 255));
    for (int i = 0; i <= 20; i += 5)
      d->AddLine(poolXY({0, float(i)}), poolXY({50, float(i)}),
                 IM_COL32(35, 64, 74, 255));
    d->AddRect(poolXY({0, 22.86f}), poolXY({50, 0}),
               IM_COL32(94, 154, 166, 255), 0, 0, 2);
    for (size_t i = 1; i < trail.size(); ++i)
      d->AddLine(xy(trail[i - 1]), xy(trail[i]), IM_COL32(53, 134, 143, 200),
                 1.5f);
    int index = 0;
    for (const auto &key :
         {"gate", "slalom_front", "slalom_back", "torpedo", "bin", "table", "octagon"})
      if (renderer->landmarks.count(key)) {
        const auto p = xy(glm::vec3(renderer->landmarks.at(key)[3]));
        const float font = interactive ? 16 : 12;
        const ImVec2 textAt(p.x + 7, p.y + (index++ % 2 ? -19 : 4));
        d->AddCircleFilled(p, interactive ? 5 : 4, color(cyan));
        if (interactive || std::string(key) != "slalom_back")
          d->AddText(interactive ? normalFont : smallFont, font, textAt,
                     color(white),
                     !interactive && std::string(key) == "slalom_front"
                         ? "slalom"
                         : key);
        if (interactive && hovered &&
            ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
            ImGui::GetMouseDragDelta().x * ImGui::GetMouseDragDelta().x +
                    ImGui::GetMouseDragDelta().y *
                        ImGui::GetMouseDragDelta().y <
                9 &&
            std::hypot(io.MousePos.x - p.x, io.MousePos.y - p.y) < 12)
          focus(key);
      }
    const auto p = xy(glm::vec3(body[3]));
    const auto tip = xy(glm::vec3(body[3]) + glm::vec3(body[0]) * 1.3f);
    d->AddCircleFilled(p, 6, IM_COL32(255, 208, 96, 255));
    d->AddLine(p, tip, IM_COL32(255, 208, 96, 255), 3);
    d->AddText(smallFont, 12, {p.x + 8, p.y - 15}, IM_COL32(255, 208, 96, 255),
               robot.c_str());
    d->PopClipRect();
  }
  void minimap(float width) {
    ImGui::BeginChild("map", {width, 225}, ImGuiChildFlags_Borders);
    heading("COURSE MAP");
    ImGui::SameLine();
    if (ImGui::SmallButton("Expand"))
      largeMap = focusMap = true;
    const auto available = ImGui::GetContentRegionAvail();
    mapCanvas(available.x, std::max(120.f, available.y), false);
    ImGui::EndChild();
  }
  void captureTf() {
    displayedFrames.clear();
    tfDifference.clear();
    tfResolved = tfMissing = 0;
    if (!showTf && !tfTreeOpen) return;
    std::map<std::string, std::string> parents;
    parents[mapFrame] = "";
    if (demo) {
      for (const auto &[name, pose] : previewFrames) {
        auto parent = robot + "/base_link";
        if (name == parent) parent = mapFrame;
        else if (name == robot + "/torpedo_0_link" || name == robot + "/torpedo_1_link")
          parent = robot + "/torpedo_link";
        parents[name] = parent;
      }
    } else {
      std::vector<std::string> frames;
      buffer->_getFrameStrings(frames);
      for (const auto &name : frames) {
        std::string parent;
        buffer->_getParent(name, tf2::TimePointZero, parent);
        parents[name] = parent;
        if (!parent.empty()) parents.try_emplace(parent, "");
      }
    }
    tfTree.update(parents);
    // Freeze the overlay before camera rendering/shadows. Looking up each axis
    // during UI drawing sampled a newer pose than the already-rendered vehicle.
    for (const auto &[name, parent] : parents) {
      auto &frame = tfTree.frames.at(name);
      try {
        const auto pose = name == mapFrame ? glm::mat4(1) : demo ? body * previewFrames.at(name) :
            matrix(buffer->lookupTransform(mapFrame, name, tf2::TimePointZero).transform);
        frame.available = true;
        if (frame.enabled) {
          displayedFrames[name] = pose;
          ++tfResolved;
        }
      } catch (const tf2::TransformException &) {
        if (frame.enabled) ++tfMissing;
      }
    }
    if (!demo) {
      try {
        const auto estimate = buffer->lookupTransform(mapFrame, robot + "/base_link", tf2::TimePointZero);
        const auto truth = buffer->lookupTransform(mapFrame, "simulator/" + robot + "/base_link", tf2::TimePointZero);
        const auto stamp = std::min(rclcpp::Time(estimate.header.stamp), rclcpp::Time(truth.header.stamp));
        const auto at = matrix(buffer->lookupTransform(mapFrame, "simulator/" + robot + "/base_link", stamp).transform);
        const auto relative = glm::inverse(at) * matrix(buffer->lookupTransform(mapFrame, robot + "/base_link", stamp).transform);
        const float angle = glm::degrees(2 * std::acos(glm::clamp(std::abs(glm::quat_cast(relative).w), 0.f, 1.f)));
        tfDifference = "ROS vs sim: " + fixed(glm::length(glm::vec3(relative[3])) * 100, 1) +
            " cm / " + fixed(angle, 1) + " deg; forward " + fixed(relative[3].x * 100, 1) + " cm";
      } catch (const tf2::TransformException &) {
        tfDifference = "Waiting for matching ROS / simulator poses";
      }
    }
  }
  void receiveDetections(const visualization_msgs::msg::MarkerArray &msg) {
    using visualization_msgs::msg::Marker;
    const auto now = Clock::now();
    for (const auto &m : msg.markers) {
      if (m.action == Marker::DELETEALL) detectionMarkers.clear();
      else if (m.action == Marker::DELETE) detectionMarkers.erase({m.ns, m.id});
      else detectionMarkers[{m.ns, m.id}] = DetectionEntry{m, now};
    }
  }
  // Camera observations belong at their actual simulator acquisition pose.
  // Missing render history falls back only to timestamped simulator TF.
  void captureDetections() {
    placedDetections.clear();
    detectionStatus.clear();
    if (demo || !detections) return;
    const auto now = Clock::now();
    for (auto it = detectionMarkers.begin(); it != detectionMarkers.end();) {
      const double life = it->second.marker.lifetime.sec + it->second.marker.lifetime.nanosec * 1e-9;
      if (life > 0 && std::chrono::duration<double>(now - it->second.received).count() > life)
        it = detectionMarkers.erase(it);
      else ++it;
    }
    int boxes = 0, unresolved = 0;
    for (auto &[key, entry] : detectionMarkers) {
      const auto &m = entry.marker;
      std::string sourceFrame = m.header.frame_id;
      const glm::mat4 *acquisitionPose = nullptr;
      const int64_t stamp = rclcpp::Time(m.header.stamp).nanoseconds();
      for (const auto &c : cameras) {
        if (sourceFrame != c.frame && sourceFrame != "simulator/" + c.frame) continue;
        sourceFrame = "simulator/" + c.frame;
        for (const auto &render : c.renders) {
          if (stamp != 0 && render.first == stamp) {
            acquisitionPose = &render.second;
            break;
          }
        }
        break;
      }
      const bool placed = entry.placement.place(m, mapFrame, *buffer,
                                               sourceFrame, acquisitionPose);
      if (placed) placedDetections.push_back({entry.placement.world(), m});
      else ++unresolved;
      if (m.type == visualization_msgs::msg::Marker::CUBE) ++boxes;
    }
    detectionStatus = "Detections: " + std::to_string(boxes) + " boxes";
    if (unresolved) detectionStatus += ", " + std::to_string(unresolved) + " waiting for timestamped TF";
  }
  void drawDetections(const pool::View &view, ImVec2 position, float width, float height) {
    if (!detections || demo) return;
    auto *draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(position, {position.x + width, position.y + height}, true);
    const auto vp = view.projection * view.view;
    auto project = [&](glm::vec4 world, ImVec2 &pixel) {
      const auto clip = vp * world;
      if (clip.w <= 0 || clip.z < -clip.w || clip.z > clip.w) return false;
      pixel = {position.x + (clip.x / clip.w * .5f + .5f) * width,
               position.y + (.5f - clip.y / clip.w * .5f) * height};
      return true;
    };
    using visualization_msgs::msg::Marker;
    for (const auto &placed : placedDetections) {
      const auto &m = placed.marker;
      const ImU32 tint = color({m.color.r, m.color.g, m.color.b, m.color.a});
      if (m.type == Marker::CUBE) {
        const float hx = m.scale.x / 2, hy = m.scale.y / 2;
        const glm::vec4 corners[] = {{-hx, -hy, 0, 1}, {hx, -hy, 0, 1}, {hx, hy, 0, 1}, {-hx, hy, 0, 1}};
        ImVec2 pixels[4];
        bool visible = true;
        for (int i = 0; i < 4 && visible; ++i) visible = project(placed.pose * corners[i], pixels[i]);
        if (!visible) continue;
        draw->AddConvexPolyFilled(pixels, 4, tint);
        draw->AddPolyline(pixels, 4, tint, ImDrawFlags_Closed, 1.5f);
      } else if (m.type == Marker::ARROW) {
        // Pose+scale arrow along the marker's +X, drawn like rviz: a shaft
        // ending at 77% of the length and a head over the final 23%. The head
        // base is projected in 3D so it foreshortens with the view; its wings
        // are spread in screen space so the head stays readable at any depth.
        ImVec2 tail, neck, tip;
        if (!project(placed.pose[3], tail) || !project(placed.pose * glm::vec4(m.scale.x * .77f, 0, 0, 1), neck) ||
            !project(placed.pose * glm::vec4(m.scale.x, 0, 0, 1), tip))
          continue;
        const float thickness = 1.5f;
        draw->AddLine(tail, neck, tint, thickness);
        const ImVec2 axis{tip.x - neck.x, tip.y - neck.y};
        const float headLength = std::sqrt(axis.x * axis.x + axis.y * axis.y);
        if (headLength < 1e-3f) {
          draw->AddCircleFilled(tip, thickness, tint);
          continue;
        }
        const float halfWidth = std::clamp(headLength * .45f, 3.f, 12.f);
        const ImVec2 normal{-axis.y / headLength * halfWidth, axis.x / headLength * halfWidth};
        const ImVec2 head[] = {tip, {neck.x + normal.x, neck.y + normal.y}, {neck.x - normal.x, neck.y - normal.y}};
        draw->AddTriangleFilled(head[0], head[1], head[2], tint);
        draw->AddTriangle(head[0], head[1], head[2], tint, 1.f);
      }
    }
    draw->PopClipRect();
    if (!detectionStatus.empty())
      draw->AddText(smallFont, 12, {position.x + 14, position.y + 101}, color(white), detectionStatus.c_str());
  }
  // Asks physics_simulator to teleport the plant to the EKF's base_link pose.
  // The viewer spins on the UI thread, so the reply lands here without locks.
  void requestSyncToEstimate() {
    if (!syncClient->service_is_ready()) {
      syncStatus = "Sync: physics simulator service unavailable";
      return;
    }
    syncStatus = "Sync: requested";
    syncClient->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          const auto reply = future.get();
          syncStatus = std::string(reply->success ? "Sync: " : "Sync failed: ") + reply->message;
        });
  }
  // Asks physics_simulator to put the plant back at its start pose, at rest.
  void requestResetToStart() {
    if (!resetStartClient->service_is_ready()) {
      syncStatus = "Reset: physics simulator service unavailable";
      return;
    }
    syncStatus = "Reset: requested";
    resetStartClient->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          const auto reply = future.get();
          syncStatus = std::string(reply->success ? "Reset: " : "Reset failed: ") + reply->message;
        });
  }
  void drawTfTree() {
    if (ImGui::Button("Show all")) tfTree.selectAll(true);
    ImGui::SameLine();
    if (ImGui::Button("Hide all")) tfTree.selectAll(false);
    ImGui::TextDisabled("Only this: one frame. With children: the frame and all descendants.");
    ImGui::BeginChild("TF frame tree", {620, 320}, ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_HorizontalScrollbar);
    if (ImGui::BeginTable("TF visibility", 3, ImGuiTableFlags_Resizable |
                         ImGuiTableFlags_BordersInnerV)) {
      ImGui::TableSetupColumn("Frame", ImGuiTableColumnFlags_WidthStretch);
      ImGui::TableSetupColumn("Only this", ImGuiTableColumnFlags_WidthFixed, 76);
      ImGui::TableSetupColumn("With children", ImGuiTableColumnFlags_WidthFixed, 112);
      ImGui::TableHeadersRow();
      std::set<std::string> visited;
      const auto row = [&](const auto &self, const std::string &name) -> void {
        if (!visited.insert(name).second) return;
        auto &frame = tfTree.frames.at(name);
        const auto children = tfTree.children.find(name);
        const bool leaf = children == tfTree.children.end() || children->second.empty();
        ImGui::PushID(name.c_str());
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                                  ImGuiTreeNodeFlags_FramePadding |
                                  ImGuiTreeNodeFlags_SpanAvailWidth;
        if (leaf) flags |= ImGuiTreeNodeFlags_Leaf;
        if (name == mapFrame) flags |= ImGuiTreeNodeFlags_DefaultOpen;
        const bool open = ImGui::TreeNodeEx("branch", flags, "%s", name.c_str());
        if (ImGui::BeginPopupContextItem("branch selection")) {
          if (ImGui::MenuItem("Show branch")) tfTree.selectBranch(name, true);
          if (ImGui::MenuItem("Hide branch")) tfTree.selectBranch(name, false);
          ImGui::EndPopup();
        }
        if (!frame.available) {
          ImGui::SameLine();
          ImGui::TextDisabled("(unavailable)");
        }
        ImGui::TableSetColumnIndex(1);
        ImGui::Checkbox("##enabled", &frame.enabled);
        ImGui::TableSetColumnIndex(2);
        if (!leaf) {
          const auto selection = tfTree.branchSelection(name);
          bool enabled = selection == pool::TfTree::Selection::Shown;
          ImGui::PushItemFlag(ImGuiItemFlags_MixedValue,
                             selection == pool::TfTree::Selection::Mixed);
          if (ImGui::Checkbox("##with_children", &enabled))
            tfTree.selectBranch(name, enabled);
          ImGui::PopItemFlag();
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Show or hide this frame and all descendants.\nA filled square means some frames are shown.");
        }
        ImGui::TableSetColumnIndex(0);
        if (open) {
          if (!leaf)
            for (const auto &child : children->second) self(self, child);
          ImGui::TreePop();
        }
        ImGui::PopID();
      };
      for (const auto &root : tfTree.roots) row(row, root);
      ImGui::EndTable();
    }
    ImGui::EndChild();
  }
  void drawTf(const pool::View &view, ImVec2 position, float width, float height) {
    if (!showTf) return;
    auto *draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(position, {position.x + width, position.y + height}, true);
    const auto vp = view.projection * view.view;
    auto project = [&](glm::vec4 world, ImVec2 &pixel) {
      const auto clip = vp * world;
      if (clip.w <= 0 || clip.z < -clip.w || clip.z > clip.w) return false;
      pixel = {position.x + (clip.x / clip.w * .5f + .5f) * width,
               position.y + (.5f - clip.y / clip.w * .5f) * height};
      return true;
    };
    const ImU32 colors[] = {IM_COL32(255, 70, 70, 255),
                            IM_COL32(75, 235, 100, 255),
                            IM_COL32(80, 150, 255, 255)};
    for (const auto &[name, frame] : displayedFrames) {
      ImVec2 origin;
      if (!project(frame[3], origin)) continue;
      for (int axis = 0; axis < 3; ++axis) {
        ImVec2 end;
        if (!project(frame[3] + frame[axis] * tfAxisLength, end)) continue;
        draw->AddLine(origin, end, colors[axis], 2.5f);
        draw->AddCircleFilled(end, 3, colors[axis]);
      }
      draw->AddCircleFilled(origin, 3, IM_COL32(255, 255, 255, 255));
      if (tfNames) {
        const ImVec2 at(origin.x + 5, origin.y + 5);
        draw->AddText(smallFont, 12, {at.x + 1, at.y + 1}, IM_COL32(0, 0, 0, 255), name.c_str());
        draw->AddText(smallFont, 12, at, color(white), name.c_str());
      }
    }
    const auto caption = demo ? "TF preview from robot config" :
        "ROS TF in " + mapFrame + ": " +
        std::to_string(tfResolved) + " frames, " +
        std::to_string(tfMissing) + " unavailable";
    draw->AddText(smallFont, 12, {position.x + 14, position.y + 50}, color(white), caption.c_str());
    if (!tfDifference.empty())
      draw->AddText(smallFont, 12, {position.x + 14, position.y + 67}, color(white), tfDifference.c_str());
    if (!syncStatus.empty())
      draw->AddText(smallFont, 12, {position.x + 14, position.y + 84}, color(white), syncStatus.c_str());
    draw->PopClipRect();
  }
  std::string runTime() const {
    const double seconds = runScore && runScore["elapsed"] ? runScore["elapsed"].as<double>() : 0.;
    char value[64];
    std::snprintf(value,sizeof(value),"%02d:%04.1f",int(seconds)/60,std::fmod(seconds,60.));
    return value;
  }
  void sendRunCommand(const YAML::Node &command) {
    // All string fields are fixed UI tokens; numeric input is validated above.
    std_msgs::msg::String msg;
    const auto action=command["action"].as<std::string>();
    msg.data="{\"action\":\""+action+"\"";
    if(command["role"]) msg.data+=",\"role\":\""+command["role"].as<std::string>()+"\"";
    if(command["heading_coin"]) msg.data+=std::string(",\"heading_coin\":")+(command["heading_coin"].as<bool>() ? "true" : "false");
    if(command["role_coin"]) msg.data+=std::string(",\"role_coin\":")+(command["role_coin"].as<bool>() ? "true" : "false");
    if(command["points"]) msg.data+=",\"points\":"+std::to_string(command["points"].as<double>());
    msg.data+="}";runCommand->publish(msg);
  }
  void drawRunControls(bool showDetailsButton=true) {
    const bool connected=!demo && std::chrono::duration<double>(Clock::now()-lastRunScore).count()<1.;
    const bool running=runScore && runScore["running"] && runScore["running"].as<bool>();
    const double total=runScore && runScore["total"] ? runScore["total"].as<double>() : 0.;
    ImGui::TextColored(cyan,"%s   |   %.1f points",runTime().c_str(),total);
    if(showDetailsButton) {ImGui::SameLine();if(ImGui::Button("Detailed scorecard")) scorecardOpen=focusScorecard=true;}
    ImGui::BeginDisabled(!connected);
    ImGui::BeginDisabled(running);
    ImGui::SetNextItemWidth(210);
    ImGui::Combo("Intended role",&intendedRole,"Survey & Repair\0Search & Rescue\0");
    ImGui::Checkbox("Heading coin flip",&headingCoin);
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("Confirm the real heading coin flip was performed before starting; this adds 300 points after the gate.");
    ImGui::SameLine();
    ImGui::Checkbox("Role coin flip",&roleCoin);
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("Select the role assigned by the coin flip above. Adds 150 points when the gate passage matches that role.");
    if(ImGui::Button("Start run")) {
      YAML::Node cmd;cmd["action"]="start";
      cmd["role"]=intendedRole==0 ? "repair" : "rescue";
      cmd["heading_coin"]=headingCoin;cmd["role_coin"]=roleCoin;sendRunCommand(cmd);
    }
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("Clear tasks and score, reload/disarm, and start a fresh timer. Vehicle position stays unchanged.");
    ImGui::EndDisabled();ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if(ImGui::Button("Stop run")) {YAML::Node cmd;cmd["action"]="stop";sendRunCommand(cmd);}
    ImGui::EndDisabled();ImGui::SameLine();
    if(ImGui::Button("Reset run & tasks")) taskReset->publish(std_msgs::msg::Empty{});
    ImGui::EndDisabled();
    if(!connected) ImGui::TextDisabled("Start the live simulator to score a run.");
    if(runScore && runScore["intended_role"]) {
      const std::string intended=runScore["intended_role"].as<std::string>();
      const std::string actual=runScore["role"].IsNull() ? "Awaiting gate passage" : runScore["role"].as<std::string>();
      ImGui::Text("Assigned: %s  |  Scored role: %s",intended.c_str(),actual.c_str());
      ImGui::TextWrapped("%s",runScore["message"].as<std::string>().c_str());
      const auto reason=runScore["ended_reason"].as<std::string>();
      if(!reason.empty()) ImGui::TextColored(ImVec4(1,.6f,.3f,1),"%s",reason.c_str());
    }
    ImGui::TextDisabled("Timer uses simulation time; pauses with physics. Stop is manual.");
  }
  void drawScorecard() {
    if(!scorecardOpen) return;
    ImGui::SetNextWindowSize({580,780},ImGuiCond_FirstUseEver);
    if (focusScorecard) {
      ImGui::SetNextWindowFocus();
      ImGui::SetNextWindowCollapsed(false);
      focusScorecard = false;
    }
    if(ImGui::Begin("RoboSub 2026 scorecard",&scorecardOpen)) {
      drawRunControls(false);
      ImGui::Separator();
      if(runScore && runScore["rows"]) {
        if(ImGui::BeginTable("Awards",2,ImGuiTableFlags_RowBg|ImGuiTableFlags_BordersInnerH)) {
          ImGui::TableSetupColumn("Award",ImGuiTableColumnFlags_WidthStretch);
          ImGui::TableSetupColumn("Points",ImGuiTableColumnFlags_WidthFixed,80);
          ImGui::TableHeadersRow();
          for(const auto &row:runScore["rows"]) {
            ImGui::TableNextRow();ImGui::TableNextColumn();
            ImGui::TextUnformatted(row["label"].as<std::string>().c_str());
            ImGui::TableNextColumn();ImGui::Text("%d",row["points"].as<int>());
          }
          ImGui::EndTable();
        }
        ImGui::Text("Manual adjustment: %.1f",runScore["adjustment"].as<double>());
        ImGui::Text("TOTAL: %.1f",runScore["total"].as<double>());
        ImGui::Text("Objects in both baskets: %d",runScore["basket_count"].as<int>());
        ImGui::TextDisabled("Time-bonus prerequisites: %s",runScore["time_bonus_eligible"].as<bool>() ? "met" : "not met");
      }
      ImGui::Separator();
      ImGui::TextWrapped("Referee / weight / other adjustments (signed points; replaces the prior adjustment):");
      ImGui::SetNextItemWidth(150);ImGui::InputFloat("##manual_points",&manualPoints,50,100,"%.1f");
      ImGui::SameLine();
      ImGui::BeginDisabled(demo || std::chrono::duration<double>(Clock::now()-lastRunScore).count()>=1. || !std::isfinite(manualPoints));
      if(ImGui::Button("Apply adjustment")) {YAML::Node cmd;cmd["action"]="adjustment";cmd["points"]=manualPoints;sendRunCommand(cmd);}
      ImGui::EndDisabled();
      ImGui::TextWrapped("Pinger awards await future acoustic integration. No session countdown or automatic time bonus is applied to individual runs.");
    }
    ImGui::End();
  }
  void drawInterface(float time, float dt) {
    auto &io = ImGui::GetIO();
    float W = io.DisplaySize.x, H = io.DisplaySize.y;
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({W, H});
    ImGui::Begin("Riptide", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::PushFont(titleFont);
    ImGui::TextUnformatted("RIPTIDE");
    ImGui::PopFont();
    ImGui::SameLine();
    ImGui::TextColored(muted, " / ");
    ImGui::SameLine();
    ImGui::TextUnformatted("ROBOSUB SIMULATION");
    ImGui::SameLine(W - 395);
    pill(status, demo               ? ImVec4(.94, .73, .35, 1)
                 : cameras[0].ready ? cyan
                                    : muted);
    ImGui::SameLine();
    if (ImGui::Button("Capture"))
      capture = true;
    ImGui::Separator();
    const float side = glm::clamp(W * .265f, 305.f, 405.f),
                left = W - side - 56, contentHeight = H - 112;
    ImGui::BeginChild("left", {left, contentHeight}, ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoScrollbar);
    ImGui::BeginChild("toolbar", {left, 46}, ImGuiChildFlags_None);
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(cyan, "POOL VIEW");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(134);
    int oldMode = mode;
    ImGui::Combo("##view", &mode,
                 "Orbit\0Free camera\0Forward camera\0Downward camera\0");
    if (mode == 1 && oldMode != 1) {
      freeEye = target + glm::vec3(-3, -3, 1);
      freeYaw = .78f;
      freePitch = -.2f;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(132);
    if (ImGui::Combo("##focus", &selectedFocus,
                     "Course\0Vehicle\0Gate\0Torpedo\0Bins\0Table\0Payloads\0Claw\0Light 1\0Light 2\0Octagon\0")) {
      const char *names[] = {"Course",  "Vehicle", "gate",
                             "torpedo", "bin",     "table", "Payloads", "Claw",
                             "magnet_target1", "magnet_target2", "octagon"};
      focus(names[selectedFocus]);
    }
    ImGui::SameLine();
    ImGui::Checkbox("Follow", &follow);
    ImGui::SameLine();
    ImGui::Checkbox("Labels", &labels);
    ImGui::SameLine();
    if (ImGui::Button("TF")) ImGui::OpenPopup("TF display");
    tfTreeOpen = false;
    if (ImGui::BeginPopup("TF display")) {
      tfTreeOpen = true;
      ImGui::Checkbox("Show TF frames", &showTf);
      ImGui::Checkbox("Frame names", &tfNames);
      ImGui::SetNextItemWidth(220);
      ImGui::SliderFloat("Axis length", &tfAxisLength, .02f, 1.f, "%.2f m");
      ImGui::TextUnformatted("X: red   Y: green   Z: blue");
      drawTfTree();
      ImGui::TextDisabled("Axes show through objects. Unavailable frames cannot reach the fixed frame.");
      ImGui::TextDisabled("%s", demo ? "Preview: configured base and payload frames only." :
          "Raw ROS TF in the fixed frame, including localization drift.");
      ImGui::EndPopup();
    }
    if (!demo) {
      ImGui::SameLine();
      ImGui::Checkbox("Detections", &detections);
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Camera detections use the simulator pose at image capture.\n"
                          "Each observation stays fixed in the simulated world.");
      ImGui::SameLine();
      if (ImGui::Button("Sync sim to ROS")) requestSyncToEstimate();
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Move the simulated vehicle to the EKF's current base_link pose.\n"
                          "Velocities are kept and the EKF is not touched, so the\n"
                          "\"ROS vs sim\" difference drops to zero from the plant's side.");
      ImGui::SameLine();
      if (ImGui::Button("Reset to start")) requestResetToStart();
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Move the simulated vehicle back to its start pose, at rest with\n"
                          "thrusters cleared, and re-seed the EKF there like set_sim_pose.\n"
                          "Start is the launch pose, or the last explicit set_sim_pose.");
    }
    if (demo) {
      ImGui::SameLine();
      ImGui::SetNextItemWidth(163);
      if (ImGui::Combo("##previewtask", &selectedDemo,
                       "Preview: gate\0Preview: torpedo\0Preview: "
                       "bins\0Preview: table\0")) {
        const char *tasks[] = {"gate", "torpedo", "bin", "table"};
        previewPose(tasks[selectedDemo]);
      }
    }
    ImGui::EndChild();
    float viewHeight = std::max(230.f, contentHeight - 391);
    ImVec2 position = ImGui::GetCursorScreenPos();
    bool hovered = ImGui::IsMouseHoveringRect(
        position, {position.x + left, position.y + viewHeight});
    // Dropdowns can overlap the viewport. Selecting Free camera must not also
    // consume that click as a mouse-capture request.
    hovered = hovered && ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) &&
              mode == oldMode &&
              !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId);
    auto view = overviewView(left / viewHeight, dt, hovered);
    // Keep sensor aspect ratios when their view is promoted to the large
    // viewport.
    float iw = left, ih = viewHeight;
    if (mode >= 2) {
      float aspect =
          float(cameras[mode - 2].k.width) / cameras[mode - 2].k.height;
      ih = std::min(viewHeight, left / aspect);
      iw = ih * aspect;
    }
    overview.resize(std::max(16, int(iw)), std::max(16, int(ih)));
    renderer->pointSize = cloudPointSize;
    renderer->pointHighlight = cloudHighlight;
    if (mode < 2)
      renderer->render(overview, view, look, time, true, true, cloudOverlay != 0 && !demo);
    const GLuint mainTexture =
        mode < 2 ? overview.final.color
                 : (cameras[mode - 2].depthPreview && cameras[mode - 2].depthTexture
                        ? cameras[mode - 2].depthTexture
                        : cameras[mode - 2].image.final.color);
    ImGui::SetCursorScreenPos(
        {position.x + (left - iw) / 2, position.y + (viewHeight - ih) / 2});
    ImGui::Image(textureID(mainTexture), {iw, ih}, {0, 1}, {1, 0});
    drawTf(view, {position.x + (left - iw) / 2, position.y + (viewHeight - ih) / 2}, iw, ih);
    drawDetections(view, {position.x + (left - iw) / 2, position.y + (viewHeight - ih) / 2}, iw, ih);
    auto *d = ImGui::GetWindowDrawList();
    d->AddRect(position, {position.x + left, position.y + viewHeight},
               IM_COL32(38, 62, 72, 255), 5, 0, 1);
    d->AddRectFilled({position.x + 14, position.y + 14},
                     {position.x + 237, position.y + 43},
                     IM_COL32(8, 22, 29, 225), 4);
    d->AddText(smallFont, 12, {position.x + 25, position.y + 22}, color(white),
               "OSU POOL   /   50.0 x 22.86 m");
    if (runScore && runScore["total"]) {
      std::string readout = fixed(runScore["total"].as<double>(),1)+" pts   /   "+runTime();
      if (runScore["running"].as<bool>()) readout += "  RUNNING";
      d->AddRectFilled({position.x+left-310,position.y+12},{position.x+left-12,position.y+43},IM_COL32(8,22,29,225),4);
      d->AddText(smallFont,14,{position.x+left-298,position.y+21},color(cyan),readout.c_str());
    }
    if (labels && mode < 2 && selectedFocus != 6 && selectedFocus != 7) {
      for (const auto &key :
           {"gate", "slalom_front", "torpedo", "bin", "table", "octagon"})
        if (renderer->landmarks.count(key)) {
          glm::vec4 p = view.projection * view.view *
                        (renderer->landmarks.at(key) * glm::vec4(0, 0, .5, 1));
          if (p.w <= 0)
            continue;
          p /= p.w;
          if (std::abs(p.x) > .94 || std::abs(p.y) > .85 || p.z > 1)
            continue;
          ImVec2 pos(position.x + (p.x * .5f + .5f) * left,
                     position.y + (.5f - p.y * .5f) * viewHeight);
          d->AddCircleFilled(pos, 3, color(cyan));
          d->AddLine(pos, {pos.x + 10, pos.y - 14}, color(cyan));
          d->AddRectFilled({pos.x + 9, pos.y - 31}, {pos.x + 105, pos.y - 11},
                           IM_COL32(8, 22, 29, 215), 3);
          d->AddText(smallFont, 12, {pos.x + 16, pos.y - 28}, color(white),
                     key);
        }
    }
    const char *controls =
        mode == 1 ? "CLICK  mouse look    WASD  move    SPACE / SHIFT  up / "
                    "down    CTRL  fast    ESC  release"
                  : "LEFT DRAG  orbit     SCROLL  zoom     RIGHT DRAG  pan";
    d->AddRectFilled({position.x, position.y + viewHeight - 30},
                     {position.x + left, position.y + viewHeight},
                     IM_COL32(6, 18, 26, 205));
    d->AddText(smallFont, 12, {position.x + 14, position.y + viewHeight - 21},
               color(white), controls);
    ImGui::SetCursorScreenPos({position.x, position.y + viewHeight + 12});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16, 8));
    ImGui::BeginChild("readouts", {left, 76}, ImGuiChildFlags_Borders);
    const auto p = glm::vec3(body[3]);
    const float headingDegrees =
        std::fmod(glm::degrees(std::atan2(body[0].y, body[0].x)) + 360, 360);
    ImGui::Columns(4, nullptr, false);
    const std::array<std::pair<std::string, std::string>, 4> values = {
        {{"VEHICLE", robot},
         {"DEPTH", fixed(-p.z, 2) + " m"},
         {"HEADING", fixed(headingDegrees, 1) + " deg"},
         {"VIEW RATE", fixed(1000 / std::max(frameMs, 1.), 0) + " fps"}}};
    for (const auto &v : values) {
      heading(v.first.c_str());
      ImGui::PushFont(numberFont);
      ImGui::TextUnformatted(v.second.c_str());
      ImGui::PopFont();
      ImGui::NextColumn();
    }
    ImGui::Columns(1);
    ImGui::EndChild();
    ImGui::BeginChild("environment", {left, 239}, ImGuiChildFlags_Borders);
    if (ImGui::BeginTabBar("Environment tabs")) {
      if (ImGui::BeginTabItem("Lighting")) {
        heading("UNDERWATER OPTICS");
        ImGui::SetNextItemWidth(left * .17f);
        ImGui::SliderFloat("Caustics", &look.caustics, 0, 1, "%.2f");
        ImGui::SameLine();
        ImGui::Checkbox("Surface", &look.surface);
        ImGui::SameLine();
        ImGui::Checkbox("Shadows", &look.shadows);
        ImGui::Separator();
        int profile = look.outdoor ? 1 : 0;
        ImGui::SetNextItemWidth(120);
        if (ImGui::Combo("Lighting", &profile, "Indoor\0Outdoor\0")) {
          look.outdoor = profile == 1;
          look.directLight = look.outdoor ? 1.4f : 1.f;
          look.ambientLight = look.outdoor ? .6f : .9f;
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(130);
        ImGui::SliderFloat("Brightness", &look.directLight, 0, 4, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(125);
        ImGui::SliderFloat("Ambient", &look.ambientLight, 0, 2, "%.2f");
        if (look.outdoor) {
          ImGui::SetNextItemWidth(160);
          ImGui::SliderFloat("Sun azimuth", &look.sunAzimuth, 0, 360,
                             "%.0f deg");
          ImGui::SameLine();
          ImGui::SetNextItemWidth(140);
          ImGui::SliderFloat("Elevation", &look.sunElevation, 5, 89,
                             "%.0f deg");
          ImGui::SameLine();
          ImGui::SetNextItemWidth(120);
          ImGui::SliderFloat("Glare", &look.glare, 0, 2, "%.2f");
        } else
          ImGui::TextDisabled("Diffuse indoor lighting. Switch to Outdoor to "
                              "adjust sun and glare.");
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Water appearance")) {
        drawWaterControls();
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Depth sensor")) {
        drawDepthControls(left);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Run score")) {
        drawRunControls();
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Tasks & claw")) {
        if (ImGui::SmallButton("Inspect launcher")) focus("Payloads");
        ImGui::SameLine();
        if (ImGui::SmallButton("Inspect claw")) focus("Claw");
        ImGui::SameLine();
        if (ImGui::SmallButton("Inspect lights")) focus("magnet_target1");
        const bool connected =
            !demo &&
            std::chrono::duration<double>(Clock::now() - lastActuator).count() <
                1.;
        ImGui::TextUnformatted(
            connected
                ? "Enable the vehicle in RViz, then arm the actuators."
                : "Start the live simulator to operate the actuators. Scene preview "
                  "sends no commands.");
        ImGui::BeginDisabled(!connected);
        if (ImGui::Button(actuatorStatus.actuators_armed ? "Disarm"
                                                         : "Arm actuators")) {
          std_msgs::msg::Bool command;
          command.data = !actuatorStatus.actuators_armed;
          taskArm->publish(command);
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!actuatorStatus.actuators_armed);
        if (ImGui::Button("Fire torpedo"))
          taskTorpedo->publish(std_msgs::msg::Empty{});
        ImGui::SameLine();
        if (ImGui::Button("Drop marker"))
          taskDropper->publish(std_msgs::msg::Empty{});
        ImGui::SameLine();
        if (ImGui::Button("Open claw")) {
          std_msgs::msg::Bool command; command.data=true; taskClaw->publish(command);
        }
        ImGui::SameLine();
        if (ImGui::Button("Close claw")) {
          std_msgs::msg::Bool command; command.data=false; taskClaw->publish(command);
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop claw")) {
          std_msgs::msg::Float32 command; command.data=0; taskClawTimed->publish(command);
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Reload / disarm"))
          taskReload->publish(std_msgs::msg::Empty{});
        if (ImGui::Button("Reset all tasks"))
          taskReset->publish(std_msgs::msg::Empty{});
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("Clear released payloads and scores, reset lights and table props, reload and disarm.");
        ImGui::EndDisabled();
        ImGui::Text("Remaining: %u torpedoes / %u markers",
                    demo ? 2u : unsigned(actuatorStatus.torpedo_available_count),
                    demo ? 2u : unsigned(actuatorStatus.dropper_available_count));
        ImGui::SameLine();
        ImGui::Text(" / Jaw gap: %.0f mm",1000*(clawMinGap+clawJoints[0]+clawJoints[1]));
        ImGui::TextWrapped("%s", taskScore.c_str());
        for (const auto &light : magnetStates)
          ImGui::TextColored(light.second ? ImVec4(.2f,1.f,.3f,1.f) : ImVec4(1.f,.3f,.25f,1.f),
                             "%s: %s",light.first.c_str(),light.second ? "GREEN" : "RED");
        for (const auto &event : taskEvents)
          ImGui::TextDisabled("%s", event.c_str());
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }
    ImGui::EndChild();
    ImGui::PopStyleVar();
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("right", {side, contentHeight}, ImGuiChildFlags_None);
    const float previewHeight = std::max(90.f, (contentHeight - 605.f) / 2.f);
    cameraCard(cameras[0], side, previewHeight);
    cameraCard(cameras[1], side, previewHeight);
    minimap(side);
    ImGui::PushFont(smallFont);
    ImGui::TextColored(muted, "RENDER PROFILE");
    ImGui::PopFont();
    ImGui::SetNextItemWidth(side - 85);
    ImGui::SliderFloat("Exposure", &look.exposure, .4f, 2.0f, "%.2f");
    ImGui::Checkbox("Calibration board", &look.tag);
    ImGui::PushFont(smallFont);
    ImGui::TextWrapped("%s", demo ? "Scene preview. Start your normal robot "
                                    "simulation to stream live cameras."
                                  : "Images follow physics TF. Observer "
                                    "controls do not move the vehicle.");
    ImGui::PopFont();
    ImGui::EndChild();
    ImGui::Separator();
    ImGui::PushFont(smallFont);
    ImGui::TextColored(
        muted, "OPENGL 3.3   /   %zu triangles   /   %s", renderer->triangles,
        reinterpret_cast<const char *>(glGetString(GL_RENDERER)));
    ImGui::SameLine();
    if (!lastCapture.empty())
      ImGui::TextColored(cyan, "   Saved capture");
    ImGui::PopFont();
    ImGui::End();
    drawScorecard();
  }
  void saveScreenshot() {
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    cv::Mat rgb(h, w, CV_8UC3);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, rgb.data);
    cv::flip(rgb, rgb, 0);
    cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
    std::filesystem::path out =
        screenshot.empty() ? std::filesystem::path("/tmp") /
                                 ("riptide-pool-" +
                                  std::to_string(now().nanoseconds()) + ".png")
                           : std::filesystem::path(screenshot);
    if (!out.parent_path().empty())
      std::filesystem::create_directories(out.parent_path());
    if (!cv::imwrite(out.string(), rgb))
      throw std::runtime_error("Cannot save screenshot: " + out.string());
    lastCapture = out.string();
    RCLCPP_INFO(get_logger(), "Saved %s", lastCapture.c_str());
    for (auto &c : cameras) {
      if (!c.image.final.fbo) continue;
      cv::Mat image(c.image.final.height, c.image.final.width, CV_8UC3);
      glBindFramebuffer(GL_FRAMEBUFFER, c.image.final.fbo);
      glReadPixels(0, 0, image.cols, image.rows, GL_RGB, GL_UNSIGNED_BYTE,
                   image.data);
      cv::flip(image, image, 0);
      cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
      cv::imwrite(
          (out.parent_path() / (out.stem().string() + "-" + c.name + ".png"))
              .string(),
          image);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<PoolViewer>();
    node->run();
  } catch (const std::exception &e) {
    std::cerr << "Pool viewer: " << e.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
