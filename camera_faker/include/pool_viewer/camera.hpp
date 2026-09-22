#pragma once

#include <cmath>
#include <filesystem>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace pool {
inline glm::mat4 pose(const glm::vec3 &p, const glm::vec3 &rpy = {}) {
  return glm::translate(glm::mat4(1), p) *
         glm::rotate(glm::mat4(1), rpy.z, glm::vec3(0, 0, 1)) *
         glm::rotate(glm::mat4(1), rpy.y, glm::vec3(0, 1, 0)) *
         glm::rotate(glm::mat4(1), rpy.x, glm::vec3(1, 0, 0));
}
inline glm::vec3 vector3(const YAML::Node &n) {
  if (!n || n.size() < 3)
    throw std::runtime_error("Expected an xyz vector");
  return {n[0].as<float>(), n[1].as<float>(), n[2].as<float>()};
}
inline glm::mat4 yamlPose(const YAML::Node &p) {
  if (!p || p.size() != 6)
    throw std::runtime_error("Expected xyz/rpy pose (radians)");
  return pose(vector3(p),
              {p[3].as<float>(), p[4].as<float>(), p[5].as<float>()});
}
// Matches robot_base.xacro's base_link -> origin joint. CAD coordinates,
// including actuator poses, must pass through this transform exactly once.
inline glm::mat4 baseToOrigin(const YAML::Node &vehicle) {
  return pose(-vector3(vehicle["base_link"]));
}
struct Intrinsics {
  int width = 1920, height = 1200;
  double fx = 0, fy = 0, cx = 0, cy = 0, rate = 15, maxDepth = 8;
  double nearPlane = 0.05, farPlane = 100;
  std::string calibration = "Approximate 2.2 mm lens";
  // Camera frame is ROS FLU. OpenGL camera looks down -Z with +Y up.
  glm::mat4 projection() const {
    glm::mat4 p(0);
    p[0][0] = 2 * fx / width;
    p[1][1] = 2 * fy / height;
    // ROS pixel coordinates index pixel centres; GL viewport coordinates index
    // edges.
    p[2][0] = 1 - 2 * (cx + .5) / width;
    p[2][1] = 2 * (cy + .5) / height - 1;
    p[2][2] = -(farPlane + nearPlane) / (farPlane - nearPlane);
    p[2][3] = -1;
    p[3][2] = -2 * farPlane * nearPlane / (farPlane - nearPlane);
    return p;
  }
  void resize(int w, int h) {
    fx *= double(w) / width;
    cx *= double(w) / width;
    fy *= double(h) / height;
    cy *= double(h) / height;
    width = w;
    height = h;
  }
  void validate() const {
    if (width < 16 || height < 16 || width > 4096 || height > 4096 ||
        !std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0 ||
        !std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(rate) ||
        rate <= 0 || rate > 120 || !std::isfinite(maxDepth) ||
        maxDepth <= nearPlane)
      throw std::runtime_error(
          "Invalid camera resolution, intrinsics, rate or depth range");
  }
};
inline Intrinsics loadCamera(const std::string &config,
                             const std::string &overrideCalibration,
                             double scale) {
  Intrinsics k;
  std::string calibrationFile = overrideCalibration;
  if (!config.empty()) {
    const auto params = YAML::LoadFile(config)["/**"]["ros__parameters"];
    const auto g = params["general"];
    const auto resolution = g["grab_resolution"].as<std::string>("HD1200");
    if (resolution == "HD1200") {
      k.width = 1920;
      k.height = 1200;
    } else if (resolution == "HD1080") {
      k.width = 1920;
      k.height = 1080;
    } else if (resolution == "HD720") {
      k.width = 1280;
      k.height = 720;
    } else if (resolution == "SVGA") {
      k.width = 960;
      k.height = 600;
    } else
      throw std::runtime_error("Unsupported camera grab_resolution: " +
                               resolution);
    k.rate =
        g["pub_frame_rate"].as<double>(g["grab_frame_rate"].as<double>(15));
    if (params["depth"])
      k.maxDepth = params["depth"]["max_depth"].as<double>(8);
    if (calibrationFile.empty())
      calibrationFile =
          g["optional_opencv_calibration_file"].as<std::string>("");
    if (g["pub_resolution"].as<std::string>("NATIVE") == "CUSTOM") {
      double downscale = g["pub_downscale_factor"].as<double>(1);
      if (!std::isfinite(downscale) || downscale < 1)
        throw std::runtime_error("Invalid ZED downscale factor");
      scale /= downscale;
    }
  }
  // Approximate rectified 2.2 mm ZED X Mini FOV. A wet calibration overrides
  // this.
  k.fx = k.width / (2 * std::tan(glm::radians(105.0) / 2));
  k.fy = k.height / (2 * std::tan(glm::radians(78.0) / 2));
  k.cx = k.width / 2.0;
  k.cy = k.height / 2.0;
  if (!calibrationFile.empty() && std::filesystem::exists(calibrationFile)) {
    cv::FileStorage f(calibrationFile, cv::FileStorage::READ);
    if (!f.isOpened())
      throw std::runtime_error("Cannot open calibration: " + calibrationFile);
    cv::Mat K, P;
    int w = 0, h = 0;
    if (!f["K_LEFT"].empty()) {
      f["K_LEFT"] >> K;
      std::vector<int> size;
      f["Size"] >> size;
      if (size.size() != 2)
        throw std::runtime_error("Calibration requires Size: [width,height]");
      w = size[0];
      h = size[1];
      cv::Mat kr, dl, dr, r, t, r1, r2, p2, q;
      f["K_RIGHT"] >> kr;
      f["D_LEFT"] >> dl;
      f["D_RIGHT"] >> dr;
      f["R"] >> r;
      f["T"] >> t;
      if (kr.empty() || dl.empty() || dr.empty() || r.empty() || t.empty())
        throw std::runtime_error(
            "Stereo calibration requires K/D_LEFT/RIGHT, R, T");
      cv::stereoRectify(K, dl, kr, dr, {w, h}, r, t, r1, r2, P, p2, q,
                        cv::CALIB_ZERO_DISPARITY, 0);
    } else {
      f["image_width"] >> w;
      f["image_height"] >> h;
      const auto p = f["projection_matrix"]["data"];
      std::vector<double> data;
      p >> data;
      if (data.size() != 12)
        throw std::runtime_error(
            "Calibration requires rectified projection_matrix.data[12]");
      P = cv::Mat(3, 4, CV_64F, data.data()).clone();
    }
    if (w <= 0 || h <= 0 || P.empty())
      throw std::runtime_error("Invalid calibration dimensions");
    P.convertTo(P, CV_64F);
    k.fx = P.at<double>(0, 0) * k.width / w;
    k.fy = P.at<double>(1, 1) * k.height / h;
    k.cx = P.at<double>(0, 2) * k.width / w;
    k.cy = P.at<double>(1, 2) * k.height / h;
    k.calibration = calibrationFile;
  } else if (!overrideCalibration.empty()) {
    throw std::runtime_error("Explicit calibration file does not exist: " +
                             overrideCalibration);
  }
  if (!std::isfinite(scale) || scale <= 0 || scale > 1)
    throw std::runtime_error("camera_scale must be in (0,1]");
  k.resize(std::lround(k.width * scale), std::lround(k.height * scale));
  k.validate();
  return k;
}
struct View {
  glm::vec3 eye{};
  glm::mat4 view{1}, projection{1};
};
inline View cameraView(const glm::mat4 &mount, const Intrinsics &k) {
  glm::vec3 p(mount[3]), forward(mount[0]), up(mount[2]);
  return {p, glm::lookAt(p, p + forward, up), k.projection()};
}
inline float linearDepth(float z, float nearPlane = 0.05f,
                         float farPlane = 100.f) {
  return 2 * nearPlane * farPlane /
         (farPlane + nearPlane - (2 * z - 1) * (farPlane - nearPlane));
}
} // namespace pool
