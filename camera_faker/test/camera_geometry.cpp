#include "pool_viewer/camera.hpp"
#include "pool_viewer/payload_mounts.hpp"
#include <iostream>

void check(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}
void close(double a, double b, double tolerance, const char *message) {
  check(std::abs(a - b) < tolerance, message);
}
int main(int argc, char **argv) {
  try {
    check(argc == 2, "Expected camera configuration directory");
    const auto vehicle = YAML::Load(R"(
base_link: [0.1, -0.2, 0.3]
torpedoes: {pose: [1, 2, 3, 0, 0, 1.5707963267948966], baseline: 0.04}
droppers: {pose: [1, 2, 3, 1.5707963267948966, 0, 0]}
)");
    const auto task = YAML::Load(R"(
torpedo:
  count: 2
  slot_offsets: [[0.03, 0, -0.02, 0, 0, 0], [0.03, 0, -0.02, 0, 0, 0]]
dropper:
  count: 2
  slot_offsets:
    - [0, -0.02, 0, 0, 1.5707963267948966, 0]
    - [0, 0.02, 0, 0, 1.5707963267948966, 0]
)");
    const auto torpedoes = pool::payloadMounts(vehicle, task, "torpedo");
    // A CAD point at the configured base-link coordinates must land exactly
    // on the world base link, regardless of yaw (or roll/pitch). This catches
    // a stale scene offset, reversed sign, or applying the lever arm twice.
    for (const auto &angles : {glm::vec3(0), glm::vec3(0, 0, glm::pi<float>()),
                               glm::vec3(.4f, -.3f, 1.2f)}) {
      const auto worldBase = pool::pose({2, -3, -.7f}, angles);
      const auto worldOrigin = worldBase * pool::baseToOrigin(vehicle);
      const auto basePoint = worldOrigin * glm::vec4(.1f, -.2f, .3f, 1);
      close(glm::distance(glm::vec3(basePoint), glm::vec3(worldBase[3])), 0,
            1e-6, "CAD origin and base link coincide at configured anchor through yaw");
      const auto fromCad = worldOrigin * pool::yamlPose(vehicle["torpedoes"]["pose"]) *
          pool::pose({0, -.02f, 0}) * pool::yamlPose(task["torpedo"]["slot_offsets"][0]);
      const auto fromBase = worldBase * torpedoes[0];
      close(glm::distance(glm::vec3(fromCad[3]), glm::vec3(fromBase[3])), 0,
            1e-6, "Payload and CAD mesh share the same origin through rotation");
    }
    for (int i = 0; i < 2; ++i) {
      const auto &m = torpedoes[i];
      close(m[3].x, i == 0 ? .92 : .88, 1e-6, "Torpedo baseline rotates in actuator frame");
      close(m[3].y, 2.23, 1e-6, "Subtract base_link once");
      close(m[3].z, 2.68, 1e-6, "Robot config sets torpedo height");
      close(glm::distance(glm::vec3(m[0]), glm::vec3(0, 1, 0)), 0, 1e-6,
            "Torpedo release direction follows actuator rotation");
    }
    const auto droppers = pool::payloadMounts(vehicle, task, "dropper");
    for (int i = 0; i < 2; ++i) {
      const auto &m = droppers[i];
      close(m[3].z, i == 0 ? 2.68 : 2.72, 1e-6, "Dropper slot offsets rotate with mount");
      close(glm::distance(glm::vec3(m[0]), glm::vec3(0, 1, 0)), 0, 1e-6,
            "Dropper ejects along rotated negative Z");
    }
    const auto k =
        pool::loadCamera(std::string(argv[1]) + "/ffc_config.yaml", "", .5);
    check(k.width == 960 && k.height == 600, "HD1200 half resolution");
    const auto native = pool::loadCamera(std::string(argv[1]) + "/ffc_config.yaml", "", 1.0);
    check(native.width == 1920 && native.height == 1200, "HD1200 native resolution");
    close(native.fx, 2*k.fx, 1e-6, "Native horizontal focal length");
    close(native.fy, 2*k.fy, 1e-6, "Native vertical focal length");
    close(native.cx, 2*k.cx, 1e-6, "Native principal point x");
    close(native.cy, 2*k.cy, 1e-6, "Native principal point y");
    pool::Intrinsics off = k;
    off.cx = 301.25;
    off.cy = 275.75;
    // Independently project an optical-frame point through the graphics matrix.
    glm::vec3 optical(.31, -.14, 2.7);
    auto clip =
        off.projection() * glm::vec4(optical.x, -optical.y, -optical.z, 1);
    auto ndc = glm::vec3(clip) / clip.w;
    close((ndc.x + 1) * off.width / 2 - .5,
          off.fx * optical.x / optical.z + off.cx, .001,
          "Off-axis horizontal registration");
    close((1 - ndc.y) * off.height / 2 - .5,
          off.fy * optical.y / optical.z + off.cy, .001,
          "Off-axis vertical registration");
    close(pool::linearDepth(ndc.z * .5 + .5), optical.z, .0001,
          "Depth must be axial metres, not ray length");
    for (float z : {.1f, 1.f, 4.f, 20.f}) {
      auto c = k.projection() * glm::vec4(0, 0, -z, 1);
      close(pool::linearDepth((c.z / c.w + 1) / 2), z, .003,
            "Depth buffer inversion");
    }
    const auto down = pool::pose({0, 0, 0}, {1.5507f, 1.5107f, 0});
    const auto view = pool::cameraView(down, k);
    auto forward = glm::vec3(down[0]);
    check(forward.z < -.99, "DFC must point down");
    auto centre = view.view * glm::vec4(forward * 2.f, 1);
    close(centre.x, 0, .00001, "DFC horizontal basis");
    close(centre.y, 0, .00001, "DFC vertical basis");
    close(centre.z, -2, .00001, "DFC forward convention");
    bool rejected = false;
    try {
      pool::loadCamera("", "", 0);
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected, "Reject zero camera scale");
    rejected = false;
    try {
      pool::loadCamera("", "/nonexistent/calibration.yaml", 1);
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected, "Never silently ignore explicit calibration");
    // A ROS rectified calibration must preserve asymmetric focal
    // lengths/principal point.
    const auto temp = std::filesystem::temp_directory_path() /
                      "riptide-camera-geometry-test.yaml";
    {
      cv::FileStorage f(temp.string(), cv::FileStorage::WRITE);
      f << "image_width" << 1920 << "image_height" << 1200;
      f << "projection_matrix" << "{" << "data"
        << std::vector<double>{1100, 0, 920, 0, 0, 1120, 570, 0, 0, 0, 1, 0}
        << "}";
    }
    const auto calibrated = pool::loadCamera("", temp.string(), .5);
    std::filesystem::remove(temp);
    close(calibrated.fx, 550, .00001, "Scaled fx");
    close(calibrated.fy, 560, .00001, "Scaled fy");
    close(calibrated.cx, 460, .00001, "Scaled cx");
    close(calibrated.cy, 285, .00001, "Scaled cy");
    std::cout << "Camera projection, depth, DFC orientation, and calibration "
                 "checks passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
