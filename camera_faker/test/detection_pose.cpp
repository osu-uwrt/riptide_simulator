#include "pool_viewer/detection_pose.hpp"
#include <cassert>
#include <iostream>

int main() {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer tf(clock);
  auto set = [&](int sec, double x) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "camera";
    t.header.stamp.sec = sec;
    t.transform.translation.x = x;
    t.transform.rotation.w = 1;
    assert(tf.setTransform(t, "test"));
  };
  set(10, 1);
  set(12, 3);
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "camera";
  m.header.stamp.sec = 11;
  m.pose.position.x = 5;
  m.pose.orientation.w = 1;
  glm::mat4 result(1);
  const auto close = [](const glm::vec3 &actual, const glm::vec3 &expected) {
    assert(glm::length(actual - expected) < 1e-5f);
  };

  // A delayed observation interpolates at acquisition time, not arrival time.
  assert(pool::resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {7, 0, 0});
  set(13, 10);
  assert(pool::resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {7, 0, 0});

  // No latest-pose fallback for future stamps: wait until TF catches up.
  m.header.stamp.sec = 14;
  assert(!pool::resolveDetectionPose(m, "map", tf, result));
  set(14, 12);
  assert(pool::resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {17, 0, 0});

  m.header.stamp.sec = 0;
  assert(pool::resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {17, 0, 0});
  m.header.stamp.sec = 11;
  m.frame_locked = true;
  assert(pool::resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {7, 0, 0});
  set(15, 14);
  assert(pool::resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {7, 0, 0});

  m.frame_locked = false;
  m.header.frame_id = "map";
  assert(pool::resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {5, 0, 0});
  m.header.frame_id = "unknown";
  assert(!pool::resolveDetectionPose(m, "map", tf, result));

  m.header.frame_id = "camera";
  m.pose.position.x = 1;
  m.pose.position.y = 2;
  m.pose.position.z = 3;

  // Exercise the same stateful placement used by the viewer over many frames.
  // Subsequent TF updates must not move an existing observation, including
  // zero-stamped/frame-locked markers.
  int nextStamp = 16;
  for (bool locked : {false, true}) {
    for (int stamp : {0, 11}) {
      m.frame_locked = locked;
      m.header.stamp.sec = stamp;
      m.lifetime.sec = 5;
      pool::DetectionPose observation;
      assert(observation.place(m, "map", tf));
      const auto initial = observation.world();
      set(nextStamp++, 30);
      for (int frame = 0; frame < 20; ++frame) {
        assert(observation.place(m, "map", tf));
        for (int column = 0; column < 4; ++column)
          assert(glm::length(observation.world()[column] - initial[column]) < 1e-5f);
      }
      assert(m.lifetime.sec == 5 && m.frame_locked == locked);
    }
  }
  // Unresolved observations can retry; replacement creates a new placement.
  m.header.stamp.sec = nextStamp;
  pool::DetectionPose pending;
  assert(!pending.place(m, "map", tf));
  set(nextStamp, 40);
  assert(pending.place(m, "map", tf));
  close(glm::vec3(pending.world()[3]), {41, 2, 3});

  // Never substitute estimated TF when the simulator frame is unavailable.
  pool::DetectionPose missingTruth;
  assert(!missingTruth.place(m, "map", tf, "simulator/camera"));

  // Repeated observations of a stationary simulator target must agree while the
  // robot translates and rotates, even when simulator and estimated poses
  // diverge. This covers replacement markers, not just a cached observation.
  const auto target = glm::translate(glm::mat4(1), glm::vec3(8, -3, -2)) *
      glm::rotate(glm::mat4(1), .7f, glm::vec3(0, 1, 0));
  const auto setPose = [&](const std::string &child, int sec, const glm::mat4 &pose) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = child;
    t.header.stamp.sec = sec;
    t.transform.translation.x = pose[3].x;
    t.transform.translation.y = pose[3].y;
    t.transform.translation.z = pose[3].z;
    const auto q = glm::quat_cast(pose);
    t.transform.rotation.w = q.w;
    t.transform.rotation.x = q.x;
    t.transform.rotation.y = q.y;
    t.transform.rotation.z = q.z;
    assert(tf.setTransform(t, "test"));
  };
  m.frame_locked = false;
  for (int step = 0; step < 20; ++step) {
    const int stamp = 40 + 2 * step;
    const auto camera = glm::translate(glm::mat4(1), glm::vec3(step * .2f, 1, -1)) *
        glm::rotate(glm::mat4(1), step * .1f, glm::vec3(0, 0, 1)) *
        pool::opticalToLink();
    const auto drift = glm::translate(glm::mat4(1), glm::vec3(1 + step * .3f, -2, 0)) *
        glm::rotate(glm::mat4(1), step * .05f, glm::vec3(0, 0, 1));
    setPose("camera", stamp, drift * camera);
    setPose("simulator/camera", stamp, camera);
    // A newer robot pose must not affect a delayed detection.
    setPose("simulator/camera", stamp + 1,
            glm::translate(glm::mat4(1), glm::vec3(20, 30, 40)));
    const auto local = glm::inverse(camera) * target;
    const auto q = glm::quat_cast(local);
    m.header.frame_id = "camera";
    m.header.stamp.sec = stamp;
    m.pose.position.x = local[3].x;
    m.pose.position.y = local[3].y;
    m.pose.position.z = local[3].z;
    m.pose.orientation.w = q.w;
    m.pose.orientation.x = q.x;
    m.pose.orientation.y = q.y;
    m.pose.orientation.z = q.z;
    pool::DetectionPose observation;
    assert(observation.place(m, "map", tf, "simulator/camera"));
    for (int column = 0; column < 4; ++column)
      assert(glm::length(observation.world()[column] - target[column]) < 1e-5f);

    // RViz's estimated placement differs from the actual simulated target.
    assert(pool::resolveDetectionPose(m, "map", tf, result));
    assert(glm::length(result[3] - target[3]) > .1f);

    // Exact rendered poses take priority over TF's sampled/interpolated pose.
    const auto offset = glm::translate(glm::mat4(1), glm::vec3(.08f, -.04f, .02f));
    auto rendered = offset * camera;
    pool::DetectionPose captured;
    assert(captured.place(m, "map", tf, "simulator/camera", &rendered));
    const auto renderedTarget = offset * target;
    for (int column = 0; column < 4; ++column)
      assert(glm::length(captured.world()[column] - renderedTarget[column]) < 1e-5f);
    rendered = glm::mat4(1);
    assert(captured.place(m, "map", tf, "simulator/camera", &rendered));
    for (int column = 0; column < 4; ++column)
      assert(glm::length(captured.world()[column] - renderedTarget[column]) < 1e-5f);

    // A marker explicitly in the simulator branch still uses that branch.
    m.header.frame_id = "simulator/camera";
    assert(pool::resolveDetectionPose(m, "map", tf, result));
    for (int column = 0; column < 4; ++column)
      assert(glm::length(result[column] - target[column]) < 1e-5f);
  }
  // Zero stamps have no matching acquisition: use simulator TF once, then
  // keep the observation fixed even when newer simulator poses arrive.
  m.header.frame_id = "camera";
  m.header.stamp.sec = 0;
  const glm::mat4 unrelatedRender(1);
  pool::DetectionPose unstamped;
  assert(unstamped.place(m, "map", tf, "simulator/camera", &unrelatedRender));
  assert(pool::resolveDetectionPose(m, "map", tf, result, "simulator/camera"));
  for (int column = 0; column < 4; ++column)
    assert(glm::length(unstamped.world()[column] - result[column]) < 1e-5f);
  setPose("simulator/camera", 81, glm::mat4(1));
  assert(unstamped.place(m, "map", tf, "simulator/camera"));
  for (int column = 0; column < 4; ++column)
    assert(glm::length(unstamped.world()[column] - result[column]) < 1e-5f);
  std::cout << "Detection timestamps, immutable placement, simulator TF fallback, and render registration passed\n";
}
