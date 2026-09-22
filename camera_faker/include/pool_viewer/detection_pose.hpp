#pragma once

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>

namespace pool {
inline glm::mat4 opticalToLink() {
    glm::mat4 m(0);
    m[0] = {0, -1, 0, 0};
    m[1] = {0, 0, -1, 0};
    m[2] = {1, 0, 0, 0};
    m[3] = {0, 0, 0, 1};
    return m;
}

// Detections describe observations in the world, even if the incoming marker
// requests frame locking. Always use acquisition time, never the current robot
// pose. A zero stamp selects the latest transform only for initial placement.
// For simulated camera observations the caller supplies the saved optical
// render pose, or the simulator TF frame if that acquisition has aged out.
inline bool resolveDetectionPose(const visualization_msgs::msg::Marker &marker, const std::string &fixedFrame,
                                 tf2_ros::Buffer &tf, glm::mat4 &worldPose, const std::string &sourceFrame = "",
                                 const glm::mat4 *acquisitionPose = nullptr) {
    glm::mat4 frame(1);
    const rclcpp::Time stamp(marker.header.stamp);
    if (acquisitionPose && stamp.nanoseconds() != 0) {
        frame = *acquisitionPose;
    } else {
        try {
            const auto t =
                tf.lookupTransform(fixedFrame, sourceFrame.empty() ? marker.header.frame_id : sourceFrame, stamp)
                    .transform;
            const auto &q = t.rotation;
            frame = glm::translate(glm::mat4(1), glm::vec3(t.translation.x, t.translation.y, t.translation.z)) *
                    glm::mat4_cast(glm::normalize(glm::quat(q.w, q.x, q.y, q.z)));
        } catch (const tf2::TransformException &) {
            return false;
        }
    }
    const auto &p = marker.pose.position;
    const auto &q = marker.pose.orientation;
    worldPose = frame * glm::translate(glm::mat4(1), glm::vec3(p.x, p.y, p.z)) *
                glm::mat4_cast(glm::normalize(glm::quat(q.w, q.x, q.y, q.z)));
    return true;
}

// Owned by one observation, not by the camera. Once resolved, its world pose
// is immutable until that observation is replaced or deleted. In particular,
// frame_locked and zero-stamped detections must not follow subsequent TF.
class DetectionPose {
  public:
    bool place(const visualization_msgs::msg::Marker &marker, const std::string &fixedFrame, tf2_ros::Buffer &tf,
               const std::string &sourceFrame = "", const glm::mat4 *acquisitionPose = nullptr) {
        if (!placed_)
            placed_ = resolveDetectionPose(marker, fixedFrame, tf, world_, sourceFrame, acquisitionPose);
        return placed_;
    }

    const glm::mat4 &world() const {
        return world_;
    }

  private:
    bool placed_ = false;
    glm::mat4 world_{1};
};
} // namespace pool
