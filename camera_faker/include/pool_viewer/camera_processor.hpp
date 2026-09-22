#pragma once
#include "pool_viewer/depth_noise.hpp"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>

namespace pool {
// Matches PointCloud2's padded xyz/rgb layout, including zeroed padding.
struct CloudPoint {
    float x, y, z, padding;
    uint8_t b, g, r, alpha;
    uint32_t reserved[3];
};
static_assert(sizeof(CloudPoint) == 32 && offsetof(CloudPoint, b) == 16);
static_assert(std::is_trivially_copyable<CloudPoint>::value);

struct CameraRequest {
    float nearPlane = .05f, farPlane = 100.f;
    DepthNoise noise;
    double fx = 1, fy = 1, cx = 0, cy = 0;
    int cloudStride = 0; // Zero skips cloud generation.
    bool jpeg = false, preview = false;
};
struct CameraProducts {
    cv::Mat depth, preview;
    std::vector<CloudPoint> cloud;
    std::vector<uint8_t> jpeg;
    int cloudWidth = 0, cloudHeight = 0;
    bool jpegOnGpu = false;
    std::string warning;
};
void encodeCameraJpeg(const cv::Mat &rgb, std::vector<uint8_t> &jpeg);

// Each camera owns a backend used only by its output worker. Inputs are
// top-down RGB and OpenGL depth; leave them untouched so CPU retry is safe.
class CameraBackend {
  public:
    virtual ~CameraBackend() = default;
    virtual std::string name() const = 0;
    virtual CameraProducts process(const cv::Mat &rgb, const cv::Mat &buffer, const CameraRequest &request,
                                   std::mt19937 &random) = 0;
};
std::unique_ptr<CameraBackend> makeCudaCameraBackend();

class CameraProcessor {
  public:
    explicit CameraProcessor(const std::string &preference = "auto");
    explicit CameraProcessor(std::unique_ptr<CameraBackend> backend);
    const std::string &description() const {
        return description_;
    }
    bool usesCuda() const {
        return bool(backend_);
    }
    // One transaction: depth, preview, cloud, and JPEG belong to the same frame.
    // A failed GPU frame is retried on CPU without consuming its random seed.
    CameraProducts processFrame(const cv::Mat &rgb, const cv::Mat &buffer, const CameraRequest &request,
                                std::mt19937 &random);

  private:
    std::unique_ptr<CameraBackend> backend_;
    std::string description_;
};
} // namespace pool
