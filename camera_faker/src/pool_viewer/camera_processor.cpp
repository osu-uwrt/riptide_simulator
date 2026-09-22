#include "pool_viewer/camera_processor.hpp"
#include "pool_viewer/camera.hpp"
#include <opencv2/imgcodecs.hpp>

namespace pool {
#ifndef POOL_HAS_CUDA
std::unique_ptr<CameraBackend> makeCudaCameraBackend() {
    throw std::runtime_error("built without a CUDA toolkit");
}
#endif

void encodeCameraJpeg(const cv::Mat &rgb, std::vector<uint8_t> &jpeg) {
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    if (!cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, 93}))
        throw std::runtime_error("JPEG encoding failed");
}

CameraProcessor::CameraProcessor(const std::string &preference) {
    if (preference != "auto" && preference != "cpu")
        throw std::invalid_argument("camera_compute must be auto or cpu");
    if (preference == "cpu") {
        description_ = "CPU (requested)";
        return;
    }
    try {
        backend_ = makeCudaCameraBackend();
        description_ = backend_->name();
    } catch (const std::exception &error) {
        backend_.reset();
        description_ = std::string("CPU (") + error.what() + ")";
    }
}
CameraProcessor::CameraProcessor(std::unique_ptr<CameraBackend> backend) : backend_(std::move(backend)) {
    description_ = backend_ ? backend_->name() : "CPU";
}

CameraProducts CameraProcessor::processFrame(const cv::Mat &rgb, const cv::Mat &buffer, const CameraRequest &request,
                                             std::mt19937 &random) {
    const auto &noise = request.noise;
    noise.validate();
    if ((!buffer.empty() && (buffer.type() != CV_32FC1 || buffer.dims != 2)) ||
        (!rgb.empty() && (rgb.type() != CV_8UC3 || rgb.dims != 2)) || !std::isfinite(request.nearPlane) ||
        !std::isfinite(request.farPlane) || request.nearPlane <= 0 || request.farPlane <= request.nearPlane ||
        request.cloudStride < 0)
        throw std::invalid_argument("Invalid camera input or clipping planes");
    if ((request.jpeg && rgb.empty()) || (request.preview && buffer.empty()))
        throw std::invalid_argument("Requested camera output is missing its image");
    if (request.cloudStride &&
        (buffer.empty() || rgb.empty() || buffer.size() != rgb.size() || !std::isfinite(request.fx) ||
         !std::isfinite(request.fy) || !std::isfinite(request.cx) || !std::isfinite(request.cy) || request.fx <= 0 ||
         request.fy <= 0))
        throw std::invalid_argument("Cloud requires matching RGB/depth and valid intrinsics");

    CameraProducts result;
    if (backend_) {
        auto candidate = random;
        try {
            auto gpu = backend_->process(rgb, buffer, request, candidate);
            random = candidate;
            return gpu;
        } catch (const std::exception &error) {
            result.warning = std::string("CUDA camera processing failed; using CPU: ") + error.what();
            backend_.reset();
            description_ = "CPU (CUDA failed)";
        }
    }
    if (!buffer.empty()) {
        result.depth = buffer.clone();
        for (int y = 0; y < buffer.rows; ++y)
            for (int x = 0; x < buffer.cols; ++x) {
                float &z = result.depth.at<float>(y, x);
                z = z >= .999999f ? NAN : linearDepth(z, request.nearPlane, request.farPlane);
            }
        noise.apply(result.depth, random);
    }
    if (request.preview) {
        cv::Mat gray;
        result.depth.convertTo(gray, CV_8U, 255 / noise.maxRange);
        cv::applyColorMap(gray, result.preview, cv::COLORMAP_TURBO);
        for (int y = 0; y < buffer.rows; ++y)
            for (int x = 0; x < buffer.cols; ++x)
                if (!std::isfinite(result.depth.at<float>(y, x)))
                    result.preview.at<cv::Vec3b>(y, x) = {16, 22, 27};
        cv::cvtColor(result.preview, result.preview, cv::COLOR_BGR2RGB);
        cv::flip(result.preview, result.preview, 0);
    }
    if (request.cloudStride) {
        const int stride = request.cloudStride;
        result.cloudWidth = (buffer.cols - 1) / stride + 1;
        result.cloudHeight = (buffer.rows - 1) / stride + 1;
        result.cloud.resize(size_t(result.cloudWidth) * result.cloudHeight);
        for (int y = 0; y < result.cloudHeight; ++y)
            for (int x = 0; x < result.cloudWidth; ++x) {
                const int u = x * stride, v = y * stride;
                const float d = result.depth.at<float>(v, u);
                const auto pixel = rgb.at<cv::Vec3b>(v, u);
                auto &p = result.cloud[size_t(y) * result.cloudWidth + x];
                p.x = (u - request.cx) * d / request.fx;
                p.y = (v - request.cy) * d / request.fy;
                p.z = d;
                p.r = pixel[0];
                p.g = pixel[1];
                p.b = pixel[2];
            }
    }
    if (request.jpeg)
        encodeCameraJpeg(rgb, result.jpeg);
    return result;
}

} // namespace pool
