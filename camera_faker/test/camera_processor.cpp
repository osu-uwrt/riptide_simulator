#include "pool_viewer/camera.hpp"
#include "pool_viewer/camera_processor.hpp"
#include <cassert>
#include <future>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

namespace {
std::string processDepth(pool::CameraProcessor &processor, cv::Mat &buffer, float nearPlane, float farPlane,
                         const pool::DepthNoise &noise, std::mt19937 &random) {
    pool::CameraRequest request;
    request.nearPlane = nearPlane;
    request.farPlane = farPlane;
    request.noise = noise;
    auto result = processor.processFrame({}, buffer, request, random);
    buffer = std::move(result.depth);
    return result.warning;
}
cv::Mat buffer(int width, int height, float metres = 2.f) {
    constexpr float nearPlane = .05f, farPlane = 100.f;
    const float z = (farPlane + nearPlane - 2 * nearPlane * farPlane / metres) / (farPlane - nearPlane);
    return cv::Mat(height, width, CV_32FC1, cv::Scalar((z + 1) / 2)).clone();
}
void equal(const cv::Mat &a, const cv::Mat &b, double tolerance = 0) {
    assert(a.size() == b.size());
    for (int y = 0; y < a.rows; ++y)
        for (int x = 0; x < a.cols; ++x) {
            const float u = a.at<float>(y, x), v = b.at<float>(y, x);
            assert((std::isnan(u) && std::isnan(v)) || std::abs(u - v) <= tolerance);
        }
}
class FailingBackend : public pool::CameraBackend {
  public:
    std::string name() const override {
        return "injected failure";
    }
    pool::CameraProducts process(const cv::Mat &, const cv::Mat &, const pool::CameraRequest &,
                                 std::mt19937 &random) override {
        random.discard(20);
        throw std::runtime_error("injected CUDA failure");
    }
};
void cpuTests() {
    pool::CameraProcessor cpu("cpu"), automatic;
    assert(!cpu.usesCuda());
    assert(!automatic.description().empty());
    pool::DepthNoise noise;
    auto input = buffer(37, 29);
    input.at<float>(0, 0) = 1.f;
    input.at<float>(1, 0) = NAN;
    auto actual = input.clone(), expected = input.clone();
    std::mt19937 a(42), b(42);
    for (int y = 0; y < expected.rows; ++y)
        for (int x = 0; x < expected.cols; ++x) {
            float &z = expected.at<float>(y, x);
            z = z >= .999999f ? NAN : pool::linearDepth(z);
        }
    noise.apply(expected, b);
    assert(processDepth(cpu, actual, .05f, 100.f, noise, a).empty());
    equal(actual, expected);
    assert(a == b);

    pool::CameraProcessor failing(std::make_unique<FailingBackend>());
    a.seed(42);
    actual = input.clone();
    const auto warning = processDepth(failing, actual, .05f, 100.f, noise, a);
    assert(warning.find("injected CUDA failure") != std::string::npos);
    assert(!failing.usesCuda());
    equal(actual, expected);
    assert(a == b); // The failed attempt must not consume the CPU RNG sequence.
    actual = input.clone();
    assert(processDepth(failing, actual, .05f, 100.f, noise, a).empty());
    bool rejected = false;
    try {
        pool::CameraProcessor invalid("typo");
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    assert(rejected);
}
void checkProducts(const pool::CameraProducts &result, const cv::Mat &rgb, const pool::CameraRequest &request) {
    assert(result.cloudWidth == (rgb.cols - 1) / request.cloudStride + 1);
    assert(result.cloudHeight == (rgb.rows - 1) / request.cloudStride + 1);
    assert(result.cloud.size() == size_t(result.cloudWidth) * result.cloudHeight);
    for (int y = 0; y < result.cloudHeight; ++y)
        for (int x = 0; x < result.cloudWidth; ++x) {
            const int u = x * request.cloudStride, v = y * request.cloudStride;
            const auto &p = result.cloud[size_t(y) * result.cloudWidth + x];
            const float z = result.depth.at<float>(v, u);
            if (std::isnan(z))
                assert(std::isnan(p.x) && std::isnan(p.y) && std::isnan(p.z));
            else {
                assert(p.z == z);
                assert(std::abs(p.x - (u - request.cx) * z / request.fx) < 1e-5);
                assert(std::abs(p.y - (v - request.cy) * z / request.fy) < 1e-5);
            }
            const auto pixel = rgb.at<cv::Vec3b>(v, u);
            assert(p.r == pixel[0] && p.g == pixel[1] && p.b == pixel[2]);
            assert(p.padding == 0 && p.alpha == 0 && p.reserved[0] == 0 && p.reserved[1] == 0 && p.reserved[2] == 0);
        }
    cv::Mat gray, preview;
    result.depth.convertTo(gray, CV_8U, 255 / request.noise.maxRange);
    cv::applyColorMap(gray, preview, cv::COLORMAP_TURBO);
    for (int y = 0; y < result.depth.rows; ++y)
        for (int x = 0; x < result.depth.cols; ++x)
            if (!std::isfinite(result.depth.at<float>(y, x)))
                preview.at<cv::Vec3b>(y, x) = {16, 22, 27};
    cv::cvtColor(preview, preview, cv::COLOR_BGR2RGB);
    cv::flip(preview, preview, 0);
    assert(cv::norm(result.preview, preview, cv::NORM_INF) <= 4); // At most one palette bin from rounding.
    cv::Mat decoded = cv::imdecode(result.jpeg, cv::IMREAD_COLOR), expected;
    cv::cvtColor(rgb, expected, cv::COLOR_RGB2BGR);
    assert(decoded.size() == rgb.size() && decoded.type() == CV_8UC3);
    cv::Mat difference;
    cv::absdiff(decoded, expected, difference);
    const auto mean = cv::mean(difference);
    for (int c = 0; c < 3; ++c)
        assert(mean[c] < 5);
}
void cameraTests(pool::CameraProcessor &processor, bool gpu) {
    // Cropped inputs exercise pitched uploads; asymmetric gradients detect RGB
    // channel swaps and accidental image flips in the JPEG/cloud paths.
    cv::Mat parent(59, 79, CV_8UC3);
    for (int y = 0; y < parent.rows; ++y)
        for (int x = 0; x < parent.cols; ++x)
            parent.at<cv::Vec3b>(y, x) = {uint8_t(30 + x * 2), uint8_t(20 + y * 3), uint8_t(220 - x)};
    const auto rgb = parent(cv::Rect(2, 3, 71, 53));
    auto depthParent = buffer(79, 59);
    auto input = depthParent(cv::Rect(2, 3, 71, 53));
    input.at<float>(0, 0) = 1.f;
    input.at<float>(1, 0) = NAN;
    auto original = input.clone();
    pool::CameraRequest request;
    request.jpeg = request.preview = true;
    request.fx = 33.7;
    request.fy = 40.1;
    request.cx = 30.2;
    request.cy = 22.4;
    std::mt19937 random(17);
    for (int stride : {1, 3, 8, 100}) {
        request.cloudStride = stride;
        auto result = processor.processFrame(rgb, input, request, random);
        assert(result.warning.empty());
        assert(processor.usesCuda() == gpu);
        checkProducts(result, rgb, request);
        equal(original, input); // Inputs must survive an attempted GPU frame.
        if (gpu && processor.description().find("JPEG: nvJPEG") != std::string::npos)
            assert(result.jpegOnGpu);
    }
    // RGB-only subscribers must not need depth or consume depth RNG state.
    const auto before = random;
    pool::CameraRequest rgbOnly;
    rgbOnly.jpeg = true;
    auto jpegOnly = processor.processFrame(rgb, {}, rgbOnly, random);
    assert(jpegOnly.warning.empty() && !jpegOnly.jpeg.empty() && jpegOnly.depth.empty() && jpegOnly.cloud.empty());
    assert(random == before);
    auto empty = processor.processFrame({}, {}, {}, random);
    assert(empty.depth.empty() && empty.jpeg.empty() && empty.cloud.empty() && random == before);

    // A failure after partial backend work retries the complete acquisition.
    pool::CameraProcessor failing(std::make_unique<FailingBackend>()), reference("cpu");
    std::mt19937 a(31), b(31);
    auto failed = failing.processFrame(rgb, input, request, a);
    auto expected = reference.processFrame(rgb, input, request, b);
    assert(!failed.warning.empty() && !failing.usesCuda() && a == b);
    equal(failed.depth, expected.depth);
    assert(failed.jpeg == expected.jpeg);
    checkProducts(failed, rgb, request);
    assert(failing.processFrame(rgb, input, request, a).warning.empty());
    bool rejected = false;
    try {
        processor.processFrame(rgb, {}, request, random);
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    assert(rejected);
}
void processGpu(pool::CameraProcessor &gpu, cv::Mat &input, const pool::DepthNoise &noise, std::mt19937 &random) {
    assert(processDepth(gpu, input, .05f, 100.f, noise, random).empty());
    assert(gpu.usesCuda()); // A fallback must fail this test, not silently pass it.
}
int gpuTests() {
    pool::CameraProcessor gpu;
    if (!gpu.usesCuda()) {
        std::cout << "SKIP: " << gpu.description() << '\n';
        return 77;
    }
    pool::CameraProcessor cpu("cpu");
    pool::DepthNoise noise;
    noise.enabled = false;
    for (const auto size : {cv::Size(1, 1), cv::Size(71, 53), cv::Size(640, 480)}) {
        auto parent = buffer(size.width + 3, size.height + 2);
        auto input = parent(cv::Rect(1, 1, size.width, size.height));
        input.at<float>(0, 0) = 1.f;
        if (size.width > 1) {
            input.at<float>(0, 1) = NAN;
            input.at<float>(1, 0) = 0.f;
            input.at<float>(2, 0) = buffer(1, 1, 12).at<float>(0, 0);
        }
        auto expected = input.clone();
        std::mt19937 a(42), b(42);
        processDepth(cpu, expected, .05f, 100.f, noise, a);
        // Exercise device binding on a different thread, as camera workers do.
        auto worker = std::async(std::launch::async, [&] { processGpu(gpu, input, noise, b); });
        worker.get();
        equal(input, expected, .0001);
        assert(a == b);
    }
    noise.enabled = true;
    noise.dropout = noise.rangeDropout = noise.edgeDropout = noise.outliers = 0;
    noise.baseSigma = .02;
    noise.rangeSigma = 0;
    for (double correlation : {0., .8, 1.}) {
        noise.correlation = correlation;
        auto input = buffer(1024, 768), repeat = input.clone();
        std::mt19937 a(72), b(72);
        processGpu(gpu, input, noise, a);
        processGpu(gpu, repeat, noise, b);
        equal(input, repeat);
        assert(a == b);
        cv::Scalar mean, sigma;
        cv::meanStdDev(input, mean, sigma);
        assert(std::abs(mean[0] - 2) < .002);
        assert(std::abs(sigma[0] - .02) < .002);
    }
    // Exercise range-dependent sigma, bias, and bounded outliers together.
    noise.correlation = 0;
    noise.baseSigma = 0;
    noise.rangeSigma = .01;
    noise.exponent = 1.5;
    noise.bias = .02;
    noise.outliers = 1;
    auto outliers = buffer(1024, 768);
    std::mt19937 outlierRandom(27);
    processGpu(gpu, outliers, noise, outlierRandom);
    cv::Scalar mean, deviation;
    cv::meanStdDev(outliers, mean, deviation);
    const double expectedSigma = std::sqrt(std::pow(noise.sigma(2), 2) + 1. / 12.);
    assert(std::abs(mean[0] - 2.02) < .003);
    assert(std::abs(deviation[0] - expectedSigma) < .003);
    noise.bias = noise.rangeSigma = noise.outliers = 0;
    noise.baseSigma = .02;

    // Separate cameras may run concurrently without sharing streams or RNGs.
    pool::CameraProcessor otherGpu;
    assert(otherGpu.usesCuda());
    auto first = buffer(320, 240), second = first.clone();
    std::mt19937 firstRandom(11), secondRandom(11);
    auto worker = std::async(std::launch::async, [&] { processGpu(gpu, first, noise, firstRandom); });
    processGpu(otherGpu, second, noise, secondRandom);
    worker.get();
    equal(first, second);

    noise.dropout = .25;
    auto input = buffer(640, 480);
    std::mt19937 random(7);
    processGpu(gpu, input, noise, random);
    int invalid = 0;
    for (size_t i = 0; i < input.total(); ++i)
        invalid += std::isnan(input.ptr<float>()[i]);
    assert(std::abs(double(invalid) / input.total() - .25) < .01);

    noise.dropout = 0;
    noise.edgeDropout = 1;
    noise.baseSigma = 0;
    input = buffer(5, 5);
    input.at<float>(2, 2) = 1;
    processGpu(gpu, input, noise, random);
    assert(std::isnan(input.at<float>(2, 2)) && std::isnan(input.at<float>(2, 1)));
    assert(std::isnan(input.at<float>(1, 2)) && std::isnan(input.at<float>(2, 3)));
    assert(std::isnan(input.at<float>(3, 2)) && std::isfinite(input.at<float>(0, 0)));
    cameraTests(gpu, true);
    std::cout << "CUDA camera tests passed: " << gpu.description() << '\n';
    return 0;
}
} // namespace
int main(int argc, char **) {
    if (argc > 1)
        return gpuTests();
    cpuTests();
    pool::CameraProcessor cpu("cpu");
    cameraTests(cpu, false);
    std::cout << "CPU camera/depth/cloud and CUDA failure recovery tests passed\n";
}
