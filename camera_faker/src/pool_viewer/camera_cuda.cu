#include "pool_viewer/camera_processor.hpp"
#include <cuda_runtime.h>
#include <cstdint>
#ifdef POOL_HAS_NVJPEG
#include <nvjpeg.h>
#endif

namespace pool {
namespace {
void check(cudaError_t error, const char *operation) {
    if (error != cudaSuccess)
        throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(error));
}

// Stateless SplitMix64 samples give each pixel its own stream. Noise has the
// same distribution as the CPU model, but the samples are not bit-for-bit equal.
__device__ double uniform(uint64_t &state) {
    uint64_t z = (state += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    z ^= z >> 31;
    return ((z >> 12) + .5) * 0x1.0p-52;
}

__global__ void linearize(const float *input, float *truth, size_t count, float nearPlane, float farPlane) {
    const size_t i = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i >= count)
        return;
    const float z = input[i];
    truth[i] = z >= .999999f ? nanf("")
                             : 2 * nearPlane * farPlane / (farPlane + nearPlane - (2 * z - 1) * (farPlane - nearPlane));
}

__device__ double patchCoordinate(int pixel, int size, int coarseSize, int &lo, int &hi) {
    const double s = (pixel + .5) * coarseSize / size - .5;
    const double bounded = fmin(fmax(s, 0.), double(coarseSize - 1));
    lo = int(floor(bounded));
    hi = min(lo + 1, coarseSize - 1);
    return bounded - lo;
}

__global__ void applyNoise(const float *truth, float *output, int width, int height, DepthNoise noise,
                           const float *coarse, int coarseWidth, int coarseHeight, uint32_t seed) {
    const size_t i = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i >= size_t(width) * height)
        return;
    float z = truth[i];
    if (!isfinite(z) || z < noise.minRange || z > noise.maxRange) {
        output[i] = nanf("");
        return;
    }
    if (!noise.enabled) {
        output[i] = z;
        return;
    }
    const int x = i % width, y = i / width;
    bool edge = false;
    const int dx[] = {-1, 1, 0, 0}, dy[] = {0, 0, -1, 1};
    for (int n = 0; n < 4; ++n) {
        const int u = x + dx[n], v = y + dy[n];
        if (u >= 0 && v >= 0 && u < width && v < height) {
            const float adjacent = truth[size_t(v) * width + u];
            edge |= !isfinite(adjacent) || fabsf(adjacent - z) > .05 + .03 * z;
        }
    }
    uint64_t state = (uint64_t(seed) << 32) | uint32_t(i);
    const double fraction = z / noise.maxRange;
    const double dropout =
        fmin(1., fmax(0., noise.dropout + noise.rangeDropout * fraction * fraction + (edge ? noise.edgeDropout : 0.)));
    if (uniform(state) < dropout) {
        output[i] = nanf("");
        return;
    }
    const double outlierChance = uniform(state);
    const double radial = sqrt(-2. * log(uniform(state)));
    const double independent = radial * cos(6.283185307179586 * uniform(state));
    double shared = 0;
    if (noise.correlation > 0) {
        int x0, x1, y0, y1;
        const double fx = patchCoordinate(x, width, coarseWidth, x0, x1);
        const double fy = patchCoordinate(y, height, coarseHeight, y0, y1);
        const double top = (1 - fx) * coarse[y0 * coarseWidth + x0] + fx * coarse[y0 * coarseWidth + x1];
        const double bottom = (1 - fx) * coarse[y1 * coarseWidth + x0] + fx * coarse[y1 * coarseWidth + x1];
        shared =
            ((1 - fy) * top + fy * bottom) / sqrt((fx * fx + (1 - fx) * (1 - fx)) * (fy * fy + (1 - fy) * (1 - fy)));
    }
    const float original = z;
    const double sigma =
        noise.baseSigma + noise.rangeSigma * (noise.exponent == 2. ? double(z) * z : pow(double(z), noise.exponent));
    z += noise.bias + sigma * (sqrt(1 - noise.correlation) * independent + sqrt(noise.correlation) * shared);
    if (outlierChance < noise.outliers)
        z += (uniform(state) * .5 - .25) * original;
    output[i] = !isfinite(z) || z < noise.minRange || z > noise.maxRange ? nanf("") : z;
}

__global__ void makeCloud(const float *depth, const uint8_t *rgb, CloudPoint *cloud, int width, int cloudWidth,
                          int cloudHeight, int stride, double fx, double fy, double cx, double cy) {
    const size_t i = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i >= size_t(cloudWidth) * cloudHeight)
        return;
    const int u = (i % cloudWidth) * stride, v = (i / cloudWidth) * stride;
    const size_t pixel = size_t(v) * width + u;
    const float d = depth[pixel];
    CloudPoint p{};
    p.x = (u - cx) * d / fx;
    p.y = (v - cy) * d / fy;
    p.z = d;
    p.r = rgb[pixel * 3];
    p.g = rgb[pixel * 3 + 1];
    p.b = rgb[pixel * 3 + 2];
    cloud[i] = p;
}

__global__ void makePreview(const float *depth, uint8_t *preview, const uint8_t *lut, int width, int height,
                            float scale) {
    const size_t i = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i >= size_t(width) * height)
        return;
    // OpenCV uses nearest-even rounding; the display texture is bottom-up RGB.
    const float z = depth[i];
    const int index = isfinite(z) ? min(255, max(0, __float2int_rn(z * scale))) : 0;
    const size_t output = (size_t(height - 1 - i / width) * width + i % width) * 3;
    preview[output] = isfinite(z) ? lut[index * 3] : 27;
    preview[output + 1] = isfinite(z) ? lut[index * 3 + 1] : 22;
    preview[output + 2] = isfinite(z) ? lut[index * 3 + 2] : 16;
}

#ifdef POOL_HAS_NVJPEG
void checkJpeg(nvjpegStatus_t status, const char *operation) {
    if (status != NVJPEG_STATUS_SUCCESS)
        throw std::runtime_error(std::string(operation) + " (nvJPEG status " + std::to_string(int(status)) + ")");
}
class JpegEncoder {
  public:
    explicit JpegEncoder(cudaStream_t stream) : stream_(stream) {
        try {
            checkJpeg(nvjpegCreateSimple(&handle_), "create JPEG encoder");
            checkJpeg(nvjpegEncoderStateCreate(handle_, &state_, stream_), "create JPEG state");
            checkJpeg(nvjpegEncoderParamsCreate(handle_, &params_, stream_), "create JPEG parameters");
            checkJpeg(nvjpegEncoderParamsSetQuality(params_, 93, stream_), "set JPEG quality");
            checkJpeg(nvjpegEncoderParamsSetSamplingFactors(params_, NVJPEG_CSS_420, stream_), "set JPEG sampling");
        } catch (...) {
            release();
            throw;
        }
    }
    ~JpegEncoder() {
        release();
    }
    void encode(uint8_t *rgb, int width, int height, std::vector<uint8_t> &output) {
        size_t length = 0;
        struct Completion {
            cudaStream_t stream;
            ~Completion() {
                cudaStreamSynchronize(stream);
            }
        } completion{stream_};
        nvjpegImage_t image{};
        image.channel[0] = rgb;
        image.pitch[0] = size_t(width) * 3;
        checkJpeg(nvjpegEncodeImage(handle_, state_, params_, &image, NVJPEG_INPUT_RGBI, width, height, stream_),
                  "encode RGB camera image");
        checkJpeg(nvjpegEncodeRetrieveBitstream(handle_, state_, nullptr, &length, stream_), "size JPEG output");
        check(cudaStreamSynchronize(stream_), "finish JPEG sizing");
        output.resize(length);
        checkJpeg(nvjpegEncodeRetrieveBitstream(handle_, state_, output.data(), &length, stream_),
                  "retrieve JPEG output");
        check(cudaStreamSynchronize(stream_), "finish JPEG output");
        output.resize(length);
    }

  private:
    void release() {
        // Handles and host bitstreams must outlive asynchronous encoder work.
        cudaStreamSynchronize(stream_);
        if (params_)
            nvjpegEncoderParamsDestroy(params_);
        if (state_)
            nvjpegEncoderStateDestroy(state_);
        if (handle_)
            nvjpegDestroy(handle_);
    }
    cudaStream_t stream_;
    nvjpegHandle_t handle_{};
    nvjpegEncoderState_t state_{};
    nvjpegEncoderParams_t params_{};
};
#endif

// One stream per camera. Cloud/preview kernels consume the noisy depth already
// on the device; RGB is uploaded once and shared by cloud coloring and JPEG.
class CudaCamera final : public CameraBackend {
  public:
    CudaCamera(int device, const std::string &name) : device_(device), name_("CUDA (" + name + ")") {
#ifdef POOL_HAS_NVJPEG
        name_ += "; JPEG: nvJPEG";
#else
        name_ += "; JPEG: CPU (nvJPEG not built)";
#endif
        cv::Mat ramp(1, 256, CV_8UC1), bgr;
        for (int i = 0; i < 256; ++i)
            ramp.at<uint8_t>(0, i) = i;
        cv::applyColorMap(ramp, bgr, cv::COLORMAP_TURBO);
        cv::cvtColor(bgr, lutHost_, cv::COLOR_BGR2RGB);
        check(cudaSetDevice(device_), "select CUDA device");
        check(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "create camera stream");
    }
    ~CudaCamera() override {
        cudaSetDevice(device_);
        cudaStreamSynchronize(stream_);
#ifdef POOL_HAS_NVJPEG
        jpeg_.reset();
#endif
        cudaFree(input_);
        cudaFree(truth_);
        cudaFree(output_);
        cudaFree(coarse_);
        cudaFree(rgb_);
        cudaFree(cloud_);
        cudaFree(preview_);
        cudaFree(lut_);
        cudaStreamDestroy(stream_);
    }
    std::string name() const override {
        return name_;
    }
    CameraProducts process(const cv::Mat &rgb, const cv::Mat &buffer, const CameraRequest &request,
                           std::mt19937 &random) override {
        // std::async may choose a new host thread for each acquisition.
        check(cudaSetDevice(device_), "select CUDA device");
        const auto &noise = request.noise;
        CameraProducts result;
        cv::Mat coarse;
        // Drain outstanding copies before host storage is destroyed on failure.
        struct Completion {
            cudaStream_t stream;
            ~Completion() {
                cudaStreamSynchronize(stream);
            }
        } completion{stream_};

        bool gpuJpeg = false;
#ifdef POOL_HAS_NVJPEG
        if (request.jpeg && !jpegFailed_) {
            try {
                if (!jpeg_)
                    jpeg_ = std::make_unique<JpegEncoder>(stream_);
                gpuJpeg = true;
            } catch (const std::exception &error) {
                disableJpeg(result, error);
            }
        }
#endif
        if (request.cloudStride || gpuJpeg) {
            reserve(rgb_, rgbCapacity_, rgb.total() * 3);
            check(cudaMemcpy2DAsync(rgb_, rgb.cols * 3, rgb.data, rgb.step, rgb.cols * 3, rgb.rows,
                                    cudaMemcpyHostToDevice, stream_),
                  "upload camera RGB");
        }
        if (!buffer.empty()) {
            const size_t count = buffer.total();
            reserve(input_, inputCapacity_, count);
            reserve(truth_, truthCapacity_, count);
            reserve(output_, outputCapacity_, count);
            result.depth.create(buffer.size(), CV_32FC1);
            if (noise.enabled && noise.correlation > 0) {
                coarse.create((buffer.rows + noise.patchSize - 1) / noise.patchSize + 1,
                              (buffer.cols + noise.patchSize - 1) / noise.patchSize + 1, CV_32FC1);
                std::normal_distribution<float> normal(0, 1);
                for (size_t i = 0; i < coarse.total(); ++i)
                    coarse.ptr<float>()[i] = normal(random);
                reserve(coarse_, coarseCapacity_, coarse.total());
                check(cudaMemcpyAsync(coarse_, coarse.data, coarse.total() * sizeof(float), cudaMemcpyHostToDevice,
                                      stream_),
                      "upload depth patches");
            }
            const uint32_t seed = noise.enabled ? random() : 0;
            check(cudaMemcpy2DAsync(input_, buffer.cols * sizeof(float), buffer.data, buffer.step,
                                    buffer.cols * sizeof(float), buffer.rows, cudaMemcpyHostToDevice, stream_),
                  "upload depth buffer");
            const unsigned blocks = (count + 255) / 256;
            linearize<<<blocks, 256, 0, stream_>>>(input_, truth_, count, request.nearPlane, request.farPlane);
            check(cudaGetLastError(), "launch depth conversion");
            applyNoise<<<blocks, 256, 0, stream_>>>(truth_, output_, buffer.cols, buffer.rows, noise, coarse_,
                                                    coarse.cols, coarse.rows, seed);
            check(cudaGetLastError(), "launch depth noise");
            if (request.cloudStride) {
                result.cloudWidth = (buffer.cols - 1) / request.cloudStride + 1;
                result.cloudHeight = (buffer.rows - 1) / request.cloudStride + 1;
                result.cloud.resize(size_t(result.cloudWidth) * result.cloudHeight);
                reserve(cloud_, cloudCapacity_, result.cloud.size());
                makeCloud<<<(result.cloud.size() + 255) / 256, 256, 0, stream_>>>(
                    output_, rgb_, cloud_, buffer.cols, result.cloudWidth, result.cloudHeight, request.cloudStride,
                    request.fx, request.fy, request.cx, request.cy);
                check(cudaGetLastError(), "launch point cloud");
                check(cudaMemcpyAsync(result.cloud.data(), cloud_, result.cloud.size() * sizeof(CloudPoint),
                                      cudaMemcpyDeviceToHost, stream_),
                      "download point cloud");
            }
            if (request.preview) {
                reserve(preview_, previewCapacity_, count * 3);
                if (!lut_) {
                    reserve(lut_, lutCapacity_, size_t(256 * 3));
                    check(cudaMemcpyAsync(lut_, lutHost_.data, 256 * 3, cudaMemcpyHostToDevice, stream_),
                          "upload preview palette");
                }
                result.preview.create(buffer.size(), CV_8UC3);
                makePreview<<<blocks, 256, 0, stream_>>>(output_, preview_, lut_, buffer.cols, buffer.rows,
                                                         255 / noise.maxRange);
                check(cudaGetLastError(), "launch depth preview");
                check(cudaMemcpyAsync(result.preview.data, preview_, count * 3, cudaMemcpyDeviceToHost, stream_),
                      "download depth preview");
            }
            check(cudaMemcpyAsync(result.depth.data, output_, count * sizeof(float), cudaMemcpyDeviceToHost, stream_),
                  "download depth");
        }
#ifdef POOL_HAS_NVJPEG
        if (gpuJpeg) {
            try {
                jpeg_->encode(rgb_, rgb.cols, rgb.rows, result.jpeg);
                result.jpegOnGpu = true;
            } catch (const std::exception &error) {
                disableJpeg(result, error);
            }
        }
#endif
        check(cudaStreamSynchronize(stream_), "finish camera processing");
        if (request.jpeg && !result.jpegOnGpu)
            encodeCameraJpeg(rgb, result.jpeg);
        return result;
    }

  private:
    template <class T> void reserve(T *&pointer, size_t &capacity, size_t count) {
        if (count <= capacity)
            return;
        T *replacement = nullptr;
        check(cudaMalloc(reinterpret_cast<void **>(&replacement), count * sizeof(T)), "allocate camera buffer");
        cudaFree(pointer);
        pointer = replacement;
        capacity = count;
    }
#ifdef POOL_HAS_NVJPEG
    void disableJpeg(CameraProducts &result, const std::exception &error) {
        // Only JPEG falls back; depth and clouds can still use CUDA. The stream
        // is drained before CPU encoding can resize a partial JPEG buffer.
        jpeg_.reset();
        jpegFailed_ = true;
        result.warning = std::string("nvJPEG failed; using CPU JPEG: ") + error.what();
    }
    std::unique_ptr<JpegEncoder> jpeg_;
    bool jpegFailed_ = false;
#endif
    int device_;
    std::string name_;
    cudaStream_t stream_{};
    cv::Mat lutHost_;
    float *input_ = nullptr, *truth_ = nullptr, *output_ = nullptr, *coarse_ = nullptr;
    uint8_t *rgb_ = nullptr, *preview_ = nullptr, *lut_ = nullptr;
    CloudPoint *cloud_ = nullptr;
    size_t inputCapacity_ = 0, truthCapacity_ = 0, outputCapacity_ = 0, coarseCapacity_ = 0;
    size_t rgbCapacity_ = 0, cloudCapacity_ = 0, previewCapacity_ = 0, lutCapacity_ = 0;
};
} // namespace

std::unique_ptr<CameraBackend> makeCudaCameraBackend() {
    int count = 0;
    check(cudaGetDeviceCount(&count), "find CUDA devices");
    std::string reason = "no usable CUDA device";
    for (int device = 0; device < count; ++device) {
        try {
            cudaDeviceProp properties{};
            check(cudaGetDeviceProperties(&properties, device), "inspect CUDA device");
            if (properties.computeMode == cudaComputeModeProhibited)
                continue;
            return std::make_unique<CudaCamera>(device, properties.name);
        } catch (const std::exception &error) {
            reason = error.what();
        }
    }
    throw std::runtime_error(reason);
}
} // namespace pool
