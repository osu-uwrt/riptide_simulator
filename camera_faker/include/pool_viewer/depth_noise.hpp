#pragma once
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <stdexcept>
#include <vector>

namespace pool {
// Empirical stereo failure model, not a replica of the ZED neural estimator.
// z is optical-axis depth in metres. One realization feeds preview, ROS depth,
// and the organized cloud, preserving their registration and invalid pixels.
struct DepthNoise {
  bool enabled = true;
  double baseSigma = .002, rangeSigma = .0015, exponent = 2.;
  double minRange = .15, maxRange = 8., bias = 0.;
  double dropout = .005, rangeDropout = .10, edgeDropout = .20;
  double outliers = .002, correlation = .5;
  int patchSize = 8;
  void validate() const {
    for (double x : {baseSigma, rangeSigma, exponent, minRange, maxRange, bias,
                     dropout, rangeDropout, edgeDropout, outliers, correlation})
      if (!std::isfinite(x))
        throw std::invalid_argument("Depth settings must be finite");
    if (baseSigma < 0 || rangeSigma < 0 || exponent < 0 || exponent > 4 ||
        minRange <= 0 || maxRange <= minRange || maxRange > 100 ||
        patchSize < 1 || patchSize > 64)
      throw std::invalid_argument("Invalid depth noise or range settings");
    for (double p : {dropout, rangeDropout, edgeDropout, outliers, correlation})
      if (p < 0 || p > 1)
        throw std::invalid_argument("Depth probabilities must be in [0,1]");
  }
  double sigma(double z) const {
    return baseSigma + rangeSigma * (exponent == 2. ? z * z : std::pow(z, exponent));
  }
  void apply(cv::Mat &depth, std::mt19937 &random) const {
    validate();
    if (depth.type() != CV_32FC1)
      throw std::invalid_argument("Expected metric float depth");
    const cv::Mat truth = depth.clone();
    std::normal_distribution<float> normal(0, 1);
    cv::Mat patches;
    std::vector<double> normalizeX, normalizeY;
    const double independentWeight = std::sqrt(1 - correlation);
    const double sharedWeight = std::sqrt(correlation);
    if (enabled && correlation > 0) {
      cv::Mat coarse((depth.rows + patchSize - 1) / patchSize + 1,
                     (depth.cols + patchSize - 1) / patchSize + 1, CV_32FC1);
      for (int y = 0; y < coarse.rows; ++y)
        for (int x = 0; x < coarse.cols; ++x)
          coarse.at<float>(y, x) = normal(random);
      cv::resize(coarse, patches, depth.size(), 0, 0, cv::INTER_LINEAR);
      // Bilinear variance is separable and depends only on image coordinates.
      // Compute these factors per row/column instead of per depth pixel.
      const auto normalization = [](int size, int coarseSize) {
        std::vector<double> factors(size);
        for (int i = 0; i < size; ++i) {
          const double s = (i + .5) * coarseSize / size - .5;
          const double f = s < 0 || s >= coarseSize - 1 ? 0 : s - std::floor(s);
          factors[i] = 1. / std::sqrt(f * f + (1 - f) * (1 - f));
        }
        return factors;
      };
      normalizeX = normalization(depth.cols, coarse.cols);
      normalizeY = normalization(depth.rows, coarse.rows);
    }
    // Generate pixel samples in row-sized batches using OpenCV's fast RNG.
    // Keep a local seeded generator: concurrent cameras must not share RNG
    // state, and temporary storage should not scale with image height.
    cv::RNG pixelRandom(enabled ? random() : 0);
    cv::Mat independent, probabilities;
    if (enabled) {
      independent.create(1, depth.cols, CV_32FC1);
      probabilities.create(1, depth.cols, CV_32FC2);
    }
    for (int y = 0; y < depth.rows; ++y) {
      if (enabled) {
        pixelRandom.fill(independent, cv::RNG::NORMAL, 0., 1.);
        pixelRandom.fill(probabilities, cv::RNG::UNIFORM, 0., 1.);
      }
      for (int x = 0; x < depth.cols; ++x) {
        float &z = depth.at<float>(y, x);
        if (!std::isfinite(z) || z < minRange || z > maxRange) {
          z = NAN;
          continue;
        }
        if (!enabled)
          continue;
        bool edge = false;
        for (const cv::Point offset : {cv::Point(-1, 0), cv::Point(1, 0),
                                       cv::Point(0, -1), cv::Point(0, 1)}) {
          const int u = x + offset.x, v = y + offset.y;
          if (u < 0 || v < 0 || u >= depth.cols || v >= depth.rows)
            continue;
          const float adjacent = truth.at<float>(v, u);
          if (!std::isfinite(adjacent) ||
              std::abs(adjacent - z) > .05 + .03 * z)
            edge = true;
        }
        const double probability =
            std::clamp(dropout + rangeDropout * std::pow(z / maxRange, 2) +
                           (edge ? edgeDropout : 0),
                       0., 1.);
        const auto chance = probabilities.at<cv::Vec2f>(0, x);
        if (chance[0] < probability) {
          z = NAN;
          continue;
        }
        // Bilinear interpolation averages four samples. Normalize its variance
        // so the displayed sigma remains the actual marginal standard
        // deviation.
        float shared = 0;
        if (!patches.empty())
          shared = patches.at<float>(y, x) * normalizeX[x] * normalizeY[y];
        const double perturbation =
            independentWeight * independent.at<float>(0, x) + sharedWeight * shared;
        const float original = z;
        z += bias + sigma(z) * perturbation;
        if (chance[1] < outliers)
          z += pixelRandom.uniform(-.25f, .25f) * original;
        if (!std::isfinite(z) || z < minRange || z > maxRange)
          z = NAN;
      }
    }
  }
};
} // namespace pool
