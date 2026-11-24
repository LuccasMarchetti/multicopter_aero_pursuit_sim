#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <limits>
#include <optional>
#include <vector>

namespace aero_pursuit::perception
{

/// Pinhole model (no distortion) from the K matrix of a CameraInfo.
struct PinholeIntrinsics
{
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  static PinholeIntrinsics fromK(const std::array<double, 9> & k)
  {
    return {k[0], k[4], k[2], k[5]};
  }

  bool valid() const {return fx > 0.0 && fy > 0.0;}

  /// Back-projects pixel (u, v) with depth Z into the optical frame
  /// (x: direita, y: baixo, z: frente).
  cv::Point3d backproject(double u, double v, double depth) const
  {
    return {(u - cx) * depth / fx, (v - cy) * depth / fy, depth};
  }
};

/// Line of sight of a pixel, used when there is no valid depth.
struct Bearing
{
  cv::Point3d direction;   ///< Unit vector in the optical frame (x: right, y: down, z: forward)
  double azimuth = 0.0;    ///< Offset around Z of camera_link (FLU) [rad]: + target to the left
  double elevation = 0.0;  ///< Offset around Y of camera_link (FLU) [rad]: + target above
};

Bearing bearingFromPixel(const PinholeIntrinsics & k, double u, double v);

/// Maps a box of the RGB camera to the depth camera, assuming
/// coincident optical centers (the OAK-D Lite case in Gazebo). Without valid
/// intrinsics, uses a scale proportional to the resolution.
cv::Rect mapBoxToDepth(
  const cv::Rect & box_rgb, const cv::Size & rgb_size, const cv::Size & depth_size,
  const PinholeIntrinsics & rgb, const PinholeIntrinsics & depth);

/// Robust ROI depth: value at the mid percentile in [lower, upper] of the
/// valid pixels (discards background and noise). Uses nth_element (O(n)).
/// `scratch` is reused between calls to avoid allocations.
/// Only pixels in [min_range, max_range) count: the Gazebo depth sensor returns
/// +inf beyond the far clip (or the far value itself, depending on the version) and −inf before the near clip.
std::optional<float> robustDepth(
  const cv::Mat & depth_m, const cv::Rect & box, double percentile_lower,
  double percentile_upper, std::vector<float> & scratch, float min_range = 0.01f,
  float max_range = std::numeric_limits<float>::infinity());

/// Classification of the depth pixels of a ROI by sensor range.
struct DepthCoverage
{
  int total = 0;    ///< ROI pixels (clipped to the image)
  int valid = 0;    ///< Within [min_range, max_range)
  int beyond = 0;   ///< +inf or ≥ max_range (beyond the far clip)
  int below = 0;    ///< −inf or < min_range (before the near clip)
  double validFraction() const {return total > 0 ? static_cast<double>(valid) / total : 0.0;}
  double beyondFraction() const {return total > 0 ? static_cast<double>(beyond) / total : 0.0;}
};
DepthCoverage depthCoverage(const cv::Mat & depth_m, const cv::Rect & box, float min_range,
  float max_range);

}  // namespace aero_pursuit::perception
