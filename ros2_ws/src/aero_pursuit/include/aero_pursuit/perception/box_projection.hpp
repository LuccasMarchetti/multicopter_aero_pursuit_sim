#pragma once

#include "aero_pursuit/perception/camera_model.hpp"

#include <opencv2/core.hpp>

#include <opencv2/core/matx.hpp>

#include <optional>

namespace aero_pursuit::perception
{

/// Box of a vehicle projected into the image from ground truth.
struct ProjectedBox
{
  cv::Rect2d box;          ///< Pixels of the original image (already clipped to the bounds)
  double depth = 0.0;      ///< Distance along the optical axis [m]
  bool center_inside = false;  ///< Center inside the image (with the margin)
};

/// Projects an ellipsoid (radius `radius` horizontally, `radius * height_ratio`
/// vertically: multirotors are wide and short) centered at `p_optical` (optical
/// frame: x right, y down, z forward). nullopt behind the camera, too close or outside
/// the image. `margin` [px] shrinks the region accepted for the center.
std::optional<ProjectedBox> projectSphere(
  const PinholeIntrinsics & k, const cv::Point3d & p_optical, double radius,
  const cv::Size & image, double margin = 0.0, double min_depth = 0.3, double height_ratio = 1.0);

/// Oriented box of a vehicle in the model's own frame (Gazebo FLU).
struct ModelBox
{
  cv::Point3d center{0.0, 0.0, 0.0};  ///< Box center in the model frame [m]
  cv::Point3d size{0.5, 0.5, 0.2};    ///< Edges (x, y, z) [m], with the propellers
};

/// Exact projection (pinhole model) of the 8 corners of the oriented box: `R_cam_obj`/`t_cam_obj`
/// take model points to the optical frame (x right, y down, z forward). Returns the
/// tight 2D box (clipped to the image); nullopt if any corner is behind `min_depth`
/// (too close / behind the camera) or if nothing falls in the image. `depth` = z of the center.
std::optional<ProjectedBox> projectOrientedBox(
  const PinholeIntrinsics & k, const cv::Matx33d & R_cam_obj, const cv::Vec3d & t_cam_obj,
  const ModelBox & box, const cv::Size & image, double margin = 0.0, double min_depth = 0.3);

/// Intersection over union of two boxes (0 without overlap).
double intersectionOverUnion(const cv::Rect2d & a, const cv::Rect2d & b);

}  // namespace aero_pursuit::perception
