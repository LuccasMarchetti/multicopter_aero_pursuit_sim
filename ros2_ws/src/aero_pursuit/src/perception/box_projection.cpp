#include "aero_pursuit/perception/box_projection.hpp"

#include <algorithm>

namespace aero_pursuit::perception
{

std::optional<ProjectedBox> projectSphere(
  const PinholeIntrinsics & k, const cv::Point3d & p, double radius,
  const cv::Size & image, double margin, double min_depth, double height_ratio)
{
  if (!k.valid() || p.z < min_depth) {
    return std::nullopt;
  }
  const double u = k.fx * p.x / p.z + k.cx;
  const double v = k.fy * p.y / p.z + k.cy;
  // Apparent radius (distant-sphere approximation; exact on the optical axis).
  const double hw = k.fx * radius / p.z;
  const double hh = k.fy * radius * height_ratio / p.z;
  const cv::Rect2d full(u - hw, v - hh, 2.0 * hw, 2.0 * hh);
  const cv::Rect2d clipped = full & cv::Rect2d(0.0, 0.0, image.width, image.height);
  if (clipped.width <= 0.0 || clipped.height <= 0.0) {
    return std::nullopt;
  }
  ProjectedBox out;
  out.box = clipped;
  out.depth = p.z;
  out.center_inside = u >= margin && v >= margin && u <= image.width - margin && v <= image.height - margin;
  return out;
}

std::optional<ProjectedBox> projectOrientedBox(
  const PinholeIntrinsics & k, const cv::Matx33d & R, const cv::Vec3d & t, const ModelBox & box,
  const cv::Size & image, double margin, double min_depth)
{
  if (!k.valid()) {
    return std::nullopt;
  }
  double u0 = 1e300, v0 = 1e300, u1 = -1e300, v1 = -1e300;
  for (int i = 0; i < 8; ++i) {
    const cv::Vec3d corner(
      box.center.x + ((i & 1) ? 0.5 : -0.5) * box.size.x,
      box.center.y + ((i & 2) ? 0.5 : -0.5) * box.size.y,
      box.center.z + ((i & 4) ? 0.5 : -0.5) * box.size.z);
    const cv::Vec3d p = R * corner + t;
    if (p[2] < min_depth) {
      return std::nullopt;
    }
    const double u = k.fx * p[0] / p[2] + k.cx;
    const double v = k.fy * p[1] / p[2] + k.cy;
    u0 = std::min(u0, u);
    u1 = std::max(u1, u);
    v0 = std::min(v0, v);
    v1 = std::max(v1, v);
  }
  const cv::Vec3d c = R * cv::Vec3d(box.center.x, box.center.y, box.center.z) + t;
  const cv::Rect2d clipped = cv::Rect2d(u0, v0, u1 - u0, v1 - v0) &
    cv::Rect2d(0.0, 0.0, image.width, image.height);
  if (clipped.width <= 0.0 || clipped.height <= 0.0) {
    return std::nullopt;
  }
  const double uc = k.fx * c[0] / c[2] + k.cx;
  const double vc = k.fy * c[1] / c[2] + k.cy;
  ProjectedBox out;
  out.box = clipped;
  out.depth = c[2];
  out.center_inside = uc >= margin && vc >= margin && uc <= image.width - margin &&
    vc <= image.height - margin;
  return out;
}

double intersectionOverUnion(const cv::Rect2d & a, const cv::Rect2d & b)
{
  const double inter = (a & b).area();
  const double uni = a.area() + b.area() - inter;
  return uni > 0.0 ? inter / uni : 0.0;
}

}  // namespace aero_pursuit::perception
