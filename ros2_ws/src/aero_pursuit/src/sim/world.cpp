#include "aero_pursuit/sim/world.hpp"

#include <cmath>
#include <regex>

namespace aero_pursuit::sim
{

double yawOf(const Eigen::Quaterniond & q)
{
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

std::string withNamespace(const std::string & sdf, const std::string & ns)
{
  static const std::regex tag(R"(<robotNamespace>[^<]*</robotNamespace>)");
  return std::regex_replace(sdf, tag, "<robotNamespace>" + ns + "</robotNamespace>");
}

}  // namespace aero_pursuit::sim
