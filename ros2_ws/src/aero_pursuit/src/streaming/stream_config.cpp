#include "aero_pursuit/streaming/stream_config.hpp"

#include <algorithm>
#include <charconv>
#include <string>

namespace aero_pursuit::streaming
{
namespace
{

/// from_chars is locale independent (strtod would read "0,5" in pt_BR).
template<typename T>
bool parseNumber(std::string_view s, T & out)
{
  T v{};
  const auto r = std::from_chars(s.data(), s.data() + s.size(), v);
  if (r.ec != std::errc{}) {return false;}
  out = v;
  return true;
}

}  // namespace

std::string_view codecName(Codec c)
{
  switch (c) {
    case Codec::kH264: return "h264";
    case Codec::kH265: return "h265";
    case Codec::kMjpeg: return "mjpeg";
  }
  return "h264";
}

std::optional<Codec> parseCodec(std::string_view s)
{
  if (s == "h264") {return Codec::kH264;}
  if (s == "h265" || s == "hevc") {return Codec::kH265;}
  if (s == "mjpeg" || s == "jpeg") {return Codec::kMjpeg;}
  return std::nullopt;
}

void applyConfigLine(std::string_view line, StreamConfig & cfg)
{
  std::size_t pos = 0;
  while (pos < line.size()) {
    const std::size_t end = std::min(line.find(' ', pos), line.size());
    const std::string_view tok = line.substr(pos, end - pos);
    pos = end + 1;
    const std::size_t eq = tok.find('=');
    if (eq == std::string_view::npos) {continue;}
    const std::string_view key = tok.substr(0, eq);
    const std::string_view val = tok.substr(eq + 1);

    if (key == "active") {
      cfg.active = (val == "1" || val == "true");
    } else if (key == "topic") {
      if (!val.empty()) {cfg.topic = std::string(val);}
    } else if (key == "codec") {
      if (auto c = parseCodec(val)) {cfg.codec = *c;}
    } else if (key == "backend") {
      cfg.backend = val == "hardware" ? Backend::kHardware :
        val == "software" ? Backend::kSoftware : Backend::kAuto;
    } else if (key == "scale") {
      double v = cfg.scale;
      if (parseNumber(val, v)) {cfg.scale = std::clamp(v, 0.05, 1.0);}
    } else if (key == "fps") {
      double v = cfg.max_fps;
      if (parseNumber(val, v)) {cfg.max_fps = std::clamp(v, 0.0, 240.0);}
    } else if (key == "bitrate") {
      int v = cfg.bitrate_kbps;
      if (parseNumber(val, v)) {cfg.bitrate_kbps = std::clamp(v, 100, 200000);}
    } else if (key == "rc") {
      cfg.rate_control = val == "vbr" ? RateControl::kVbr :
        val == "cqp" ? RateControl::kCqp : RateControl::kCbr;
    } else if (key == "quality") {
      int v = cfg.quality;
      if (parseNumber(val, v)) {cfg.quality = std::clamp(v, 1, 100);}
    } else if (key == "gop") {
      int v = cfg.gop;
      if (parseNumber(val, v)) {cfg.gop = std::clamp(v, 1, 1000);}
    } else if (key == "depth_min") {
      double v = cfg.depth_min;
      if (parseNumber(val, v)) {cfg.depth_min = std::max(0.0, v);}
    } else if (key == "depth_max") {
      double v = cfg.depth_max;
      if (parseNumber(val, v)) {cfg.depth_max = std::max(0.01, v);}
    }
  }
  if (cfg.depth_max <= cfg.depth_min) {cfg.depth_max = cfg.depth_min + 0.1;}
}

}  // namespace aero_pursuit::streaming
