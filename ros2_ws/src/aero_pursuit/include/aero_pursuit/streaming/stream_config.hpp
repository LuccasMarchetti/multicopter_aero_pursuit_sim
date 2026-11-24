#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace aero_pursuit::streaming
{

enum class Codec : std::uint8_t {kH264 = 1, kH265 = 2, kMjpeg = 3};
enum class Backend : std::uint8_t {kAuto, kHardware, kSoftware};
enum class RateControl : std::uint8_t {kCbr, kVbr, kCqp};

/// Stream configuration, sent by the GCS (line protocol "cfg key=value ...").
struct StreamConfig
{
  bool active = false;
  std::string topic = "/camera";
  Codec codec = Codec::kH264;
  Backend backend = Backend::kAuto;
  double scale = 1.0;          ///< Fraction of the original resolution (0.1 - 1.0)
  double max_fps = 30.0;       ///< 0: no limit
  int bitrate_kbps = 8000;
  RateControl rate_control = RateControl::kCbr;
  int quality = 80;            ///< JPEG: 1-100; CQP: QP (0-51)
  int gop = 30;                ///< Frames between keyframes
  double depth_min = 0.2;      ///< [m] depth color-map range
  double depth_max = 20.0;

  /// Fields that require rebuilding the pipeline (the rest is applied live).
  bool needsRebuild(const StreamConfig & o) const
  {
    return codec != o.codec || backend != o.backend || scale != o.scale ||
           rate_control != o.rate_control || quality != o.quality || gop != o.gop;
  }
};

/// Applies key=value pairs (space separated) on `cfg`. Unknown keys
/// are ignored; invalid values keep the previous one.
void applyConfigLine(std::string_view line, StreamConfig & cfg);

std::string_view codecName(Codec c);
std::optional<Codec> parseCodec(std::string_view s);

}  // namespace aero_pursuit::streaming
