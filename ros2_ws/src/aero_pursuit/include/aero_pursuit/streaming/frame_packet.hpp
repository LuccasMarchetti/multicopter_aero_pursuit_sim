#pragma once

#include <array>
#include <cstdint>
#include <cstring>

namespace aero_pursuit::streaming
{

/// Binary header (64 bytes, little-endian) of each streamer -> GCS packet. The GCS
/// passes the same header to the browser, which uses it to decode and measure latency.
///
///  off  tipo  campo
///   0   u32   magic 'AGV1'
///   4   u8    type (1 = quadro, 2 = info texto, 3 = erro texto)
///   5   u8    flags (bit 0 = keyframe)
///   6   u8    codec (Codec)
///   7   u8    reservado
///   8   u32   largura codificada      12  u32 altura codificada
///  16   u32   largura original        20  u32 altura original
///  24   u32   seq                     28  u32 payload size
///  32   i64   ROS header stamp [ns] (simulation time)
///  40   i64   reception at the streamer [ns, system clock]
///  48   i64   end of encoding [ns, system clock]
///  56   i64   frame age on arrival (ROS now - stamp) [ns]
struct FrameHeader
{
  static constexpr std::uint32_t kMagic = 0x31564741;  // "AGV1"
  static constexpr std::size_t kSize = 64;
  static constexpr std::uint8_t kTypeFrame = 1;
  static constexpr std::uint8_t kTypeInfo = 2;
  static constexpr std::uint8_t kTypeError = 3;

  std::uint8_t type = kTypeFrame;
  std::uint8_t flags = 0;
  std::uint8_t codec = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t src_width = 0;
  std::uint32_t src_height = 0;
  std::uint32_t seq = 0;
  std::uint32_t payload_size = 0;
  std::int64_t stamp_ns = 0;
  std::int64_t recv_ns = 0;
  std::int64_t encoded_ns = 0;
  std::int64_t sim_age_ns = 0;

  std::array<std::uint8_t, kSize> serialize() const
  {
    std::array<std::uint8_t, kSize> b{};
    auto put = [&b](std::size_t off, const auto & v) {std::memcpy(b.data() + off, &v, sizeof(v));};
    put(0, kMagic);
    b[4] = type;
    b[5] = flags;
    b[6] = codec;
    put(8, width);
    put(12, height);
    put(16, src_width);
    put(20, src_height);
    put(24, seq);
    put(28, payload_size);
    put(32, stamp_ns);
    put(40, recv_ns);
    put(48, encoded_ns);
    put(56, sim_age_ns);
    return b;  // x86/ARM little-endian: memcpy direto
  }
};

}  // namespace aero_pursuit::streaming
