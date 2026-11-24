#pragma once

#include "aero_pursuit/streaming/stream_config.hpp"

#include <gst/gst.h>

#include <deque>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace aero_pursuit::streaming
{

/// Raw frame to encode. `keepalive` keeps the `data` memory alive until
/// GStreamer releases the buffer (zero copy of the ROS message).
struct RawFrame
{
  const std::uint8_t * data = nullptr;
  std::size_t size = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t stride = 0;
  std::string format;          ///< Formato GStreamer: RGB, BGR, GRAY8, RGBA, BGRA, GRAY16_LE
  std::shared_ptr<const void> keepalive;
  std::int64_t stamp_ns = 0;
  std::int64_t recv_ns = 0;
  std::int64_t sim_age_ns = 0;
};

/// Encoded frame delivered on the GStreamer thread.
struct EncodedFrame
{
  const std::uint8_t * data = nullptr;
  std::size_t size = 0;
  bool keyframe = false;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t src_width = 0;
  std::uint32_t src_height = 0;
  Codec codec = Codec::kH264;
  std::int64_t stamp_ns = 0;
  std::int64_t recv_ns = 0;
  std::int64_t encoded_ns = 0;
  std::int64_t sim_age_ns = 0;
};

/// GStreamer pipeline appsrc -> conversion/scaling -> encoder (VA-API or software)
/// -> appsink. Rebuilds itself when the input resolution/format or structural
/// configuration fields change; the bitrate is changed live when supported.
class VideoEncoder
{
public:
  using FrameHandler = std::function<void(const EncodedFrame &)>;
  /// Text info ("codec=avc1.640028 encoder=vah264enc width=...") or error.
  using InfoHandler = std::function<void(bool error, const std::string &)>;

  VideoEncoder(FrameHandler on_frame, InfoHandler on_info);
  ~VideoEncoder();
  VideoEncoder(const VideoEncoder &) = delete;
  VideoEncoder & operator=(const VideoEncoder &) = delete;

  void configure(const StreamConfig & cfg);
  /// Returns false if the frame was dropped (pipeline unavailable).
  bool push(RawFrame frame);
  void requestKeyframe();
  void stop();

private:
  struct Meta
  {
    std::uint64_t pts = GST_CLOCK_TIME_NONE;
    std::int64_t stamp_ns = 0;
    std::int64_t recv_ns = 0;
    std::int64_t sim_age_ns = 0;
  };
  /// Description of the current stream, read on the streaming thread.
  struct StreamInfo
  {
    Codec codec = Codec::kH264;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t src_width = 0;
    std::uint32_t src_height = 0;
    bool hardware = false;
    bool announced = false;
  };

  bool build(const RawFrame & first);
  void teardown();
  std::string encoderDescription(bool hardware) const;
  void setBitrate(int kbps);
  static GstFlowReturn onNewSample(GstElement * sink, gpointer self);
  void handleSample(GstSample * sample);

  FrameHandler on_frame_;
  InfoHandler on_info_;

  // mutex_: pipeline state (ROS thread and GCS thread). meta_mutex_: what the
  // streaming thread reads; separate because teardown waits for that thread holding mutex_.
  std::mutex mutex_;
  StreamConfig cfg_;
  GstElement * pipeline_ = nullptr;
  GstElement * appsrc_ = nullptr;
  GstElement * encoder_ = nullptr;
  GstElement * appsink_ = nullptr;
  bool hardware_ = false;
  bool nvenc_ = false;           ///< hardware encoder is NVENC (nvh26xenc), not VA-API
  bool hw_failed_ = false;       ///< hardware failed in this configuration: uses software
  std::string in_format_;
  std::uint32_t in_width_ = 0;
  std::uint32_t in_height_ = 0;
  std::uint32_t out_width_ = 0;
  std::uint32_t out_height_ = 0;
  std::uint64_t base_ns_ = 0;
  std::uint64_t last_pts_ = 0;

  std::mutex meta_mutex_;
  std::deque<Meta> meta_;        ///< Frames in the encoder, in input order
  StreamInfo stream_;
};

}  // namespace aero_pursuit::streaming
