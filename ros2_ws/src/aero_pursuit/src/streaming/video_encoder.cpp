#include "aero_pursuit/streaming/video_encoder.hpp"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sstream>
#include <utility>

namespace aero_pursuit::streaming
{
namespace
{

std::int64_t systemNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::uint64_t steadyNowNs()
{
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::uint32_t even(double v)
{
  const auto i = static_cast<std::uint32_t>(v + 0.5);
  return std::max<std::uint32_t>(16, i & ~1u);
}

const char * rateControlName(RateControl rc)
{
  switch (rc) {
    case RateControl::kVbr: return "vbr";
    case RateControl::kCqp: return "cqp";
    case RateControl::kCbr: break;
  }
  return "cbr";
}

bool elementExists(const char * name)
{
  GstElementFactory * f = gst_element_factory_find(name);
  if (!f) {return false;}
  gst_object_unref(f);
  return true;
}

/// "4.1" -> 41 (H.264) ; "5.1" -> 153 (H.265, 30 * level).
int parseLevel(const char * level, int major_mult, int minor_mult)
{
  if (!level) {return 0;}
  int major = 0;
  int minor = 0;
  if (std::sscanf(level, "%d.%d", &major, &minor) < 1) {return 0;}
  return major * major_mult + minor * minor_mult;
}

/// WebCodecs codec string from the parser caps.
std::string codecString(Codec codec, const GstStructure * s)
{
  char buf[64];
  if (codec == Codec::kH264) {
    const char * profile = s ? gst_structure_get_string(s, "profile") : nullptr;
    int idc = 0x64;
    int constraints = 0x00;
    if (profile) {
      const std::string p(profile);
      if (p == "constrained-baseline") {idc = 0x42; constraints = 0xE0;}
      else if (p == "baseline") {idc = 0x42;}
      else if (p == "main") {idc = 0x4D; constraints = 0x40;}
      else if (p == "high-10") {idc = 0x6E;}
      else if (p == "high-4:2:2") {idc = 0x7A;}
      else if (p == "high-4:4:4") {idc = 0xF4;}
    }
    int level = parseLevel(s ? gst_structure_get_string(s, "level") : nullptr, 10, 1);
    if (level == 0) {level = 51;}
    std::snprintf(buf, sizeof(buf), "avc1.%02X%02X%02X", idc, constraints, level);
    return buf;
  }
  if (codec == Codec::kH265) {
    const char * profile = s ? gst_structure_get_string(s, "profile") : nullptr;
    const char * tier = s ? gst_structure_get_string(s, "tier") : nullptr;
    const bool main10 = profile && std::strcmp(profile, "main-10") == 0;
    int level = parseLevel(s ? gst_structure_get_string(s, "level") : nullptr, 30, 3);
    if (level == 0) {level = 153;}
    std::snprintf(buf, sizeof(buf), "hev1.%d.%d.%c%d.B0", main10 ? 2 : 1, main10 ? 4 : 6,
      (tier && std::strcmp(tier, "high") == 0) ? 'H' : 'L', level);
    return buf;
  }
  return "mjpeg";
}

}  // namespace

VideoEncoder::VideoEncoder(FrameHandler on_frame, InfoHandler on_info)
: on_frame_(std::move(on_frame)), on_info_(std::move(on_info))
{
  static std::once_flag once;
  std::call_once(once, [] {gst_init(nullptr, nullptr);});
}

VideoEncoder::~VideoEncoder()
{
  stop();
}

void VideoEncoder::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  teardown();
}

void VideoEncoder::configure(const StreamConfig & cfg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const StreamConfig old = cfg_;
  cfg_ = cfg;
  if (cfg.backend != old.backend || cfg.codec != old.codec) {
    hw_failed_ = false;  // explicit new attempt by the user
  }
  if (!cfg.active) {
    teardown();
    return;
  }
  if (!pipeline_) {return;}  // built on the next frame
  if (cfg.needsRebuild(old)) {
    teardown();
  } else if (cfg.bitrate_kbps != old.bitrate_kbps && cfg.codec != Codec::kMjpeg) {
    setBitrate(cfg.bitrate_kbps);
  }
}

void VideoEncoder::setBitrate(int kbps)
{
  if (!encoder_) {return;}
  GParamSpec * spec = g_object_class_find_property(G_OBJECT_GET_CLASS(encoder_), "bitrate");
  if (spec && (spec->flags & GST_PARAM_MUTABLE_PLAYING)) {
    g_object_set(encoder_, "bitrate", static_cast<guint>(kbps), nullptr);
  } else {
    teardown();  // encoder without live tuning: rebuilds on the next frame
  }
}

std::string VideoEncoder::encoderDescription(bool hardware) const
{
  const auto & c = cfg_;
  std::ostringstream e;
  const bool cqp = c.rate_control == RateControl::kCqp;
  const int qp = std::clamp(c.quality, 0, 51);
  switch (c.codec) {
    case Codec::kH264:
    case Codec::kH265: {
      const bool h264 = c.codec == Codec::kH264;
      if (hardware && nvenc_) {
        // NVENC (NVIDIA, gst nvcodec): low-latency preset, no B-frames, no reordering delay.
        e << (h264 ? "nvh264enc" : "nvh265enc") << " name=enc preset=low-latency-hp"
          << " zerolatency=true bframes=0 gop-size=" << c.gop;
        if (cqp) {
          e << " rc-mode=constqp qp-const=" << qp;
        } else {
          e << " rc-mode=" << rateControlName(c.rate_control) << " bitrate=" << c.bitrate_kbps;
          if (c.rate_control == RateControl::kCbr) {e << " vbv-buffer-size=" << c.bitrate_kbps / 2;}
        }
      } else if (hardware) {
        // No B-frames, 1 reference and target-usage 7 (fastest): 1 frame of delay.
        e << (h264 ? "vah264enc" : "vah265enc") << " name=enc rate-control="
          << rateControlName(c.rate_control) << " key-int-max=" << c.gop
          << " b-frames=0 ref-frames=1 target-usage=7";
        if (cqp) {
          e << " qpi=" << qp << " qpp=" << qp << " qpb=" << qp;
        } else {
          e << " bitrate=" << c.bitrate_kbps;
          // Short CPB: limits the size of the peaks (keyframes) and the network delay.
          if (c.rate_control == RateControl::kCbr) {e << " cpb-size=" << c.bitrate_kbps / 2;}
        }
      } else if (h264) {
        e << "x264enc name=enc tune=zerolatency speed-preset=superfast bframes=0 threads=0"
          << " sliced-threads=true key-int-max=" << c.gop;
        if (cqp) {
          e << " pass=quant quantizer=" << qp;
        } else {
          e << " pass=cbr bitrate=" << c.bitrate_kbps << " vbv-buf-capacity="
            << (c.rate_control == RateControl::kCbr ? 500 : 1500);
        }
      } else {
        e << "x265enc name=enc tune=zerolatency speed-preset=superfast key-int-max=" << c.gop
          << " option-string=\"bframes=0\"";
        if (cqp) {e << " qp=" << qp;} else {e << " bitrate=" << c.bitrate_kbps;}
      }
      e << (h264 ? " ! h264parse config-interval=-1 ! video/x-h264" :
        " ! h265parse config-interval=-1 ! video/x-h265")
        << ",stream-format=byte-stream,alignment=au";
      break;
    }
    case Codec::kMjpeg:
      e << "jpegenc name=enc idct-method=ifast quality=" << std::clamp(c.quality, 1, 100);
      break;
  }
  return e.str();
}

bool VideoEncoder::build(const RawFrame & first)
{
  const bool want_hw = cfg_.codec != Codec::kMjpeg && cfg_.backend != Backend::kSoftware &&
    !(cfg_.backend == Backend::kAuto && hw_failed_);
  // VA-API (Intel/AMD) first, else NVENC (NVIDIA; in a container it needs the driver mounted,
  // docker/aero.sh). A VA element exists only when a VA device was found.
  const bool h264 = cfg_.codec == Codec::kH264;
  const char * hw_name = h264 ? "vah264enc" : "vah265enc";
  nvenc_ = false;
  if (want_hw && !elementExists(hw_name) && elementExists(h264 ? "nvh264enc" : "nvh265enc")) {
    hw_name = h264 ? "nvh264enc" : "nvh265enc";
    nvenc_ = true;
  }
  hardware_ = want_hw && elementExists(hw_name);
  if (want_hw && !hardware_ && cfg_.backend == Backend::kHardware && on_info_) {
    on_info_(true, std::string("hardware encoder unavailable: ") + hw_name);
  }

  in_format_ = first.format;
  in_width_ = first.width;
  in_height_ = first.height;
  out_width_ = even(first.width * cfg_.scale);
  out_height_ = even(first.height * cfg_.scale);
  // VA-API and NVENC accept NV12 from system memory; the software encoders, I420.
  const char * out_format = hardware_ ? "NV12" : "I420";
  const int fps = cfg_.max_fps > 0.0 ? static_cast<int>(cfg_.max_fps + 0.5) : 30;

  std::ostringstream p;
  p << "appsrc name=src is-live=true format=time block=false max-buffers=2"
    << " caps=video/x-raw,format=" << first.format
    << ",width=" << first.width << ",height=" << first.height << ",framerate=" << fps << "/1"
    << " ! videoconvertscale n-threads=4 add-borders=false"
    << " ! video/x-raw,format=" << out_format << ",width=" << out_width_
    << ",height=" << out_height_
    << " ! " << encoderDescription(hardware_)
    << " ! appsink name=sink sync=false async=false max-buffers=1 drop=false emit-signals=true";

  GError * err = nullptr;
  GstElement * pipeline = gst_parse_launch(p.str().c_str(), &err);
  if (!pipeline || err) {
    const std::string msg = err ? err->message : "gst_parse_launch failed";
    if (err) {g_error_free(err);}
    if (pipeline) {gst_object_unref(pipeline);}
    if (on_info_) {on_info_(true, "invalid pipeline: " + msg);}
    if (hardware_) {hw_failed_ = true; return build(first);}
    return false;
  }
  pipeline_ = pipeline;
  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
  encoder_ = gst_bin_get_by_name(GST_BIN(pipeline_), "enc");
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
  g_signal_connect(appsink_, "new-sample", G_CALLBACK(&VideoEncoder::onNewSample), this);

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    teardown();
    if (on_info_) {on_info_(true, std::string("could not start ") + (hardware_ ? hw_name : "software"));}
    if (hardware_) {hw_failed_ = true; return build(first);}
    return false;
  }
  base_ns_ = steadyNowNs();
  last_pts_ = 0;
  {
    std::lock_guard<std::mutex> lock(meta_mutex_);
    meta_.clear();
    stream_ = StreamInfo{cfg_.codec, out_width_, out_height_, in_width_, in_height_,
      hardware_, false};
  }
  return true;
}

void VideoEncoder::teardown()
{
  if (!pipeline_) {return;}
  gst_element_set_state(pipeline_, GST_STATE_NULL);  // waits for the streaming threads
  for (GstElement ** e : {&appsrc_, &encoder_, &appsink_}) {
    if (*e) {gst_object_unref(*e); *e = nullptr;}
  }
  gst_object_unref(pipeline_);
  pipeline_ = nullptr;
}

bool VideoEncoder::push(RawFrame frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!cfg_.active || !frame.data || frame.width == 0 || frame.height == 0) {return false;}
  if (pipeline_ && (frame.format != in_format_ || frame.width != in_width_ ||
    frame.height != in_height_))
  {
    teardown();  // source resolution/format changed
  }
  if (!pipeline_ && !build(frame)) {return false;}
  // Encoder still busy with the previous frame: drops this one (the newest
  // comes next) instead of queueing latency. Dropping here, and not at appsrc,
  // keeps the 1:1 input/output correspondence used by the metadata.
  if (gst_app_src_get_current_level_buffers(GST_APP_SRC(appsrc_)) > 0) {return false;}

  GstVideoInfo info;
  if (!gst_video_info_set_format(&info, gst_video_format_from_string(frame.format.c_str()),
    frame.width, frame.height))
  {
    return false;
  }
  const auto packed_stride = static_cast<std::uint32_t>(GST_VIDEO_INFO_PLANE_STRIDE(&info, 0));
  GstBuffer * buffer = nullptr;
  if (frame.stride == packed_stride && frame.size >= GST_VIDEO_INFO_SIZE(&info)) {
    // Zero copy: the buffer points to the ROS message, kept alive by keepalive.
    auto * hold = new std::shared_ptr<const void>(std::move(frame.keepalive));
    buffer = gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY,
      const_cast<std::uint8_t *>(frame.data), GST_VIDEO_INFO_SIZE(&info), 0,
      GST_VIDEO_INFO_SIZE(&info), hold,
      [](gpointer p) {delete static_cast<std::shared_ptr<const void> *>(p);});
  } else {
    // Padded stride: copies row by row into the compact layout.
    buffer = gst_buffer_new_allocate(nullptr, GST_VIDEO_INFO_SIZE(&info), nullptr);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    const std::size_t row = std::min<std::size_t>(packed_stride, frame.stride);
    for (std::uint32_t r = 0; r < frame.height; ++r) {
      std::memcpy(map.data + r * packed_stride, frame.data + r * frame.stride, row);
    }
    gst_buffer_unmap(buffer, &map);
  }

  std::uint64_t pts = steadyNowNs() - base_ns_;
  if (pts <= last_pts_) {pts = last_pts_ + 1000;}
  last_pts_ = pts;
  GST_BUFFER_PTS(buffer) = pts;
  GST_BUFFER_DTS(buffer) = pts;
  {
    std::lock_guard<std::mutex> meta_lock(meta_mutex_);
    meta_.push_back(Meta{pts, frame.stamp_ns, frame.recv_ns, frame.sim_age_ns});
    while (meta_.size() > 16) {meta_.pop_front();}  // out of sync: start over
  }
  return gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer) == GST_FLOW_OK;
}

void VideoEncoder::requestKeyframe()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!appsink_) {return;}
  gst_element_send_event(appsink_,
    gst_video_event_new_upstream_force_key_unit(GST_CLOCK_TIME_NONE, TRUE, 0));
}

GstFlowReturn VideoEncoder::onNewSample(GstElement * sink, gpointer self)
{
  GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
  if (!sample) {return GST_FLOW_OK;}
  static_cast<VideoEncoder *>(self)->handleSample(sample);
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

void VideoEncoder::handleSample(GstSample * sample)
{
  // GStreamer streaming thread: does not take mutex_ (teardown holds it while
  // waiting for this thread to finish).
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  if (!buffer) {return;}
  EncodedFrame out;
  out.encoded_ns = systemNowNs();
  out.keyframe = !GST_BUFFER_FLAG_IS_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);
  bool announce_now = false;
  StreamInfo stream;
  {
    std::lock_guard<std::mutex> lock(meta_mutex_);
    // Without B-frames, every frame enters and leaves the encoder in the same order (dropping
    // happens before appsrc, in push()). Matching by PTS does not work: x264/VA
    // rewrite the PTS from the framerate and add 1000 h (set_min_pts).
    if (!meta_.empty()) {
      const Meta & m = meta_.front();
      out.stamp_ns = m.stamp_ns;
      out.recv_ns = m.recv_ns;
      out.sim_age_ns = m.sim_age_ns;
      meta_.pop_front();
    }
    announce_now = !stream_.announced;
    stream_.announced = true;
    stream = stream_;
  }
  if (announce_now && on_info_) {
    GstCaps * caps = gst_sample_get_caps(sample);
    const GstStructure * s = caps ? gst_caps_get_structure(caps, 0) : nullptr;
    std::ostringstream info;
    info << "codec=" << codecString(stream.codec, s) << " name=" << codecName(stream.codec)
         << " hardware=" << (stream.hardware ? 1 : 0)
         << " width=" << stream.width << " height=" << stream.height
         << " src_width=" << stream.src_width << " src_height=" << stream.src_height;
    on_info_(false, info.str());
  }
  out.width = stream.width;
  out.height = stream.height;
  out.src_width = stream.src_width;
  out.src_height = stream.src_height;
  out.codec = stream.codec;

  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {return;}
  out.data = map.data;
  out.size = map.size;
  if (on_frame_) {on_frame_(out);}
  gst_buffer_unmap(buffer, &map);
}

}  // namespace aero_pursuit::streaming
