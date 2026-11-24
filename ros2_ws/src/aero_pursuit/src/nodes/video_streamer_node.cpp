#include "aero_pursuit/nodes/video_streamer_node.hpp"

#include "aero_pursuit/streaming/frame_packet.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <string>

namespace aero_pursuit
{
namespace
{

std::int64_t systemNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// One socket per channel (drone video, free camera...): see the GCS video_hub.py.
std::string defaultSocketPath(const std::string & channel)
{
  const std::string file = channel == "main" ? "video.sock" : "video_" + channel + ".sock";
  if (const char * xdg = std::getenv("XDG_RUNTIME_DIR"); xdg && *xdg) {
    return std::string(xdg) + "/aero_gcs/" + file;
  }
  return "/tmp/aero_gcs-" + std::to_string(::getuid()) + "/" + file;
}

/// ROS encoding -> GStreamer format (empty: needs conversion).
std::string gstFormat(const std::string & encoding)
{
  if (encoding == "rgb8") {return "RGB";}
  if (encoding == "bgr8") {return "BGR";}
  if (encoding == "rgba8") {return "RGBA";}
  if (encoding == "bgra8") {return "BGRA";}
  if (encoding == "mono8" || encoding == "8UC1") {return "GRAY8";}
  return {};
}

/// Depth (32FC1 in m or 16UC1 in mm) -> TURBO color map; invalid ones in black.
std::shared_ptr<cv::Mat> colorizeDepth(const sensor_msgs::msg::Image & msg, double dmin, double dmax)
{
  const bool f32 = msg.encoding == "32FC1";
  const bool u16 = msg.encoding == "16UC1" || msg.encoding == "mono16";
  if (!f32 && !u16) {return nullptr;}
  const cv::Mat src(static_cast<int>(msg.height), static_cast<int>(msg.width),
    f32 ? CV_32FC1 : CV_16UC1, const_cast<std::uint8_t *>(msg.data.data()), msg.step);
  const double unit = f32 ? 1.0 : 1000.0;  // 16UC1 in millimeters
  const double alpha = 255.0 / ((dmax - dmin) * unit);
  cv::Mat u8;
  src.convertTo(u8, CV_8UC1, alpha, -dmin * unit * alpha);
  auto out = std::make_shared<cv::Mat>();
  cv::applyColorMap(u8, *out, cv::COLORMAP_TURBO);
  cv::Mat invalid;
  if (f32) {
    cv::compare(src, src, invalid, cv::CMP_NE);  // NaN
    cv::Mat nonpos;
    cv::compare(src, 0.0, nonpos, cv::CMP_LE);
    invalid |= nonpos;
  } else {
    cv::compare(src, 0, invalid, cv::CMP_EQ);
  }
  out->setTo(cv::Scalar::all(0), invalid);
  return out;
}

}  // namespace

VideoStreamerNode::VideoStreamerNode(const rclcpp::NodeOptions & options)
: Node("video_streamer", options)
{
  const auto socket_path = declare_parameter<std::string>("socket_path", "");
  const auto channel = declare_parameter<std::string>("channel", "main");
  pending_.topic = declare_parameter<std::string>("default_topic", "/camera");

  encoder_ = std::make_unique<streaming::VideoEncoder>(
    [this](const streaming::EncodedFrame & f) {onEncoded(f);},
    [this](bool error, const std::string & text) {
      sendText(error, text);
      if (error) {RCLCPP_WARN(get_logger(), "Streamer: %s", text.c_str());}
    });

  link_ = std::make_unique<streaming::GcsLink>(
    socket_path.empty() ? defaultSocketPath(channel) : socket_path,
    [this](std::string_view line) {onLine(line);},
    [this](bool connected) {
      RCLCPP_INFO(get_logger(), "GCS %s", connected ? "conectada" : "desconectada");
      if (!connected) {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_.active = false;  // no GCS, no encoding
        dirty_ = true;
      }
    });

  // Subscriptions are created/removed on the executor thread.
  apply_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {applyPending();});
}

VideoStreamerNode::~VideoStreamerNode()
{
  encoder_->stop();  // no frame callbacks from here on
  link_.reset();
}

void VideoStreamerNode::onLine(std::string_view line)
{
  if (line == "key") {
    encoder_->requestKeyframe();
    return;
  }
  if (line.rfind("cfg ", 0) == 0) {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    streaming::applyConfigLine(line.substr(4), pending_);
    dirty_ = true;
  }
}

void VideoStreamerNode::applyPending()
{
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!dirty_) {return;}
    cfg_ = pending_;
    dirty_ = false;
  }
  encoder_->configure(cfg_);

  const std::string wanted = cfg_.active ? cfg_.topic : std::string();
  if (wanted == subscribed_topic_) {return;}
  sub_.reset();
  subscribed_topic_ = wanted;
  if (wanted.empty()) {
    RCLCPP_INFO(get_logger(), "Stream stopped");
    return;
  }
  // Queue of 1 and best-effort: always the newest frame; with intra-process, no copy.
  sub_ = create_subscription<Image>(wanted, rclcpp::SensorDataQoS().keep_last(1),
    [this](std::shared_ptr<const Image> msg) {onImage(msg);});
  RCLCPP_INFO(get_logger(), "Streaming %s", wanted.c_str());
}

void VideoStreamerNode::onImage(const std::shared_ptr<const Image> & msg)
{
  const std::int64_t now_steady = steadyNowNs();
  if (cfg_.max_fps > 0.0) {
    // Drift-free schedule with a 1/4-period tolerance: with equal source and limit
    // (30 Hz and 30 fps) no frame is dropped because of arrival jitter.
    const auto period = static_cast<std::int64_t>(1e9 / cfg_.max_fps);
    if (now_steady < next_frame_ns_ - period / 4) {return;}
    next_frame_ns_ = std::max(next_frame_ns_ + period, now_steady - period / 4);
  }

  streaming::RawFrame frame;
  frame.recv_ns = systemNowNs();
  frame.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  frame.sim_age_ns = now().nanoseconds() - frame.stamp_ns;

  if (std::string fmt = gstFormat(msg->encoding); !fmt.empty()) {
    frame.format = std::move(fmt);
    frame.data = msg->data.data();
    frame.size = msg->data.size();
    frame.width = msg->width;
    frame.height = msg->height;
    frame.stride = msg->step;
    frame.keepalive = msg;
  } else if (auto color = colorizeDepth(*msg, cfg_.depth_min, cfg_.depth_max)) {
    frame.format = "BGR";
    frame.data = color->data;
    frame.size = color->total() * color->elemSize();
    frame.width = static_cast<std::uint32_t>(color->cols);
    frame.height = static_cast<std::uint32_t>(color->rows);
    frame.stride = static_cast<std::uint32_t>(color->step);
    frame.keepalive = color;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Encoding %s not supported by the streamer", msg->encoding.c_str());
    return;
  }
  encoder_->push(std::move(frame));
}

void VideoStreamerNode::onEncoded(const streaming::EncodedFrame & f)
{
  streaming::FrameHeader h;
  h.type = streaming::FrameHeader::kTypeFrame;
  h.flags = f.keyframe ? 1 : 0;
  h.codec = static_cast<std::uint8_t>(f.codec);
  h.width = f.width;
  h.height = f.height;
  h.src_width = f.src_width;
  h.src_height = f.src_height;
  h.seq = seq_++;
  h.payload_size = static_cast<std::uint32_t>(f.size);
  h.stamp_ns = f.stamp_ns;
  h.recv_ns = f.recv_ns;
  h.encoded_ns = f.encoded_ns;
  h.sim_age_ns = f.sim_age_ns;
  const auto bytes = h.serialize();
  if (link_) {link_->send(bytes.data(), bytes.size(), f.data, f.size);}
}

void VideoStreamerNode::sendText(bool error, const std::string & text)
{
  if (!link_) {return;}
  streaming::FrameHeader h;
  h.type = error ? streaming::FrameHeader::kTypeError : streaming::FrameHeader::kTypeInfo;
  h.payload_size = static_cast<std::uint32_t>(text.size());
  const auto bytes = h.serialize();
  link_->send(bytes.data(), bytes.size(), text.data(), text.size());
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::VideoStreamerNode)
