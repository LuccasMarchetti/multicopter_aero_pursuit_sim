#include "aero_pursuit/perception/image_conversions.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cstring>

namespace aero_pursuit::perception
{

namespace enc = sensor_msgs::image_encodings;

cv::Mat wrapImage(const sensor_msgs::msg::Image & msg)
{
  int type = -1;
  if (msg.encoding == enc::RGB8 || msg.encoding == enc::BGR8) {
    type = CV_8UC3;
  } else if (msg.encoding == enc::RGBA8 || msg.encoding == enc::BGRA8) {
    type = CV_8UC4;
  } else if (msg.encoding == enc::MONO8) {
    type = CV_8UC1;
  } else if (msg.encoding == enc::TYPE_32FC1) {
    type = CV_32FC1;
  } else if (msg.encoding == enc::TYPE_16UC1 || msg.encoding == enc::MONO16) {
    type = CV_16UC1;
  }
  if (type < 0 || msg.data.empty()) {
    return {};
  }
  // Safe const_cast: the Mat is only read.
  return cv::Mat(
    static_cast<int>(msg.height), static_cast<int>(msg.width), type,
    const_cast<uint8_t *>(msg.data.data()), msg.step);
}

bool toBgr(const sensor_msgs::msg::Image & msg, cv::Mat & out)
{
  const cv::Mat src = wrapImage(msg);
  if (src.empty()) {
    return false;
  }
  if (msg.encoding == enc::RGB8) {
    cv::cvtColor(src, out, cv::COLOR_RGB2BGR);
  } else if (msg.encoding == enc::BGR8) {
    src.copyTo(out);
  } else if (msg.encoding == enc::RGBA8) {
    cv::cvtColor(src, out, cv::COLOR_RGBA2BGR);
  } else if (msg.encoding == enc::BGRA8) {
    cv::cvtColor(src, out, cv::COLOR_BGRA2BGR);
  } else if (msg.encoding == enc::MONO8) {
    cv::cvtColor(src, out, cv::COLOR_GRAY2BGR);
  } else {
    return false;
  }
  return true;
}

cv::Mat toDepthMeters(const sensor_msgs::msg::Image & msg, cv::Mat & conversion_buffer)
{
  const cv::Mat src = wrapImage(msg);
  if (src.type() == CV_32FC1) {
    return src;
  }
  if (src.type() == CV_16UC1) {
    src.convertTo(conversion_buffer, CV_32FC1, 0.001);
    return conversion_buffer;
  }
  return {};
}

void toImageMsg(
  const cv::Mat & bgr, const std_msgs::msg::Header & header, sensor_msgs::msg::Image & out)
{
  out.header = header;
  out.height = static_cast<uint32_t>(bgr.rows);
  out.width = static_cast<uint32_t>(bgr.cols);
  out.encoding = enc::BGR8;
  out.is_bigendian = false;
  out.step = static_cast<uint32_t>(bgr.cols * bgr.elemSize());
  out.data.resize(static_cast<size_t>(out.step) * out.height);
  if (bgr.isContinuous()) {
    std::memcpy(out.data.data(), bgr.data, out.data.size());
  } else {
    for (int r = 0; r < bgr.rows; ++r) {
      std::memcpy(out.data.data() + r * out.step, bgr.ptr(r), out.step);
    }
  }
}

bool fromCompressedMsg(const sensor_msgs::msg::CompressedImage & msg, cv::Mat & out)
{
  if (msg.data.empty()) {
    return false;
  }
  out = cv::imdecode(msg.data, cv::IMREAD_COLOR);
  return !out.empty();
}

bool toCompressedMsg(
  const cv::Mat & bgr, const std_msgs::msg::Header & header, int jpeg_quality,
  sensor_msgs::msg::CompressedImage & out)
{
  out.header = header;
  out.format = "jpeg";
  return cv::imencode(".jpg", bgr, out.data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality});
}

bool toCompressedDepthMsg(
  const sensor_msgs::msg::Image & depth, int png_level, cv::Mat & buffer,
  sensor_msgs::msg::CompressedImage & out)
{
  const cv::Mat src = wrapImage(depth);
  cv::Mat mm;
  if (src.type() == CV_32FC1) {
    src.copyTo(buffer);
    cv::patchNaNs(buffer, 0.0);
    cv::min(buffer, 65.535, buffer);  // +inf -> 65535 mm
    buffer.convertTo(mm, CV_16UC1, 1000.0);
  } else if (src.type() == CV_16UC1) {
    mm = src;
  } else {
    return false;
  }
  out.header = depth.header;
  out.format = "16UC1; png";
  return cv::imencode(".png", mm, out.data, {cv::IMWRITE_PNG_COMPRESSION, png_level});
}

bool fromCompressedDepthMsg(
  const sensor_msgs::msg::CompressedImage & msg, sensor_msgs::msg::Image & out)
{
  if (msg.data.empty()) {
    return false;
  }
  const cv::Mat mm = cv::imdecode(msg.data, cv::IMREAD_UNCHANGED);
  if (mm.empty() || mm.type() != CV_16UC1) {
    return false;
  }
  out.header = msg.header;
  out.height = static_cast<uint32_t>(mm.rows);
  out.width = static_cast<uint32_t>(mm.cols);
  out.encoding = enc::TYPE_16UC1;
  out.is_bigendian = false;
  out.step = static_cast<uint32_t>(mm.cols * mm.elemSize());
  out.data.assign(mm.datastart, mm.dataend);
  return true;
}

}  // namespace aero_pursuit::perception
