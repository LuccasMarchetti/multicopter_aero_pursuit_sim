#include "aero_pursuit/nodes/image_downscaler_node.hpp"

#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>

#include "aero_pursuit/perception/image_conversions.hpp"

namespace aero_pursuit
{

namespace
{
/// Image message from a cv::Mat, with the header and encoding of the source.
std::unique_ptr<sensor_msgs::msg::Image> toMsg(
  const cv::Mat & m, const std_msgs::msg::Header & header, const std::string & encoding)
{
  auto out = std::make_unique<sensor_msgs::msg::Image>();
  out->header = header;
  out->height = static_cast<uint32_t>(m.rows);
  out->width = static_cast<uint32_t>(m.cols);
  out->encoding = encoding;
  out->step = static_cast<uint32_t>(m.cols * m.elemSize());
  out->data.resize(out->step * out->height);
  for (int r = 0; r < m.rows; ++r) {
    std::copy(m.ptr<uint8_t>(r), m.ptr<uint8_t>(r) + out->step, out->data.begin() + r * out->step);
  }
  return out;
}
}  // namespace

ImageDownscalerNode::ImageDownscalerNode(const rclcpp::NodeOptions & options)
: Node("image_downscaler", options)
{
  const auto in_image = declare_parameter<std::string>("input_topic", "/camera");
  const auto in_info = declare_parameter<std::string>("input_info_topic", "/camera/camera_info");
  const auto out_image = declare_parameter<std::string>("output_topic", "/camera/remote");
  const auto out_info = declare_parameter<std::string>("output_info_topic", "/camera/remote/camera_info");
  const auto in_depth = declare_parameter<std::string>("depth_input_topic", "/depth_camera");
  const auto in_depth_info = declare_parameter<std::string>("depth_input_info_topic", "/depth_camera/camera_info");
  const auto out_depth = declare_parameter<std::string>("depth_output_topic", "/depth_camera/remote");
  const auto out_depth_info = declare_parameter<std::string>(
    "depth_output_info_topic", "/depth_camera/remote/camera_info");
  scale_ = std::clamp(declare_parameter<double>("scale", 0.5), 0.05, 1.0);
  depth_scale_ = std::clamp(declare_parameter<double>("depth_scale", 0.5), 0.05, 1.0);
  jpeg_quality_ = std::clamp(static_cast<int>(declare_parameter<int>("jpeg_quality", 80)), 0, 100);
  depth_png_level_ = std::clamp(static_cast<int>(declare_parameter<int>("depth_png_level", 1)), -1, 9);
  const double rate = declare_parameter<double>("max_rate_hz", 0.0);
  min_period_ = rate > 0.0 ? 1.0 / rate : 0.0;

  // Reliable, queue of 1 (as the bridge): best-effort subscribers still match.
  const auto qos = rclcpp::QoS(1).reliable();
  if (jpeg_quality_ > 0) {
    pub_compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>(out_image + "/compressed", qos);
  } else {
    pub_image_ = create_publisher<Image>(out_image, qos);
  }
  pub_info_ = create_publisher<CameraInfo>(out_info, qos);
  if (depth_png_level_ >= 0) {
    pub_depth_compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>(out_depth + "/compressed", qos);
  } else {
    pub_depth_ = create_publisher<Image>(out_depth, qos);
  }
  pub_depth_info_ = create_publisher<CameraInfo>(out_depth_info, qos);

  const auto in_qos = rclcpp::SensorDataQoS().keep_last(1);
  sub_image_ = create_subscription<Image>(in_image, in_qos, [this](Image::UniquePtr m) {onImage(std::move(m));});
  sub_depth_ = create_subscription<Image>(in_depth, in_qos, [this](Image::UniquePtr m) {onDepth(std::move(m));});
  sub_info_ = create_subscription<CameraInfo>(
    in_info, rclcpp::SensorDataQoS(), [this](const CameraInfo & m) {pub_info_->publish(scaled(m, scale_));});
  sub_depth_info_ = create_subscription<CameraInfo>(
    in_depth_info, rclcpp::SensorDataQoS(),
    [this](const CameraInfo & m) {pub_depth_info_->publish(scaled(m, depth_scale_));});
}

ImageDownscalerNode::CameraInfo ImageDownscalerNode::scaled(const CameraInfo & info, double scale)
{
  CameraInfo out = info;
  out.width = static_cast<uint32_t>(std::lround(info.width * scale));
  out.height = static_cast<uint32_t>(std::lround(info.height * scale));
  // Pixel centers: x' = s (x + 0.5) - 0.5.
  for (int r : {0, 1}) {
    out.k[3 * r + 0] = info.k[3 * r + 0] * scale;
    out.k[3 * r + 1] = info.k[3 * r + 1] * scale;
    out.k[3 * r + 2] = (info.k[3 * r + 2] + 0.5) * scale - 0.5;
    out.p[4 * r + 0] = info.p[4 * r + 0] * scale;
    out.p[4 * r + 1] = info.p[4 * r + 1] * scale;
    out.p[4 * r + 2] = (info.p[4 * r + 2] + 0.5) * scale - 0.5;
    out.p[4 * r + 3] = info.p[4 * r + 3] * scale;
  }
  out.roi.x_offset = static_cast<uint32_t>(info.roi.x_offset * scale);
  out.roi.y_offset = static_cast<uint32_t>(info.roi.y_offset * scale);
  out.roi.width = static_cast<uint32_t>(info.roi.width * scale);
  out.roi.height = static_cast<uint32_t>(info.roi.height * scale);
  return out;
}

void ImageDownscalerNode::onImage(Image::UniquePtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (min_period_ > 0.0 && last_out_.nanoseconds() > 0 && (stamp - last_out_).seconds() < min_period_) {
    return;
  }
  last_out_ = stamp;
  if (!pub_compressed_ && scale_ >= 0.999) {
    pub_image_->publish(std::move(msg));
    return;
  }
  if (!perception::toBgr(*msg, bgr_)) {
    return;
  }
  const cv::Mat * out = &bgr_;
  if (scale_ < 0.999) {
    cv::resize(bgr_, resized_, cv::Size(), scale_, scale_, cv::INTER_AREA);
    out = &resized_;
  }
  if (pub_compressed_) {
    sensor_msgs::msg::CompressedImage c;
    if (perception::toCompressedMsg(*out, msg->header, jpeg_quality_, c)) {
      pub_compressed_->publish(std::move(c));
    }
  } else {
    pub_image_->publish(toMsg(*out, msg->header, "bgr8"));
  }
}

void ImageDownscalerNode::onDepth(Image::UniquePtr msg)
{
  if (depth_scale_ < 0.999) {
    const cv::Mat in = perception::wrapImage(*msg);
    if (in.empty()) {
      return;
    }
    cv::resize(in, depth_resized_, cv::Size(), depth_scale_, depth_scale_, cv::INTER_NEAREST);
    msg = toMsg(depth_resized_, msg->header, msg->encoding);
  }
  if (pub_depth_compressed_) {
    sensor_msgs::msg::CompressedImage c;
    if (perception::toCompressedDepthMsg(*msg, depth_png_level_, depth_buffer_, c)) {
      pub_depth_compressed_->publish(std::move(c));
    }
  } else {
    pub_depth_->publish(std::move(msg));
  }
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::ImageDownscalerNode)
