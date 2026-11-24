// video_streamer protocol and encoder (GCS).

#include "aero_pursuit/streaming/frame_packet.hpp"
#include "aero_pursuit/streaming/stream_config.hpp"
#include "aero_pursuit/streaming/video_encoder.hpp"

#include <gst/gst.h>
#include <gtest/gtest.h>

#include <chrono>
#include <clocale>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace aero_pursuit::streaming;

TEST(StreamConfig, ParsesAllFields)
{
  StreamConfig c;
  applyConfigLine("active=1 topic=/depth_camera codec=h265 backend=software scale=0.5 fps=15 "
    "bitrate=4000 rc=cqp quality=30 gop=10 depth_min=0.5 depth_max=12", c);
  EXPECT_TRUE(c.active);
  EXPECT_EQ(c.topic, "/depth_camera");
  EXPECT_EQ(c.codec, Codec::kH265);
  EXPECT_EQ(c.backend, Backend::kSoftware);
  EXPECT_DOUBLE_EQ(c.scale, 0.5);
  EXPECT_DOUBLE_EQ(c.max_fps, 15.0);
  EXPECT_EQ(c.bitrate_kbps, 4000);
  EXPECT_EQ(c.rate_control, RateControl::kCqp);
  EXPECT_EQ(c.quality, 30);
  EXPECT_EQ(c.gop, 10);
  EXPECT_DOUBLE_EQ(c.depth_min, 0.5);
  EXPECT_DOUBLE_EQ(c.depth_max, 12.0);
}

TEST(StreamConfig, ClampsAndKeepsInvalid)
{
  StreamConfig c;
  applyConfigLine("scale=3 fps=-1 bitrate=abc codec=vp9 depth_min=5 depth_max=1", c);
  EXPECT_DOUBLE_EQ(c.scale, 1.0);
  EXPECT_DOUBLE_EQ(c.max_fps, 0.0);
  EXPECT_EQ(c.bitrate_kbps, 8000);          // invalid: keeps
  EXPECT_EQ(c.codec, Codec::kH264);          // unknown codec: keeps
  EXPECT_GT(c.depth_max, c.depth_min);
}

TEST(StreamConfig, PartialLineOnlyChangesGivenFields)
{
  StreamConfig c;
  applyConfigLine("active=1 codec=mjpeg scale=0.3", c);
  applyConfigLine("bitrate=1234", c);       // adaptive bitrate adjustment
  EXPECT_TRUE(c.active);
  EXPECT_EQ(c.codec, Codec::kMjpeg);
  EXPECT_DOUBLE_EQ(c.scale, 0.3);
  EXPECT_EQ(c.bitrate_kbps, 1234);
}

TEST(StreamConfig, LocaleIndependent)
{
  // In pt_BR the decimal separator is a comma; the protocol uses a dot.
  const char * old = std::setlocale(LC_NUMERIC, nullptr);
  const std::string saved = old ? old : "C";
  std::setlocale(LC_NUMERIC, "pt_BR.UTF-8");
  StreamConfig c;
  applyConfigLine("scale=0.25", c);
  std::setlocale(LC_NUMERIC, saved.c_str());
  EXPECT_DOUBLE_EQ(c.scale, 0.25);
}

TEST(StreamConfig, RebuildOnlyForStructuralChanges)
{
  StreamConfig a;
  StreamConfig b = a;
  b.bitrate_kbps = 1000;
  b.max_fps = 10;
  b.topic = "/x";
  EXPECT_FALSE(a.needsRebuild(b));
  b.scale = 0.5;
  EXPECT_TRUE(a.needsRebuild(b));
}

TEST(FrameHeader, LayoutMatchesPythonAndJs)
{
  FrameHeader h;
  h.flags = 1;
  h.codec = 2;
  h.width = 640;
  h.height = 480;
  h.src_width = 1920;
  h.src_height = 1080;
  h.seq = 77;
  h.payload_size = 1234;
  h.stamp_ns = 1;
  h.recv_ns = 2;
  h.encoded_ns = 3;
  h.sim_age_ns = -4;
  const auto b = h.serialize();
  auto u32 = [&b](std::size_t o) {std::uint32_t v; std::memcpy(&v, b.data() + o, 4); return v;};
  auto i64 = [&b](std::size_t o) {std::int64_t v; std::memcpy(&v, b.data() + o, 8); return v;};
  EXPECT_EQ(u32(0), FrameHeader::kMagic);
  EXPECT_EQ(b[4], FrameHeader::kTypeFrame);
  EXPECT_EQ(b[5], 1);
  EXPECT_EQ(b[6], 2);
  EXPECT_EQ(u32(8), 640u);
  EXPECT_EQ(u32(12), 480u);
  EXPECT_EQ(u32(16), 1920u);
  EXPECT_EQ(u32(20), 1080u);
  EXPECT_EQ(u32(24), 77u);
  EXPECT_EQ(u32(28), 1234u);
  EXPECT_EQ(i64(32), 1);
  EXPECT_EQ(i64(40), 2);
  EXPECT_EQ(i64(48), 3);
  EXPECT_EQ(i64(56), -4);
}

namespace
{

bool hasElement(const char * name)
{
  gst_init(nullptr, nullptr);
  GstElementFactory * f = gst_element_factory_find(name);
  if (f) {gst_object_unref(f);}
  return f != nullptr;
}

/// Encodes `n` RGB frames and returns the metadata that came out of the encoder.
struct EncodeRun
{
  std::vector<EncodedFrame> frames;
  std::vector<std::string> infos;
};

EncodeRun encode(const StreamConfig & cfg, int n, std::uint32_t w = 320, std::uint32_t h = 240)
{
  EncodeRun run;
  std::mutex m;
  std::condition_variable cv;
  VideoEncoder enc(
    [&](const EncodedFrame & f) {
      std::lock_guard<std::mutex> lock(m);
      EncodedFrame copy = f;
      copy.data = nullptr;   // the buffer is only valid inside the callback
      run.frames.push_back(copy);
      cv.notify_all();
    },
    [&](bool, const std::string & text) {
      std::lock_guard<std::mutex> lock(m);
      run.infos.push_back(text);
    });
  enc.configure(cfg);
  auto pixels = std::make_shared<std::vector<std::uint8_t>>(w * h * 3, 128);
  for (int i = 0; i < n; ++i) {
    (*pixels)[static_cast<std::size_t>(i) % pixels->size()] = static_cast<std::uint8_t>(i);
    RawFrame f;
    f.data = pixels->data();
    f.size = pixels->size();
    f.width = w;
    f.height = h;
    f.stride = w * 3;
    f.format = "RGB";
    f.keepalive = pixels;
    f.recv_ns = 1000 + i;       // identifies the frame at the output
    f.stamp_ns = 5000 + i;
    // Waits for the encoder to release appsrc (push drops frames while it is busy).
    for (int t = 0; t < 200 && !enc.push(f); ++t) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  std::unique_lock<std::mutex> lock(m);
  cv.wait_for(lock, std::chrono::seconds(5), [&] {return run.frames.size() >= static_cast<std::size_t>(n);});
  lock.unlock();
  enc.stop();
  return run;
}

void expectInOrder(const EncodeRun & run, int n, std::uint32_t w, std::uint32_t h)
{
  ASSERT_EQ(run.frames.size(), static_cast<std::size_t>(n));
  EXPECT_TRUE(run.frames.front().keyframe);
  for (int i = 0; i < n; ++i) {
    // Metadata matched in order even with the encoder rewriting the PTS.
    EXPECT_EQ(run.frames[i].recv_ns, 1000 + i) << "quadro " << i;
    EXPECT_EQ(run.frames[i].stamp_ns, 5000 + i);
    EXPECT_GE(run.frames[i].encoded_ns, 0);
    EXPECT_EQ(run.frames[i].width, w);
    EXPECT_EQ(run.frames[i].height, h);
    EXPECT_EQ(run.frames[i].src_width, 320u);
  }
  ASSERT_FALSE(run.infos.empty());
}

}  // namespace

TEST(VideoEncoder, SoftwareH264KeepsMetadataOrder)
{
  if (!hasElement("x264enc")) {GTEST_SKIP() << "x264enc ausente";}
  StreamConfig c;
  applyConfigLine("active=1 codec=h264 backend=software scale=1 gop=5", c);
  const auto run = encode(c, 12);
  expectInOrder(run, 12, 320, 240);
  EXPECT_NE(run.infos.front().find("codec=avc1."), std::string::npos) << run.infos.front();
  EXPECT_NE(run.infos.front().find("hardware=0"), std::string::npos);
}

TEST(VideoEncoder, MjpegScalesToEvenSize)
{
  if (!hasElement("jpegenc")) {GTEST_SKIP() << "jpegenc ausente";}
  StreamConfig c;
  applyConfigLine("active=1 codec=mjpeg scale=0.5 quality=70", c);
  const auto run = encode(c, 6);
  expectInOrder(run, 6, 160, 120);
  for (const auto & f : run.frames) {EXPECT_TRUE(f.keyframe);}
}

TEST(VideoEncoder, HardwareH264WhenAvailable)
{
  if (!hasElement("vah264enc")) {GTEST_SKIP() << "VA-API ausente";}
  StreamConfig c;
  applyConfigLine("active=1 codec=h264 backend=hardware scale=1 gop=5", c);
  const auto run = encode(c, 10);
  expectInOrder(run, 10, 320, 240);
}

TEST(VideoEncoder, InactiveDropsFrames)
{
  if (!hasElement("jpegenc")) {GTEST_SKIP() << "jpegenc ausente";}
  VideoEncoder enc([](const EncodedFrame &) {}, nullptr);
  StreamConfig c;  // active=false
  enc.configure(c);
  auto px = std::make_shared<std::vector<std::uint8_t>>(32 * 32 * 3);
  RawFrame f{px->data(), px->size(), 32, 32, 96, "RGB", px, 0, 0, 0};
  EXPECT_FALSE(enc.push(f));
}
