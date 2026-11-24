#include "aero_pursuit/perception/track_verifier.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

using aero_pursuit::perception::TrackVerifier;
using Verdict = TrackVerifier::Verdict;

namespace
{
constexpr double kSize = 0.5;  // X4 ~0.5 m

TrackVerifier::Frame frame(double t, double range, bool with_depth = true, double score = 0.7)
{
  TrackVerifier::Frame f;
  f.time = t;
  if (with_depth) {
    f.depth = range;
  }
  f.angular_size = 2.0 * std::atan(kSize / (2.0 * range));
  f.score = score;
  return f;
}
}  // namespace

TEST(TrackVerifier, ConsistentApproachIsAccepted)
{
  TrackVerifier v;
  for (int k = 0; k < 100; ++k) {
    const double t = k * 0.05, r = 15.0 - 2.0 * t;  // closing at 2 m/s
    ASSERT_EQ(v.update(frame(t, r)).verdict, Verdict::kOk) << "k " << k;
  }
  EXPECT_NEAR(v.learnedSize(), kSize, 0.02);
}

TEST(TrackVerifier, TargetFartherThanDepthRangeIsAccepted)
{
  // Approach from 30 m: no depth, ROI beyond range, no memory yet => no evidence against it.
  TrackVerifier v;
  for (int k = 0; k < 40; ++k) {
    auto f = frame(k * 0.05, 30.0, false);
    f.beyond_range = true;
    EXPECT_EQ(v.update(f).verdict, Verdict::kOk);
  }
}

TEST(TrackVerifier, DriftToBackgroundAfterCloseRangeIsRejected)
{
  // Field failure: target measured at ~7 m, then the box sits on the background
  // (whole ROI beyond the 19 m far limit) with a high score.
  TrackVerifier::Params p;
  p.hold_time = 0.0;  // discard at once
  TrackVerifier v(p);
  double t = 0.0;
  for (; t < 2.0; t += 0.05) {
    ASSERT_EQ(v.update(frame(t, 7.0)).verdict, Verdict::kOk);
  }
  TrackVerifier::Frame bg;
  bg.beyond_range = true;
  bg.angular_size = 0.03;
  bg.score = 0.8;
  bg.time = t;
  EXPECT_EQ(v.update(bg).verdict, Verdict::kSuspect);
  bg.time += 0.05;
  EXPECT_EQ(v.update(bg).verdict, Verdict::kSuspect);
  bg.time += 0.05;
  const auto r = v.update(bg);
  EXPECT_EQ(r.verdict, Verdict::kFailed);
  EXPECT_EQ(r.reason, "depth_beyond");
}

TEST(TrackVerifier, DepthJumpToAnotherObjectIsRejected)
{
  TrackVerifier::Params p;
  p.hold_time = 0.0;
  TrackVerifier v(p);
  double t = 0.0;
  for (; t < 1.0; t += 0.05) {
    v.update(frame(t, 6.0));
  }
  // Box jumps to a decoy 14 m away within 0.1 s (needs > 70 m/s).
  int failed = 0;
  for (int k = 0; k < 3; ++k) {
    t += 0.05;
    failed += v.update(frame(t, 14.0)).verdict == Verdict::kFailed;
  }
  EXPECT_EQ(failed, 1);
}

TEST(TrackVerifier, ScaleInconsistentWithLearnedSizeIsRejected)
{
  // Same depth, but the ROI is 4x larger than the learned object at that range (box grew
  // onto a large background region with spurious valid depth).
  TrackVerifier v;
  double t = 0.0;
  for (; t < 2.0; t += 0.05) {
    v.update(frame(t, 8.0));
  }
  auto f = frame(t, 8.0);
  f.angular_size *= 4.0;
  EXPECT_EQ(v.update(f).reason, "scale");
}

TEST(TrackVerifier, MemoryExpiresAndResetClears)
{
  TrackVerifier::Params p;
  p.memory = 1.0;
  TrackVerifier v(p);
  for (double t = 0.0; t < 1.0; t += 0.05) {
    v.update(frame(t, 7.0));
  }
  // 3 s later the target may legitimately be beyond the range.
  TrackVerifier::Frame far;
  far.time = 4.0;
  far.beyond_range = true;
  far.angular_size = NAN;
  EXPECT_EQ(v.update(far).verdict, Verdict::kOk);
  v.reset();
  EXPECT_TRUE(std::isnan(v.learnedSize()));
}

TEST(TrackVerifier, ScoreDropIsOptional)
{
  TrackVerifier::Params p;
  p.score_check = true;
  TrackVerifier v(p);
  double t = 0.0;
  for (; t < 2.0; t += 0.05) {
    v.update(frame(t, 6.0, true, 0.8));
  }
  EXPECT_EQ(v.update(frame(t + 0.05, 6.0, true, 0.2)).reason, "score_drop");
}

TEST(TrackVerifier, AppearanceBelowMinimumIsRejected)
{
  TrackVerifier v;
  auto f = frame(0.0, 6.0);
  f.appearance = 0.9;
  EXPECT_EQ(v.update(f).verdict, Verdict::kOk);
  f.time = 0.05;
  f.appearance = 0.1;
  EXPECT_EQ(v.update(f).reason, "appearance");
}

TEST(TrackVerifier, BoxOverWholeImageIsRejectedWithoutLearnedSize)
{
  // Recorded ViT failure: the box grows to ~1600x700 px of 1920x1080 with score ~0.4.
  TrackVerifier v;
  auto f = frame(0.0, 6.0, false);
  f.angular_size = NAN;
  f.box_fraction = 0.85;
  EXPECT_EQ(v.update(f).reason, "box_size");
  f.box_fraction = 0.15;
  f.time = 0.05;
  EXPECT_EQ(v.update(f).verdict, Verdict::kOk);
}

TEST(TrackVerifier, HoldRecoversWhenTrackerReturnsToTarget)
{
  TrackVerifier::Params p;
  p.hold_time = 3.0;
  p.recover_frames = 5;
  TrackVerifier v(p);
  double t = 0.0;
  for (; t < 2.0; t += 0.05) {
    ASSERT_EQ(v.update(frame(t, 5.0)).verdict, Verdict::kOk);
  }
  auto bad = frame(t, 5.0);
  bad.box_fraction = 0.9;
  for (int k = 0; k < 10; ++k, t += 0.05) {
    bad.time = t;
    ASSERT_EQ(v.update(bad).verdict, Verdict::kSuspect) << k;  // never failed while holding
  }
  EXPECT_TRUE(v.holding());
  // Back on the target: suspect until recover_frames consistent frames, then ok.
  for (int k = 0; k < 4; ++k, t += 0.05) {
    EXPECT_EQ(v.update(frame(t, 5.0)).reason, "recovering");
  }
  EXPECT_EQ(v.update(frame(t, 5.0)).verdict, Verdict::kOk);
  EXPECT_FALSE(v.holding());
}

TEST(TrackVerifier, InconsistentFrameRestartsRecovery)
{
  TrackVerifier v;
  double t = 0.0;
  for (; t < 1.0; t += 0.05) {
    v.update(frame(t, 5.0));
  }
  auto bad = frame(t, 5.0);
  bad.box_fraction = 0.9;
  for (int k = 0; k < 3; ++k, t += 0.05) {
    bad.time = t;
    v.update(bad);
  }
  ASSERT_TRUE(v.holding());
  for (int k = 0; k < 3; ++k, t += 0.05) {
    v.update(frame(t, 5.0));
  }
  bad.time = t;
  t += 0.05;
  EXPECT_EQ(v.update(bad).verdict, Verdict::kSuspect);
  for (int k = 0; k < 4; ++k, t += 0.05) {
    EXPECT_EQ(v.update(frame(t, 5.0)).reason, "recovering") << k;
  }
}

TEST(TrackVerifier, HoldTimesOut)
{
  TrackVerifier::Params p;
  p.hold_time = 1.0;
  TrackVerifier v(p);
  double t = 0.0;
  for (; t < 1.0; t += 0.05) {
    v.update(frame(t, 5.0));
  }
  auto bad = frame(t, 5.0);
  bad.box_fraction = 0.9;
  Verdict last = Verdict::kOk;
  std::string reason;
  for (; t < 2.5 && last != Verdict::kFailed; t += 0.05) {
    bad.time = t;
    const auto r = v.update(bad);
    last = r.verdict;
    reason = r.reason;
  }
  EXPECT_EQ(last, Verdict::kFailed);
  EXPECT_EQ(reason, "hold_timeout:box_size");
  EXPECT_TRUE(v.holdExpired(t));
}
