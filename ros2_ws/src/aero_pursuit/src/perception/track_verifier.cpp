#include "aero_pursuit/perception/track_verifier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aero_pursuit::perception
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// Angular size of an object of size s at range r.
double angularSize(double s, double r)
{
  return 2.0 * std::atan(s / (2.0 * std::max(r, 1e-3)));
}
}  // namespace

const char * toString(TrackVerifier::Verdict v)
{
  switch (v) {
    case TrackVerifier::Verdict::kSuspect: return "suspect";
    case TrackVerifier::Verdict::kFailed: return "failed";
    case TrackVerifier::Verdict::kOk: break;
  }
  return "ok";
}

void TrackVerifier::reset()
{
  last_depth_.reset();
  size_ = kNaN;
  score_mean_ = kNaN;
  last_time_ = kNaN;
  failing_ = 0;
  holding_ = false;
  hold_start_ = 0.0;
  recovered_ = 0;
}

std::string TrackVerifier::check(const Frame & f) const
{
  const bool memory = last_depth_ && f.time - last_depth_->first <= params_.memory &&
    f.time >= last_depth_->first;
  const double age = memory ? f.time - last_depth_->first : 0.0;
  const double r_last = memory ? last_depth_->second : kNaN;
  const double r_lo = memory ? std::max(0.0, r_last - params_.max_closing_speed * age) : kNaN;
  const double r_hi = memory ? r_last + params_.max_closing_speed * age : kNaN;

  if (params_.box_check && std::isfinite(f.box_fraction) && f.box_fraction > params_.max_box_fraction) {
    return "box_size";
  }

  if (params_.depth_check && memory) {
    if (f.depth) {
      const double tol = params_.depth_jump_abs;
      if (*f.depth < r_lo - tol || *f.depth > r_hi + tol) {
        return "depth_jump";
      }
    } else if (f.beyond_range && r_hi < params_.max_range - params_.range_margin) {
      // The target was well inside the depth range: it cannot be beyond it now.
      return "depth_beyond";
    }
  }

  if (params_.scale_check && std::isfinite(size_) && std::isfinite(f.angular_size) &&
    f.angular_size > 0.0)
  {
    double lo = kNaN, hi = kNaN;  // range interval allowed for this frame
    if (f.depth) {
      lo = hi = *f.depth;
    } else if (memory) {
      lo = std::max(r_lo, 0.2);
      hi = r_hi;
    }
    if (std::isfinite(lo)) {
      const double theta_max = angularSize(size_, lo);
      const double theta_min = angularSize(size_, hi);
      if (f.angular_size > params_.scale_ratio * theta_max ||
        f.angular_size < theta_min / params_.scale_ratio)
      {
        return "scale";
      }
    }
  }

  if (params_.appearance_check && std::isfinite(f.appearance) && f.appearance < params_.appearance_min) {
    return "appearance";
  }

  if (params_.score_check && std::isfinite(score_mean_) &&
    f.score < params_.score_drop_ratio * score_mean_)
  {
    return "score_drop";
  }
  return {};
}

void TrackVerifier::learn(const Frame & f)
{
  const double dt = std::isfinite(last_time_) ? std::max(0.0, f.time - last_time_) : 0.0;
  if (f.depth) {
    last_depth_ = std::make_pair(f.time, *f.depth);
    if (std::isfinite(f.angular_size) && f.angular_size > 0.0) {
      const double sample = 2.0 * *f.depth * std::tan(0.5 * f.angular_size);
      if (!std::isfinite(size_)) {
        size_ = sample;
      } else {
        const double k = std::clamp(dt / 2.0, 0.02, 1.0);  // ~2 s average
        size_ += k * (sample - size_);
      }
    }
  }
  if (!std::isfinite(score_mean_)) {
    score_mean_ = f.score;
  } else {
    const double k = dt / (params_.score_mean_time + dt);
    score_mean_ += k * (f.score - score_mean_);
  }
}

TrackVerifier::Result TrackVerifier::fail(const Frame & f, const std::string & reason)
{
  Result r;
  r.reason = reason;
  r.verdict = Verdict::kSuspect;
  if (holding_) {
    recovered_ = 0;
    if (f.time - hold_start_ > params_.hold_time) {
      r.verdict = Verdict::kFailed;
      r.reason = "hold_timeout:" + reason;
    }
    return r;
  }
  if (++failing_ < params_.fail_frames) {
    return r;
  }
  if (params_.hold_time > 0.0) {
    holding_ = true;
    hold_start_ = f.time;
    recovered_ = 0;
  } else {
    r.verdict = Verdict::kFailed;
  }
  return r;
}

TrackVerifier::Result TrackVerifier::update(const Frame & f)
{
  const std::string reason = check(f);
  Result r;
  if (!reason.empty()) {
    r = fail(f, reason);
  } else if (holding_ && f.time - hold_start_ > params_.hold_time) {
    // Consistent frames that came too late: the memories are stale, start over.
    r.verdict = Verdict::kFailed;
    r.reason = "hold_timeout";
  } else if (holding_ && ++recovered_ < std::max(1, params_.recover_frames)) {
    // Consistent again, but not for long enough: still nothing is learned or published.
    r.verdict = Verdict::kSuspect;
    r.reason = "recovering";
  } else {
    holding_ = false;
    recovered_ = 0;
    failing_ = 0;
    learn(f);
  }
  last_time_ = f.time;
  return r;
}

}  // namespace aero_pursuit::perception
