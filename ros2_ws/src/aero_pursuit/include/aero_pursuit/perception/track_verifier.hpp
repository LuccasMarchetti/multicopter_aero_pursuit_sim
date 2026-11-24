#pragma once

#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace aero_pursuit::perception
{

/// Physical consistency checks of a visual track, independent of the tracker model.
///
/// A class-agnostic tracker (ViTTrack, NanoTrack, DaSiamRPN) keeps a high score on
/// whatever texture it drifted to (sky, ground, a decoy). The checks below use what the
/// pursuer knows about the target to detect such a drift:
///
///  - depth continuity: a target measured at r a moment ago cannot be farther than
///    r + v_max·Δt; in particular it cannot jump "beyond the depth range" while r is well
///    inside it (the box went to the background);
///  - scale/range consistency: the ROI angular size θ and the physical size learned while
///    there was depth predict the range; θ off by more than `scale_ratio` from what the
///    range allows means a different object (or the background);
///  - appearance: similarity of the ROI centre with the initial template below
///    `appearance_min` (colour + edges; see AppearanceModel);
///  - score drop: score below `score_drop_ratio` × its running mean (an APCE-like
///    confidence test, relative to the tracker's own scale);
///  - box size: ROI side above `max_box_fraction` of the image (the ViT box grows onto the
///    whole frame when it loses the target; caught even before any size is learned).
///
/// A check failing for `fail_frames` consecutive frames starts a hold (`hold_time` > 0): the
/// track stays suspect, the memories are kept and nothing is learned until `recover_frames`
/// consecutive consistent frames (the tracker found the target again by itself). A hold
/// longer than `hold_time` (or any failure with `hold_time` = 0) declares the track failed.
class TrackVerifier
{
public:
  struct Params
  {
    bool depth_check = true;
    bool scale_check = true;
    bool score_check = false;
    bool appearance_check = true;
    bool box_check = true;
    double max_box_fraction = 0.5;    ///< Max ROI width/height as a fraction of the image
    double appearance_min = 0.35;     ///< Minimum template similarity [0, 1]
    double max_range = 19.1;          ///< Depth camera far limit [m]
    double range_margin = 3.0;        ///< Memory must be this far inside the range [m]
    double memory = 2.0;              ///< Max age of the last depth used in the checks [s]
    double max_closing_speed = 8.0;   ///< Bound of |dr/dt| (target + pursuer) [m/s]
    double depth_jump_abs = 1.5;      ///< Extra tolerance on the depth jump [m]
    double scale_ratio = 2.0;         ///< Allowed factor between θ and the predicted θ
    double score_drop_ratio = 0.5;    ///< Score below this fraction of its mean => suspect
    double score_mean_time = 1.0;     ///< Time constant of the score mean [s]
    int fail_frames = 3;              ///< Consecutive failing frames to declare a failure
    double hold_time = 3.0;           ///< Max hold waiting for recovery [s] (0: fail at once)
    int recover_frames = 5;           ///< Consecutive consistent frames that end a hold
  };

  /// One tracker frame. `depth` = valid ROI depth (none if absent); `beyond_range` = the
  /// ROI is (almost) all beyond the far limit; `angular_size` [rad] (NaN if unknown).
  struct Frame
  {
    double time = 0.0;
    std::optional<double> depth;
    bool beyond_range = false;
    double angular_size = 0.0;
    double box_fraction = std::numeric_limits<double>::quiet_NaN();  ///< max(w/W, h/H); NaN = unknown
    double score = 0.0;
    double appearance = std::numeric_limits<double>::quiet_NaN();  ///< [0, 1]; NaN = not computed
  };

  enum class Verdict { kOk, kSuspect, kFailed };

  struct Result
  {
    Verdict verdict = Verdict::kOk;
    std::string reason;  ///< Check that failed (empty when kOk)
  };

  TrackVerifier() : TrackVerifier(Params{}) {}
  explicit TrackVerifier(const Params & params) : params_(params) {}

  void setParams(const Params & p) {params_ = p;}
  const Params & params() const {return params_;}

  /// New track (ROI selection or re-acquisition): clears the memories.
  void reset();

  Result update(const Frame & f);

  /// Physical size learned from depth × angular size [m] (NaN before any depth).
  double learnedSize() const {return size_;}

  /// Waiting for the tracker to come back to a consistent target (no measurement leaves).
  bool holding() const {return holding_;}

  /// Hold older than `hold_time` at `time` (also while the tracker reports no target).
  bool holdExpired(double time) const {return holding_ && time - hold_start_ > params_.hold_time;}

private:
  std::string check(const Frame & f) const;
  void learn(const Frame & f);
  Result fail(const Frame & f, const std::string & reason);

  Params params_;
  std::optional<std::pair<double, double>> last_depth_;  ///< (time, range)
  double size_ = std::numeric_limits<double>::quiet_NaN();
  double score_mean_ = std::numeric_limits<double>::quiet_NaN();
  double last_time_ = std::numeric_limits<double>::quiet_NaN();
  int failing_ = 0;
  bool holding_ = false;
  double hold_start_ = 0.0;
  int recovered_ = 0;
};

const char * toString(TrackVerifier::Verdict v);

}  // namespace aero_pursuit::perception
