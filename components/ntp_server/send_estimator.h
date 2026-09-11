#pragma once

#include <algorithm>
#include <cstdint>

namespace esphome {
namespace ntp_server {

/// Predicts how long after t0 the W5500 is told to transmit (Sn_CR = SEND), so the reply's T3
/// can name the instant the packet actually leaves even though it is written before sendto().
///
/// Dependency-free on purpose: the rules below are unit-tested on the host
/// (tests/send_estimator_test.cpp), because the failure they guard against only shows up under
/// live network load.
class SendEstimator {
 public:
  /// Below the floor the packet was queued (ARP miss); above the ceiling something stalled.
  static constexpr int32_t US_MIN = 50;
  static constexpr int32_t US_MAX = 5000;
  /// Largest innovation accepted as normal. A single outlier beyond it is ignored rather than
  /// clamped: clamping still drags the estimate toward a bad sample.
  static constexpr int32_t STEP_MAX_US = 100;
  /// EWMA smoothing as a right-shift. 3 (alpha 1/8) measured best: settled error RMS 9.85 us,
  /// against 11.12 at 1/4, 11.91 at 1/16 and 15.27 at 1/64.
  static constexpr uint8_t EWMA_SHIFT = 3;
  /// Rejects in a row before a re-seed is considered.
  static constexpr uint8_t RESEED_AFTER = 4;
  /// ...and how long that run must last. Under outbound network load the send path stalls for
  /// ~1 s; four stalled sends in a row used to re-seed the estimate milliseconds high and give
  /// the following replies a late T3 (measured 2026-09-10). A real change persists far longer.
  static constexpr int64_t RESEED_MIN_SPAN_US = 3000000;

  enum class Result : uint8_t { IGNORED, SEEDED, LEARNED, REJECTED, RESEEDED };

  int32_t estimate() const { return this->estimate_us_; }

  /// A hardware-measured send delay (t0 to the Sn_CR = SEND write), observed at now_us.
  Result learn(int32_t actual_us, int64_t now_us) {
    if (actual_us <= US_MIN || actual_us >= US_MAX)
      return Result::IGNORED;
    if (this->estimate_us_ == 0) {
      this->estimate_us_ = actual_us;
      this->reject_run_ = 0;
      return Result::SEEDED;
    }
    const int32_t innov = actual_us - this->estimate_us_;
    if (innov > -STEP_MAX_US && innov < STEP_MAX_US) {
      this->estimate_us_ += innov >> EWMA_SHIFT;
      this->reject_run_ = 0;
      return Result::LEARNED;
    }
    // A reject that widens the run's spread past STEP_MAX_US starts a new run: stalled sends
    // disagree with each other as much as with the estimate, a changed send path does not.
    if (this->reject_run_ == 0 ||
        std::max(this->reject_max_, actual_us) - std::min(this->reject_min_, actual_us) >= STEP_MAX_US) {
      this->reject_min_ = actual_us;
      this->reject_max_ = actual_us;
      this->reject_first_us_ = now_us;
      this->reject_run_ = 1;
    } else {
      this->reject_min_ = std::min(this->reject_min_, actual_us);
      this->reject_max_ = std::max(this->reject_max_, actual_us);
      if (this->reject_run_ < UINT8_MAX)
        this->reject_run_++;
    }
    if (this->reject_run_ >= RESEED_AFTER && now_us - this->reject_first_us_ >= RESEED_MIN_SPAN_US) {
      this->estimate_us_ = this->reject_min_ + (this->reject_max_ - this->reject_min_) / 2;
      this->reject_run_ = 0;
      return Result::RESEEDED;
    }
    return Result::REJECTED;
  }

  /// Fallback when no hardware stamp was available: the sendto() duration. Same bounds, so a
  /// queued packet (returns immediately, departs late) cannot drag the estimate down.
  void learn_fallback(int32_t dur_us) {
    if (dur_us > US_MIN && dur_us < US_MAX)
      this->estimate_us_ += (dur_us - this->estimate_us_) >> EWMA_SHIFT;
  }

 private:
  int32_t estimate_us_{0};
  uint8_t reject_run_{0};
  int32_t reject_min_{0};
  int32_t reject_max_{0};
  int64_t reject_first_us_{0};
};

}  // namespace ntp_server
}  // namespace esphome
