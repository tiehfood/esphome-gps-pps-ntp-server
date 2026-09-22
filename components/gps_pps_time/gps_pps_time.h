#pragma once

#include <atomic>

#include "esphome/components/gps/gps.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace gps_pps_time {

class GPSPPSTime : public time::RealTimeClock, public gps::GPSListener {
 public:
  void set_pps_pin(InternalGPIOPin *pin) { this->pps_pin_ = pin; }
  void set_satellites_sensor(sensor::Sensor *sensor) { this->satellites_sensor_ = sensor; }
  void set_clock_offset_sensor(sensor::Sensor *sensor) { this->clock_offset_sensor_ = sensor; }
  void set_pps_drift_sensor(sensor::Sensor *sensor) { this->pps_drift_sensor_ = sensor; }
  void set_gps_time_sensor(text_sensor::TextSensor *sensor) { this->gps_time_sensor_ = sensor; }
  void set_gps_satellites_sensor(sensor::Sensor *sensor) { this->gps_satellites_sensor_ = sensor; }
  void set_glonass_satellites_sensor(sensor::Sensor *sensor) { this->glonass_satellites_sensor_ = sensor; }
  void set_galileo_satellites_sensor(sensor::Sensor *sensor) { this->galileo_satellites_sensor_ = sensor; }
  void set_beidou_satellites_sensor(sensor::Sensor *sensor) { this->beidou_satellites_sensor_ = sensor; }
  void set_crash_info_sensor(text_sensor::TextSensor *sensor) { this->crash_info_sensor_ = sensor; }
  void set_nmea_clock_delta_sensor(sensor::Sensor *sensor) { this->nmea_clock_delta_sensor_ = sensor; }
  /// Design K diagnostic (docs/superpowers/plans/2026-09-09-p4-ntp-probe.md): how far the
  /// PREVIOUS anchor's prediction would have been from the actual GPS second at THIS edge --
  /// i.e. the served-time error that accumulates BETWEEN two PPS edges, not at the edge
  /// itself (where NTPServer::anchor_epoch_us_() is exact by construction). Positive means
  /// the server would have served time ahead of GPS. Diagnostic only: computed in
  /// apply_pps_correction_(), never read by the correction loop or fed back into the anchor.
  void set_anchor_pred_error_sensor(sensor::Sensor *sensor) { this->anchor_pred_error_sensor_ = sensor; }

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  /// Returns true if PPS-disciplined time is active and recent
  bool is_synchronized() const;

  /// Epoch of the last PPS correction, 0 if never synced.
  /// NTP reference timestamp (RFC 5905 7.3) — not the current time.
  /// A consistent (epoch, micros) pair: at esp_timer time `micros`, true UTC was exactly
  /// `epoch` seconds and 0 microseconds -- the PPS edge IS the second boundary.
  /// `drift_ppb` is the hardware-measured crystal rate error in parts per billion,
  /// used to correct raw micros() elapsed time between edges. It is a RATE, not the
  /// position error at the edge -- the two differ by roughly 2x (the discipline loop's
  /// sawtooth centres at D/2, so the old position mean under-corrected the rate).
  ///
  /// Published as a unit under a seqlock because the pair is briefly inconsistent: the ISR
  /// stamps micros for edge N+1 before the main loop advances the epoch, and reading across
  /// that window yields a NEW timestamp against an OLD epoch -- a full second wrong.
  struct PpsAnchor {
    time_t epoch;
    uint32_t micros;
    int32_t drift_ppb;
  };

  /// False if no anchor has been published yet, or if a writer kept interrupting.
  /// A/B switch for the anchor's frequency term: the MCPWM-captured crystal rate (default)
  /// or the legacy position-mean estimate it replaced. The capture EMA keeps updating in
  /// both states, so switching back takes effect at the next PPS edge.
  void set_use_hw_rate(bool use) { this->use_hw_rate_ = use; }

  bool get_pps_anchor(PpsAnchor &out) const {
    for (int attempt = 0; attempt < 4; attempt++) {
      const uint32_t before = this->anchor_seq_.load(std::memory_order_acquire);
      if (before & 1u)
        continue;  // a write is in progress
      out = this->anchor_;
      if (this->anchor_seq_.load(std::memory_order_acquire) == before)
        return out.epoch != 0;
    }
    return false;
  }

  time_t get_last_sync_epoch() const {
    return this->pps_synced_ ? static_cast<time_t>(this->last_gps_epoch_) : 0;
  }

  void on_update(TinyGPSPlus &tiny_gps) override;

 protected:
  void apply_pps_correction_();
  void set_pps_time_(time_t epoch, uint32_t pps_micros, int32_t compensation_us = 0);

  InternalGPIOPin *pps_pin_{nullptr};
  sensor::Sensor *satellites_sensor_{nullptr};
  sensor::Sensor *clock_offset_sensor_{nullptr};
  sensor::Sensor *pps_drift_sensor_{nullptr};
  text_sensor::TextSensor *gps_time_sensor_{nullptr};
  sensor::Sensor *gps_satellites_sensor_{nullptr};
  sensor::Sensor *glonass_satellites_sensor_{nullptr};
  sensor::Sensor *galileo_satellites_sensor_{nullptr};
  sensor::Sensor *beidou_satellites_sensor_{nullptr};
  text_sensor::TextSensor *crash_info_sensor_{nullptr};
  sensor::Sensor *nmea_clock_delta_sensor_{nullptr};
  sensor::Sensor *anchor_pred_error_sensor_{nullptr};

  /// System clock minus the NMEA epoch, ms. NMEA carries absolute time and always
  /// arrives a sub-second delay AFTER the edge it describes, so a correct clock puts
  /// this in (0, 1000). An integer-second value means the PPS epoch counter is off --
  /// which last_drift_us_ cannot see, because drift is measured against that counter.
  int32_t nmea_clock_delta_ms_{0};
  bool nmea_clock_delta_valid_{false};
  /// Consecutive NMEA updates agreeing on the same integer-second epoch error.
  /// Requiring several stops a single glitched sentence from stepping the clock.
  int8_t epoch_error_streak_{0};
  int8_t epoch_error_last_{0};

  /// Pre-crash state from RTC NOINIT memory (populated in setup, published in first update)
  std::string crash_report_;
  bool crash_report_pending_{false};

  /// TinyGPSCustom objects for per-constellation satellite counts (GSV sentences)
  TinyGPSCustom *gp_gsv_sats_{nullptr};
  TinyGPSCustom *gl_gsv_sats_{nullptr};
  TinyGPSCustom *ga_gsv_sats_{nullptr};
  TinyGPSCustom *bd_gsv_sats_{nullptr};

  /// Last per-constellation satellite counts
  uint16_t last_gps_sat_count_{0};
  uint16_t last_glonass_sat_count_{0};
  uint16_t last_galileo_sat_count_{0};
  uint16_t last_beidou_sat_count_{0};

  /// Last GPS epoch extracted from NMEA sentences
  volatile time_t last_gps_epoch_{0};
  /// Whether we have received valid GPS time
  volatile bool gps_time_valid_{false};
  /// Flag set by ISR when PPS pulse detected
  volatile bool pps_flag_{false};
  /// Microsecond timestamp of last PPS pulse (from micros()) — used for interval/elapsed calc
  volatile uint32_t last_pps_micros_{0};
  /// ISR guard anchor — re-anchored after hard-sync, separate from interval tracking
  volatile uint32_t isr_anchor_micros_{0};
  /// Whether PPS-disciplined time has been applied at least once.
  /// volatile: read cross-thread by NTPServer::recv_task_() via is_synchronized()
  /// and get_last_sync_epoch() (single bool/word, atomic on Xtensa -- benign
  /// without volatile, but this documents the cross-thread read and stops the
  /// compiler from ever caching it across the loop() write path).
  volatile bool pps_synced_{false};

  // ---- Hardware PPS edge capture (MCPWM group 1; the W5500 INTn capture uses group 0).
  //
  // The GPIO ISR reads micros(): 1 us resolution plus interrupt-latency jitter. MCPWM
  // latches the edge in hardware at a 12.5 ns tick (APB 80 MHz, the only source on S3,
  // and the prescaler is not adjustable), so consecutive captures measure the PPS
  // interval far more precisely than the ISR can.
  //
  // The capture register is 32-bit and wraps every 2^32 / 80e6 = 53.687 s. Consecutive
  // PPS edges are ~1 s apart, so plain uint32 subtraction gives the interval correctly
  // across a wrap; only a gap longer than 53 s would alias, and that is a lost-PPS case
  // which is rejected below.
  static const uint32_t PPS_CAPTURE_HZ = 80000000UL;
  volatile uint32_t pps_cap_prev_{0};
  volatile uint32_t pps_cap_interval_{0};  ///< ticks between the last two edges
  volatile uint32_t pps_cap_count_{0};
  sensor::Sensor *pps_interval_sensor_{nullptr};
  bool pps_cap_started_{false};
  bool pps_interval_pending_{false};

 public:
  void set_pps_interval_sensor(sensor::Sensor *s) { this->pps_interval_sensor_ = s; }
  /// Crystal error in ppb, from the hardware-captured PPS interval. The GPS second is the
  /// reference, so any deviation from PPS_CAPTURE_HZ ticks is our oscillator, measured to
  /// ~12.5 ns in 1 s = 0.0125 ppm resolution -- about 80x finer than the ISR can manage.
  int32_t pps_cap_ppb_mean_x256_{0};
  bool use_hw_rate_{true};  ///< A/B: false selects the legacy position-mean rate term

  /// Smoothed crystal rate error. A single reading quantises to one 12.5 ns capture
  /// tick (12.5 ppb); averaging resolves below the tick.
  int32_t pps_capture_ppb_mean() const { return this->pps_cap_ppb_mean_x256_ / 256; }

  int32_t pps_capture_ppb() const {
    const uint32_t iv = this->pps_cap_interval_;
    if (iv == 0)
      return 0;
    return static_cast<int32_t>((static_cast<int64_t>(iv) - PPS_CAPTURE_HZ) * 1000000000LL /
                                PPS_CAPTURE_HZ);
  }
  void start_pps_capture_();

 protected:

  /// Seqlock-protected anchor; see get_pps_anchor(). Odd seq means a write is in flight.
  std::atomic<uint32_t> anchor_seq_{0};
  PpsAnchor anchor_{};
  void publish_pps_anchor_(time_t epoch, uint32_t micros, int32_t drift_ppb) {
    this->anchor_seq_.fetch_add(1, std::memory_order_release);
    this->anchor_ = PpsAnchor{epoch, micros, drift_ppb};
    this->anchor_seq_.fetch_add(1, std::memory_order_release);
  }
  /// Whether coarse GPS time has been set (once)
  bool has_gps_time_{false};
  /// PPS pulse counter for throttling settimeofday calls
  uint32_t pps_count_{0};
  /// Last satellite count from GPS (published in update(), not on_update())
  uint16_t last_satellite_count_{0};
  /// Last measured drift in microseconds (raw, unfiltered)
  int64_t last_drift_us_{0};
  /// Last clock offset in microseconds (filtered, what NTP clients see)
  int64_t last_clock_offset_us_{0};
  /// Estimated per-second crystal drift for pre-compensation (non-adjtime platforms)
  int32_t drift_compensation_us_{0};
  /// Previous PPS micros timestamp for ISR latency detection
  uint32_t prev_pps_micros_{0};
  /// Running mean of drift in fixed-point x256 for display centering (ESP-IDF)
  int64_t drift_mean_x256_{0};

  /// Design K diagnostic: most recent previous-anchor-vs-new-edge prediction error (see
  /// set_anchor_pred_error_sensor()), computed in apply_pps_correction_() and published in
  /// update() alongside the other PPS sensors. The arithmetic is PINNED to
  /// NTPServer::anchor_epoch_us_() (components/ntp_server/ntp_server.cpp) -- duplicated
  /// rather than shared because gps_pps_time cannot depend on ntp_server (the dependency
  /// runs the other way: ntp_server forward-declares GPSPPSTime) and because reusing it would
  /// mean editing the live serving function for a diagnostic-only feature. Any change to that
  /// function's arithmetic must be mirrored here.
  float anchor_pred_error_us_{0};
  bool anchor_pred_error_valid_{false};
  /// Set when on_update()'s NMEA epoch-streak correction steps last_gps_epoch_ and the wall
  /// clock between two PPS edges -- a deliberate counter correction, not a real prediction
  /// failure. Consumed and cleared by the next apply_pps_correction_() normal-branch pass to
  /// skip exactly one otherwise-fake multi-second sample. Diagnostic-only: never read outside
  /// the anchor_pred_error_us_ computation.
  bool anchor_pred_error_skip_next_{false};
  /// Millis timestamp of last processed PPS (for timeout detection).
  /// volatile: read cross-thread by NTPServer::recv_task_() via is_synchronized().
  volatile uint32_t last_pps_millis_{0};
  /// PPS timeout threshold in milliseconds
  static const uint32_t PPS_TIMEOUT_MS = 10000;

  static void IRAM_ATTR pps_isr(GPSPPSTime *self);
};

}  // namespace gps_pps_time
}  // namespace esphome
