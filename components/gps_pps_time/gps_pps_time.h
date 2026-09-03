#pragma once

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
  void set_crash_info_sensor(text_sensor::TextSensor *sensor) { this->crash_info_sensor_ = sensor; }

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  /// Returns true if PPS-disciplined time is active and recent
  bool is_synchronized() const;

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
  text_sensor::TextSensor *crash_info_sensor_{nullptr};

  /// Pre-crash state from RTC NOINIT memory (populated in setup, published in first update)
  std::string crash_report_;
  bool crash_report_pending_{false};

  /// TinyGPSCustom objects for per-constellation satellite counts (GSV sentences)
  TinyGPSCustom *gp_gsv_sats_{nullptr};
  TinyGPSCustom *gl_gsv_sats_{nullptr};
  TinyGPSCustom *ga_gsv_sats_{nullptr};

  /// Last per-constellation satellite counts
  uint16_t last_gps_sat_count_{0};
  uint16_t last_glonass_sat_count_{0};
  uint16_t last_galileo_sat_count_{0};

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
  /// Whether PPS-disciplined time has been applied at least once
  bool pps_synced_{false};
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
  /// Millis timestamp of last processed PPS (for timeout detection)
  uint32_t last_pps_millis_{0};
  /// PPS timeout threshold in milliseconds
  static const uint32_t PPS_TIMEOUT_MS = 10000;

  static void IRAM_ATTR pps_isr(GPSPPSTime *self);
};

}  // namespace gps_pps_time
}  // namespace esphome
