#include "gps_pps_time.h"
#include "esphome/core/log.h"
#include <sys/time.h>
#include <cstdlib>
#ifdef USE_ESP_IDF
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#ifdef USE_ESP_IDF
// RTC NOINIT memory survives panic resets — stores pre-crash diagnostics.
// Written periodically in loop(); read in setup() to report the pre-crash state.
struct CrashContext {
  uint32_t magic;           // 0xDEAD5AFE when valid
  uint32_t heap_free;       // esp_get_free_heap_size()
  uint32_t heap_min;        // esp_get_minimum_free_heap_size()
  uint32_t uptime_sec;      // millis() / 1000
  uint32_t pps_count;       // PPS pulse counter
  uint32_t main_stack_hwm;  // uxTaskGetStackHighWaterMark(NULL) for main task
  uint32_t loop_count;      // incremented each loop() call
};
static RTC_NOINIT_ATTR CrashContext crash_ctx;
#endif

namespace esphome {
namespace gps_pps_time {

static const char *const TAG = "gps_pps_time";

void IRAM_ATTR GPSPPSTime::pps_isr(GPSPPSTime *self) {
  uint32_t now = micros();

  // 900ms guard: reject any edge arriving less than 900ms after the last accepted PPS.
  // Edge dump confirmed the PPS signal is clean (exactly 1 rising edge per second,
  // 1,000,010µs apart, zero spurious edges), so this is a safety margin only.
  // isr_anchor_micros_ == 0 on first boot → always accepts the very first pulse.
  if (self->isr_anchor_micros_ != 0 && (now - self->isr_anchor_micros_) < 900000) {
    return;
  }
  // NOTE: gettimeofday() is intentionally NOT called here. On ESP-IDF it acquires
  // s_time_lock/s_boot_time_lock via _lock_acquire(), which is not ISR-safe: if the
  // main loop holds that lock (adjtime()/settimeofday() in apply_pps_correction_(),
  // or NTPServer::get_ntp_timestamp_()) when this edge arrives, xSemaphoreTakeFromISR
  // fails and newlib's lock implementation calls abort() -> panic reboot. micros()
  // (esp_timer_get_time(), lock-free) is the only time source safe to read here.
  // The wall-clock time at this edge is reconstructed in apply_pps_correction_()
  // from this micros() timestamp instead.
  self->last_pps_micros_ = now;
  self->isr_anchor_micros_ = now;
  self->pps_flag_ = true;
}

void GPSPPSTime::setup() {
  ESP_LOGI(TAG, "Setting up GPS PPS time source...");
  this->pps_pin_->setup();
  this->pps_pin_->attach_interrupt(&GPSPPSTime::pps_isr, this, gpio::INTERRUPT_RISING_EDGE);
#ifdef USE_ESP_IDF
  esp_reset_reason_t reason = esp_reset_reason();
  if (reason == ESP_RST_PANIC && crash_ctx.magic == 0xDEAD5AFE) {
    char buf[200];
    snprintf(buf, sizeof(buf),
             "PANIC at %us uptime | Heap: %u free, %u min | Stack HWM: %u words | PPS#%u | Loops: %u",
             crash_ctx.uptime_sec, crash_ctx.heap_free, crash_ctx.heap_min,
             crash_ctx.main_stack_hwm, crash_ctx.pps_count, crash_ctx.loop_count);
    this->crash_report_ = buf;
    this->crash_report_pending_ = true;
    ESP_LOGW(TAG, "Previous crash: %s", buf);
  } else if (reason == ESP_RST_PANIC) {
    this->crash_report_ = "PANIC (no RTC context — cold boot or power loss)";
    this->crash_report_pending_ = true;
    ESP_LOGW(TAG, "Previous crash detected but no valid crash context");
  }
  // Initialise for this boot
  crash_ctx.magic = 0xDEAD5AFE;
  crash_ctx.loop_count = 0;
#endif
}

void GPSPPSTime::loop() {
  if (this->pps_flag_) {
    this->pps_flag_ = false;
    this->apply_pps_correction_();
  }
#ifdef USE_ESP_IDF
  // Update crash context in RTC NOINIT memory every ~10 seconds so that on a panic
  // reset the last known state is available in setup() for diagnostics.
  static uint32_t last_ctx_update_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms - last_ctx_update_ms >= 10000) {
    last_ctx_update_ms = now_ms;
    crash_ctx.heap_free = esp_get_free_heap_size();
    crash_ctx.heap_min = esp_get_minimum_free_heap_size();
    crash_ctx.uptime_sec = now_ms / 1000;
    crash_ctx.pps_count = this->pps_count_;
    crash_ctx.main_stack_hwm = uxTaskGetStackHighWaterMark(nullptr);
  }
  crash_ctx.loop_count++;
#endif
}

void GPSPPSTime::set_pps_time_(time_t epoch, uint32_t pps_micros, int32_t compensation_us) {
  // Account for time elapsed since PPS edge using the snapshot captured before loop()
  // delay, minus drift pre-compensation. Using the passed pps_micros (not the volatile
  // this->last_pps_micros_) prevents a concurrent ISR overwrite from corrupting the
  // elapsed calculation.
  int32_t elapsed_us = static_cast<int32_t>(micros() - pps_micros);
  int32_t adjusted_us = elapsed_us - compensation_us;
  struct timeval tv;
  tv.tv_sec = epoch;
  tv.tv_usec = (adjusted_us > 0) ? adjusted_us : 0;
  struct timezone tz = {0, 0};
  settimeofday(&tv, &tz);
}

void GPSPPSTime::apply_pps_correction_() {
  // Reconstruct the wall-clock time at the PPS edge without calling gettimeofday()
  // from the ISR (see pps_isr() for why that is forbidden on ESP-IDF).
  //
  // On ESP-IDF: gettimeofday() == adjusted_boot_time() + esp_timer_get_time(), and
  // micros() == (uint32_t) esp_timer_get_time() (esphome/components/esp32/core.cpp).
  // Reading both back-to-back here and subtracting the micros() delta since the edge
  // algebraically cancels the esp_timer_get_time() term, leaving:
  //   system_us_at_pps = adjusted_boot_time(now) + esp_timer_get_time(edge)
  // which equals gettimeofday() as it would have read exactly at the edge, PROVIDED
  // adjusted_boot_time() has not changed between the edge and this read. adjtime()'s
  // slew (components/newlib/src/time.c, ADJTIME_CORRECTION_FACTOR=6) advances
  // adjusted_boot_time by (elapsed_us >> 6) each time it's queried, i.e. it can apply
  // at most ~15,625us of correction per second of elapsed real time. Our steady-state
  // per-PPS corrections (tens of µs) therefore fully complete within a few ms of being
  // issued — long before the ~1s until this code runs again — so adjusted_boot_time is
  // stable across the edge-to-read window and the residual error is ~0 in steady state.
  // Worst case (correction still converging, e.g. drift near the 100ms hard-sync
  // threshold): bounded by (1/64) * loop_delay_us, a few ms at most.
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const uint32_t read_micros = micros();

  // Snapshot ISR-written volatile to a local — the ISR can fire again
  // after pps_flag_ was cleared in loop(), so reading it multiple
  // times without a snapshot risks mixing old and new PPS values.
  const uint32_t pps_micros = this->last_pps_micros_;

  // Wraparound-safe: elapsed since the edge, in microseconds.
  const int32_t elapsed_since_edge_us = static_cast<int32_t>(read_micros - pps_micros);
  const int64_t system_us_at_pps_raw =
      static_cast<int64_t>(tv.tv_sec) * 1000000LL + tv.tv_usec - elapsed_since_edge_us;

  if (!this->gps_time_valid_) {
    ESP_LOGD(TAG, "PPS pulse received but no valid GPS time yet");
    return;
  }

  // Guard: if pps_flag_ blocked NMEA from setting the epoch, last_gps_epoch_ is still 0.
  // Skip this PPS and wait for the next NMEA + PPS cycle.
  if (this->last_gps_epoch_ == 0) {
    ESP_LOGD(TAG, "PPS pulse skipped, waiting for NMEA epoch");
    return;
  }

  // Satellite guard: the LEA-M8T continues outputting PPS from its internal oscillator
  // even with zero satellites (no GNSS fix). These undisciplined pulses cause growing
  // clock drift. Refuse to trust PPS unless at least one satellite is reported.
  // Use both GGA (satellites in use) and GSV (satellites in view) counts — GGA may
  // not be parsed yet at boot while GSV is already reporting 20+ satellites.
  uint16_t gsv_total = this->last_gps_sat_count_ + this->last_glonass_sat_count_
                       + this->last_galileo_sat_count_;
  if (this->last_satellite_count_ == 0 && gsv_total == 0) {
    if (this->pps_synced_) {
      // GPS fix lost while previously synced — treat same as epoch diverged so that
      // NMEA re-establishes epoch and coarse clock once satellites are reacquired.
      ESP_LOGW(TAG, "GPS fix lost (0 satellites): suspending PPS sync");
      this->pps_synced_ = false;
      this->gps_time_valid_ = false;
      this->has_gps_time_ = false;
      this->last_gps_epoch_ = 0;
      this->last_drift_us_ = 0;
      this->drift_mean_x256_ = 0;
      this->pps_count_ = 0;
    }
    // Log at most once per 10 seconds to avoid flooding during long satellite outages
    if (this->pps_count_ == 0 && (millis() - this->last_pps_millis_) > 10000) {
      ESP_LOGD(TAG, "PPS pulse skipped, no satellites (GGA=%u GSV=%u)",
               this->last_satellite_count_, gsv_total);
      this->last_pps_millis_ = millis();
    }
    return;
  }

  this->pps_count_++;
  this->last_pps_millis_ = millis();

  if (this->pps_count_ == 1) {
    // First accepted PPS after boot or re-sync: record timestamp and wait for
    // the second PPS to compute an interval. The 900ms ISR guard already rejects
    // spurious edges — no additional usec-based filtering needed.
    this->prev_pps_micros_ = pps_micros;
    // Normalize the reconstructed edge time to [0, 1e6) purely for diagnostic logging.
    int32_t usec_estimate = static_cast<int32_t>(((system_us_at_pps_raw % 1000000) + 1000000) % 1000000);
    ESP_LOGD(TAG, "First PPS accepted (usec=%d), waiting for second", usec_estimate);
    return;
  }

  uint32_t pps_interval_us = pps_micros - this->prev_pps_micros_;

  // Detect missed PPS pulses: if loop() was delayed, the ISR may have captured a later
  // pulse while an earlier one was overwritten. The micros() interval reveals how many
  // real 1-second periods elapsed. Advance the epoch counter accordingly so it stays
  // aligned with real GPS time even when loop() is slow.
  int extra_epochs = static_cast<int>((pps_interval_us + 500000u) / 1000000u) - 1;
  if (extra_epochs > 0) {
    ESP_LOGW(TAG, "Missed %d PPS pulse(s) (interval: %u us), advancing epoch",
             extra_epochs, pps_interval_us);
  }

  // Detect ISR latency from slight PPS interval deviation (only for single-interval pulses).
  // If UART ISR delays the PPS ISR, micros() captures a late timestamp.
  int32_t isr_latency_us = 0;
  if (extra_epochs == 0) {
    int32_t deviation = static_cast<int32_t>(pps_interval_us) - 1000000;
    if (deviation > 500 && deviation < 10000) {
      isr_latency_us = deviation;
      ESP_LOGD(TAG, "PPS ISR latency detected: %d us", isr_latency_us);
    }
  }
  this->prev_pps_micros_ = pps_micros;

  // PPS marks the start of the next second after the last GPS epoch.
  // extra_epochs compensates for any missed intermediate pulses.
  time_t corrected_epoch = this->last_gps_epoch_ + 1 + extra_epochs;
  // Advance epoch to corrected_epoch (already includes extra_epochs skips)
  this->last_gps_epoch_ = corrected_epoch;

  // Wall-clock time was reconstructed (not captured in the ISR) from gettimeofday()
  // and micros() read back-to-back above — no loop-delay compensation needed since
  // elapsed_since_edge_us is already subtracted. Only subtract ISR latency if the
  // PPS ISR itself was delayed by a higher-priority ISR (detected via PPS interval
  // deviation above).
  int64_t system_us_at_pps = system_us_at_pps_raw - isr_latency_us;
  // Expected time at PPS edge: corrected_epoch seconds, 0 microseconds
  int64_t expected_us = static_cast<int64_t>(corrected_epoch) * 1000000LL;
  this->last_drift_us_ = system_us_at_pps - expected_us;

  if (!this->pps_synced_) {
    // First PPS: hard-sync the clock to the exact PPS second boundary.
    // Do NOT call time_sync_callback_ here — it triggers the HA API to push its own
    // integer-second time back (~643ms behind actual), overriding our correction.
#ifdef USE_ESP_IDF
    // Set to exact second boundary. The small loop-delay error (typically 1-5ms)
    // is corrected by adjtime on the next PPS. Using set_pps_time_() here would
    // bake the loop delay (50-200ms when HA API blocks) into a permanent clock
    // offset that adjtime then fights against, causing oscillation.
    struct timeval sync_tv = {corrected_epoch, 0};
    struct timezone sync_tz = {0, 0};
    settimeofday(&sync_tv, &sync_tz);
    // Cancel any pending adjtime so stale correction doesn't corrupt new baseline
    struct timeval zero_adj = {0, 0};
    adjtime(&zero_adj, nullptr);
#else
    this->set_pps_time_(corrected_epoch, pps_micros);
#endif
    this->last_drift_us_ = 0;
    ESP_LOGI(TAG, "PPS-disciplined time synchronized");
    this->pps_synced_ = true;
  } else if (this->last_drift_us_ > 2000000 || this->last_drift_us_ < -2000000) {
    // Drift > 2 seconds: epoch counter has diverged (GPS module reset, large gap).
    // Full reset — NMEA must re-establish epoch AND re-set the coarse clock before
    // next PPS. Resetting has_gps_time_ is critical: without it, the coarse clock
    // re-set in on_update() is skipped (guarded by !has_gps_time_), so the clock
    // stays wrong, next PPS sees drift > 2s again, and the device never re-syncs.
    ESP_LOGW(TAG, "Epoch diverged (%lld us), re-syncing from NMEA+PPS",
             (long long) this->last_drift_us_);
    this->pps_synced_ = false;
    this->gps_time_valid_ = false;
    this->has_gps_time_ = false;
    this->last_gps_epoch_ = 0;
    this->last_drift_us_ = 0;
    this->drift_mean_x256_ = 0;
    this->pps_count_ = 0;
  } else if (this->last_drift_us_ > 100000 || this->last_drift_us_ < -100000) {
    // Drift 100ms–2s: hard-sync the clock without resetting sync state.
    // adjtime() slews at ~1ms/s and each PPS replaces the pending correction,
    // so any drift > ~100ms never converges through adjtime alone. Hard-sync
    // immediately and let fine correction resume from the next PPS.
    ESP_LOGW(TAG, "Large drift (%lld us): hard-syncing clock",
             (long long) this->last_drift_us_);
#ifdef USE_ESP_IDF
    {
      struct timeval sync_tv = {corrected_epoch, 0};
      struct timezone sync_tz = {0, 0};
      settimeofday(&sync_tv, &sync_tz);
      struct timeval zero_adj = {0, 0};
      adjtime(&zero_adj, nullptr);
    }
#else
    this->set_pps_time_(corrected_epoch, pps_micros);
#endif
    this->last_drift_us_ = 0;
    this->drift_mean_x256_ = 0;
  } else {
    // Normal operation: correct every PPS pulse
#ifdef USE_ESP_IDF
    // adjtime() gradually slews the clock without jumping — ideal for NTP serving.
    // Always apply a correction so the clock never free-runs, even on noisy measurements.
    // The EMA (overcorrection mean) is only updated from clean measurements within ±50µs
    // to prevent ISR-contaminated readings from poisoning the mean estimate.
    int64_t overcorrection = this->drift_mean_x256_ / 256;
    int64_t correction_us = -(this->last_drift_us_ + overcorrection);
    struct timeval delta;
    delta.tv_sec = correction_us / 1000000LL;
    delta.tv_usec = correction_us % 1000000LL;
    // Normalize: ESP-IDF adjtime requires tv_usec in [0, 999999].
    // C++ rounds -51345/1000000 toward zero → tv_sec=0, tv_usec=-51345.
    // Negative tv_usec with tv_sec=0 causes ESP-IDF to misinterpret the sign,
    // applying the correction in the wrong direction (~35ms/s drift growth).
    if (delta.tv_usec < 0) {
      delta.tv_sec -= 1;
      delta.tv_usec += 1000000;
    }
    adjtime(&delta, nullptr);
    if (this->last_drift_us_ > -50 && this->last_drift_us_ < 50) {
      // Clean measurement: update the running mean and record exact offset
      // Track running mean of measured drift (fixed-point x256, alpha 1/128)
      this->drift_mean_x256_ += (this->last_drift_us_ * 256 - this->drift_mean_x256_) / 128;
      this->last_clock_offset_us_ = this->last_drift_us_;
    } else {
      // Spike: adjtime still applied above, but EMA protected — estimate offset from mean
      ESP_LOGD(TAG, "Drift spike filtered from EMA: %lld us",
               (long long) this->last_drift_us_);
      this->last_clock_offset_us_ = this->drift_mean_x256_ / 256;
    }
#else
    // Platforms without adjtime(): always correct, protect EMA from spikes
    // Subtract ISR latency from compensation so set_pps_time_ adds it to elapsed time
    this->set_pps_time_(corrected_epoch, pps_micros, this->drift_compensation_us_ - isr_latency_us);
    // Only update drift estimate when measurement is clean (no ISR latency, small drift)
    if (isr_latency_us == 0 && this->last_drift_us_ > -50 && this->last_drift_us_ < 50) {
      this->drift_compensation_us_ += static_cast<int32_t>(this->last_drift_us_) / 4;
    } else {
      ESP_LOGD(TAG, "PPS drift spike filtered from EMA: %lld us",
               (long long) this->last_drift_us_);
    }
#endif
  }

  ESP_LOGD(TAG, "PPS #%lu, epoch: %ld, drift: %lld us",
           (unsigned long) this->pps_count_, (long) corrected_epoch,
           (long long) this->last_drift_us_);
}

void GPSPPSTime::on_update(TinyGPSPlus &tiny_gps) {
  // Register TinyGPSCustom extractors on first call (needs TinyGPSPlus reference)
  if (this->gp_gsv_sats_ == nullptr) {
    this->gp_gsv_sats_ = new TinyGPSCustom(tiny_gps, "GPGSV", 3);
    this->gl_gsv_sats_ = new TinyGPSCustom(tiny_gps, "GLGSV", 3);
    this->ga_gsv_sats_ = new TinyGPSCustom(tiny_gps, "GAGSV", 3);
  }

  if (tiny_gps.satellites.isValid()) {
    this->last_satellite_count_ = tiny_gps.satellites.value();
  }

  // Update per-constellation satellite counts from GSV sentences
  if (this->gp_gsv_sats_->isUpdated())
    this->last_gps_sat_count_ = atoi(this->gp_gsv_sats_->value());
  if (this->gl_gsv_sats_->isUpdated())
    this->last_glonass_sat_count_ = atoi(this->gl_gsv_sats_->value());
  if (this->ga_gsv_sats_->isUpdated())
    this->last_galileo_sat_count_ = atoi(this->ga_gsv_sats_->value());

  if (!tiny_gps.time.isValid() || !tiny_gps.date.isValid() ||
      !tiny_gps.time.isUpdated() || !tiny_gps.date.isUpdated() ||
      tiny_gps.date.year() < 2025) {
    return;
  }

  ESPTime val{};
  val.year = tiny_gps.date.year();
  val.month = tiny_gps.date.month();
  val.day_of_month = tiny_gps.date.day();
  val.day_of_week = 1;
  val.day_of_year = 1;
  val.hour = tiny_gps.time.hour();
  val.minute = tiny_gps.time.minute();
  val.second = tiny_gps.time.second();
  val.recalc_timestamp_utc(false);

  this->gps_time_valid_ = true;

  // Epoch sanity: compare the free-running clock against NMEA's absolute time.
  // Expected range is (0, 1000) ms -- the NMEA parse delay. An integer-second
  // excursion is a wrong PPS epoch counter, which drift reports as ~0 because it
  // measures against that same counter.
  {
    struct timeval now_tv;
    gettimeofday(&now_tv, nullptr);
    int64_t delta_ms = (static_cast<int64_t>(now_tv.tv_sec) - static_cast<int64_t>(val.timestamp)) * 1000 +
                       now_tv.tv_usec / 1000;
    this->nmea_clock_delta_ms_ = static_cast<int32_t>(delta_ms);
    this->nmea_clock_delta_valid_ = true;
    if (this->pps_synced_ && (delta_ms < -150 || delta_ms > 1150)) {
      ESP_LOGW(TAG, "NMEA/clock delta %lld ms (expect 0..1000): epoch counter looks %+d s out",
               (long long) delta_ms, static_cast<int>((delta_ms - 350) / 1000));
    }
  }

  // Once PPS is synced, it manages the epoch counter via incrementing.
  // NMEA must not overwrite it, as stale timestamps would reset the counter backward.
  // Skip update when a PPS ISR is pending — prevents race where NMEA for the next
  // second arrives between the PPS ISR and loop() processing (would cause +1s error).
  if (!this->pps_synced_ && !this->pps_flag_) {
    // Before PPS sync, NMEA manages the epoch counter directly.
    this->last_gps_epoch_ = val.timestamp;
  } else if (this->pps_synced_ && !this->pps_flag_) {
    // While PPS is synced, validate epoch against NMEA to detect runaway drift.
    // If the ISR has locked onto spurious edges (e.g. at 1063ms or 1225ms instead
    // of 1000ms), the PPS-managed epoch counter advances faster than real GPS time.
    // NMEA provides the authoritative GPS second; under normal operation they agree
    // within ±1s (NMEA may lag by one second relative to the next PPS). A divergence
    // of more than ±2s means the ISR is tracking spurious edges — trigger a full
    // re-sync so NMEA re-establishes the epoch and coarse clock from scratch.
    int64_t epoch_diff = static_cast<int64_t>(val.timestamp)
                         - static_cast<int64_t>(this->last_gps_epoch_);
    if (epoch_diff > 2 || epoch_diff < -2) {
      ESP_LOGW(TAG, "NMEA/PPS epoch diverged: NMEA=%ld PPS=%ld diff=%lld — re-syncing",
               (long) val.timestamp, (long) this->last_gps_epoch_, (long long) epoch_diff);
      this->pps_synced_ = false;
      this->gps_time_valid_ = false;
      this->has_gps_time_ = false;
      this->last_gps_epoch_ = 0;
      this->last_drift_us_ = 0;
      this->drift_mean_x256_ = 0;
      this->pps_count_ = 0;
    }
  }

  // Set coarse GPS time once as fallback until PPS takes over.
  // Use settimeofday() directly — NOT synchronize_epoch_() — to avoid triggering
  // time_sync_callback_, which causes the HA API to push integer-second time back
  // and override the GPS-corrected clock (~643ms behind actual time).
  if (!this->pps_synced_ && !this->has_gps_time_) {
    struct timeval tv = {.tv_sec = static_cast<time_t>(val.timestamp), .tv_usec = 0};
    settimeofday(&tv, nullptr);
    this->has_gps_time_ = true;
    ESP_LOGI(TAG, "GPS time set (coarse, waiting for PPS): %04d-%02d-%02d %02d:%02d:%02d",
             val.year, val.month, val.day_of_month, val.hour, val.minute, val.second);
  }
}

void GPSPPSTime::update() {
  // Publish crash report from previous boot (once, after HA connection is ready)
  if (this->crash_report_pending_ && this->crash_info_sensor_ != nullptr) {
    this->crash_info_sensor_->publish_state(this->crash_report_);
    this->crash_report_pending_ = false;
  }

  if (this->satellites_sensor_ != nullptr) {
    // Sum per-constellation GSV counts for consistent "in view" total
    uint16_t total = this->last_gps_sat_count_ + this->last_glonass_sat_count_
                     + this->last_galileo_sat_count_;
    this->satellites_sensor_->publish_state(total);
  }

  if (this->gps_satellites_sensor_ != nullptr && this->gp_gsv_sats_ != nullptr && this->gp_gsv_sats_->isValid()) {
    this->gps_satellites_sensor_->publish_state(this->last_gps_sat_count_);
  }
  if (this->glonass_satellites_sensor_ != nullptr && this->gl_gsv_sats_ != nullptr && this->gl_gsv_sats_->isValid()) {
    this->glonass_satellites_sensor_->publish_state(this->last_glonass_sat_count_);
  }
  if (this->galileo_satellites_sensor_ != nullptr && this->ga_gsv_sats_ != nullptr && this->ga_gsv_sats_->isValid()) {
    this->galileo_satellites_sensor_->publish_state(this->last_galileo_sat_count_);
  }

  if (this->nmea_clock_delta_sensor_ != nullptr && this->nmea_clock_delta_valid_) {
    this->nmea_clock_delta_sensor_->publish_state(static_cast<float>(this->nmea_clock_delta_ms_));
  }

  if (this->clock_offset_sensor_ != nullptr && this->pps_synced_) {
    // Filtered clock offset: actual error NTP clients see. Only updated on clean measurements;
    // on spikes, estimates drift from crystal rate (since adjtime skipped the correction).
    this->clock_offset_sensor_->publish_state(static_cast<float>(this->last_clock_offset_us_));
  }
  if (this->pps_drift_sensor_ != nullptr && this->pps_synced_) {
    // Raw PPS drift: unfiltered measurement at each PPS edge.
    // Shows ISR contention and measurement anomalies for diagnostics.
    this->pps_drift_sensor_->publish_state(static_cast<float>(this->last_drift_us_));
  }

  if (this->pps_synced_) {
    auto time = this->now();
    ESP_LOGD(TAG, "PPS-disciplined time: %04d-%02d-%02d %02d:%02d:%02d",
             time.year, time.month, time.day_of_month, time.hour, time.minute, time.second);

    if (this->gps_time_sensor_ != nullptr) {
      char buf[20];
      snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
               time.year, time.month, time.day_of_month, time.hour, time.minute, time.second);
      this->gps_time_sensor_->publish_state(buf);
    }
  }
#ifdef USE_ESP_IDF
  UBaseType_t stack_hwm_words = uxTaskGetStackHighWaterMark(nullptr);
  ESP_LOGD(TAG, "Main stack HWM: %u words (%u bytes)",
           (unsigned) stack_hwm_words, (unsigned) stack_hwm_words * 4);
#endif
}

bool GPSPPSTime::is_synchronized() const {
  if (!this->pps_synced_)
    return false;
  return (millis() - this->last_pps_millis_) < PPS_TIMEOUT_MS;
}

void GPSPPSTime::dump_config() {
  ESP_LOGCONFIG(TAG, "GPS PPS Time:");
  LOG_PIN("  PPS Pin: ", this->pps_pin_);
  ESP_LOGCONFIG(TAG, "  PPS Synced: %s", YESNO(this->pps_synced_));
}

}  // namespace gps_pps_time
}  // namespace esphome
