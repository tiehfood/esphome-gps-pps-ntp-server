#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#ifdef USE_ESP_IDF
#include <sys/socket.h>
#include <netinet/in.h>
#include <atomic>
#include "esp_eth_driver.h"
#include "esp_netif.h"
#else
#include <WiFiUdp.h>
#endif

namespace esphome {
namespace gps_pps_time {
class GPSPPSTime;
}  // namespace gps_pps_time

namespace ntp_server {

/// NTP timestamp: seconds since 1900-01-01
struct NTPTimestamp {
  uint32_t seconds;
  uint32_t fraction;
};

class NTPServer : public Component {
 public:
  void set_port(uint16_t port) { this->port_ = port; }
  void set_time_source(gps_pps_time::GPSPPSTime *source) { this->time_source_ = source; }
#ifdef USE_ESP_IDF
  void set_hook_latency_sensor(sensor::Sensor *sensor) { this->hook_latency_sensor_ = sensor; }
  void set_rx_stamp_gap_sensor(sensor::Sensor *sensor) { this->rx_stamp_gap_sensor_ = sensor; }
#endif

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

 protected:
  // receive_ts is sampled by the caller right after recvfrom()/parsePacket() returns,
  // not taken internally -- on the ESP-IDF path that call site is recv_task_(), and
  // T2 must reflect arrival, not whatever this function does first.
  void build_ntp_response_(const uint8_t *request, uint8_t *response, const NTPTimestamp &receive_ts);
  NTPTimestamp get_ntp_timestamp_(int32_t offset_us = 0);
  bool is_time_synchronized_();

  uint16_t port_{123};
  gps_pps_time::GPSPPSTime *time_source_{nullptr};

#ifdef USE_ESP_IDF
  int socket_fd_{-1};

  /// log2(s) of max(clock resolution, clock read cost), measured in setup (RFC 5905 11.1).
  int8_t precision_{-20};

  /// EWMA of sendto() duration, us. LWIP_TCPIP_CORE_LOCKING + the W5500's
  /// spi_device_polling_transmit mean sendto() runs the SPI write inline, so the
  /// packet leaves this long after T3 is stamped. Added to T3 to compensate.
  int32_t send_us_{0};

  /// Dedicated FreeRTOS task blocked in recvfrom() -- stamps T2 on return instead of
  /// whenever ESPHome's shared loop next polls us. Runs for the component's lifetime;
  /// no handle is kept, matching esp32_camera's framebuffer_task precedent.
  static void recv_task_(void *param);

  // ---- NTP receive-timestamp input-path hook ----
  // Stamps T2 in the W5500 driver's own task, at the point esp_eth hands the frame
  // to the network stack -- before lwIP, before recv_task_() wakes. Removes the
  // FreeRTOS scheduling gap between "frame delivered" and "our task resumes" from
  // the T2 measurement. See docs/superpowers/plans/2026-09-03-ntp-serving-latency.md.

  /// Ring slot: maps a client's own NTP transmit timestamp (echoed back to us in the
  /// request, and unique enough per request for this purpose) to the arrival time
  /// eth_input_hook_() captured for it. eth_input_hook_() is the sole writer -- the
  /// W5500 driver delivers one frame at a time from its own task, so slot writes never
  /// race each other -- recv_task_() is the sole reader. `seq` is a seqlock: odd means
  /// a write is in progress, even means key/t are a consistent snapshot. Needed because
  /// a 64-bit `t` (and the 8-byte `key`) tears on Xtensa; see .claude/rules/firmware.md.
  struct HookEntry {
    std::atomic<uint32_t> seq{0};
    uint8_t key[8]{};
    int64_t t{0};
  };
  static const int HOOK_RING_SIZE = 8;
  HookEntry hook_ring_[HOOK_RING_SIZE];
  /// Next slot to write. Touched only by eth_input_hook_() (single writer).
  uint8_t hook_ring_next_{0};

  /// esp_netif every frame must be forwarded to. Captured once in setup(); null means
  /// the hook was never installed (get_eth_netif() failed) -- eth_input_hook_() is then
  /// also never registered as the input path, so it never runs with this still null.
  esp_netif_t *eth_netif_{nullptr};

  /// Diagnostic only: microseconds between the hook's stamp and recv_task_()'s own
  /// stamp on the most recent ring hit -- the latency this hook exists to remove.
  /// Written only by recv_task_(); published from loop() (the main ESPHome task) --
  /// never from recv_task_() itself, which must make no ESPHome API calls.
  volatile int32_t last_hook_latency_us_{0};
  volatile bool hook_latency_pending_{false};
  /// Phase 3 Step 3: microseconds between the driver reading Sn_RX_RSR and the input
  /// hook stamping T2 -- the SPI transfer + dispatch cost currently inside T2, and so
  /// the upper bound on what stamping T2 earlier could recover.
  volatile int32_t last_rx_stamp_gap_us_{0};
  volatile bool rx_stamp_gap_pending_{false};
  sensor::Sensor *rx_stamp_gap_sensor_{nullptr};
  sensor::Sensor *hook_latency_sensor_{nullptr};

  /// Registered with esp_eth_update_input_path() as the driver's stack_input. Runs in
  /// the W5500 driver's own task for EVERY received frame -- no ESPHome API calls, no
  /// blocking, no allocation, no gettimeofday(). Must call esp_netif_receive() on every
  /// path: failing to forward a frame here takes down all networking on the device.
  static esp_err_t eth_input_hook_(esp_eth_handle_t eth_handle, uint8_t *buffer, uint32_t length, void *priv);
  /// Records (key, t) in the ring. Called only from eth_input_hook_() (driver task).
  void hook_record_(const uint8_t *key, int64_t t);
  /// Looks up a key written by hook_record_(). Called only from recv_task_() (our task).
  bool hook_lookup_(const uint8_t *key, int64_t *t_out);
  /// Converts an absolute Unix-epoch microsecond value to an NTPTimestamp.
  static NTPTimestamp micros_epoch_to_ntp_timestamp_(int64_t unix_us);
  /// Reconstructs the wall-clock time at a past esp_timer_get_time() reading, the same
  /// way GPSPPSTime::apply_pps_correction_() reconstructs the PPS edge's wall-clock time
  /// -- gettimeofday()+esp_timer_get_time() read back-to-back here, minus the elapsed
  /// delta since hook_us. Never calls gettimeofday() at the hook itself.
  NTPTimestamp hook_to_ntp_timestamp_(int64_t hook_us);
#else
  WiFiUDP udp_;
#endif
};

}  // namespace ntp_server
}  // namespace esphome
