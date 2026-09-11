#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "send_estimator.h"

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
  void set_arp_primes_sensor(sensor::Sensor *sensor) { this->arp_primes_sensor_ = sensor; }
  void set_t3_error_sensor(sensor::Sensor *sensor) { this->t3_error_sensor_ = sensor; }
  /// A/B: derive served timestamps from the PPS anchor instead of gettimeofday().
  void set_use_pps_anchor(bool enable) { this->use_pps_anchor_ = enable; }
  void set_int_lead_sensor(sensor::Sensor *sensor) { this->int_lead_sensor_ = sensor; }
  void set_refused_sensor(sensor::Sensor *sensor) { this->refused_sensor_ = sensor; }
  void set_w5500_cmd_retries_sensor(sensor::Sensor *sensor) { this->w5500_cmd_retries_sensor_ = sensor; }
  void set_w5500_cmd_max_sensor(sensor::Sensor *sensor) { this->w5500_cmd_max_sensor_ = sensor; }
  /// Stall investigation: per-request records and the W5500 network recorder, pulled over
  /// UDP DIAG_PORT after a run. Off by default; adds no traffic while on.
  void set_diagnostics(bool enable);
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
  /// Precision of the PPS-anchor path (esp_timer), measured separately at setup.
  int8_t precision_anchor_{-20};
  /// Default ON. Measured against the gettimeofday path over 7 interleaved blocks:
  /// precision -16 -> -19 (8x), root dispersion 244 -> 92 us (2.7x), and IQR 27.6 -> 22.9 us
  /// (tighter in 10 of 12 pairwise block comparisons). Falls back automatically whenever no
  /// usable anchor exists, so this is safe to leave on.
  volatile bool use_pps_anchor_{true};

  /// Predicts how long after the reply is built the W5500 is told to transmit, so T3 names the
  /// instant the packet leaves. LWIP_TCPIP_CORE_LOCKING + the W5500's polling SPI mean sendto()
  /// runs the SPI write inline. Rules and their tests: send_estimator.h,
  /// tests/send_estimator_test.cpp.
  SendEstimator send_estimator_;

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
  /// What the hook learned about one request, handed from the driver task to recv_task_().
  struct HookInfo {
    int64_t t{0};
    /// The receive burst began long after the W5500 raised INTn: T2 is late, refuse.
    bool rx_stalled{false};
    int32_t int_lead_us{-1};
    int32_t rx_gap_us{-1};
  };
  struct HookEntry {
    std::atomic<uint32_t> seq{0};
    uint8_t key[8]{};
    HookInfo info;
  };
  static const int HOOK_RING_SIZE = 8;
  HookEntry hook_ring_[HOOK_RING_SIZE];
  /// Next slot to write. Touched only by eth_input_hook_() (single writer).
  uint8_t hook_ring_next_{0};

  /// esp_netif every frame must be forwarded to. Captured once in setup(); null means
  /// the hook was never installed (get_eth_netif() failed) -- eth_input_hook_() is then
  /// also never registered as the input path, so it never runs with this still null.
  esp_netif_t *eth_netif_{nullptr};

  /// Largest-magnitude sample since the last publish, and how many samples fed it. Written
  /// from the driver task and ntp_recv, drained by loop() every TELEMETRY_INTERVAL_MS -- the
  /// main ESPHome task, since the API must not be called from the other two. Lock-free, so
  /// neither writer can block. Adjacent windows may trade a single sample; that is fine for
  /// a diagnostic whose purpose is to show the worst case.
  struct WindowMax {
    std::atomic<int32_t> value{0};
    std::atomic<uint32_t> count{0};
    void add(int32_t v) {
      int32_t cur = this->value.load(std::memory_order_relaxed);
      while ((v < 0 ? -v : v) > (cur < 0 ? -cur : cur) &&
             !this->value.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {
      }
      this->count.fetch_add(1, std::memory_order_relaxed);
    }
    bool take(int32_t &out) {
      if (this->count.exchange(0, std::memory_order_relaxed) == 0)
        return false;
      out = this->value.exchange(0, std::memory_order_relaxed);
      return true;
    }
  };
  /// Microseconds between the hook's stamp and recv_task_()'s own stamp on a ring hit --
  /// the latency this hook exists to remove.
  WindowMax hook_latency_win_;
  /// Phase 3 Step 3: microseconds between the driver reading Sn_RX_RSR and the input
  /// hook stamping T2 -- the SPI transfer + dispatch cost currently inside T2, and so
  /// the upper bound on what stamping T2 earlier could recover.
  /// Phase 3 Step 10 (ARP priming). lwIP does not learn a peer's MAC from an incoming
  /// IP packet (ETHARP_TRUST_IP_MAC is off), and entries age out after ARP_MAXAGE = 300 s.
  /// A client polling less often than that finds a cold cache, so our sendto() triggers an
  /// ARP round trip mid-reply -- a rare but large outlier between T2 and T3. Re-resolving
  /// recent clients on a timer keeps their entries warm so the reply never waits.
  static const uint8_t ARP_CLIENT_SLOTS = 8;
  static const uint32_t ARP_REFRESH_INTERVAL_MS = 60000;  // ARP_MAXAGE is 300 s
  uint32_t arp_clients_[ARP_CLIENT_SLOTS]{};  // IPv4, network byte order; 0 = empty slot
  uint8_t arp_client_next_{0};
  uint32_t arp_last_refresh_ms_{0};
  uint32_t arp_primes_{0};
  bool arp_primes_pending_{false};
  sensor::Sensor *arp_primes_sensor_{nullptr};
  void note_arp_client_(uint32_t addr);
  void refresh_arp_entries_();

  /// Microseconds from the hardware INTn edge to our burst-start T2 stamp: GPIO ISR latency
  /// plus driver task wake, normally 3-25 us. A stalled receive shows here as ~100 ms.
  WindowMax int_lead_win_;
  sensor::Sensor *int_lead_sensor_{nullptr};
  /// (actual Sn_CR=SEND instant) - (the send estimate). Positive means we stamped T3
  /// EARLIER than the packet really departed, i.e. we under-predict the send cost.
  WindowMax t3_error_win_;
  sensor::Sensor *t3_error_sensor_{nullptr};
  WindowMax rx_stamp_gap_win_;
  sensor::Sensor *rx_stamp_gap_sensor_{nullptr};
  sensor::Sensor *hook_latency_sensor_{nullptr};
  /// Requests refused because their receive stalled (total since boot).
  std::atomic<uint32_t> refused_{0};
  sensor::Sensor *refused_sensor_{nullptr};
  sensor::Sensor *w5500_cmd_retries_sensor_{nullptr};
  sensor::Sensor *w5500_cmd_max_sensor_{nullptr};
  uint32_t telemetry_last_ms_{0};

  // ---- Stall investigation diagnostics (switch-gated, pulled over UDP DIAG_PORT) ----
  /// One served or refused request, as the server saw it. client_tx joins it to the client's
  /// own record of the same exchange; -1 means not measured.
  struct DiagRecord {
    int64_t t2_us;          ///< esp_timer at T2 (hook stamp; at lookup time if the hook missed)
    uint8_t client_tx[8];   ///< the client's transmit field
    uint8_t flags;          ///< DIAG_* bits
    int32_t int_lead_us;
    int32_t rx_gap_us;
    int32_t hook_latency_us;
    int32_t send_us;        ///< t0 to Sn_CR = SEND, unclipped (stalled sends included)
    int32_t sendto_us;
    int32_t lock_wait_us;   ///< taking lwIP's core lock just before the reply was built
    int32_t estimate_us;    ///< send estimate this reply's T3 used
  };
  static constexpr uint8_t DIAG_HOOK_HIT = 0x01;
  static constexpr uint8_t DIAG_RX_STALLED = 0x02;
  static constexpr uint8_t DIAG_REFUSED = 0x04;
  static constexpr uint8_t DIAG_RESEEDED = 0x08;
  static constexpr uint8_t DIAG_SEND_LONG = 0x10;
  static constexpr uint8_t DIAG_FALLBACK = 0x20;
  /// 1024 records: 4.3 min at a 4 Hz client, so a whole test run joins without a mid-run dump
  /// (a dump is itself outbound traffic).
  static constexpr uint16_t DIAG_RING_SIZE = 1024;
  struct DiagSlot {
    std::atomic<uint32_t> seq{0};
    DiagRecord rec{};
  };
  /// Allocated on first enable by the main task, before diagnostics_ turns on.
  DiagSlot *diag_ring_{nullptr};
  /// Next slot to write. recv_task_() is the only writer.
  uint16_t diag_next_{0};
  volatile bool diagnostics_{false};
  int diag_socket_{-1};
  /// recv_task_() only.
  void diag_record_(const DiagRecord &rec);
  /// loop() only: opens/closes the dump socket with the switch and answers CLK / REQ / NET.
  void diag_serve_();

  /// Registered with esp_eth_update_input_path() as the driver's stack_input. Runs in
  /// the W5500 driver's own task for EVERY received frame -- no ESPHome API calls, no
  /// blocking, no allocation, no gettimeofday(). Must call esp_netif_receive() on every
  /// path: failing to forward a frame here takes down all networking on the device.
  static esp_err_t eth_input_hook_(esp_eth_handle_t eth_handle, uint8_t *buffer, uint32_t length, void *priv);
  /// Records (key, t) in the ring. Called only from eth_input_hook_() (driver task).
  void hook_record_(const uint8_t *key, const HookInfo &info);
  /// Looks up a key written by hook_record_(). Called only from recv_task_() (our task).
  bool hook_lookup_(const uint8_t *key, HookInfo *out);
  /// Converts an absolute Unix-epoch microsecond value to an NTPTimestamp.
  static NTPTimestamp micros_epoch_to_ntp_timestamp_(int64_t unix_us);
  /// Reconstructs the wall-clock time at a past esp_timer_get_time() reading, the same
  /// way GPSPPSTime::apply_pps_correction_() reconstructs the PPS edge's wall-clock time
  /// -- gettimeofday()+esp_timer_get_time() read back-to-back here, minus the elapsed
  /// delta since hook_us. Never calls gettimeofday() at the hook itself.
  NTPTimestamp hook_to_ntp_timestamp_(int64_t hook_us);
  bool anchor_epoch_us_(int64_t at_us, int64_t &out_us);
#else
  WiFiUDP udp_;
#endif
};

}  // namespace ntp_server
}  // namespace esphome
