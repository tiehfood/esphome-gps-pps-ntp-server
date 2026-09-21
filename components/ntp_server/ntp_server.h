#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "send_estimator.h"
#include "hook_ring.h"
#include "rx_admission.h"
#include "deadman.h"
#include "ntp_tx_patch.h"

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
  void set_arp_waits_sensor(sensor::Sensor *sensor) { this->arp_waits_sensor_ = sensor; }
  void set_w5500_cmd_retries_sensor(sensor::Sensor *sensor) { this->w5500_cmd_retries_sensor_ = sensor; }
  void set_w5500_cmd_max_sensor(sensor::Sensor *sensor) { this->w5500_cmd_max_sensor_ = sensor; }
  /// Stall investigation: per-request records and the W5500 network recorder, pulled over
  /// UDP DIAG_PORT after a run. Off by default; adds no traffic while on.
  void set_diagnostics(bool enable);
  /// Refuse on a tight (200us) or missing INTn edge lead, not just the legacy 1ms one. On by
  /// default since 2026-09-11 (verified by interleaved A/B); kept switchable for later A/Bs. The
  /// verdict is re-evaluated per request, so flipping it takes effect immediately.
  void set_strict_rx_admission(bool enable) { this->strict_rx_admission_ = enable; }

  /// Design E ("NTP INT Edge T2"): stamp T2 from the W5500's own INTn hardware edge instead of
  /// the burst-start SPI transaction, when the edge is usable, its lead is a sane
  /// 0 < lead_us < RX_EDGE_LEAD_STRICT_US, and it has not already been consumed for another
  /// frame in the same read burst (see eth_input_hook_()'s dedup against
  /// last_int_edge_seq_for_t2_). Default OFF for the pre-registered A/B; see
  /// docs/superpowers/plans/2026-09-09-p4-ntp-probe.md. Read on the receive path
  /// (eth_input_hook_(), the W5500 driver's own task), written only from the main task via this
  /// setter -- same volatile-bool cross-task pattern as strict_rx_admission_ above, chosen for
  /// the same reason: a single plain flag, evaluated fresh per frame, needs no stronger
  /// synchronisation than that.
  void set_int_edge_t2(bool enable) { this->int_edge_t2_ = enable; }
  /// For the YAML switch's state lambda (mirrors short_int_wait_active()'s role for that
  /// switch), so the UI reflects the policy actually in effect rather than assuming turn_on/
  /// turn_off always succeeded.
  bool int_edge_t2_active() const { return this->int_edge_t2_; }

  /// Design G ("NTP T2 Frame Start"), pre-registered 2026-09-21: after T2 is settled (whatever
  /// design E left it at -- the INTn edge if enabled and usable, otherwise the burst-start/
  /// gap-refined hook stamp), subtract the frame's own on-wire duration so T2 names the frame's
  /// FIRST bit (the SFD) instead of its last. That matches where the GPS-referenced P4 probe
  /// stamps T1/T4, removing a ~+3.8 us client-visible bias for a typical 90-byte request. Default
  /// OFF for the pre-registered A/B; see docs/superpowers/plans/2026-09-09-p4-ntp-probe.md.
  /// Composes with design E rather than replacing it -- see eth_input_hook_() for why the order
  /// (edge substitution first, duration subtraction second) is the physically correct one. Same
  /// cross-task volatile-bool pattern as int_edge_t2_ above: read on the receive path
  /// (eth_input_hook_(), the W5500 driver's own task), written only from the main task here.
  void set_t2_frame_start(bool enable) { this->t2_frame_start_ = enable; }
  /// For the YAML switch's state lambda.
  bool t2_frame_start_active() const { return this->t2_frame_start_; }

  /// Design A'' ("W5500 Short Interrupt Wait"): shortens the chip's own INTLEVEL from the
  /// driver's boot default (0xFFFF, ~1.748 ms) to 0x0FFF (~109 us). On by default since
  /// 2026-09-13 (verified interleaved A/B: replies served late > 150 us 0.64 -> 0.19%, for
  /// +0.18 points of refusals). A live register write with no independent recovery path if it
  /// went wrong, so the policy is applied only after a deferred boot delay -- see loop() and
  /// SHORT_INT_WAIT_BOOT_DELAY_MS -- which guarantees every boot an OTA window with the
  /// driver's own 0xFFFF in place, rather than the time-bounded dead-man revert this replaced.
  /// Always sets short_int_wait_policy_, whether or not the register write itself succeeds.
  /// Main task only.
  void set_short_int_wait(bool enable);
  /// True once the last INTLEVEL write AND read-back actually verified the short value is live
  /// in hardware (short_wait_effective_) -- not just "was asked for". Public and const-qualified
  /// for callers outside the hot path (the YAML switch's state lambda, the INT diag command);
  /// recv_task_() reads short_wait_effective_ directly instead.
  bool short_int_wait_active() const;

  /// Design B ("NTP Post-Write T3"): rewrites T3 (and the UDP checksum, if present) into the
  /// reply's bytes already queued in the W5500 TX buffer, immediately before the SEND that
  /// transmits it -- closer to the wire than any timestamp written before sendto(). On by
  /// default since 2026-09-11 (enabled at the end of setup()); main task only.
  void set_post_write_t3(bool enable);
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
  /// `clock_read_us`, if non-null, receives the esp_timer_get_time() reading this timestamp was
  /// actually computed from (the anchor path's input, or one taken immediately next to the
  /// gettimeofday() fallback) -- ESP-IDF only; see the "T3-computation gap" diagnostic (design C)
  /// in recv_task_(). Left untouched on non-ESP-IDF builds.
  NTPTimestamp get_ntp_timestamp_(int32_t offset_us = 0, int64_t *clock_read_us = nullptr);
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

  /// Design C ("T3-computation gap" diagnostic): the exact esp_timer_get_time() reading the
  /// estimate path's most recent T3 was computed from -- set by get_ntp_timestamp_()'s
  /// out-parameter, via build_ntp_response_(), and read back in recv_task_() against its own
  /// `t0` to measure the gap the old (pre-design-B) T3 was early by. Diagnostic only; never
  /// read on the served-timestamp path.
  int64_t last_t3_clock_read_us_{0};

  /// Dedicated FreeRTOS task blocked in recvfrom() -- stamps T2 on return instead of
  /// whenever ESPHome's shared loop next polls us. Runs for the component's lifetime;
  /// no handle is kept, matching esp32_camera's framebuffer_task precedent.
  static void recv_task_(void *param);

  // ---- NTP receive-timestamp input-path hook ----
  // Stamps T2 in the W5500 driver's own task, at the point esp_eth hands the frame
  // to the network stack -- before lwIP, before recv_task_() wakes. Removes the
  // FreeRTOS scheduling gap between "frame delivered" and "our task resumes" from
  // the T2 measurement. See docs/superpowers/plans/2026-09-03-ntp-serving-latency.md.

  /// What the hook learned about one request, handed from the driver task to recv_task_()
  /// via hook_ring_. eth_input_hook_() is the sole writer -- the W5500 driver delivers one
  /// frame at a time from its own task, so writes never race each other -- recv_task_() is
  /// the sole reader. `t` also serves as hook_ring_'s newest-wins ordering key.
  struct HookInfo {
    int64_t t{0};
    /// Diagnostic only: the usable INTn-to-burst lead, or -1 (kept for the existing sensor and
    /// REQ dump; the admission verdict is computed fresh in recv_task_() from edge_usable/lead_us
    /// below, against whatever set_strict_rx_admission() is currently set to).
    int32_t int_lead_us{-1};
    int32_t rx_gap_us{-1};
    /// Raw evaluate_rx_edge() output for this request's INTn edge, unfiltered by int_lead_us's
    /// usable-only convention -- rx_admission() needs the lead even when it is not "usable".
    bool edge_usable{false};
    int32_t lead_us{0};
    /// Whether an INTn edge had EVER been captured (w5500_int_stamp().seq != 0) as of this
    /// request -- distinguishes "no edge because it's early boot" from "no edge, and it should
    /// have had one".
    bool capture_armed{false};
    /// Sn_RX_RSR value and the hook's own `length` argument at this frame's arrival --
    /// diagnostic-only, set only when the size-read stamp provably belongs to this frame (same
    /// condition as the T2 stamp above). -1 when not measured.
    int32_t rx_rsr{-1};
    int32_t frame_len{-1};
    /// Design E ("NTP INT Edge T2"): true when this request's T2 (the `t` field above) was
    /// taken from the INTn hardware edge rather than the burst-start SPI transaction. Set only
    /// in eth_input_hook_(); recv_task_() copies it into DiagRecord::flags's DIAG_INT_EDGE_T2
    /// bit and nowhere else -- it does not change how T2 is used, only where it diagnostically
    /// says the value came from.
    bool t2_from_edge{false};
    /// Design G ("NTP T2 Frame Start"): true when this request's T2 (the `t` field above) was
    /// further shifted from "frame received" to "frame started" by subtracting the frame's
    /// on-wire duration. Set only in eth_input_hook_(); recv_task_() copies it into
    /// DiagRecord::flags's DIAG_T2_FRAME_START bit and nowhere else.
    bool t2_from_frame_start{false};
  };
  /// Keyed on the client's transmit timestamp + source IP + source port; see hook_ring.h for why.
  HookRing<HookInfo, 8> hook_ring_;

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

  /// Requests whose reply needed to resolve ARP before it could be sent -- i.e. wait_for_arp()
  /// had a cache miss, whether or not it went on to resolve within ARP_WAIT_MAX_US.
  std::atomic<uint32_t> arp_waits_{0};
  sensor::Sensor *arp_waits_sensor_{nullptr};

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

  /// On by default: refuse on a tight (200us) or missing INTn edge lead. See rx_admission.h.
  /// Verified 2026-09-11 in an interleaved A/B against a pre-registered prediction: replies off
  /// by more than 1 ms 6.86 -> 0.53 per 1000, refusals 1.41 -> 2.79 %. Kept switchable for later
  /// A/Bs; evaluated fresh per request, so flipping it takes effect immediately.
  volatile bool strict_rx_admission_{true};

  /// Design E ("NTP INT Edge T2"), pre-registered 2026-09-17, **ON by default since 2026-09-18**.
  /// Stamps T2 from the W5500's own INTn edge rather than the burst-start SPI transaction, which
  /// is a measured 4-18 us (median 13) later -- worth ~6.5 us of client-visible offset, since a
  /// T2 stamped L late biases the offset by +L/2. Verification: E1 100 % of served requests
  /// corrected; two interleaved Pi A/Bs at -7.23 and -7.00 us against a -6.5 us prediction made
  /// before the code existed; 23 h soak clean (refusals 12.3/h, hook latency max 6.4 ms, no
  /// reboot). The accuracy gain itself could NOT be verified end-to-end -- that needed the P4
  /// probe, which was repurposed on 2026-09-17 -- so it rests on those two agreeing measurements
  /// plus the physics: the INTn edge provably precedes our stamp. See set_int_edge_t2() for the
  /// cross-task treatment.
  volatile bool int_edge_t2_{true};

  /// Design G ("NTP T2 Frame Start"), **ON by default since 2026-09-21**. Subtracts
  /// (frame_len + FCS) x 80 ns/byte from T2 -- see set_t2_frame_start() and eth_input_hook_() for
  /// the rationale and guard bounds. Verified by a GPS-referenced interleaved A/B: ON +3.67 us vs
  /// OFF +6.27 us, difference **-2.60 us** against a -3.8 +/- 2 us prediction committed before the
  /// code existed; the OFF arm reproduced that day's independent baseline (+8.16 us over 3 runs).
  /// Every block 100 % clean. Composes with design E; both are needed for the full correction.
  volatile bool t2_frame_start_{true};
  /// Dedup state for design E: the last INTn edge `seq` (w5500_int_stamp()) already consumed as
  /// a T2 stamp. evaluate_rx_edge() has no sequence de-duplication of its own -- two frames in
  /// one driver read burst would otherwise both see the same edge, back-dating the second
  /// frame's T2 by the inter-frame gap (the exact failure mode that shelved this feature in
  /// 2026-09-04). Written and read ONLY inside eth_input_hook_(), which the W5500 driver calls
  /// once per frame, sequentially, from its own single task -- so this needs no volatile/atomic,
  /// unlike the cross-task flags above. 0 means "never consumed" and is never a real seq (a real
  /// edge's seq is nonzero, same convention as HookInfo::capture_armed).
  uint32_t last_int_edge_seq_for_t2_{0};

  /// Reasons a request can be refused, recorded in DiagRecord::refuse_reason and the REQ dump.
  static constexpr uint8_t REFUSE_NONE = 0;
  static constexpr uint8_t REFUSE_OLD_EDGE = 1;
  static constexpr uint8_t REFUSE_NO_EDGE = 2;
  static constexpr uint8_t REFUSE_HOOK_MISS = 3;
  static constexpr uint8_t REFUSE_ARP_UNRESOLVED = 4;

  // ---- Stall investigation diagnostics (switch-gated, pulled over UDP DIAG_PORT) ----
  /// One served or refused request, as the server saw it. client_tx joins it to the client's
  /// own record of the same exchange; -1 means not measured.
  struct DiagRecord {
    int64_t t2_us;          ///< esp_timer at T2 (hook stamp; at lookup time if the hook missed)
    uint8_t client_tx[8];   ///< the client's transmit field
    /// DIAG_* bits. Widened from uint8_t to uint16_t 2026-09-17 (design E): the 8 bits below
    /// were all already assigned, so DIAG_INT_EDGE_T2 needed a 9th. The REQ dump's `%x` format
    /// is unaffected -- it already prints however many hex digits the value needs.
    uint16_t flags;
    int32_t int_lead_us;
    int32_t rx_gap_us;
    int32_t hook_latency_us;
    int32_t send_us;        ///< t0 to Sn_CR = SEND, unclipped (stalled sends included)
    int32_t sendto_us;
    int32_t lock_wait_us;   ///< taking lwIP's core lock just before the reply was built
    int32_t estimate_us;    ///< send estimate this reply's T3 used
    int32_t arp_wait_us;    ///< -1 not evaluated (refused before the ARP step), 0 cached, >0 waited
    uint8_t refuse_reason;  ///< REFUSE_* above; REFUSE_NONE when served
    int8_t send_class;      ///< frame class of the stamped SEND (ethernet::W5500FrameClass), -1 if none
    /// t0 to the start/end of the SPI transfer that wrote this reply's payload into the W5500 TX
    /// buffer -- splits the send path into "before the frame reached the W5500" (build + lwIP +
    /// core lock, ending at tx_write_start_us) vs "SPI transfer + Sn_CR=SEND" (tx_write_end_us to
    /// send_us). -1 unless this SEND is provably this reply (send_class == NTP) and both
    /// snapshots were measured (see w5500_send_stamp()).
    int32_t tx_write_start_us;
    int32_t tx_write_end_us;
    /// Sn_RX_RSR value and hook `length` at this request's arrival; see HookInfo. -1 if the hook
    /// missed or the size-read stamp did not provably belong to this frame.
    int32_t rx_rsr;
    int32_t frame_len;
    /// Design B ("NTP Post-Write T3"): whether this reply's T3 (and checksum, if present) was
    /// rewritten in the TX buffer just before SEND. patch_to_send_us/patch_pred_us are -1 when
    /// patched == 0.
    uint8_t patched;
    int32_t patch_to_send_us;  ///< send_cmd_us - patch_us for this SEND
    int32_t patch_pred_us;     ///< the patch_delay_ estimate this reply's patched T3 used
    /// Design C ("T3-computation gap" diagnostic): t0 - last_t3_clock_read_us_, i.e. how much
    /// earlier the estimate path's T3 read its clock than recv_task_()'s own t0. -1 when not
    /// measured (e.g. refused requests, which never reach build_ntp_response_()).
    int32_t t3_calc_to_t0_us;
  };
  static constexpr uint16_t DIAG_HOOK_HIT = 0x01;
  static constexpr uint16_t DIAG_RX_STALLED = 0x02;
  static constexpr uint16_t DIAG_REFUSED = 0x04;
  static constexpr uint16_t DIAG_RESEEDED = 0x08;
  static constexpr uint16_t DIAG_SEND_LONG = 0x10;
  static constexpr uint16_t DIAG_FALLBACK = 0x20;
  static constexpr uint16_t DIAG_ARP_MISS = 0x40;
  static constexpr uint16_t DIAG_SEND_NOT_NTP = 0x80;
  /// Design E ("NTP INT Edge T2"): this reply's T2 came from the INTn hardware edge rather than
  /// the burst-start SPI transaction (HookInfo::t2_from_edge). The 8 bits above were all taken,
  /// hence the field's uint8_t -> uint16_t widening above.
  static constexpr uint16_t DIAG_INT_EDGE_T2 = 0x100;
  /// Design G ("NTP T2 Frame Start"): this reply's T2 was further shifted to the frame's start
  /// by subtracting its on-wire duration (HookInfo::t2_from_frame_start).
  static constexpr uint16_t DIAG_T2_FRAME_START = 0x200;
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
  /// Records one request in hook_ring_, keyed on (tx, src_ip, src_port). Called only from
  /// eth_input_hook_() (driver task).
  void hook_record_(const uint8_t *tx, uint32_t src_ip, uint16_t src_port, const HookInfo &info);
  /// Looks up the newest hook_record_() entry for (tx, src_ip, src_port) no older than
  /// HookRing::MAX_AGE_US as of now_us. Called only from recv_task_() (our task).
  bool hook_lookup_(const uint8_t *tx, uint32_t src_ip, uint16_t src_port, int64_t now_us, HookInfo *out);
  /// Converts an absolute Unix-epoch microsecond value to an NTPTimestamp.
  static NTPTimestamp micros_epoch_to_ntp_timestamp_(int64_t unix_us);
  /// Reconstructs the wall-clock time at a past esp_timer_get_time() reading, the same
  /// way GPSPPSTime::apply_pps_correction_() reconstructs the PPS edge's wall-clock time
  /// -- gettimeofday()+esp_timer_get_time() read back-to-back here, minus the elapsed
  /// delta since hook_us. Never calls gettimeofday() at the hook itself.
  NTPTimestamp hook_to_ntp_timestamp_(int64_t hook_us);
  bool anchor_epoch_us_(int64_t at_us, int64_t &out_us);

  // ---- Design A'': "W5500 Short Interrupt Wait", on by default since 2026-09-13 ----
  /// The policy the switch asked for, independent of short_wait_effective_ (the verified
  /// hardware state). Load-bearing: without this separation, loop()'s deferred boot-enable
  /// would silently re-enable an A/B "off" block 60 s later, corrupting every future A/B.
  /// Defaults on to match the switch's ALWAYS_ON restore_mode. Set only from
  /// set_short_int_wait(); main task only.
  bool short_int_wait_policy_{true};
  /// The value actually read back after the last write, so short_int_wait_active() and the INT
  /// diag command reflect hardware state rather than merely "was asked for".
  uint16_t last_int_level_readback_{0};
  /// Design A' (edge-aware admission): true only once set_short_int_wait(true) has both written
  /// AND read back W5500_INT_LEVEL_SHORT -- i.e. reflects hardware state, not just "was asked
  /// for". Set false on disable and on any failed or mismatched read-back (see loop()'s
  /// steady-state re-check). recv_task_() reads this plain bool directly rather than calling
  /// short_int_wait_active() (a public, const-qualified convenience for callers outside the hot
  /// path, e.g. the INT diag command) -- one fewer function call per request, same value.
  volatile bool short_wait_effective_{false};
  /// millis() deadline for the next INTLEVEL action: loop()'s deferred boot-enable (while
  /// !short_wait_effective_) or its steady-state re-check (while short_wait_effective_) -- the
  /// two are mutually exclusive, so one timer serves both. Wrap-safe via the same subtraction
  /// idiom used elsewhere (e.g. the ARP refresh timer). Main task only.
  uint32_t int_level_recheck_ms_{0};

  // ---- Design B: "NTP Post-Write T3" ----
  /// EWMA estimate of the patch-callback-to-SEND delay, learned from every SEND that was
  /// actually patched (see recv_task_()). Independent of send_estimator_, which predicts a much
  /// longer interval (build through sendto()).
  PatchDelayEstimator patch_delay_;
  /// Trampoline registered with ethernet::w5500_set_tx_patch(); runs in the transmitting task's
  /// context, inside sendto() -- see build_patch_t3_() for what that forbids.
  static bool tx_patch_trampoline_(void *ctx, uint32_t now_us, uint8_t t3_out[8]);
  bool build_patch_t3_(uint32_t now_us, uint8_t t3_out[8]);
#else
  WiFiUDP udp_;
#endif
};

}  // namespace ntp_server
}  // namespace esphome
