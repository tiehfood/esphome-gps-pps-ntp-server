#include "ntp_server.h"
#include "esphome/core/log.h"
#include "esphome/components/gps_pps_time/gps_pps_time.h"

#include <sys/time.h>
#include <cerrno>
#include <cstring>

#ifdef USE_ESP_IDF
#include <unistd.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <atomic>
#include "esphome/components/ethernet/ethernet_component.h"
#include <lwip/etharp.h>
#include <lwip/tcpip.h>
#include <esp_netif_net_stack.h>
#endif

namespace esphome {
namespace ntp_server {

static const char *const TAG = "ntp_server";

/// Offset between Unix epoch (1970) and NTP epoch (1900) in seconds
static const uint32_t NTP_UNIX_OFFSET = 2208988800UL;

/// NTP packet size
static const int NTP_PACKET_SIZE = 48;

/// Root dispersion budget, seconds. Reflects measured server-added error;
/// revise down once serving latency is fixed. Floor matches ntpd MINDISTANCE.
/// RFC 5905 s11.1: root dispersion is a BOUND on our maximum error relative to the
/// reference clock, not a typical value. 5 ms was picked before anything had been
/// measured. Budget as actually measured on this device:
///   PPS -> system clock discipline   ~6 us typical; +/-50 us is the spike-filter bound
///   timestamp precision (2^-15 s)     30.5 us
///   T2 stamping residual              unmeasured -- no absolute reference at the wire;
///                                     bounded above by the 287 us SPI gap Step 3 removed
/// 1 ms keeps roughly 5x margin over everything measurable while still covering the term
/// that is not. Do not go below this without a same-segment reference to measure T2's
/// absolute accuracy: under-advertising makes us a falseticker, which is the exact bug
/// that motivated advertising a non-zero dispersion in the first place.
static const float ROOT_DISP_BASE_S = 0.001f;
/// Dispersion growth per second since last PPS (~10ppm crystal).
static const float ROOT_DISP_RATE_S_PER_S = 10.0e-6f;
/// NTP short format is 16.16 fixed point; 1 LSB = 15.259us.
static const float NTP_SHORT_SCALE = 65536.0f;

/// Bounds for learning the send-duration EWMA. Below the floor the packet was
/// queued (ARP miss), above the ceiling something stalled; neither is typical.
static const int32_t SEND_US_MIN = 50;
static const int32_t SEND_US_MAX = 5000;

// ---- Platform-specific setup / loop ----

#ifdef USE_ESP_IDF

void NTPServer::setup() {
  ESP_LOGI(TAG, "Setting up NTP server on port %u...", this->port_);

  this->socket_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->socket_fd_ < 0) {
    ESP_LOGE(TAG, "Failed to create UDP socket: %d", errno);
    this->mark_failed();
    return;
  }

  // Blocking socket, on purpose: recv_task_() parks in recvfrom() so T2 is
  // stamped the instant a packet arrives, not on the next poll of the shared
  // loop. No fcntl(O_NONBLOCK) here.

  struct sockaddr_in server_addr {};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(this->port_);
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(this->socket_fd_, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind UDP socket to port %u: %d", this->port_, errno);
    close(this->socket_fd_);
    this->socket_fd_ = -1;
    this->mark_failed();
    return;
  }

  // Install the receive-timestamp input-path hook: replaces esp_eth's default
  // stack_input (which the netif glue set up when esp_eth_start() ran, per
  // EthernetComponent::get_setup_priority() == WIFI, higher than our own
  // AFTER_CONNECTION -- ethernet setup() has already completed by the time we get
  // here). eth_input_hook_() stamps T2 before lwIP and before recv_task_() wakes.
  // If the netif isn't available for any reason, skip installing it entirely --
  // serving must keep working without this, at the previous precision only.
  if (ethernet::global_eth_component != nullptr) {
    this->eth_netif_ = ethernet::global_eth_component->get_eth_netif();
  }
  if (this->eth_netif_ != nullptr) {
    esp_err_t err = esp_eth_update_input_path(ethernet::global_eth_component->get_eth_handle(),
                                               &NTPServer::eth_input_hook_, this);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Installed NTP receive-timestamp input-path hook");
    } else {
      ESP_LOGE(TAG, "Failed to install NTP input-path hook: %d", err);
      this->eth_netif_ = nullptr;
    }
  } else {
    ESP_LOGE(TAG, "Ethernet netif unavailable; NTP receive-timestamp hook not installed");
  }

  // RFC 5905 11.1: precision is max(resolution, read cost). Smallest non-zero
  // delta over repeated reads, as chrony does.
  uint32_t best_us = UINT32_MAX;
  for (int i = 0; i < 100; i++) {
    struct timeval a, b;
    gettimeofday(&a, nullptr);
    gettimeofday(&b, nullptr);
    int32_t d = static_cast<int32_t>((b.tv_sec - a.tv_sec) * 1000000 + (b.tv_usec - a.tv_usec));
    if (d > 0 && static_cast<uint32_t>(d) < best_us)
      best_us = static_cast<uint32_t>(d);
  }
  if (best_us == UINT32_MAX)
    best_us = 1;
  int8_t p = -20;
  while (p < 0 && (1.0f / static_cast<float>(1u << -p)) < (best_us / 1000000.0f))
    p++;
  this->precision_ = p;

  ESP_LOGI(TAG, "NTP server listening on port %u (clock read %uus, precision %d)", this->port_,
           best_us, this->precision_);

  // Serving moves off the shared loop entirely: Application::loop() sleeps out a
  // 16ms loop_interval_ between component polls, so stamping T2 there adds a mean
  // ~4.4ms queueing delay to BOTH T2 and T3. That shifts the client's computed
  // offset by the full delay (not half -- see the plan's RFC 5905 s8 algebra) and
  // leaves round-trip delay untouched, so no client can detect or filter it.
  //
  // Core 1: ESPHome's main task is pinned to core 0 (CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0),
  // so this task can never preempt gps_pps_time's apply_pps_correction_() between its
  // gettimeofday() and micros() reads -- preemption there would inflate
  // elapsed_since_edge_us and inject a spurious drift measurement, undoing the ISR fix.
  // Priority 7, below lwIP's task at 18 (CONFIG_LWIP_TCPIP_TASK_PRIO): Espressif's
  // guidance is that tasks doing socket I/O run below the TCP/IP task.
  xTaskCreatePinnedToCore(&NTPServer::recv_task_, "ntp_recv", 4096, this, 7, nullptr, 1);
}

void NTPServer::loop() {
  // Nothing to do on the request path itself: recv_task_() serves every request on
  // its own task, blocked in recvfrom(). The one exception is the hook-latency
  // diagnostic: recv_task_() can only record the value (a single volatile int32_t,
  // atomic on this hardware -- see .claude/rules/firmware.md), publishing it is an
  // ESPHome API call and must happen from this main-thread loop() instead.
  this->refresh_arp_entries_();

  if (this->arp_primes_pending_) {
    this->arp_primes_pending_ = false;
    if (this->arp_primes_sensor_ != nullptr)
      this->arp_primes_sensor_->publish_state(this->arp_primes_);
  }

  if (this->t3_error_pending_) {
    int32_t err_us = this->last_t3_error_us_;
    this->t3_error_pending_ = false;
    if (this->t3_error_sensor_ != nullptr)
      this->t3_error_sensor_->publish_state(err_us);
  }

  if (this->rx_burst_lead_pending_) {
    int32_t lead_us = this->last_rx_burst_lead_us_;
    this->rx_burst_lead_pending_ = false;
    if (this->rx_burst_lead_sensor_ != nullptr)
      this->rx_burst_lead_sensor_->publish_state(lead_us);
  }

  if (this->rx_stamp_gap_pending_) {
    int32_t gap_us = this->last_rx_stamp_gap_us_;
    this->rx_stamp_gap_pending_ = false;
    if (this->rx_stamp_gap_sensor_ != nullptr)
      this->rx_stamp_gap_sensor_->publish_state(gap_us);
  }
  if (this->hook_latency_pending_) {
    int32_t latency_us = this->last_hook_latency_us_;
    this->hook_latency_pending_ = false;
    if (this->hook_latency_sensor_ != nullptr)
      this->hook_latency_sensor_->publish_state(latency_us);
  }
}

void NTPServer::recv_task_(void *param) {
  auto *self = static_cast<NTPServer *>(param);
  uint8_t buffer[NTP_PACKET_SIZE];
  uint8_t response[NTP_PACKET_SIZE];

  while (true) {
    struct sockaddr_in client_addr {};
    socklen_t client_len = sizeof(client_addr);

    int received = recvfrom(self->socket_fd_, buffer, sizeof(buffer), 0,
                            (struct sockaddr *) &client_addr, &client_len);
    // T2 fallback: stamped the instant recvfrom() returns -- this replaces the
    // shared-loop poll and is the entire reason this task exists. Overridden below
    // if the input-path hook already stamped this same request earlier.
    NTPTimestamp receive_ts = self->get_ntp_timestamp_();

    if (received < 0) {
      // Back off. If the socket goes permanently bad (EBADF) an unguarded
      // continue would spin this task forever and peg core 1.
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (received < NTP_PACKET_SIZE)
      continue;  // runt packet -- drop, keep serving

    // The client's own transmit timestamp (bytes 40-47) is unique per request and
    // is exactly what eth_input_hook_() keyed its ring entry on -- it saw this same
    // request arrive, before lwIP, before this task woke.
    int64_t hook_t;
    if (self->hook_lookup_(&buffer[40], &hook_t)) {
      receive_ts = self->hook_to_ntp_timestamp_(hook_t);
      // Diagnostic only: how much later this task observed the same request vs.
      // the hook. Stored here (not published -- ESPHome's API is not task-safe from
      // a non-main task) and picked up by loop() on the main task.
      self->last_hook_latency_us_ = static_cast<int32_t>(esp_timer_get_time() - hook_t);
      self->hook_latency_pending_ = true;
    }

    // No ESP_LOGD/publish_state here: ESPHome's logger and API are not task-safe
    // from a non-main task. Drop silently rather than working around it.
    if (!self->is_time_synchronized_())
      continue;

    // Remember who asked, so loop() can keep their ARP entry warm. Plain store only --
    // no lwIP calls from this task beyond the socket API.
    self->note_arp_client_(client_addr.sin_addr.s_addr);

    self->build_ntp_response_(buffer, response, receive_ts);

    ethernet::W5500SendStamp before = ethernet::w5500_send_stamp();
    int64_t t0 = esp_timer_get_time();
    sendto(self->socket_fd_, response, NTP_PACKET_SIZE, 0,
           (struct sockaddr *) &client_addr, client_len);
    int64_t t_after = esp_timer_get_time();
    int32_t dur = static_cast<int32_t>(t_after - t0);

    // T3 was written as t0 + send_us_, a prediction. The W5500 was actually told to
    // transmit when the driver wrote Sn_CR = SEND. Measure how wrong the prediction was;
    // a non-zero mean here is a systematic, correctable error in every reply we serve.
    ethernet::W5500SendStamp after = ethernet::w5500_send_stamp();
    if (after.seq != before.seq) {
      int32_t actual_us = static_cast<int32_t>(after.send_cmd_us - static_cast<uint32_t>(t0));
      if (actual_us > 0 && actual_us < SEND_US_MAX) {
        self->last_t3_error_us_ = actual_us - self->send_us_;
        self->t3_error_pending_ = true;
      }
    }

    // Only learn from sends that actually reached the wire. On an ARP miss lwIP
    // queues the packet and returns immediately, which would drag the estimate
    // down even though that packet departs late.
    if (dur > SEND_US_MIN && dur < SEND_US_MAX)
      self->send_us_ += (dur - self->send_us_) / 8;
  }
}

esp_err_t NTPServer::eth_input_hook_(esp_eth_handle_t eth_handle, uint8_t *buffer, uint32_t length, void *priv) {
  // First line, nothing before it: this is as close to "frame arrived" as the W5500
  // driver gives us.
  int64_t t = esp_timer_get_time();

  auto *self = static_cast<NTPServer *>(priv);

  // Identify an NTP request without touching anything but `buffer`/`length`, and
  // bounds-check at every step -- this callback sits in the path of EVERY received
  // frame, NTP or not, and runs in the W5500 driver's own task: no ESP_LOGx, no
  // publish_state, no blocking, no allocation, no gettimeofday() here.
  //
  // Ethernet header (14B): dst MAC(6) src MAC(6) EtherType(2) @12.
  // IPv4 header starts @14; low nibble of the version/IHL byte @14 is the header
  // length in 32-bit words (min 5 = 20B); protocol @14+9 must be UDP (17).
  // UDP header (8B) starts @14+IHL*4; destination port @+2; payload @+8.
  // NTP transmit timestamp (the client's own, echoed back to us) is payload[40..47].
  static const uint32_t ETH_HDR_LEN = 14;
  static const uint32_t MIN_IPV4_HDR_LEN = 20;
  static const uint32_t UDP_HDR_LEN = 8;
  static const uint16_t ETHERTYPE_IPV4 = 0x0800;
  static const uint8_t IP_PROTO_UDP = 17;
  static const uint16_t NTP_PORT = 123;

  if (length >= ETH_HDR_LEN + MIN_IPV4_HDR_LEN) {
    uint16_t ethertype = (static_cast<uint16_t>(buffer[12]) << 8) | buffer[13];
    if (ethertype == ETHERTYPE_IPV4) {
      uint8_t ihl = buffer[ETH_HDR_LEN] & 0x0F;
      uint32_t ip_hdr_len = static_cast<uint32_t>(ihl) * 4;
      uint32_t udp_offset = ETH_HDR_LEN + ip_hdr_len;
      if (ihl >= 5 && length >= udp_offset + UDP_HDR_LEN && buffer[ETH_HDR_LEN + 9] == IP_PROTO_UDP) {
        uint16_t dst_port = (static_cast<uint16_t>(buffer[udp_offset + 2]) << 8) | buffer[udp_offset + 3];
        uint32_t ntp_offset = udp_offset + UDP_HDR_LEN;
        if (dst_port == NTP_PORT && length >= ntp_offset + NTP_PACKET_SIZE) {
          // Phase 3 Step 3: stamp T2 at the Sn_RX_RSR read rather than here, after the
          // frame has been clocked over SPI. Measured gap 287 +/- 16 us; a one-sided T2
          // shift costs half in client-visible offset, so this recovers ~144 us.
          //
          // micros() and esp_timer_get_time() share a timebase, so the difference is
          // valid; t - gap reconstructs the earlier instant in 64-bit without exposing
          // the 32-bit wrap. Only trusted when the stamp provably belongs to THIS frame
          // (exactly one payload read since that Sn_RX_RSR) and the gap is sane --
          // otherwise fall back to stamping here, which is never worse than before.
          int64_t t2 = t;
          ethernet::W5500RxStamps st = ethernet::w5500_rx_stamps();
          // How much earlier still the burst began. This is the remaining T2 headroom
          // after Step 3 -- the driver touches SIR/Sn_IR before asking Sn_RX_RSR.
          if (st.burst_start_us != 0 && st.size_read_us != 0) {
            int32_t lead = static_cast<int32_t>(st.size_read_us - st.burst_start_us);
            if (lead >= 0 && lead < 20000) {
              self->last_rx_burst_lead_us_ = lead;
              self->rx_burst_lead_pending_ = true;
            }
          }
          if (self->use_early_t2_ && st.size_read_us != 0 && st.payloads_since_size_read == 1) {
            uint32_t stamp = st.size_read_us;
            // Earlier still: the interrupt-service transaction that opened this burst.
            // Requires a plausible lead, so a stale or misdetected burst start cannot
            // back-date T2 by an arbitrary amount.
            if (self->use_burst_start_t2_ && st.burst_start_us != 0) {
              int32_t lead = static_cast<int32_t>(st.size_read_us - st.burst_start_us);
              if (lead > 0 && lead < 1000)
                stamp = st.burst_start_us;
            }
            int32_t gap = static_cast<int32_t>(static_cast<uint32_t>(t) - stamp);
            if (gap > 0 && gap < 20000) {
              t2 = t - gap;
              self->last_rx_stamp_gap_us_ = gap;
              self->rx_stamp_gap_pending_ = true;
            }
          }
          self->hook_record_(&buffer[ntp_offset + 40], t2);
        }
      }
    }
  }

  // Unconditional on every path, NTP or not: failing to forward here takes down all
  // networking on the device. Buffer ownership passes to esp_netif_receive(); it is
  // not freed here.
  return esp_netif_receive(self->eth_netif_, buffer, length, NULL);
}

void NTPServer::hook_record_(const uint8_t *key, int64_t t) {
  HookEntry &slot = this->hook_ring_[this->hook_ring_next_];
  this->hook_ring_next_ = static_cast<uint8_t>((this->hook_ring_next_ + 1) % HOOK_RING_SIZE);

  // Seqlock write. eth_input_hook_() (via this function) is the sole writer -- the
  // W5500 driver delivers one frame at a time from its own task, so this never races
  // another write to the same slot -- only against hook_lookup_()'s reads from
  // recv_task_(), possibly on the other core. Odd seq = write in progress; even = a
  // consistent snapshot. Release fences ensure the payload writes cannot be observed
  // out of order around the seq transitions on the reader's core.
  uint32_t seq = slot.seq.load(std::memory_order_relaxed);
  slot.seq.store(seq + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  memcpy(slot.key, key, sizeof(slot.key));
  slot.t = t;
  std::atomic_thread_fence(std::memory_order_release);
  slot.seq.store(seq + 2, std::memory_order_relaxed);
}

bool NTPServer::hook_lookup_(const uint8_t *key, int64_t *t_out) {
  for (int i = 0; i < HOOK_RING_SIZE; i++) {
    HookEntry &slot = this->hook_ring_[i];
    uint32_t s1 = slot.seq.load(std::memory_order_relaxed);
    if (s1 & 1)
      continue;  // write in progress on this slot -- skip rather than spin
    std::atomic_thread_fence(std::memory_order_acquire);
    uint8_t k[8];
    memcpy(k, slot.key, sizeof(k));
    int64_t t = slot.t;
    std::atomic_thread_fence(std::memory_order_acquire);
    uint32_t s2 = slot.seq.load(std::memory_order_relaxed);
    if (s1 != s2)
      continue;  // torn read (write happened mid-copy) -- skip, don't trust it
    if (memcmp(k, key, sizeof(k)) == 0) {
      *t_out = t;
      return true;
    }
  }
  return false;
}

NTPTimestamp NTPServer::micros_epoch_to_ntp_timestamp_(int64_t unix_us) {
  int64_t sec = unix_us / 1000000;
  int64_t usec = unix_us % 1000000;
  if (usec < 0) {
    usec += 1000000;
    sec -= 1;
  }
  NTPTimestamp ts;
  ts.seconds = static_cast<uint32_t>(sec) + NTP_UNIX_OFFSET;
  ts.fraction = static_cast<uint32_t>((static_cast<uint64_t>(usec) << 32) / 1000000ULL);
  return ts;
}

NTPTimestamp NTPServer::hook_to_ntp_timestamp_(int64_t hook_us) {
  // Reconstructs the wall-clock time at the hook's stamp the same way
  // GPSPPSTime::apply_pps_correction_() reconstructs the PPS edge's wall-clock time --
  // gettimeofday() is never called at the hook itself (driver task; must stay
  // non-blocking and allocation-free), so instead read gettimeofday() and
  // esp_timer_get_time() back-to-back here and subtract the elapsed delta since
  // hook_us. Valid on the same grounds documented there: adjtime()'s slew is bounded
  // and settimeofday() is rare, so adjusted_boot_time() is stable across this window.
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const int64_t now_us = esp_timer_get_time();
  const int64_t elapsed_since_hook_us = now_us - hook_us;
  const int64_t system_us_at_hook =
      static_cast<int64_t>(tv.tv_sec) * 1000000LL + tv.tv_usec - elapsed_since_hook_us;
  return NTPServer::micros_epoch_to_ntp_timestamp_(system_us_at_hook);
}

#else  // Arduino platforms (RP2040, ESP32-Arduino, etc.)

void NTPServer::setup() {
  ESP_LOGI(TAG, "Setting up NTP server on port %u...", this->port_);

  if (!this->udp_.begin(this->port_)) {
    ESP_LOGE(TAG, "Failed to bind UDP to port %u", this->port_);
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "NTP server listening on port %u", this->port_);
}

void NTPServer::loop() {
  int packet_size = this->udp_.parsePacket();
  if (packet_size < NTP_PACKET_SIZE)
    return;

  uint8_t buffer[NTP_PACKET_SIZE];
  this->udp_.read(buffer, NTP_PACKET_SIZE);
  NTPTimestamp receive_ts = this->get_ntp_timestamp_();

  uint8_t response[NTP_PACKET_SIZE];
  this->build_ntp_response_(buffer, response, receive_ts);

  this->udp_.beginPacket(this->udp_.remoteIP(), this->udp_.remotePort());
  this->udp_.write(response, NTP_PACKET_SIZE);
  this->udp_.endPacket();

  ESP_LOGD(TAG, "NTP response sent");
}

#endif  // USE_ESP_IDF

// ---- Shared NTP logic ----

void NTPServer::build_ntp_response_(const uint8_t *request, uint8_t *response,
                                    const NTPTimestamp &receive_ts) {
  memset(response, 0, NTP_PACKET_SIZE);

  // Echo the client's version (RFC 5905 Fig 31: x.version <-- r.version).
  const uint8_t vn = (request[0] >> 3) & 0x07;
  bool synced = this->is_time_synchronized_();
  if (synced) {
    response[0] = static_cast<uint8_t>((vn << 3) | 4);  // LI=0, Mode=4
    response[1] = 1;                                    // stratum 1
  } else {
    response[0] = static_cast<uint8_t>((3 << 6) | (vn << 3) | 4);  // LI=3, Mode=4
    response[1] = 16;
  }
  // Poll interval (copy from request)
  response[2] = request[2];
  response[3] = static_cast<uint8_t>(this->precision_);

  // Reference ID: "GPS\0"
  response[12] = 'G';
  response[13] = 'P';
  response[14] = 'S';
  response[15] = 0;

  // Reference timestamp: when the clock was last corrected (RFC 5905 7.3), not
  // now. Also keeps reftime <= rec <= xmt, which chrony checks. PPS corrections
  // land on a second boundary, so fraction stays 0.
  uint32_t ref_seconds = 0;
  if (this->time_source_ != nullptr) {
    time_t last_sync = this->time_source_->get_last_sync_epoch();
    if (last_sync > 0)
      ref_seconds = static_cast<uint32_t>(last_sync) + NTP_UNIX_OFFSET;
  }
  response[16] = (ref_seconds >> 24) & 0xFF;
  response[17] = (ref_seconds >> 16) & 0xFF;
  response[18] = (ref_seconds >> 8) & 0xFF;
  response[19] = ref_seconds & 0xFF;

  // Root dispersion. Zero would claim a perfect clock, which shrinks the
  // client's correctness interval (RFC 5905 11.2.1) and gets honest peers
  // rejected in our favour. Root delay stays 0: stratum 1 has no upstream.
  float disp_s = ROOT_DISP_BASE_S;
  if (ref_seconds != 0)
    disp_s += static_cast<float>(receive_ts.seconds - ref_seconds) * ROOT_DISP_RATE_S_PER_S;
  uint32_t root_disp = static_cast<uint32_t>(disp_s * NTP_SHORT_SCALE);
  response[8] = (root_disp >> 24) & 0xFF;
  response[9] = (root_disp >> 16) & 0xFF;
  response[10] = (root_disp >> 8) & 0xFF;
  response[11] = root_disp & 0xFF;

  // Origin timestamp (copy client's transmit timestamp)
  memcpy(&response[24], &request[40], 8);

  // Receive timestamp
  response[32] = (receive_ts.seconds >> 24) & 0xFF;
  response[33] = (receive_ts.seconds >> 16) & 0xFF;
  response[34] = (receive_ts.seconds >> 8) & 0xFF;
  response[35] = receive_ts.seconds & 0xFF;
  response[36] = (receive_ts.fraction >> 24) & 0xFF;
  response[37] = (receive_ts.fraction >> 16) & 0xFF;
  response[38] = (receive_ts.fraction >> 8) & 0xFF;
  response[39] = receive_ts.fraction & 0xFF;

  // Transmit timestamp, advanced by the measured send duration so it names the
  // instant the packet actually leaves rather than when we built the response.
  NTPTimestamp transmit_ts = this->get_ntp_timestamp_(this->send_us_);
  response[40] = (transmit_ts.seconds >> 24) & 0xFF;
  response[41] = (transmit_ts.seconds >> 16) & 0xFF;
  response[42] = (transmit_ts.seconds >> 8) & 0xFF;
  response[43] = transmit_ts.seconds & 0xFF;
  response[44] = (transmit_ts.fraction >> 24) & 0xFF;
  response[45] = (transmit_ts.fraction >> 16) & 0xFF;
  response[46] = (transmit_ts.fraction >> 8) & 0xFF;
  response[47] = transmit_ts.fraction & 0xFF;
}

NTPTimestamp NTPServer::get_ntp_timestamp_(int32_t offset_us) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);

  time_t sec = tv.tv_sec;
  int64_t usec = static_cast<int64_t>(tv.tv_usec) + offset_us;
  while (usec >= 1000000) {
    usec -= 1000000;
    sec++;
  }
  while (usec < 0) {
    usec += 1000000;
    sec--;
  }
  tv.tv_sec = sec;
  tv.tv_usec = static_cast<suseconds_t>(usec);

  NTPTimestamp ts;
  ts.seconds = static_cast<uint32_t>(tv.tv_sec) + NTP_UNIX_OFFSET;
  // Convert microseconds to NTP fraction: usec * 2^32 / 1000000
  ts.fraction = static_cast<uint32_t>((static_cast<uint64_t>(tv.tv_usec) << 32) / 1000000ULL);
  return ts;
}

bool NTPServer::is_time_synchronized_() {
  if (this->time_source_ == nullptr)
    return false;  // misconfiguration: refuse to serve rather than serve unverified time
  return this->time_source_->is_synchronized();
}

void NTPServer::dump_config() {
  ESP_LOGCONFIG(TAG, "NTP Server:");
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  ESP_LOGCONFIG(TAG, "  Time source: %s", this->time_source_ != nullptr ? "configured" : "none");
}

void NTPServer::note_arp_client_(uint32_t addr) {
  if (addr == 0)
    return;
  for (uint8_t i = 0; i < ARP_CLIENT_SLOTS; i++) {
    if (this->arp_clients_[i] == addr)
      return;  // already tracked
  }
  this->arp_clients_[this->arp_client_next_] = addr;
  this->arp_client_next_ = (this->arp_client_next_ + 1) % ARP_CLIENT_SLOTS;
}

void NTPServer::refresh_arp_entries_() {
  const uint32_t now = millis();
  if (this->arp_last_refresh_ms_ != 0 && now - this->arp_last_refresh_ms_ < ARP_REFRESH_INTERVAL_MS)
    return;
  this->arp_last_refresh_ms_ = now;
  if (this->eth_netif_ == nullptr)
    return;

  auto *netif = static_cast<struct netif *>(esp_netif_get_netif_impl(this->eth_netif_));
  if (netif == nullptr)
    return;

  uint32_t primed = 0;
  for (uint8_t i = 0; i < ARP_CLIENT_SLOTS; i++) {
    uint32_t addr = this->arp_clients_[i];
    if (addr == 0)
      continue;
    // Only a client on our own subnet is reached by ARPing its address. An off-subnet
    // client goes via the default gateway, so that is the entry the reply actually needs
    // -- ARPing the client itself would never resolve and would broadcast forever.
    // (Caught by the arp_primes counter refusing to settle: the test client sits on a
    // different /24.)
    ip4_addr_t ip;
    const uint32_t netmask = netif_ip4_netmask(netif)->addr;
    const uint32_t our_ip = netif_ip4_addr(netif)->addr;
    if (((addr ^ our_ip) & netmask) == 0) {
      ip.addr = addr;
    } else {
      ip.addr = netif_ip4_gw(netif)->addr;
      if (ip.addr == 0)
        continue;  // no gateway configured; nothing sensible to resolve
    }

    // etharp_* are not thread-safe; this runs on the ESPHome main task, not the tcpip
    // thread, so the core lock is required (CONFIG_LWIP_TCPIP_CORE_LOCKING=y).
    LOCK_TCPIP_CORE();
    struct eth_addr *eth_ret = nullptr;
    const ip4_addr_t *ip_ret = nullptr;
    bool cached = etharp_find_addr(netif, &ip, &eth_ret, &ip_ret) >= 0;
    if (!cached) {
      // q == nullptr: resolve only, queue nothing. Fires an ARP request now so the
      // entry is warm long before the next reply needs it.
      etharp_query(netif, &ip, nullptr);
      primed++;
    }
    UNLOCK_TCPIP_CORE();
  }

  if (primed > 0) {
    this->arp_primes_ += primed;
    this->arp_primes_pending_ = true;
    ESP_LOGD(TAG, "ARP priming: re-resolved %u cold client(s), %u total", primed, this->arp_primes_);
  }
}

}  // namespace ntp_server
}  // namespace esphome
