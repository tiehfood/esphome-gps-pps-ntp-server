#include "ntp_server.h"
#include "esphome/core/log.h"
#include "esphome/components/gps_pps_time/gps_pps_time.h"
#include "arp_wait.h"

#include <sys/time.h>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <new>

#ifdef USE_ESP_IDF
#include <unistd.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <atomic>
#include "esphome/components/ethernet/ethernet_component.h"
#include "esphome/components/ethernet/w5500_custom_spi.h"
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
/// RFC 5905 s11.1: root dispersion BOUNDS our error against the reference clock. It does
/// NOT cover network path asymmetry -- that is the client's to discover, not ours to
/// declare. Every term below is measured on this device.
///
/// Serving from the system clock (gettimeofday path):
///   clock vs GPS        50 us  -- 7-day mean 5.0, worst daily extreme +/-41, and the
///                                 drift spike filter accepts up to 50
///   T2 stamping         21 us  -- vs the hardware INTn edge (MCPWM capture)
///   T3 prediction       23 us  -- vs the measured Sn_CR=SEND instant
///   timestamp grain     31 us  -- 2^-15 s
///   worst-case sum     125 us  -> 250 us with 2x margin
static const float ROOT_DISP_BASE_S = 0.000250f;

/// Serving from the PPS anchor. The system clock, and with it the adjtime sawtooth, is out
/// of the path entirely -- time comes from the GPS second boundary directly:
///   GPS PPS itself      ~0     -- ~30 ns RMS, tAcc measured at 19 ns
///   PPS edge capture     3 us  -- micros() in the ISR
///   extrapolation        5 us  -- residual drift between edges after the frequency term
///   T2 stamping         21 us  -- unchanged
///   T3 prediction       23 us  -- unchanged
///   timestamp grain      2 us  -- esp_timer resolution
///   worst-case sum      54 us  -> 100 us with a comparable margin
/// Still deliberately conservative: the wire-to-INTn latency inside the W5500 is not
/// measurable from here, and under-advertising makes us a falseticker.
static const float ROOT_DISP_ANCHOR_S = 0.000100f;
/// Dispersion growth per second since last PPS (~10ppm crystal).
static const float ROOT_DISP_RATE_S_PER_S = 10.0e-6f;
/// NTP short format is 16.16 fixed point; 1 LSB = 15.259us.
static const float NTP_SHORT_SCALE = 65536.0f;

// The send estimate's bounds, EWMA rate and re-seed rules live in send_estimator.h.
// The receive-admission thresholds (RX_STALL_MAX_US, RX_STALL_EDGE_MAX_AGE_US,
// RX_EDGE_LEAD_STRICT_US) live in rx_admission.h, alongside the pure functions that use them.

/// How long recv_task_() waits for an ARP entry to resolve before giving up and refusing the
/// request outright. lwIP's own ARP round trip is ~1.5 ms on this network; 20 ms is a wide
/// margin that still fits comfortably inside a task's own budget (this is not the main loop,
/// so there is no watchdog concern -- see .claude/CLAUDE.md).
static const int64_t ARP_WAIT_MAX_US = 20000;
/// Per-request diagnostics are published as the largest-magnitude value per window. Publishing
/// on every request put ~16 state messages/s through API and SSE at a 4 Hz client: outbound
/// load of exactly the kind that stalls the receive path.
static const uint32_t TELEMETRY_INTERVAL_MS = 10000;
/// UDP port answering CLK / REQ / NET while the diagnostics switch is on.
static const uint16_t DIAG_PORT = 12301;

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
    this->eth_netif_ = ethernet::global_eth_component->get_esp_netif();
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

  // Same measurement for the PPS-anchor path. It reads esp_timer_get_time() -- IRAM,
  // lock-free -- instead of gettimeofday(), which takes s_time_lock. Measured separately
  // because the two paths have genuinely different resolution and RFC 5905 11.1 asks for
  // the cost of the clock we actually read.
  uint32_t best_anchor_us = UINT32_MAX;
  for (int i = 0; i < 100; i++) {
    const int64_t a2 = esp_timer_get_time();
    const int64_t b2 = esp_timer_get_time();
    const int32_t d = static_cast<int32_t>(b2 - a2);
    if (d > 0 && static_cast<uint32_t>(d) < best_anchor_us)
      best_anchor_us = static_cast<uint32_t>(d);
  }
  if (best_anchor_us == UINT32_MAX)
    best_anchor_us = 1;
  int8_t pa = -20;
  while (pa < 0 && (1.0f / static_cast<float>(1u << -pa)) < (best_anchor_us / 1000000.0f))
    pa++;
  this->precision_anchor_ = pa;

  ESP_LOGI(TAG, "NTP server listening on port %u (clock read %uus precision %d; anchor %uus precision %d)",
           this->port_, (unsigned) best_us, this->precision_, (unsigned) best_anchor_us, this->precision_anchor_);

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
  // its own task, blocked in recvfrom(). Diagnostics are recorded there and in the driver
  // task, and published here -- publishing is an ESPHome API call, only safe from this
  // main-thread loop() -- once per TELEMETRY_INTERVAL_MS as the window's worst case.
  this->refresh_arp_entries_();
  this->diag_serve_();

  if (this->arp_primes_pending_) {
    this->arp_primes_pending_ = false;
    if (this->arp_primes_sensor_ != nullptr)
      this->arp_primes_sensor_->publish_state(this->arp_primes_);
  }

  const uint32_t now_ms = millis();
  if (now_ms - this->telemetry_last_ms_ < TELEMETRY_INTERVAL_MS)
    return;
  this->telemetry_last_ms_ = now_ms;

  int32_t v;
  if (this->t3_error_win_.take(v) && this->t3_error_sensor_ != nullptr)
    this->t3_error_sensor_->publish_state(v);
  if (this->int_lead_win_.take(v) && this->int_lead_sensor_ != nullptr)
    this->int_lead_sensor_->publish_state(v);
  if (this->rx_stamp_gap_win_.take(v) && this->rx_stamp_gap_sensor_ != nullptr)
    this->rx_stamp_gap_sensor_->publish_state(v);
  if (this->hook_latency_win_.take(v) && this->hook_latency_sensor_ != nullptr)
    this->hook_latency_sensor_->publish_state(v);
  if (this->refused_sensor_ != nullptr)
    this->refused_sensor_->publish_state(this->refused_.load(std::memory_order_relaxed));
  if (this->arp_waits_sensor_ != nullptr)
    this->arp_waits_sensor_->publish_state(this->arp_waits_.load(std::memory_order_relaxed));

  const ethernet::W5500CmdStats cmd = ethernet::w5500_take_cmd_stats();
  if (this->w5500_cmd_retries_sensor_ != nullptr)
    this->w5500_cmd_retries_sensor_->publish_state(cmd.retried);
  if (this->w5500_cmd_max_sensor_ != nullptr && cmd.commands > 0)
    this->w5500_cmd_max_sensor_->publish_state(cmd.max_us);
  if (cmd.overlaps > 0)
    ESP_LOGD(TAG, "W5500 Sn_CR: %u commands, %u retried, max %u us, %u overlapping", (unsigned) cmd.commands,
             (unsigned) cmd.retried, (unsigned) cmd.max_us, (unsigned) cmd.overlaps);
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

    // The client's own transmit timestamp (bytes 40-47) plus its source IP/port is exactly
    // what eth_input_hook_() keyed its ring entry on -- it saw this same request arrive,
    // before lwIP, before this task woke. Both fields are already network-order, matching the
    // raw bytes the hook copied out of the frame, so no ntohs()/ntohl() is needed here.
    HookInfo hook;
    const int64_t lookup_now_us = esp_timer_get_time();
    const bool hook_hit = self->hook_lookup_(&buffer[40], client_addr.sin_addr.s_addr,
                                             client_addr.sin_port, lookup_now_us, &hook);
    int32_t hook_latency_us = -1;
    if (hook_hit) {
      receive_ts = self->hook_to_ntp_timestamp_(hook.t);
      // Diagnostic only: how much later this task observed the same request vs. the hook.
      // Recorded here, published by loop() -- ESPHome's API is not task-safe from here.
      hook_latency_us = static_cast<int32_t>(esp_timer_get_time() - hook.t);
      self->hook_latency_win_.add(hook_latency_us);
    }

    // No ESP_LOGD/publish_state here: ESPHome's logger and API are not task-safe
    // from a non-main task. Drop silently rather than working around it.
    if (!self->is_time_synchronized_())
      continue;

    const bool diagnostics = self->diagnostics_;
    DiagRecord diag{};
    if (diagnostics) {
      diag.t2_us = hook_hit ? hook.t : esp_timer_get_time();
      memcpy(diag.client_tx, &buffer[40], sizeof(diag.client_tx));
      diag.flags = hook_hit ? DIAG_HOOK_HIT : 0;
      diag.int_lead_us = hook_hit ? hook.int_lead_us : -1;
      diag.rx_gap_us = hook_hit ? hook.rx_gap_us : -1;
      diag.hook_latency_us = hook_latency_us;
      diag.rx_rsr = hook_hit ? hook.rx_rsr : -1;
      diag.frame_len = hook_hit ? hook.frame_len : -1;
      diag.send_us = -1;
      diag.sendto_us = -1;
      diag.lock_wait_us = -1;
      diag.estimate_us = self->send_estimator_.estimate();
      diag.arp_wait_us = -1;  // not evaluated yet
      diag.refuse_reason = REFUSE_NONE;
      diag.send_class = -1;
      diag.tx_write_start_us = -1;
      diag.tx_write_end_us = -1;
    }

    // ---- Admission: hook miss, or an INTn edge that is stale/missing (rx_admission.h) ----
    // A stale/missing edge means T2 is unverifiable or was late by up to ~100 ms; a hook miss
    // (only possible when the hook IS installed -- eth_netif_ != nullptr) means this exact
    // request never crossed the input-path hook, so its T2 fallback could be minutes old (a
    // repeated or all-zero client transmit field hitting a stale ring entry, or T2 stamped only
    // on this task's own recvfrom() return). No reply beats a wrong one; clients retry.
    uint8_t refuse_reason = REFUSE_NONE;
    if (self->eth_netif_ != nullptr && !hook_hit) {
      refuse_reason = REFUSE_HOOK_MISS;
    } else if (hook_hit) {
      const RxVerdict verdict =
          rx_admission(hook.capture_armed, hook.edge_usable, hook.lead_us, self->strict_rx_admission_);
      if (verdict == RxVerdict::OLD_EDGE)
        refuse_reason = REFUSE_OLD_EDGE;
      else if (verdict == RxVerdict::NO_EDGE)
        refuse_reason = REFUSE_NO_EDGE;
    }
    if (refuse_reason != REFUSE_NONE) {
      self->refused_.fetch_add(1, std::memory_order_relaxed);
      if (diagnostics) {
        diag.flags |= DIAG_REFUSED;
        if (refuse_reason == REFUSE_OLD_EDGE || refuse_reason == REFUSE_NO_EDGE)
          diag.flags |= DIAG_RX_STALLED;
        diag.refuse_reason = refuse_reason;
        self->diag_record_(diag);
      }
      continue;
    }

    // Remember who asked, so loop() can keep their ARP entry warm. Plain store only --
    // no lwIP calls from this task beyond the socket API.
    self->note_arp_client_(client_addr.sin_addr.s_addr);

    // ---- ARP resolution ----
    // sendto() would otherwise queue the reply behind an ARP round trip (~1.5 ms) if lwIP has
    // no entry for the next hop, and the SEND stamp read afterwards could then belong to the
    // ARP request rather than our reply -- silently mislearning the T3 estimate (confirmed on
    // the live device 2026-09-11). Wait for it to resolve (or give up) BEFORE the reply is
    // built, so T3 is computed after any wait and pays no accuracy cost.
    int32_t arp_wait_us = -1;
    if (self->eth_netif_ != nullptr) {
      auto *netif = static_cast<struct netif *>(esp_netif_get_netif_impl(self->eth_netif_));
      if (netif != nullptr) {
        const uint32_t our_ip = netif_ip4_addr(netif)->addr;
        const uint32_t netmask = netif_ip4_netmask(netif)->addr;
        const uint32_t gateway = netif_ip4_gw(netif)->addr;
        const uint32_t hop = next_hop_ipv4(client_addr.sin_addr.s_addr, our_ip, netmask, gateway);
        // hop == 0: off-subnet client with no gateway configured -- nothing sensible to
        // resolve, and refusing every such request would be worse than the rare ARP stall.
        if (hop != 0) {
          ip4_addr_t hop_ip;
          hop_ip.addr = hop;
          auto lookup = [netif, &hop_ip]() {
            LOCK_TCPIP_CORE();
            struct eth_addr *eth_ret = nullptr;
            const ip4_addr_t *ip_ret = nullptr;
            const bool ok = etharp_find_addr(netif, &hop_ip, &eth_ret, &ip_ret) >= 0;
            UNLOCK_TCPIP_CORE();
            return ok;
          };
          auto query = [netif, &hop_ip]() {
            LOCK_TCPIP_CORE();
            etharp_query(netif, &hop_ip, nullptr);
            UNLOCK_TCPIP_CORE();
          };
          // Never sleeps with the core lock held: the tcpip thread needs it to process the
          // ARP reply. FREERTOS_HZ is 1000, so vTaskDelay(1) is exactly 1 ms.
          const ArpWaitResult res =
              wait_for_arp(lookup, query, []() { vTaskDelay(1); },
                           []() { return esp_timer_get_time(); }, ARP_WAIT_MAX_US);
          arp_wait_us = res.waited_us;
          if (res.missed)
            self->arp_waits_.fetch_add(1, std::memory_order_relaxed);
          if (!res.resolved) {
            self->refused_.fetch_add(1, std::memory_order_relaxed);
            if (diagnostics) {
              diag.flags |= DIAG_REFUSED | DIAG_ARP_MISS;
              diag.arp_wait_us = arp_wait_us;
              diag.refuse_reason = REFUSE_ARP_UNRESOLVED;
              self->diag_record_(diag);
            }
            continue;
          }
          if (res.missed && diagnostics)
            diag.flags |= DIAG_ARP_MISS;
        }
      }
    }
    if (diagnostics)
      diag.arp_wait_us = arp_wait_us;

    if (diagnostics) {
      // How contended lwIP's core lock is right now -- sendto() has to take it too. Probed
      // before the reply is built, so a wait here happens before T3 is computed and cannot leak
      // into the served timestamp. The lock is not recursive: it must not be held into sendto().
      const int64_t lock_start_us = esp_timer_get_time();
      LOCK_TCPIP_CORE();
      diag.lock_wait_us = static_cast<int32_t>(esp_timer_get_time() - lock_start_us);
      UNLOCK_TCPIP_CORE();
    }

    self->build_ntp_response_(buffer, response, receive_ts);

    ethernet::W5500SendStamp before = ethernet::w5500_send_stamp();
    int64_t t0 = esp_timer_get_time();
    sendto(self->socket_fd_, response, NTP_PACKET_SIZE, 0,
           (struct sockaddr *) &client_addr, client_len);
    int64_t t_after = esp_timer_get_time();
    int32_t dur = static_cast<int32_t>(t_after - t0);

    // T3 was written as build time + the send estimate, a prediction. The W5500 was actually
    // told to transmit when the driver wrote Sn_CR = SEND: measure how wrong it was and learn --
    // but only when that SEND is provably OUR reply (frame_class == NTP). A SEND for some other
    // frame (an ARP request queued behind ours, for instance) would mislearn the estimate in
    // either direction, and there is no way to retroactively tell how long ours actually took.
    ethernet::W5500SendStamp after = ethernet::w5500_send_stamp();
    SendEstimator::Result learned = SendEstimator::Result::IGNORED;
    int32_t actual_us = -1;
    bool send_not_ntp = false;
    if (after.seq != before.seq) {
      if (after.frame_class == ethernet::W5500_FC_NTP) {
        // Time from t0 to the Sn_CR = SEND write -- NOT how long sendto() took to return, which
        // is strictly later and made the estimate over-predict by ~180 us.
        actual_us = static_cast<int32_t>(after.send_cmd_us - static_cast<uint32_t>(t0));
        if (actual_us > 0 && actual_us < SendEstimator::US_MAX)
          self->t3_error_win_.add(actual_us - self->send_estimator_.estimate());
        learned = self->send_estimator_.learn(actual_us, t0);
      } else {
        send_not_ntp = true;
      }
    } else {
      // No SEND happened at all this round: fall back to learning from the sendto() duration,
      // under the same bounds. Deliberately NOT used when a SEND happened but for the wrong
      // frame, or was out of SendEstimator's accepted range -- either way it says nothing about
      // how long our own reply took to go out.
      self->send_estimator_.learn_fallback(dur);
    }

    if (diagnostics) {
      diag.send_us = actual_us;
      diag.sendto_us = dur;
      diag.send_class = (after.seq != before.seq) ? static_cast<int8_t>(after.frame_class) : -1;
      // TX-buffer-write split: only meaningful when this SEND is provably our own reply and
      // the SPI callback actually measured it (both 0 means the recorder was off for that write).
      if (after.seq != before.seq && after.frame_class == ethernet::W5500_FC_NTP &&
          after.txbuf_start_us != 0 && after.txbuf_end_us != 0) {
        diag.tx_write_start_us = static_cast<int32_t>(after.txbuf_start_us - static_cast<uint32_t>(t0));
        diag.tx_write_end_us = static_cast<int32_t>(after.txbuf_end_us - static_cast<uint32_t>(t0));
      }
      if (send_not_ntp)
        diag.flags |= DIAG_SEND_NOT_NTP;
      if (actual_us >= SendEstimator::US_MAX || dur >= SendEstimator::US_MAX)
        diag.flags |= DIAG_SEND_LONG;
      if (learned == SendEstimator::Result::RESEEDED)
        diag.flags |= DIAG_RESEEDED;
      if (learned == SendEstimator::Result::IGNORED && after.seq == before.seq)
        diag.flags |= DIAG_FALLBACK;
      self->diag_record_(diag);
    }
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

  if (length >= ETH_HDR_LEN + MIN_IPV4_HDR_LEN) {
    uint16_t ethertype = (static_cast<uint16_t>(buffer[12]) << 8) | buffer[13];
    if (ethertype == ETHERTYPE_IPV4) {
      uint8_t ihl = buffer[ETH_HDR_LEN] & 0x0F;
      uint32_t ip_hdr_len = static_cast<uint32_t>(ihl) * 4;
      uint32_t udp_offset = ETH_HDR_LEN + ip_hdr_len;
      if (ihl >= 5 && length >= udp_offset + UDP_HDR_LEN && buffer[ETH_HDR_LEN + 9] == IP_PROTO_UDP) {
        uint16_t dst_port = (static_cast<uint16_t>(buffer[udp_offset + 2]) << 8) | buffer[udp_offset + 3];
        uint32_t ntp_offset = udp_offset + UDP_HDR_LEN;
        // The port the socket is actually bound to, not a hard-coded 123: recv_task_() refuses any
        // request the hook did not record, so a mismatch here would refuse every request.
        if (dst_port == self->port_ && length >= ntp_offset + NTP_PACKET_SIZE) {
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
          // The W5500 asserted INTn before any of this: hardware timestamp vs our stamp.
          // Normally 3-25 us (GPIO ISR plus driver task wake). When the receive path stalls,
          // the burst starts up to ~100 ms after the edge, T2 is that late, and rx_admission()
          // in recv_task_() refuses the request. capture_armed/edge_usable/lead_us are the raw
          // measurement; the admission verdict itself is computed later, against whatever
          // set_strict_rx_admission() is set to AT REQUEST TIME, not now.
          HookInfo info;
          ethernet::W5500IntStamp ist = ethernet::w5500_int_stamp();
          info.capture_armed = ist.seq != 0;
          const RxEdgeEval edge_eval = evaluate_rx_edge(ist.seq, ist.edge_us, st.burst_start_us);
          info.edge_usable = edge_eval.edge_usable;
          info.lead_us = edge_eval.lead_us;
          if (edge_eval.edge_usable) {
            self->int_lead_win_.add(edge_eval.lead_us);
            info.int_lead_us = edge_eval.lead_us;
          }

          if (st.size_read_us != 0 && st.payloads_since_size_read == 1) {
            // Diagnostic only: what Sn_RX_RSR reported at this frame's arrival, and our own
            // frame length -- rx_rsr > frame_len means another frame was already queued behind
            // ours in the W5500's RX buffer when it arrived.
            info.rx_rsr = static_cast<int32_t>(st.size_value);
            info.frame_len = static_cast<int32_t>(length);
            uint32_t stamp = st.size_read_us;
            // Earlier still: the interrupt-service transaction that opened this burst.
            // Requires a plausible lead, so a stale or misdetected burst start cannot
            // back-date T2 by an arbitrary amount.
            if (st.burst_start_us != 0) {
              int32_t lead = static_cast<int32_t>(st.size_read_us - st.burst_start_us);
              if (lead > 0 && lead < 1000)
                stamp = st.burst_start_us;
            }

            int32_t gap = static_cast<int32_t>(static_cast<uint32_t>(t) - stamp);
            if (gap > 0 && gap < 20000) {
              t2 = t - gap;
              self->rx_stamp_gap_win_.add(gap);
              info.rx_gap_us = gap;
            }
          }
          info.t = t2;
          // Raw copies, network byte order, straight off the wire -- these must compare equal
          // to sockaddr_in::sin_addr/sin_port in recv_task_(), which are also network order, so
          // no ntohs()/ntohl() here.
          uint32_t src_ip;
          memcpy(&src_ip, &buffer[ETH_HDR_LEN + 12], 4);
          uint16_t src_port;
          memcpy(&src_port, &buffer[udp_offset + 0], 2);
          self->hook_record_(&buffer[ntp_offset + 40], src_ip, src_port, info);
        }
      }
    }
  }

  // Unconditional on every path, NTP or not: failing to forward here takes down all
  // networking on the device. Buffer ownership passes to esp_netif_receive(); it is
  // not freed here.
  return esp_netif_receive(self->eth_netif_, buffer, length, NULL);
}

void NTPServer::set_diagnostics(bool enable) {
  if (enable && this->diag_ring_ == nullptr) {
    // ~53 KB, PSRAM first: leave the internal heap the network stack draws on alone.
    void *mem = heap_caps_calloc(DIAG_RING_SIZE, sizeof(DiagSlot), MALLOC_CAP_SPIRAM);
    if (mem == nullptr)
      mem = heap_caps_calloc(DIAG_RING_SIZE, sizeof(DiagSlot), MALLOC_CAP_8BIT);
    if (mem != nullptr) {
      auto *ring = static_cast<DiagSlot *>(mem);
      for (uint16_t i = 0; i < DIAG_RING_SIZE; i++)
        new (&ring[i]) DiagSlot();
      this->diag_ring_ = ring;
    }
    if (this->diag_ring_ == nullptr) {
      ESP_LOGW(TAG, "Diagnostics: cannot allocate the request ring");
      return;
    }
  }
  ethernet::w5500_set_net_recorder(enable);
  this->diagnostics_ = enable;
  ESP_LOGI(TAG, "Diagnostics %s", enable ? "on: request ring, network recorder, UDP 12301" : "off");
}

void NTPServer::diag_record_(const DiagRecord &rec) {
  DiagSlot *ring = this->diag_ring_;
  if (!this->diagnostics_ || ring == nullptr)
    return;
  DiagSlot &slot = ring[this->diag_next_];
  this->diag_next_ = static_cast<uint16_t>((this->diag_next_ + 1) % DIAG_RING_SIZE);
  // Seqlock write, as in hook_record_(): the dump in loop() may read concurrently.
  const uint32_t seq = slot.seq.load(std::memory_order_relaxed);
  slot.seq.store(seq + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  slot.rec = rec;
  std::atomic_thread_fence(std::memory_order_release);
  slot.seq.store(seq + 2, std::memory_order_relaxed);
}

void NTPServer::diag_serve_() {
  if (!this->diagnostics_) {
    if (this->diag_socket_ >= 0) {
      close(this->diag_socket_);
      this->diag_socket_ = -1;
    }
    return;
  }
  if (this->diag_socket_ < 0) {
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0)
      return;
    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(DIAG_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
      close(fd);
      return;
    }
    this->diag_socket_ = fd;
  }

  char cmd[16];
  struct sockaddr_in peer {};
  socklen_t peer_len = sizeof(peer);
  const int received = recvfrom(this->diag_socket_, cmd, sizeof(cmd) - 1, MSG_DONTWAIT,
                                (struct sockaddr *) &peer, &peer_len);
  if (received <= 0)
    return;
  cmd[received] = '\0';

  // Dumps are pulled after a run, so a short block of the main loop here costs nothing measured.
  char batch[1400];
  size_t used = 0;
  const auto flush = [&]() {
    if (used > 0) {
      sendto(this->diag_socket_, batch, used, 0, (struct sockaddr *) &peer, peer_len);
      used = 0;
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  };
  const auto emit = [&](const char *line, int len) {
    if (len <= 0 || static_cast<size_t>(len) >= sizeof(batch))
      return;
    if (used + len > sizeof(batch))
      flush();
    memcpy(batch + used, line, len);
    used += len;
  };
  char line[240];
  uint32_t lines = 0;

  if (strncmp(cmd, "CLK", 3) == 0) {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    emit(line, snprintf(line, sizeof(line), "CLK %lld %u %lld\n", (long long) esp_timer_get_time(),
                        (unsigned) micros(), (long long) tv.tv_sec * 1000000LL + tv.tv_usec));
    lines = 1;
  } else if (strncmp(cmd, "REQ", 3) == 0 && this->diag_ring_ != nullptr) {
    const uint16_t start = this->diag_next_;
    for (uint16_t i = 0; i < DIAG_RING_SIZE; i++) {
      DiagSlot &slot = this->diag_ring_[(start + i) % DIAG_RING_SIZE];
      DiagRecord rec{};
      bool consistent = false;
      for (int attempt = 0; attempt < 3 && !consistent; attempt++) {
        const uint32_t s1 = slot.seq.load(std::memory_order_relaxed);
        if (s1 & 1)
          continue;
        std::atomic_thread_fence(std::memory_order_acquire);
        rec = slot.rec;
        std::atomic_thread_fence(std::memory_order_acquire);
        consistent = slot.seq.load(std::memory_order_relaxed) == s1;
      }
      if (!consistent || rec.t2_us == 0)
        continue;
      char tx_hex[17];
      for (int b = 0; b < 8; b++)
        snprintf(tx_hex + 2 * b, 3, "%02x", rec.client_tx[b]);
      emit(line, snprintf(line, sizeof(line), "R %lld %s %x %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                          (long long) rec.t2_us, tx_hex, (unsigned) rec.flags, (int) rec.int_lead_us,
                          (int) rec.rx_gap_us, (int) rec.hook_latency_us, (int) rec.send_us, (int) rec.sendto_us,
                          (int) rec.lock_wait_us, (int) rec.estimate_us, (int) rec.arp_wait_us,
                          (int) rec.refuse_reason, (int) rec.send_class, (int) rec.tx_write_start_us,
                          (int) rec.tx_write_end_us, (int) rec.rx_rsr, (int) rec.frame_len));
      lines++;
    }
  } else if (strncmp(cmd, "NET", 3) == 0) {
    ethernet::W5500NetBin bin;
    for (uint16_t i = 0; i < ethernet::W5500_NET_BINS; i++) {
      if (!ethernet::w5500_net_bin(i, bin))
        continue;
      if (bin.tx_bytes == 0 && bin.rx_bytes == 0 && bin.lock_wait_max_us == 0 && bin.cr_retried == 0)
        continue;
      int len = snprintf(line, sizeof(line), "N %u", (unsigned) bin.start_us);
      for (int k = 0; k < ethernet::W5500_FC_COUNT; k++)
        len += snprintf(line + len, sizeof(line) - len, " %u", (unsigned) bin.tx[k]);
      len += snprintf(line + len, sizeof(line) - len, " %u", (unsigned) bin.tx_bytes);
      for (int k = 0; k < ethernet::W5500_FC_COUNT; k++)
        len += snprintf(line + len, sizeof(line) - len, " %u", (unsigned) bin.rx[k]);
      len += snprintf(line + len, sizeof(line) - len, " %u %u %u\n", (unsigned) bin.rx_bytes,
                      (unsigned) bin.lock_wait_max_us, (unsigned) bin.cr_retried);
      emit(line, len);
      lines++;
    }
  } else if (strncmp(cmd, "TLK", 3) == 0) {
    for (uint8_t i = 0; i < ethernet::W5500_TALKER_TABLE_SIZE; i++) {
      ethernet::W5500Talker t;
      if (!ethernet::w5500_talker(i, t))
        continue;
      char src[18];
      char dst[18];
      snprintf(src, sizeof(src), "%02x:%02x:%02x:%02x:%02x:%02x", t.src_mac[0], t.src_mac[1], t.src_mac[2],
               t.src_mac[3], t.src_mac[4], t.src_mac[5]);
      snprintf(dst, sizeof(dst), "%02x:%02x:%02x:%02x:%02x:%02x", t.first_dst_mac[0], t.first_dst_mac[1],
               t.first_dst_mac[2], t.first_dst_mac[3], t.first_dst_mac[4], t.first_dst_mac[5]);
      emit(line, snprintf(line, sizeof(line), "T %s %04x %s %u %u %u %u %u\n", src, (unsigned) t.ethertype, dst,
                          (unsigned) t.frames, (unsigned) t.burst_frames, (unsigned) t.bytes,
                          (unsigned) t.first_us, (unsigned) t.last_us));
      lines++;
    }
  } else {
    emit(line, snprintf(line, sizeof(line), "ERR usage: CLK | REQ | NET | TLK\n"));
  }
  char kind[4] = {cmd[0], cmd[1], cmd[2], '\0'};
  emit(line, snprintf(line, sizeof(line), "END %s %u\n", kind, (unsigned) lines));
  flush();
}

void NTPServer::hook_record_(const uint8_t *tx, uint32_t src_ip, uint16_t src_port, const HookInfo &info) {
  this->hook_ring_.record(tx, src_ip, src_port, info);
}

bool NTPServer::hook_lookup_(const uint8_t *tx, uint32_t src_ip, uint16_t src_port, int64_t now_us,
                             HookInfo *out) {
  return this->hook_ring_.lookup(tx, src_ip, src_port, now_us, out);
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

/// UTC microseconds at a given esp_timer value, derived from the PPS anchor rather than
/// the system clock.
///
/// The PPS edge IS the second boundary, so (epoch, micros) fixes the mapping exactly. This
/// avoids gettimeofday() entirely: no s_time_lock, ~1 us to read instead of ~15, and --
/// the real point -- the served time no longer rides the adjtime slew. adjtime() corrects
/// offset only at a bounded rate, which leaves a sawtooth in the system clock; anchoring
/// to PPS sidesteps that instead of halving it.
///
/// drift_us is the measured clock gain per second. esp_timer shares the crystal, so its
/// elapsed reading is long by that much per second; subtracting it removes the residual
/// between edges rather than letting it accumulate for a full second.
///
/// Returns false when there is no usable anchor -- unsynced, or the anchor is stale
/// because PPS stopped -- and the caller must fall back to the system clock.
bool NTPServer::anchor_epoch_us_(int64_t at_us, int64_t &out_us) {
  auto *src = static_cast<gps_pps_time::GPSPPSTime *>(this->time_source_);
  if (src == nullptr)
    return false;
  gps_pps_time::GPSPPSTime::PpsAnchor a;
  if (!src->get_pps_anchor(a))
    return false;

  // uint32 subtraction handles the micros() wrap (71.6 min) correctly for the sub-second
  // spans we care about. A larger span means PPS has stopped; refuse rather than
  // extrapolate a stale anchor into a confidently wrong timestamp.
  const uint32_t elapsed = static_cast<uint32_t>(at_us) - a.micros;
  if (elapsed > 2000000u)
    return false;

  // drift_ppb is a RATE (crystal error in parts per billion), so it scales with elapsed
  // time. Guard at +/-100 ppm; anything beyond that is a broken measurement, not a crystal.
  int64_t corrected = elapsed;
  if (a.drift_ppb > -100000 && a.drift_ppb < 100000)
    corrected -= (static_cast<int64_t>(elapsed) * a.drift_ppb) / 1000000000LL;
  out_us = static_cast<int64_t>(a.epoch) * 1000000LL + corrected;
  return true;
}

NTPTimestamp NTPServer::hook_to_ntp_timestamp_(int64_t hook_us) {
  // Reconstructs the wall-clock time at the hook's stamp the same way
  // GPSPPSTime::apply_pps_correction_() reconstructs the PPS edge's wall-clock time --
  // gettimeofday() is never called at the hook itself (driver task; must stay
  // non-blocking and allocation-free), so instead read gettimeofday() and
  // esp_timer_get_time() back-to-back here and subtract the elapsed delta since
  // hook_us. Valid on the same grounds documented there: adjtime()'s slew is bounded
  // and settimeofday() is rare, so adjusted_boot_time() is stable across this window.
  int64_t anchored_us;
  if (this->use_pps_anchor_ && this->anchor_epoch_us_(hook_us, anchored_us))
    return NTPServer::micros_epoch_to_ntp_timestamp_(anchored_us);

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
  // Advertise the precision of the clock we actually read for this reply.
  response[3] = static_cast<uint8_t>(this->use_pps_anchor_ ? this->precision_anchor_ : this->precision_);

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
  float disp_s = this->use_pps_anchor_ ? ROOT_DISP_ANCHOR_S : ROOT_DISP_BASE_S;
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
  NTPTimestamp transmit_ts = this->get_ntp_timestamp_(this->send_estimator_.estimate());
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
  int64_t anchored_us;
  if (this->use_pps_anchor_ && this->anchor_epoch_us_(esp_timer_get_time(), anchored_us))
    return NTPServer::micros_epoch_to_ntp_timestamp_(anchored_us + offset_us);

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
    const uint32_t netmask = netif_ip4_netmask(netif)->addr;
    const uint32_t our_ip = netif_ip4_addr(netif)->addr;
    const uint32_t gateway = netif_ip4_gw(netif)->addr;
    const uint32_t hop = next_hop_ipv4(addr, our_ip, netmask, gateway);
    if (hop == 0)
      continue;  // off-subnet with no gateway configured; nothing sensible to resolve
    ip4_addr_t ip;
    ip.addr = hop;

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
    ESP_LOGD(TAG, "ARP priming: re-resolved %u cold client(s), %u total", (unsigned) primed,
             (unsigned) this->arp_primes_);
  }
}

}  // namespace ntp_server
}  // namespace esphome
