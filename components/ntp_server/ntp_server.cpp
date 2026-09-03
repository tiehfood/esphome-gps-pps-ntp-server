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
static const float ROOT_DISP_BASE_S = 0.005f;
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
  // Nothing to do: recv_task_() serves every request on its own task, blocked
  // in recvfrom(). Kept as a no-op because Component::loop() is pure virtual.
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
    // T2: stamped the instant recvfrom() returns -- this replaces the shared-loop
    // poll and is the entire reason this task exists.
    NTPTimestamp receive_ts = self->get_ntp_timestamp_();

    if (received < 0) {
      // Back off. If the socket goes permanently bad (EBADF) an unguarded
      // continue would spin this task forever and peg core 1.
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (received < NTP_PACKET_SIZE)
      continue;  // runt packet -- drop, keep serving

    // No ESP_LOGD/publish_state here: ESPHome's logger and API are not task-safe
    // from a non-main task. Drop silently rather than working around it.
    if (!self->is_time_synchronized_())
      continue;

    self->build_ntp_response_(buffer, response, receive_ts);

    int64_t t0 = esp_timer_get_time();
    sendto(self->socket_fd_, response, NTP_PACKET_SIZE, 0,
           (struct sockaddr *) &client_addr, client_len);
    int32_t dur = static_cast<int32_t>(esp_timer_get_time() - t0);

    // Only learn from sends that actually reached the wire. On an ARP miss lwIP
    // queues the packet and returns immediately, which would drag the estimate
    // down even though that packet departs late.
    if (dur > SEND_US_MIN && dur < SEND_US_MAX)
      self->send_us_ += (dur - self->send_us_) / 8;
  }
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
    return true;  // No time source configured, assume synchronized
  return this->time_source_->is_synchronized();
}

void NTPServer::dump_config() {
  ESP_LOGCONFIG(TAG, "NTP Server:");
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  ESP_LOGCONFIG(TAG, "  Time source: %s", this->time_source_ != nullptr ? "configured" : "none");
}

}  // namespace ntp_server
}  // namespace esphome
