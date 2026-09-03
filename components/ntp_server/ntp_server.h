#pragma once

#include "esphome/core/component.h"

#ifdef USE_ESP_IDF
#include <sys/socket.h>
#include <netinet/in.h>
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

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

 protected:
  // receive_ts is sampled by the caller right after recvfrom()/parsePacket() returns,
  // not taken internally -- on the ESP-IDF path that call site is recv_task_(), and
  // T2 must reflect arrival, not whatever this function does first.
  void build_ntp_response_(const uint8_t *request, uint8_t *response, const NTPTimestamp &receive_ts);
  NTPTimestamp get_ntp_timestamp_();
  bool is_time_synchronized_();

  uint16_t port_{123};
  gps_pps_time::GPSPPSTime *time_source_{nullptr};

#ifdef USE_ESP_IDF
  int socket_fd_{-1};

  /// log2(s) of max(clock resolution, clock read cost), measured in setup (RFC 5905 11.1).
  int8_t precision_{-20};

  /// Dedicated FreeRTOS task blocked in recvfrom() -- stamps T2 on return instead of
  /// whenever ESPHome's shared loop next polls us. Runs for the component's lifetime;
  /// no handle is kept, matching esp32_camera's framebuffer_task precedent.
  static void recv_task_(void *param);
#else
  WiFiUDP udp_;
#endif
};

}  // namespace ntp_server
}  // namespace esphome
