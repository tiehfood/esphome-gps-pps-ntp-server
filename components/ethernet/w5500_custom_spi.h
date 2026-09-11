#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && defined(USE_ETHERNET_W5500)

#include <esp_idf_version.h>
// IDF 6.0 moved the per-chip SPI MAC drivers to the Espressif Component Registry; eth_w5500_config_t
// is no longer reachable through esp_eth.h and needs the explicit header.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#include <esp_eth_mac_w5500.h>
#else
#include <esp_eth.h>
#endif

namespace esphome::ethernet {

// Installs a custom W5500 SPI driver that offloads the bulk frame transfers off the busy-wait path.
//
// The stock W5500 driver runs every SPI transfer through spi_device_polling_transmit(), which
// busy-waits the CPU for the whole transfer. The frame payload (one large read per received frame,
// one large write per transmitted frame) is by far the biggest transfer, so the RX task and the TX
// caller each spin for hundreds of microseconds per frame. This driver sends payload transfers
// through the blocking, interrupt-driven spi_device_transmit() instead, so the calling task sleeps
// while DMA moves the bytes. Small register accesses stay on the polling path, where the busy-wait
// is cheaper than an interrupt round-trip.
//
// Must be called before esp_eth_mac_new_w5500(). The driver reads spi_host_id and spi_devcfg back
// out of `config` in its init() callback, so `config` (and the spi_devcfg it points at) must stay
// alive until esp_eth_mac_new_w5500() returns.
void install_w5500_async_spi(eth_w5500_config_t &config);

// ---------------------------------------------------------------------------------------
// LOCAL DELTA vs upstream: NTP timestamping taps.
//
// The whole reason this component is forked. ntp_server needs to know WHEN a frame arrived
// and WHEN the chip was told to transmit, and the only place both are visible is inside the
// SPI callbacks. Upstream keeps small register accesses on the polling path, so both taps
// below sit on that path and are unaffected by the bulk/DMA split.
//
// Keep this delta confined to this file so re-forking stays a one-file diff.

/// micros() at the earliest points of a W5500 receive: the first SPI transaction of the
/// burst (the driver servicing INTn), the Sn_RX_RSR read, and the payload read.
/// payloads_since_size_read == 1 means the size_read stamp belongs to the frame being
/// delivered right now rather than to an earlier one in the same burst.
struct W5500RxStamps {
  uint32_t size_read_us;
  uint32_t payload_us;
  uint32_t payloads_since_size_read;
  uint32_t burst_start_us;
};
W5500RxStamps w5500_rx_stamps();

/// micros() at the Sn_CR = SEND write for socket 0 -- when the chip was actually told to
/// transmit. seq lets a reader tell a fresh stamp from a stale one.
struct W5500SendStamp {
  uint32_t send_cmd_us;
  uint32_t seq;
};
W5500SendStamp w5500_send_stamp();

/// micros() captured in hardware (MCPWM) at the W5500 INTn falling edge. Normally 3-25 us
/// ahead of the burst-start stamp. Its real use is the exception: when the receive path
/// stalls, the burst starts up to ~100 ms after this edge, and ntp_server refuses the request.
struct W5500IntStamp {
  uint32_t edge_us;
  uint32_t seq;
};
W5500IntStamp w5500_int_stamp();
void w5500_start_int_capture(int gpio_num);

/// Sn_CR command handshakes since the last call; reading resets them. The stock driver writes a
/// command -- SEND for every transmitted frame, RECV after every received one -- then polls
/// Sn_CR until the chip clears it, sleeping vTaskDelay(10 ms) between polls (100 ms timeout).
///   commands  handshakes seen to complete
///   retried   of those, how many needed more than one poll: each cost the caller >= 10 ms
///   max_us    longest write-to-cleared time
///   overlaps  commands written while the previous one had not yet been seen to clear
struct W5500CmdStats {
  uint32_t commands;
  uint32_t retried;
  uint32_t max_us;
  uint32_t overlaps;
};
W5500CmdStats w5500_take_cmd_stats();

/// Network activity recorder: 100 ms bins over the last 120 s, counting frames through the W5500
/// by class and direction, their bytes, the longest wait for the SPI device lock, and Sn_CR
/// commands that needed a retry. Off until enabled; the bins are allocated (PSRAM first) on the
/// first enable. Built to find what the server is doing when its receive path stalls, without
/// adding any traffic of its own. Diagnostic-grade: a bin rolls under a spinlock, but a frame
/// racing the roll may land in the neighbouring bin.
enum W5500FrameClass : uint8_t {
  W5500_FC_API = 0,  // TCP 6053, ESPHome native API (Home Assistant, esphome logs)
  W5500_FC_HTTP,     // TCP 80, the web server
  W5500_FC_NTP,      // UDP 123
  W5500_FC_MDNS,     // UDP 5353
  W5500_FC_ARP,
  W5500_FC_TCP,      // any other TCP
  W5500_FC_UDP,      // any other UDP
  W5500_FC_OTHER,
  W5500_FC_COUNT
};
struct W5500NetBin {
  uint32_t start_us;  // micros() at the bin start; 0 = never used
  uint16_t tx[W5500_FC_COUNT];
  uint16_t rx[W5500_FC_COUNT];
  uint32_t tx_bytes;
  uint32_t rx_bytes;
  uint32_t lock_wait_max_us;
  uint16_t cr_retried;
};
static constexpr uint32_t W5500_NET_BIN_US = 100000;
static constexpr uint16_t W5500_NET_BINS = 1200;
void w5500_set_net_recorder(bool enable);
bool w5500_net_recorder_enabled();
/// Copies bin i, oldest first (0 .. W5500_NET_BINS-1). False if the recorder never ran or the
/// bin is unused.
bool w5500_net_bin(uint16_t i, W5500NetBin &out);

}  // namespace esphome::ethernet

#endif  // USE_ESP32 && USE_ETHERNET_W5500
