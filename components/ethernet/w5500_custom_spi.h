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

#include "w5500_talker_table.h"

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
/// size_value is the Sn_RX_RSR value itself (bytes waiting in the RX buffer, including the
/// frame this size_read stamp belongs to) -- diagnostic-only, to tell whether another frame
/// was already queued behind ours when it arrived.
struct W5500RxStamps {
  uint32_t size_read_us;
  uint32_t payload_us;
  uint32_t payloads_since_size_read;
  uint32_t burst_start_us;
  uint16_t size_value;
};
W5500RxStamps w5500_rx_stamps();

/// micros() at the Sn_CR = SEND write for socket 0 -- when the chip was actually told to
/// transmit. seq lets a reader tell a fresh stamp from a stale one. frame_class is the class of
/// the frame most recently written to the TX buffer before this SEND -- ntp_server only learns
/// its T3 estimate from a SEND provably for W5500_FC_NTP, never from an ARP request (or
/// anything else) that happened to be sent around the same time. txbuf_start_us/txbuf_end_us
/// bracket the SPI transfer that wrote that frame's payload into the TX buffer -- diagnostic
/// only (0 when the network recorder is off, meaning "not measured"), to split the send path
/// into "before the frame reached the W5500" vs "between that and Sn_CR = SEND".
struct W5500SendStamp {
  uint32_t send_cmd_us;
  uint32_t seq;
  uint8_t frame_class;
  uint32_t txbuf_start_us;
  uint32_t txbuf_end_us;
  /// Design B ("NTP Post-Write T3"): micros() when the patch callback ran for this SEND's
  /// frame, and whether the patch was fully applied (T3 written, and the checksum too if it was
  /// present) -- 0/0 when no patch fn was registered or the frame did not pass
  /// plan_ntp_tx_patch(). patched can be 1 even when frame_class != W5500_FC_NTP is never the
  /// case in practice (plan_ntp_tx_patch already requires an NTP frame), but callers should
  /// still gate on frame_class == W5500_FC_NTP the same way they already do for T3 learning.
  uint32_t patch_us;
  uint8_t patched;
  /// Design H ("SEND->wire latency", measurement only): micros() at the first SPI read of
  /// socket 0's Sn_IR that observed the chip's own SEND_OK bit set for this SEND -- i.e. the
  /// driver's own completion poll in emac_w5500_transmit(), read passively. sendok_seq is the
  /// value of `seq` above at the moment that read happened; a caller must compare it against
  /// this same W5500SendStamp's `seq` to know the SEND_OK provably belongs to THIS SEND rather
  /// than being stale (never re-observed since an earlier one, e.g. the driver's own retry cap
  /// fired and the frame never got a fresh SEND_OK before the next SEND was issued). sendok_us
  /// is meaningless unless sendok_seq == seq.
  uint32_t sendok_us;
  uint32_t sendok_seq;
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
  W5500_FC_IPV6,     // EtherType 0x86DD -- lwIP's own IPv6 is off; identifies the :07 multicast burst
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

/// Talkers seen sending a broadcast/multicast frame -- see w5500_talker_table.h. Cleared
/// whenever the recorder is (re-)enabled, alongside the bins above.
static constexpr uint8_t W5500_TALKER_TABLE_SIZE = W5500TalkerTable::SIZE;
/// Copies out entry i (insertion order). False if i is out of range or unused.
bool w5500_talker(uint8_t i, W5500Talker &out);

// ---------------------------------------------------------------------------------------
// LOCAL DELTA: design A, "W5500 Short Interrupt Wait".
//
// INTLEVEL (common register 0x0013, 2 bytes big-endian) sets the chip's own Interrupt Assert
// Wait Time = (INTLEVEL + 1) * 4 / 150 MHz. The driver writes 0xFFFF at init (1.748 ms) to avoid
// missing a quickly re-asserted interrupt on its NEGEDGE GPIO; 0x0FFF (~109 us) still leaves
// that margin while cutting the wait sixteen-fold. A live register write with no independent
// recovery path if it went wrong, so ntp_server gates it behind a switch plus a dead-man timer
// (see ntp_server/deadman.h) -- these two functions are the raw register access only.
///
/// False when no W5500 SPI context exists yet (e.g. called before esp_eth_start()) or the SPI
/// transfer itself failed.
bool w5500_write_int_level(uint16_t value);
bool w5500_read_int_level(uint16_t *out);
/// The value the driver's own init wrote to INTLEVEL (normally 0xFFFF), or 0 if never observed
/// -- what set_short_int_wait(false) restores to.
uint16_t w5500_driver_int_level();

// ---------------------------------------------------------------------------------------
// LOCAL DELTA: design B, "NTP Post-Write T3".
//
// Rewrites T3 (and, if present, the UDP checksum) into an NTP reply's bytes already sitting in
// the W5500 TX buffer, immediately before the Sn_CR = SEND write that actually transmits it --
// closer to the wire than any timestamp written before sendto() can be. The pure planning and
// checksum-update math lives in ntp_server/ntp_tx_patch.h (host-tested); this is only the
// callback wiring and the extra SPI writes.
///
/// fn(ctx, now_us, t3_out): called with a micros() reading taken just before the SEND write, for
/// a frame already confirmed (by plan_ntp_tx_patch) to be our own NTP reply on `server_port`.
/// Returns false to leave the frame untouched (e.g. clock not synchronized) -- the SEND still
/// proceeds either way. Runs in the transmitting task's context, inside sendto(): no
/// ESP_LOGx/publish_state, no blocking, no allocation, no gettimeofday().
///
/// fn == nullptr disables the feature. While disabled this costs nothing: the outgoing-frame
/// tracking it depends on (capturing each frame's TX-buffer header bytes) only runs while a fn
/// is registered.
using W5500TxPatchFn = bool (*)(void *ctx, uint32_t now_us, uint8_t t3_out[8]);
void w5500_set_tx_patch(W5500TxPatchFn fn, void *ctx, uint16_t server_port);

}  // namespace esphome::ethernet

#endif  // USE_ESP32 && USE_ETHERNET_W5500
