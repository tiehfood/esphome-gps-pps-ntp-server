#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace esphome {
namespace ntp_server {

/// Dependency-free planning for design B ("NTP Post-Write T3"): rewriting T3 (and the UDP
/// checksum) into an NTP reply already sitting in the W5500's TX buffer, right before the
/// Sn_CR = SEND write that transmits it. Kept free of ESP-IDF/ESPHome types so the addressing
/// and checksum math are host-testable (tests/ntp_tx_patch_test.cpp); the SPI writes themselves
/// live in the fork, components/ethernet/w5500_custom_spi.cpp.

/// What a candidate outgoing frame needs patched, and the bytes it had before the patch (for a
/// best-effort restore if writing the new checksum fails after T3 was already rewritten).
struct NtpTxPatchPlan {
  bool ok;
  uint16_t t3_off;      ///< frame-relative offset of the 8-byte transmit timestamp
  uint16_t csum_off;     ///< frame-relative offset of the 2-byte UDP checksum
  bool csum_present;     ///< false when the sender left UDP checksum offloaded (field == 0)
  uint16_t old_csum;
  uint8_t old_t3[8];
};

/// Identifies our own NTP reply among everything written to the TX buffer, purely from the
/// header bytes captured for this frame (see w5500_custom_spi.cpp's outgoing-frame tracking).
/// `frame` need not be the whole frame -- only enough of it to cover t3_off + 8 bytes.
///
/// Checks (frame-relative, Ethernet header at offset 0):
///   EtherType == 0x0800 (IPv4) @12
///   IPv4 version == 4, IHL >= 5                        @14
///   protocol == 17 (UDP)                               @14+9
///   UDP source port == server_port                     @14+ihl*4
///   UDP length >= 8 (UDP header) + 48 (NTP packet)      @14+ihl*4+4
///   NTP mode (payload[0] & 0x07) == 4 (server)          @ntp_offset
inline NtpTxPatchPlan plan_ntp_tx_patch(const uint8_t *frame, uint32_t len, uint16_t server_port) {
  NtpTxPatchPlan plan{};
  plan.ok = false;

  static constexpr uint32_t ETH_HDR_LEN = 14;
  static constexpr uint32_t MIN_IPV4_HDR_LEN = 20;
  static constexpr uint32_t UDP_HDR_LEN = 8;
  static constexpr uint32_t NTP_PACKET_SIZE = 48;
  static constexpr uint16_t ETHERTYPE_IPV4 = 0x0800;
  static constexpr uint8_t IP_PROTO_UDP = 17;

  if (frame == nullptr || len < ETH_HDR_LEN + MIN_IPV4_HDR_LEN)
    return plan;

  const uint16_t ethertype = (static_cast<uint16_t>(frame[12]) << 8) | frame[13];
  if (ethertype != ETHERTYPE_IPV4)
    return plan;

  const uint8_t ver_ihl = frame[ETH_HDR_LEN];
  const uint8_t version = (ver_ihl >> 4) & 0x0F;
  const uint8_t ihl = ver_ihl & 0x0F;
  if (version != 4 || ihl < 5)
    return plan;

  const uint32_t ip_hdr_len = static_cast<uint32_t>(ihl) * 4;
  const uint32_t udp_offset = ETH_HDR_LEN + ip_hdr_len;
  if (len < udp_offset + UDP_HDR_LEN)
    return plan;
  if (frame[ETH_HDR_LEN + 9] != IP_PROTO_UDP)
    return plan;

  const uint16_t src_port = (static_cast<uint16_t>(frame[udp_offset]) << 8) | frame[udp_offset + 1];
  if (src_port != server_port)
    return plan;

  const uint16_t udp_len = (static_cast<uint16_t>(frame[udp_offset + 4]) << 8) | frame[udp_offset + 5];
  if (udp_len < UDP_HDR_LEN + NTP_PACKET_SIZE)
    return plan;

  const uint32_t ntp_offset = udp_offset + UDP_HDR_LEN;
  if (len < ntp_offset + NTP_PACKET_SIZE)
    return plan;

  const uint8_t mode = frame[ntp_offset] & 0x07;
  if (mode != 4)
    return plan;

  const uint16_t t3_off = static_cast<uint16_t>(ntp_offset + 40);
  const uint16_t csum_off = static_cast<uint16_t>(udp_offset + 6);
  if (len < static_cast<uint32_t>(t3_off) + 8)
    return plan;

  plan.ok = true;
  plan.t3_off = t3_off;
  plan.csum_off = csum_off;
  const uint16_t csum = (static_cast<uint16_t>(frame[csum_off]) << 8) | frame[csum_off + 1];
  plan.csum_present = csum != 0;
  plan.old_csum = csum;
  memcpy(plan.old_t3, &frame[t3_off], 8);
  return plan;
}

/// Incremental UDP/IP checksum update, RFC 1624 eqn. 3: HC' = ~(~HC + ~m + m'), i.e. replacing
/// an old 16-bit big-endian field `old8` with `new8` inside a region already covered by
/// `old_csum`. Handles any even number of changed bytes (here: the 8-byte T3 field, as four
/// 16-bit words) by accumulating every word's contribution before folding the carry once, which
/// is equivalent to redoing the one's-complement sum in any order.
///
/// Only meaningful when the checksum was actually present (non-zero) -- callers must check
/// NtpTxPatchPlan::csum_present first, since 0 means "offloaded", not "checksum of zero".
inline uint16_t udp_csum_update(uint16_t old_csum, const uint8_t old8[8], const uint8_t new8[8]) {
  uint32_t sum = static_cast<uint16_t>(~old_csum);
  for (int i = 0; i < 8; i += 2) {
    const uint16_t old_word = (static_cast<uint16_t>(old8[i]) << 8) | old8[i + 1];
    const uint16_t new_word = (static_cast<uint16_t>(new8[i]) << 8) | new8[i + 1];
    sum += static_cast<uint16_t>(~old_word);
    sum += new_word;
  }
  while ((sum >> 16) != 0)
    sum = (sum & 0xFFFFu) + (sum >> 16);
  const uint16_t result = static_cast<uint16_t>(~sum);
  // UDP rule: a computed checksum of 0 is transmitted as all-ones (0 on the wire means "no
  // checksum"), so a genuine zero result must not be confused with "not present".
  return result == 0 ? 0xFFFFu : result;
}

/// One SPI write's worth of a ring-buffer patch: `len` bytes from the patch source buffer
/// starting at `src`, written to TX-buffer ring address `addr`.
struct RingWrite {
  uint16_t addr;
  uint16_t len;
  uint16_t src;
};

/// Mirrors the stock W5500 driver's own TX-buffer addressing (emac_w5500_transmit /
/// w5500_write_buffer): the ring is W5500_TX_MEM_SIZE (16 KB) bytes, addressed mod that size,
/// and a write that would cross the wrap point is split with the remainder written at ring
/// address 0x4000 (not 0x0000 -- the chip masks the top bits itself, so 0x4000+k aliases to k).
///
/// `frame_start_offset` is the ring address the frame's very first TX-buffer write used (i.e.
/// what the driver passed as its `offset` argument, before any %= 0x4000 the driver itself may
/// already have applied -- this function re-derives `base` the same way). `rel_off`/`n` are the
/// frame-relative byte range to patch. Returns the number of writes written into `out` (1 or 2).
inline uint8_t plan_ring_writes(uint16_t frame_start_offset, uint16_t rel_off, uint16_t n, RingWrite out[2]) {
  static constexpr uint32_t TX_MEM_SIZE = 0x4000;
  const uint32_t base = frame_start_offset % TX_MEM_SIZE;
  const uint32_t start = base + rel_off;
  if (start < TX_MEM_SIZE && start + n > TX_MEM_SIZE) {
    const uint16_t first_len = static_cast<uint16_t>(TX_MEM_SIZE - start);
    out[0] = {static_cast<uint16_t>(start), first_len, 0};
    out[1] = {static_cast<uint16_t>(TX_MEM_SIZE), static_cast<uint16_t>(n - first_len), first_len};
    return 2;
  }
  // Not just "start < TX_MEM_SIZE, no split": also covers start >= TX_MEM_SIZE outright, which
  // is exactly the wrapped-part addressing the driver itself produces (see file comment above) --
  // a single write at that address is correct as-is, no further wrapping needed for the small
  // (<= 8 byte) ranges this function is used for.
  out[0] = {static_cast<uint16_t>(start), n, 0};
  return 1;
}

/// EWMA estimate of how long after the patch callback runs the Sn_CR = SEND write actually
/// happens, so T3 can be predicted forward to the instant the packet leaves rather than the
/// instant it was computed -- same idea as SendEstimator, deliberately kept separate since it
/// measures a different, much shorter interval (a handful of SPI register writes, not a full
/// sendto()).
class PatchDelayEstimator {
 public:
  static constexpr int32_t US_MIN = 10;
  static constexpr int32_t US_MAX = 500;
  static constexpr uint8_t EWMA_SHIFT = 3;
  static constexpr int32_t INITIAL_US = 60;

  int32_t estimate() const { return this->estimate_us_; }

  void learn(int32_t us) {
    if (us < US_MIN || us > US_MAX)
      return;
    this->estimate_us_ += (us - this->estimate_us_) >> EWMA_SHIFT;
  }

 private:
  int32_t estimate_us_{INITIAL_US};
};

}  // namespace ntp_server
}  // namespace esphome
