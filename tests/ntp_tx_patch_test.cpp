// Host unit tests for design B's ("NTP Post-Write T3") dependency-free planning: identifying an
// outgoing NTP reply from the bytes already queued for the W5500's TX buffer, updating its UDP
// checksum incrementally after rewriting T3, and mirroring the stock driver's own TX-buffer ring
// addressing when a patch write straddles the wrap point.
//
//   clang++ -std=c++17 -Wall -Wextra -Werror -I components/ntp_server -I components/ethernet \
//       tests/ntp_tx_patch_test.cpp -o /tmp/ntp_tx_patch_test && /tmp/ntp_tx_patch_test

#include "ntp_tx_patch.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

using esphome::ntp_server::NtpTxPatchPlan;
using esphome::ntp_server::PatchDelayEstimator;
using esphome::ntp_server::RingWrite;
using esphome::ntp_server::plan_ntp_tx_patch;
using esphome::ntp_server::plan_ring_writes;
using esphome::ntp_server::udp_csum_update;

static int g_failures = 0;

static void check(bool ok, const char *name) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok)
    g_failures++;
}

static constexpr uint32_t ETH_HDR_LEN = 14;
static constexpr uint16_t SERVER_PORT = 123;

// Full, from-scratch UDP checksum (RFC 768 pseudo-header) -- forward-declared so make_frame()
// below can use it; defined after make_frame() since it is the "ground truth" the incremental
// update is checked against, not part of what is being built.
static uint16_t udp_checksum_full(const std::vector<uint8_t> &frame);

// Builds a well-formed Ethernet/IPv4/UDP/NTP frame: `ihl` 32-bit words of IPv4 header (options
// zero-filled beyond the mandatory 20 bytes), the client's port as UDP destination and
// SERVER_PORT as source (i.e. as our own reply would be), and NTP mode 4 (server) unless
// overridden. The UDP checksum is left at a real, non-zero value computed the same way a fresh
// recompute would, so csum_present is true by default.
static std::vector<uint8_t> make_frame(uint8_t ihl = 5, uint16_t src_port = SERVER_PORT, uint8_t mode = 4,
                                        bool with_checksum = true) {
  const uint32_t ip_hdr_len = static_cast<uint32_t>(ihl) * 4;
  const uint32_t udp_offset = ETH_HDR_LEN + ip_hdr_len;
  const uint32_t ntp_offset = udp_offset + 8;
  const uint32_t total_len = ntp_offset + 48;
  std::vector<uint8_t> f(total_len, 0);

  // Ethernet header: dst/src MAC arbitrary, EtherType IPv4.
  f[12] = 0x08;
  f[13] = 0x00;

  // IPv4 header.
  f[ETH_HDR_LEN] = static_cast<uint8_t>((4 << 4) | ihl);
  f[ETH_HDR_LEN + 9] = 17;  // protocol UDP
  // Source/destination IP, arbitrary but distinct.
  f[ETH_HDR_LEN + 12] = 192;
  f[ETH_HDR_LEN + 13] = 168;
  f[ETH_HDR_LEN + 14] = 1;
  f[ETH_HDR_LEN + 15] = 1;
  f[ETH_HDR_LEN + 16] = 192;
  f[ETH_HDR_LEN + 17] = 168;
  f[ETH_HDR_LEN + 18] = 1;
  f[ETH_HDR_LEN + 19] = 2;

  // UDP header.
  const uint16_t udp_len = static_cast<uint16_t>(8 + 48);
  f[udp_offset + 0] = static_cast<uint8_t>(src_port >> 8);
  f[udp_offset + 1] = static_cast<uint8_t>(src_port & 0xFF);
  f[udp_offset + 2] = 0xEA;  // arbitrary destination port (60000 high byte-ish)
  f[udp_offset + 3] = 0x60;
  f[udp_offset + 4] = static_cast<uint8_t>(udp_len >> 8);
  f[udp_offset + 5] = static_cast<uint8_t>(udp_len & 0xFF);
  f[udp_offset + 6] = 0;  // checksum filled below
  f[udp_offset + 7] = 0;

  // NTP payload: mode in the low 3 bits of byte 0, a distinguishable T3 field.
  f[ntp_offset + 0] = static_cast<uint8_t>((4 << 3) | mode);
  for (int i = 0; i < 8; i++)
    f[ntp_offset + 40 + i] = static_cast<uint8_t>(0xA0 + i);

  if (with_checksum) {
    const uint16_t csum = udp_checksum_full(f);
    f[udp_offset + 6] = static_cast<uint8_t>(csum >> 8);
    f[udp_offset + 7] = static_cast<uint8_t>(csum & 0xFF);
  }
  return f;
}

// Full, from-scratch UDP checksum (RFC 768 pseudo-header), independent of udp_csum_update's
// incremental math -- the ground truth the incremental update is checked against.
static uint16_t udp_checksum_full(const std::vector<uint8_t> &frame) {
  const uint8_t ihl = frame[ETH_HDR_LEN] & 0x0F;
  const uint32_t udp_offset = ETH_HDR_LEN + static_cast<uint32_t>(ihl) * 4;
  const uint16_t udp_len = (static_cast<uint16_t>(frame[udp_offset + 4]) << 8) | frame[udp_offset + 5];

  uint32_t sum = 0;
  // Pseudo-header: source IP, destination IP, zero + protocol, UDP length.
  for (int i = 0; i < 4; i += 2)
    sum += (static_cast<uint16_t>(frame[ETH_HDR_LEN + 12 + i]) << 8) | frame[ETH_HDR_LEN + 12 + i + 1];
  for (int i = 0; i < 4; i += 2)
    sum += (static_cast<uint16_t>(frame[ETH_HDR_LEN + 16 + i]) << 8) | frame[ETH_HDR_LEN + 16 + i + 1];
  sum += 17u;  // protocol UDP, as the low byte of a 16-bit pseudo-header word
  sum += udp_len;

  // UDP header + payload, with the checksum field itself treated as zero.
  std::vector<uint8_t> body(frame.begin() + udp_offset, frame.begin() + udp_offset + udp_len);
  body[6] = 0;
  body[7] = 0;
  for (size_t i = 0; i < body.size(); i += 2) {
    uint16_t word = static_cast<uint16_t>(body[i]) << 8;
    if (i + 1 < body.size())
      word |= body[i + 1];
    sum += word;
  }
  while ((sum >> 16) != 0)
    sum = (sum & 0xFFFFu) + (sum >> 16);
  const uint16_t result = static_cast<uint16_t>(~sum);
  return result == 0 ? 0xFFFFu : result;
}

static void test_plan_accept_reject() {
  {
    const auto f = make_frame();
    const auto plan = plan_ntp_tx_patch(f.data(), static_cast<uint32_t>(f.size()), SERVER_PORT);
    check(plan.ok, "plain frame (IHL 5, mode 4, matching port) is accepted");
    check(plan.t3_off == ETH_HDR_LEN + 20 + 8 + 40, "t3_off lands on the transmit timestamp field");
    check(plan.csum_off == ETH_HDR_LEN + 20 + 6, "csum_off lands on the UDP checksum field");
    check(plan.csum_present, "a non-zero checksum field is reported present");
  }
  {
    auto f = make_frame();
    f[12] = 0x86;
    f[13] = 0xDD;  // IPv6 EtherType
    const auto plan = plan_ntp_tx_patch(f.data(), static_cast<uint32_t>(f.size()), SERVER_PORT);
    check(!plan.ok, "wrong EtherType is rejected");
  }
  {
    // IPv4 options: IHL 6 (24-byte header) instead of the minimum 5.
    const auto f = make_frame(/*ihl=*/6);
    const auto plan = plan_ntp_tx_patch(f.data(), static_cast<uint32_t>(f.size()), SERVER_PORT);
    check(plan.ok, "IPv4 options (IHL 6) are still accepted");
    check(plan.t3_off == ETH_HDR_LEN + 24 + 8 + 40, "t3_off accounts for the longer IPv4 header");
  }
  {
    const auto f = make_frame(/*ihl=*/5, /*src_port=*/8888);
    const auto plan = plan_ntp_tx_patch(f.data(), static_cast<uint32_t>(f.size()), SERVER_PORT);
    check(!plan.ok, "a UDP source port other than the server's own is rejected");
  }
  {
    const auto f = make_frame(/*ihl=*/5, /*src_port=*/SERVER_PORT, /*mode=*/3);
    const auto plan = plan_ntp_tx_patch(f.data(), static_cast<uint32_t>(f.size()), SERVER_PORT);
    check(!plan.ok, "NTP mode 3 (client) is rejected");
  }
  {
    auto f = make_frame();
    f.resize(f.size() - 1);  // one byte short of covering t3_off + 8
    const auto plan = plan_ntp_tx_patch(f.data(), static_cast<uint32_t>(f.size()), SERVER_PORT);
    check(!plan.ok, "a frame too short to cover the T3 field is rejected");
  }
  {
    const auto f = make_frame(/*ihl=*/5, SERVER_PORT, /*mode=*/4, /*with_checksum=*/false);
    const auto plan = plan_ntp_tx_patch(f.data(), static_cast<uint32_t>(f.size()), SERVER_PORT);
    check(plan.ok && !plan.csum_present, "a zero UDP checksum field is reported as not present");
  }
}

static void test_csum_unchanged_is_identity() {
  const uint8_t zero8[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (uint16_t old_csum : {static_cast<uint16_t>(1), static_cast<uint16_t>(0x1234), static_cast<uint16_t>(0xFFFF)}) {
    const uint16_t updated = udp_csum_update(old_csum, zero8, zero8);
    char name[96];
    std::snprintf(name, sizeof(name), "unchanged 8 bytes leaves checksum 0x%04x unchanged", old_csum);
    check(updated == old_csum, name);
  }
}

static void test_csum_zero_result_maps_to_ffff() {
  // Constructed so the RFC 1624 sum folds to exactly 0xFFFF pre-complement: old_csum == 0xFFFF
  // and old8 == new8 (see test_csum_unchanged_is_identity for why unchanged data is a fixed
  // point) -- the one input in that family where the fixed point itself sits at the UDP
  // zero/all-ones boundary, so it also exercises the "never emit 0x0000" mapping.
  const uint8_t word[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  check(udp_csum_update(0xFFFF, word, word) == 0xFFFF, "a result that would be 0x0000 is reported as 0xFFFF");
}

static void test_csum_against_full_recompute() {
  std::mt19937 rng(0xC0FFEE);
  std::uniform_int_distribution<int> byte_dist(0, 255);
  std::uniform_int_distribution<int> ihl_dist(5, 8);
  int checked = 0;
  for (int trial = 0; trial < 1000; trial++) {
    auto f = make_frame(static_cast<uint8_t>(ihl_dist(rng)));
    // Randomize the old T3 field and the checksum-irrelevant destination port, then recompute a
    // correct starting checksum for THIS frame.
    const uint8_t ihl = f[ETH_HDR_LEN] & 0x0F;
    const uint32_t udp_offset = ETH_HDR_LEN + static_cast<uint32_t>(ihl) * 4;
    const uint32_t ntp_offset = udp_offset + 8;
    const uint32_t t3_off = ntp_offset + 40;
    uint8_t old_t3[8];
    for (auto &b : old_t3) {
      b = static_cast<uint8_t>(byte_dist(rng));
      f[t3_off + (&b - old_t3)] = b;
    }
    const uint16_t old_csum = udp_checksum_full(f);
    f[udp_offset + 6] = static_cast<uint8_t>(old_csum >> 8);
    f[udp_offset + 7] = static_cast<uint8_t>(old_csum & 0xFF);

    const auto plan = plan_ntp_tx_patch(f.data(), static_cast<uint32_t>(f.size()), SERVER_PORT);
    if (!plan.ok) {
      check(false, "random frame accepted by plan_ntp_tx_patch");
      continue;
    }

    uint8_t new_t3[8];
    for (auto &b : new_t3)
      b = static_cast<uint8_t>(byte_dist(rng));

    const uint16_t incremental = udp_csum_update(plan.old_csum, plan.old_t3, new_t3);

    auto patched = f;
    std::memcpy(&patched[t3_off], new_t3, 8);
    const uint16_t full = udp_checksum_full(patched);

    if (incremental != full) {
      char name[128];
      std::snprintf(name, sizeof(name), "trial %d: incremental 0x%04x != full recompute 0x%04x", trial, incremental,
                    full);
      check(false, name);
    }
    checked++;
  }
  check(checked == 1000, "all 1000 random frames were exercised (plan accepted every one)");
}

static void test_ring_writes() {
  {
    RingWrite w[2];
    const uint8_t n = plan_ring_writes(0x1000, 0x10, 8, w);
    check(n == 1 && w[0].addr == 0x1010 && w[0].len == 8 && w[0].src == 0, "no-split write lands at base+rel_off");
  }
  {
    // Straddles the wrap with a zero base.
    RingWrite w[2];
    const uint8_t n = plan_ring_writes(0, 0x3FFA, 8, w);
    check(n == 2, "a write crossing 0x4000 splits into two");
    check(w[0].addr == 0x3FFA && w[0].len == 6 && w[0].src == 0, "first half covers up to the wrap point");
    check(w[1].addr == 0x4000 && w[1].len == 2 && w[1].src == 6, "second half resumes at ring address 0x4000");
  }
  {
    // Straddles the wrap with a non-zero base (frame itself started mid-buffer).
    RingWrite w[2];
    const uint8_t n = plan_ring_writes(0x3FFA, 4, 8, w);
    check(n == 2, "a non-zero base write crossing 0x4000 also splits");
    check(w[0].addr == 0x3FFE && w[0].len == 2 && w[0].src == 0, "first half of the offset base case");
    check(w[1].addr == 0x4000 && w[1].len == 6 && w[1].src == 2, "second half of the offset base case");
  }
  {
    // Exactly at the 0x3FFF boundary: one byte fits before the wrap, no split needed.
    RingWrite w[2];
    const uint8_t n = plan_ring_writes(0x3FFF, 0, 1, w);
    check(n == 1 && w[0].addr == 0x3FFF && w[0].len == 1, "a write ending exactly at 0x4000 does not split");
  }
  {
    // Exactly at the 0x4000 boundary: base+rel_off itself is 0x4000, single write there.
    RingWrite w[2];
    const uint8_t n = plan_ring_writes(0, 0x4000, 8, w);
    check(n == 1 && w[0].addr == 0x4000 && w[0].len == 8, "a write starting exactly at 0x4000 is a single write there");
  }
}

static void test_patch_delay_estimator() {
  PatchDelayEstimator e;
  check(e.estimate() == PatchDelayEstimator::INITIAL_US, "starts at the initial estimate");
  e.learn(5);
  check(e.estimate() == PatchDelayEstimator::INITIAL_US, "a sample below US_MIN is ignored");
  e.learn(600);
  check(e.estimate() == PatchDelayEstimator::INITIAL_US, "a sample above US_MAX is ignored");
  e.learn(60);
  check(e.estimate() == PatchDelayEstimator::INITIAL_US, "a sample equal to the current estimate leaves it unchanged");
  e.learn(100);
  check(e.estimate() == 65, "a normal sample moves the estimate by 1/8 (EWMA_SHIFT)");
  PatchDelayEstimator boundary;
  boundary.learn(PatchDelayEstimator::US_MIN);
  check(boundary.estimate() != PatchDelayEstimator::INITIAL_US, "US_MIN itself is accepted (inclusive bound)");
  PatchDelayEstimator boundary_max;
  boundary_max.learn(PatchDelayEstimator::US_MAX);
  check(boundary_max.estimate() != PatchDelayEstimator::INITIAL_US, "US_MAX itself is accepted (inclusive bound)");
}

int main() {
  test_plan_accept_reject();
  test_csum_unchanged_is_identity();
  test_csum_zero_result_maps_to_ffff();
  test_csum_against_full_recompute();
  test_ring_writes();
  test_patch_delay_estimator();

  std::printf("%s: %d failure(s)\n", g_failures ? "FAILED" : "OK", g_failures);
  return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
