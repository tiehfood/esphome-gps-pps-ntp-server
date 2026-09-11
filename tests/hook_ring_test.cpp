// Host unit tests for ntp_server's receive-timestamp hook ring.
//
//   clang++ -std=c++17 -Wall -Wextra -Werror -I components/ntp_server \
//       tests/hook_ring_test.cpp -o /tmp/hook_ring_test && /tmp/hook_ring_test
//
// Pins the fix for the stale-hook-match bug found on 2026-09-11: hook_lookup_() used to return
// the FIRST ring match on the bare client transmit timestamp, with no age check, so a client
// that repeats (or zeroes) its transmit field could get a T2 up to minutes old. The ring is now
// keyed on transmit timestamp + source IP + source port, returns the NEWEST match, and rejects
// anything older than MAX_AGE_US.

#include "hook_ring.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using esphome::ntp_server::HookRing;

namespace {
struct TestInfo {
  int64_t t{0};
  int marker{0};
};
}  // namespace

static int g_failures = 0;

static void check(bool ok, const char *name) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok)
    g_failures++;
}

static void make_tx(uint8_t *tx, uint8_t v) { memset(tx, v, 8); }

int main() {
  {
    HookRing<TestInfo> ring;
    uint8_t tx[8];
    make_tx(tx, 0xAB);
    TestInfo in{100, 1};
    ring.record(tx, 0x0A000001u, 1000, in);

    TestInfo out{};
    check(ring.lookup(tx, 0x0A000001u, 1000, 200, &out) && out.marker == 1, "exact key match hits");
  }
  {
    HookRing<TestInfo> ring;
    uint8_t tx[8];
    make_tx(tx, 0xAB);
    ring.record(tx, 0x0A000001u, 1000, TestInfo{100, 1});

    TestInfo out{};
    check(!ring.lookup(tx, 0x0A000002u, 1000, 200, &out), "different source IP does not match");
    check(!ring.lookup(tx, 0x0A000001u, 1001, 200, &out), "different source port does not match");
  }
  {
    // Repeated (e.g. all-zero) transmit field from the same client: newest record wins.
    HookRing<TestInfo> ring;
    uint8_t tx[8];
    make_tx(tx, 0x00);
    ring.record(tx, 0x0A000001u, 1000, TestInfo{100, 1});
    ring.record(tx, 0x0A000001u, 1000, TestInfo{500, 2});
    ring.record(tx, 0x0A000001u, 1000, TestInfo{300, 3});  // written after, but older timestamp

    TestInfo out{};
    check(ring.lookup(tx, 0x0A000001u, 1000, 600, &out) && out.marker == 2,
          "newest-by-time record wins among repeated keys, not most-recently-written");
  }
  {
    // An otherwise-matching entry older than MAX_AGE_US (1,000,000 us) is rejected.
    HookRing<TestInfo> ring;
    uint8_t tx[8];
    make_tx(tx, 0x11);
    ring.record(tx, 0x0A000001u, 1000, TestInfo{0, 1});

    TestInfo out{};
    check(ring.lookup(tx, 0x0A000001u, 1000, 999999, &out) && out.marker == 1,
          "just under the age bound still matches");
    check(!ring.lookup(tx, 0x0A000001u, 1000, 1000001, &out), "older than the age bound is a miss");
  }
  {
    // Ring wraps after more than SIZE (8) records; only the newest entries survive.
    HookRing<TestInfo, 8> ring;
    uint8_t tx[8];
    for (int i = 0; i < 10; i++) {
      make_tx(tx, static_cast<uint8_t>(i));
      ring.record(tx, 0x0A000001u, 1000, TestInfo{static_cast<int64_t>(i), i});
    }
    TestInfo out{};
    make_tx(tx, 0);
    check(!ring.lookup(tx, 0x0A000001u, 1000, 10, &out), "the two oldest entries were overwritten");
    make_tx(tx, 9);
    check(ring.lookup(tx, 0x0A000001u, 1000, 10, &out) && out.marker == 9, "the newest entry survives a wrap");
  }

  std::printf("%s: %d failure(s)\n", g_failures ? "FAILED" : "OK", g_failures);
  return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
