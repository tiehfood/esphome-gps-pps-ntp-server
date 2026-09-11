// Host unit tests for ntp_server's ARP-miss handling.
//
//   clang++ -std=c++17 -Wall -Wextra -Werror -I components/ntp_server -I components/ethernet \
//       tests/arp_wait_test.cpp -o /tmp/arp_wait_test && /tmp/arp_wait_test
//
// Pins the policy that fixed the ARP-miss bug found on 2026-09-11: sendto() would otherwise
// queue the reply behind an ARP round trip and a T3 stamped for the ARP request, not the NTP
// frame, could be mislearned. next_hop_ipv4() decides who needs resolving; wait_for_arp() waits
// for it once, deterministically, without ever holding a lock across the sleep.

#include "arp_wait.h"

#include <cstdio>
#include <cstdlib>

using esphome::ntp_server::ArpWaitResult;
using esphome::ntp_server::next_hop_ipv4;
using esphome::ntp_server::wait_for_arp;

static int g_failures = 0;

static void check(bool ok, const char *name) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok)
    g_failures++;
}

// Fake IPv4s, host byte order for readability -- next_hop_ipv4() only ever XORs/ANDs them, so
// byte order does not matter for these tests as long as it is consistent.
static constexpr uint32_t OUR_IP = 0xC0A80101;      // 192.168.1.1
static constexpr uint32_t NETMASK = 0xFFFFFF00;     // /24
static constexpr uint32_t GATEWAY = 0xC0A801FE;     // 192.168.1.254
static constexpr uint32_t ON_SUBNET = 0xC0A80142;   // 192.168.1.66
static constexpr uint32_t OFF_SUBNET = 0x0A000005;  // 10.0.0.5

int main() {
  check(next_hop_ipv4(ON_SUBNET, OUR_IP, NETMASK, GATEWAY) == ON_SUBNET,
        "on-subnet client resolves to itself");
  check(next_hop_ipv4(OFF_SUBNET, OUR_IP, NETMASK, GATEWAY) == GATEWAY,
        "off-subnet client resolves via the gateway");
  check(next_hop_ipv4(OFF_SUBNET, OUR_IP, NETMASK, 0) == 0,
        "off-subnet client with no gateway configured resolves to nothing");

  {
    // Cached: lookup() succeeds immediately, query() must never be called.
    int queries = 0;
    auto lookup = [&]() { return true; };
    auto query = [&]() { queries++; };
    int sleeps = 0;
    auto sleep_1ms = [&]() { sleeps++; };
    int64_t now = 0;
    auto now_us = [&]() { return now; };
    ArpWaitResult r = wait_for_arp(lookup, query, sleep_1ms, now_us, 20000);
    check(r.resolved && !r.missed && r.waited_us == 0, "a cached entry resolves with no wait");
    check(queries == 0, "a cached entry never queries");
    check(sleeps == 0, "a cached entry never sleeps");
  }
  {
    // Resolves after N milliseconds of waiting.
    bool resolved_after = false;
    int elapsed_ms = 0;
    auto lookup = [&]() { return resolved_after; };
    int queries = 0;
    auto query = [&]() { queries++; };
    int64_t now = 1000;
    auto sleep_1ms = [&]() {
      now += 1000;
      elapsed_ms++;
      if (elapsed_ms >= 5)
        resolved_after = true;
    };
    auto now_us = [&]() { return now; };
    ArpWaitResult r = wait_for_arp(lookup, query, sleep_1ms, now_us, 20000);
    check(r.resolved && r.missed && r.waited_us == 5000, "resolves after N ms of waiting");
    check(queries == 1, "query() is called exactly once on a miss");
  }
  {
    // Never resolves: times out at the configured bound.
    auto lookup = [&]() { return false; };
    int queries = 0;
    auto query = [&]() { queries++; };
    int64_t now = 0;
    auto sleep_1ms = [&]() { now += 1000; };
    auto now_us = [&]() { return now; };
    ArpWaitResult r = wait_for_arp(lookup, query, sleep_1ms, now_us, 20000);
    check(!r.resolved && r.missed, "times out when the entry never resolves");
    check(r.waited_us >= 20000, "waited at least the configured bound");
    check(queries == 1, "still only queried once even though it never resolved");
  }
  {
    // query() called exactly once regardless of how many poll iterations follow.
    auto lookup = [&]() { return false; };
    int queries = 0;
    auto query = [&]() { queries++; };
    int64_t now = 0;
    int iterations = 0;
    auto sleep_1ms = [&]() {
      now += 1000;
      iterations++;
    };
    auto now_us = [&]() { return now; };
    wait_for_arp(lookup, query, sleep_1ms, now_us, 5000);
    check(queries == 1 && iterations >= 5, "one query across a multi-iteration wait");
  }

  std::printf("%s: %d failure(s)\n", g_failures ? "FAILED" : "OK", g_failures);
  return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
