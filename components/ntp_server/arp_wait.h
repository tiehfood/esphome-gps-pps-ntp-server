#pragma once

#include <cstdint>

namespace esphome {
namespace ntp_server {

/// Which IPv4 address (network byte order) the reply's sendto() actually needs an ARP entry
/// for: the client itself when it is on our subnet, otherwise the default gateway. Returns 0
/// when the client is off-subnet and no gateway is configured -- nothing sensible to resolve.
inline uint32_t next_hop_ipv4(uint32_t client, uint32_t our_ip, uint32_t netmask, uint32_t gateway) {
  if (((client ^ our_ip) & netmask) == 0)
    return client;
  return gateway;
}

struct ArpWaitResult {
  bool resolved;
  /// True whenever query() was called, i.e. the cache did not already have the entry --
  /// regardless of whether it went on to resolve within max_wait_us.
  bool missed;
  int32_t waited_us;
};

/// Waits for an ARP entry to resolve, dependency-free so the policy (query at most once, never
/// sleep with a lock held, bounded wait) is host-testable -- the lwIP/FreeRTOS calls themselves
/// live in the caller's Lookup/Query/Sleep/Now callables.
///
/// lookup(): true if the entry is already resolved. query(): fires exactly one ARP request when
/// lookup() first fails. sleep_1ms(): must not hold any lock the tcpip thread needs to process the
/// ARP reply. now_us(): a monotonic clock. max_wait_us: give up and report unresolved past this.
template <typename Lookup, typename Query, typename Sleep, typename Now>
ArpWaitResult wait_for_arp(Lookup lookup, Query query, Sleep sleep_1ms, Now now_us, int64_t max_wait_us) {
  if (lookup())
    return {true, false, 0};
  query();
  const int64_t start = now_us();
  while (true) {
    sleep_1ms();
    const int32_t elapsed = static_cast<int32_t>(now_us() - start);
    if (lookup())
      return {true, true, elapsed};
    if (elapsed >= max_wait_us)
      return {false, true, elapsed};
  }
}

}  // namespace ntp_server
}  // namespace esphome
