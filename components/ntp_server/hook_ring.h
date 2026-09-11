#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

namespace esphome {
namespace ntp_server {

/// Ring of "who sent this and when did the driver see it" records, written by the ethernet
/// driver's own task (the input-path hook, one frame at a time -- writes never race each
/// other) and read by the NTP task. Dependency-free so it is host-testable
/// (tests/hook_ring_test.cpp).
///
/// Keyed on the client's own NTP transmit timestamp PLUS its source IPv4 address and UDP
/// source port: a bare transmit-timestamp match let a repeated or all-zero transmit field
/// from any client return a stale hit belonging to someone else. `Info` must have an
/// `int64_t t` member naming when the hook recorded it -- lookup() returns the NEWEST match
/// no older than MAX_AGE_US; older matches are treated as a miss rather than a wrong answer.
///
/// Each slot is a seqlock (odd seq = write in progress, even = a consistent snapshot): needed
/// because the 64-bit `t` and the key both tear on Xtensa. See .claude/rules/firmware.md.
template <typename Info, int SIZE = 8>
class HookRing {
 public:
  static constexpr int KEY_SIZE = 14;  // 8B transmit timestamp + 4B source IPv4 + 2B source port
  static constexpr int64_t MAX_AGE_US = 1000000;

  /// Called only from the driver task (single writer).
  void record(const uint8_t *tx, uint32_t src_ip, uint16_t src_port, const Info &info) {
    uint8_t key[KEY_SIZE];
    build_key(key, tx, src_ip, src_port);
    Entry &slot = this->ring_[this->next_];
    this->next_ = static_cast<uint8_t>((this->next_ + 1) % SIZE);

    const uint32_t seq = slot.seq.load(std::memory_order_relaxed);
    slot.seq.store(seq + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    memcpy(slot.key, key, KEY_SIZE);
    slot.info = info;
    std::atomic_thread_fence(std::memory_order_release);
    slot.seq.store(seq + 2, std::memory_order_relaxed);
  }

  /// Called only from the reader task. Returns the newest entry matching
  /// (tx, src_ip, src_port) whose age at now_us is within MAX_AGE_US; an older match is
  /// ignored, not returned -- the caller should treat that the same as no match at all.
  bool lookup(const uint8_t *tx, uint32_t src_ip, uint16_t src_port, int64_t now_us, Info *out) const {
    uint8_t key[KEY_SIZE];
    build_key(key, tx, src_ip, src_port);
    bool found = false;
    Info best{};
    for (int i = 0; i < SIZE; i++) {
      const Entry &slot = this->ring_[i];
      const uint32_t s1 = slot.seq.load(std::memory_order_relaxed);
      if (s1 & 1)
        continue;  // write in progress on this slot -- skip rather than spin
      std::atomic_thread_fence(std::memory_order_acquire);
      uint8_t k[KEY_SIZE];
      memcpy(k, slot.key, KEY_SIZE);
      Info info = slot.info;
      std::atomic_thread_fence(std::memory_order_acquire);
      const uint32_t s2 = slot.seq.load(std::memory_order_relaxed);
      if (s1 != s2)
        continue;  // torn read (write happened mid-copy) -- skip, don't trust it
      if (memcmp(k, key, KEY_SIZE) != 0)
        continue;
      if (now_us - info.t > MAX_AGE_US)
        continue;  // stale -- an old match must not win over no match at all
      if (!found || info.t > best.t) {
        found = true;
        best = info;
      }
    }
    if (found)
      *out = best;
    return found;
  }

 private:
  struct Entry {
    std::atomic<uint32_t> seq{0};
    uint8_t key[KEY_SIZE]{};
    Info info{};
  };

  static void build_key(uint8_t *key, const uint8_t *tx, uint32_t src_ip, uint16_t src_port) {
    memcpy(key, tx, 8);
    memcpy(key + 8, &src_ip, 4);
    memcpy(key + 12, &src_port, 2);
  }

  Entry ring_[SIZE]{};
  uint8_t next_{0};
};

}  // namespace ntp_server
}  // namespace esphome
