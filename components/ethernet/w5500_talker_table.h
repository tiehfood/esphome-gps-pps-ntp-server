#pragma once

#include <cstdint>
#include <cstring>

namespace esphome {
namespace ethernet {

/// One (source MAC, EtherType) talker seen sending a broadcast/multicast frame. Diagnostic-only:
/// built to identify the source of the recurring :07 multicast burst without the recorder
/// needing to keep every frame's addresses.
struct W5500Talker {
  uint8_t src_mac[6];
  uint16_t ethertype;
  uint8_t first_dst_mac[6];
  uint32_t frames;
  uint32_t burst_frames;
  uint32_t bytes;
  uint32_t first_us;
  uint32_t last_us;
};

/// Fixed-size table of talkers, keyed on (source MAC, EtherType). Dependency-free so it is
/// host-testable (tests/talker_table_test.cpp); the caller (w5500_custom_spi.cpp) is
/// responsible for serializing access -- this class itself does no locking.
class W5500TalkerTable {
 public:
  static constexpr int SIZE = 32;

  /// Records one frame from (src, ethertype), inserting a new entry if this pair has not been
  /// seen before. When the table is full, evicts the entry with the fewest frames seen so far
  /// (ties broken by the oldest last_us) to make room.
  void note(const uint8_t *src, const uint8_t *dst, uint16_t ethertype, uint32_t len, uint32_t now_us,
            bool burst) {
    int idx = this->find(src, ethertype);
    if (idx < 0)
      idx = this->make_room(src, dst, ethertype, now_us);
    Entry &e = this->entries_[idx];
    e.frames++;
    if (burst)
      e.burst_frames++;
    e.bytes += len;
    e.last_us = now_us;
  }

  void clear() {
    for (auto &e : this->entries_)
      e = Entry{};
  }

  /// Copies out entry i (insertion order, not sorted). False if i is out of range or unused.
  bool get(int i, W5500Talker &out) const {
    if (i < 0 || i >= SIZE || !this->entries_[i].used)
      return false;
    const Entry &e = this->entries_[i];
    memcpy(out.src_mac, e.src_mac, 6);
    out.ethertype = e.ethertype;
    memcpy(out.first_dst_mac, e.first_dst_mac, 6);
    out.frames = e.frames;
    out.burst_frames = e.burst_frames;
    out.bytes = e.bytes;
    out.first_us = e.first_us;
    out.last_us = e.last_us;
    return true;
  }

 private:
  struct Entry {
    bool used{false};
    uint8_t src_mac[6]{};
    uint16_t ethertype{0};
    uint8_t first_dst_mac[6]{};
    uint32_t frames{0};
    uint32_t burst_frames{0};
    uint32_t bytes{0};
    uint32_t first_us{0};
    uint32_t last_us{0};
  };

  int find(const uint8_t *src, uint16_t ethertype) const {
    for (int i = 0; i < SIZE; i++) {
      if (this->entries_[i].used && this->entries_[i].ethertype == ethertype &&
          memcmp(this->entries_[i].src_mac, src, 6) == 0)
        return i;
    }
    return -1;
  }

  /// Finds a free slot, or evicts the least-active one. Initializes the slot for (src, ethertype)
  /// and returns its index.
  int make_room(const uint8_t *src, const uint8_t *dst, uint16_t ethertype, uint32_t now_us) {
    int idx = -1;
    for (int i = 0; i < SIZE; i++) {
      if (!this->entries_[i].used) {
        idx = i;
        break;
      }
    }
    if (idx < 0) {
      idx = 0;
      for (int i = 1; i < SIZE; i++) {
        if (this->entries_[i].frames < this->entries_[idx].frames ||
            (this->entries_[i].frames == this->entries_[idx].frames &&
             this->entries_[i].last_us < this->entries_[idx].last_us)) {
          idx = i;
        }
      }
    }
    Entry &e = this->entries_[idx];
    e = Entry{};
    e.used = true;
    memcpy(e.src_mac, src, 6);
    e.ethertype = ethertype;
    memcpy(e.first_dst_mac, dst, 6);
    e.first_us = now_us;
    e.last_us = now_us;
    return idx;
  }

  Entry entries_[SIZE]{};
};

}  // namespace ethernet
}  // namespace esphome
