// Host unit tests for the ethernet fork's talker table.
//
//   clang++ -std=c++17 -Wall -Wextra -Werror -I components/ethernet \
//       tests/talker_table_test.cpp -o /tmp/talker_table_test && /tmp/talker_table_test
//
// Built to identify the source of the recurring :07 multicast burst. Keyed on (source MAC,
// EtherType); the caller is responsible for locking, this class does none.

#include "w5500_talker_table.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using esphome::ethernet::W5500Talker;
using esphome::ethernet::W5500TalkerTable;

static int g_failures = 0;

static void check(bool ok, const char *name) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok)
    g_failures++;
}

static void mac(uint8_t *out, uint8_t last) {
  uint8_t m[6] = {0x02, 0x00, 0x00, 0x00, 0x00, last};
  memcpy(out, m, 6);
}

int main() {
  {
    W5500TalkerTable t;
    uint8_t src[6], dst[6];
    mac(src, 1);
    mac(dst, 0xFF);
    t.note(src, dst, 0x0800, 100, 1000, false);
    t.note(src, dst, 0x0800, 200, 2000, true);

    W5500Talker out{};
    check(t.get(0, out), "first entry retrievable");
    check(out.frames == 2, "frame count accumulates");
    check(out.burst_frames == 1, "burst_frames only counts frames flagged as burst");
    check(out.bytes == 300, "bytes accumulate");
    check(out.first_us == 1000, "first_us kept from the first frame");
    check(out.last_us == 2000, "last_us updated to the most recent frame");
    check(memcmp(out.first_dst_mac, dst, 6) == 0, "first_dst_mac kept from the first frame");
  }
  {
    // Same source MAC, different EtherType: a separate entry.
    W5500TalkerTable t;
    uint8_t src[6], dst[6];
    mac(src, 1);
    mac(dst, 0xFF);
    t.note(src, dst, 0x0800, 100, 1000, false);
    t.note(src, dst, 0x86DD, 100, 1000, false);

    int count = 0;
    W5500Talker out{};
    for (int i = 0; i < W5500TalkerTable::SIZE; i++)
      if (t.get(i, out))
        count++;
    check(count == 2, "same source MAC with a different EtherType is a separate entry");
  }
  {
    // A second frame from a different dst does not change first_dst_mac.
    W5500TalkerTable t;
    uint8_t src[6], dst1[6], dst2[6];
    mac(src, 1);
    mac(dst1, 0xFF);
    mac(dst2, 0xFE);
    t.note(src, dst1, 0x0800, 10, 1, false);
    t.note(src, dst2, 0x0800, 10, 2, false);
    W5500Talker out{};
    t.get(0, out);
    check(memcmp(out.first_dst_mac, dst1, 6) == 0, "first_dst_mac never changes after the first frame");
  }
  {
    // Fill the table, then force an eviction of the least-active entry.
    W5500TalkerTable t;
    uint8_t src[6], dst[6];
    mac(dst, 0xFF);
    for (int i = 0; i < W5500TalkerTable::SIZE; i++) {
      mac(src, static_cast<uint8_t>(i));
      t.note(src, dst, 0x0800, 10, static_cast<uint32_t>(1000 + i), false);
    }
    // Entry 0 has the fewest frames (1, tied with everyone) and the oldest last_us -- it must
    // be the one evicted.
    uint8_t new_src[6];
    mac(new_src, 200);
    t.note(new_src, dst, 0x0800, 10, 5000, false);

    int count = 0;
    bool found_old0 = false, found_new = false;
    for (int i = 0; i < W5500TalkerTable::SIZE; i++) {
      W5500Talker out{};
      if (!t.get(i, out))
        continue;
      count++;
      uint8_t old0[6];
      mac(old0, 0);
      if (memcmp(out.src_mac, old0, 6) == 0)
        found_old0 = true;
      if (memcmp(out.src_mac, new_src, 6) == 0)
        found_new = true;
    }
    check(count == W5500TalkerTable::SIZE, "table stays at capacity after an eviction");
    check(!found_old0, "the oldest, least-active entry was evicted");
    check(found_new, "the new talker took its place");
  }
  {
    // Eviction prefers fewest frames over oldest last_us when they differ.
    W5500TalkerTable t;
    uint8_t src[6], dst[6];
    mac(dst, 0xFF);
    for (int i = 0; i < W5500TalkerTable::SIZE; i++) {
      mac(src, static_cast<uint8_t>(i));
      t.note(src, dst, 0x0800, 10, static_cast<uint32_t>(1000 + i), false);
    }
    // Give entry 5 a second frame so it is no longer the minimum; entry 0 (oldest, still at 1
    // frame) must be evicted instead even though it isn't the very first note() call above.
    uint8_t src5[6];
    mac(src5, 5);
    t.note(src5, dst, 0x0800, 10, 9999, false);

    uint8_t new_src[6];
    mac(new_src, 201);
    t.note(new_src, dst, 0x0800, 10, 6000, false);

    uint8_t old0[6];
    mac(old0, 0);
    bool found_old0 = false;
    for (int i = 0; i < W5500TalkerTable::SIZE; i++) {
      W5500Talker out{};
      if (t.get(i, out) && memcmp(out.src_mac, old0, 6) == 0)
        found_old0 = true;
    }
    check(!found_old0, "eviction picks the entry with the fewest frames, not merely the oldest");
  }
  {
    W5500TalkerTable t;
    uint8_t src[6], dst[6];
    mac(src, 1);
    mac(dst, 0xFF);
    t.note(src, dst, 0x0800, 10, 1, false);
    t.clear();
    W5500Talker out{};
    check(!t.get(0, out), "clear() empties the table");
  }

  std::printf("%s: %d failure(s)\n", g_failures ? "FAILED" : "OK", g_failures);
  return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
