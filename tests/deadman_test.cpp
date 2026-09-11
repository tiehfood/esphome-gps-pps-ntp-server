// Host unit tests for the W5500 short-interrupt-wait dead-man timer.
//
//   clang++ -std=c++17 -Wall -Wextra -Werror -I components/ntp_server \
//       tests/deadman_test.cpp -o /tmp/deadman_test && /tmp/deadman_test
//
// The one property that matters on real hardware: a register write made unattended MUST revert
// itself even if nobody calls disarm() again -- these tests exercise arm/expire/disarm/remaining
// and the millis() 32-bit wrap, since arm()/expired() are pure uint32_t arithmetic and any wrap
// bug would silently disable the safety net for ~24 days after every ~49.7-day wrap.

#include "deadman.h"

#include <cstdio>
#include <cstdlib>

using esphome::ntp_server::DeadmanTimer;
using esphome::ntp_server::int_level_still_short;

static int g_failures = 0;

static void check(bool ok, const char *name) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok)
    g_failures++;
}

int main() {
  {
    DeadmanTimer t;
    check(!t.armed(), "starts disarmed");
    check(!t.expired(0), "never expired while disarmed");
    check(t.remaining_ms(0) == 0, "no remaining time while disarmed");
  }
  {
    DeadmanTimer t;
    t.arm(1000, 5000);
    check(t.armed(), "armed() true after arm()");
    check(!t.expired(1000), "not expired right at arm time");
    check(!t.expired(5999), "not expired 1ms before the deadline");
    check(t.expired(6000), "expired exactly at the deadline");
    check(t.expired(6001), "still expired past the deadline");
  }
  {
    DeadmanTimer t;
    t.arm(1000, 5000);
    check(t.remaining_ms(1000) == 5000, "remaining_ms at arm time equals the duration");
    check(t.remaining_ms(4000) == 2000, "remaining_ms counts down");
    check(t.remaining_ms(6000) == 0, "remaining_ms floors at 0 once expired");
    check(t.remaining_ms(9000) == 0, "remaining_ms stays 0 well past expiry");
  }
  {
    DeadmanTimer t;
    t.arm(1000, 5000);
    t.disarm();
    check(!t.armed(), "disarm() clears armed()");
    check(!t.expired(999999), "disarm() makes expired() false at any time");
    check(t.remaining_ms(1000) == 0, "disarm() zeroes remaining_ms");
  }
  {
    // millis() wraps every ~49.7 days (2^32 ms). Arm just before the wrap, deadline lands
    // just after it.
    DeadmanTimer t;
    const uint32_t now = 0xFFFFFFF0u;  // 15 ms before the wrap
    t.arm(now, 100);                   // deadline = 0xFFFFFFF0 + 100, wraps to 84
    check(!t.expired(0xFFFFFFFFu), "not yet expired right before the wrap");
    check(!t.expired(50), "not yet expired just after the wrap (still short of the deadline)");
    check(t.expired(84), "expired exactly at the wrapped deadline");
    check(t.expired(200), "still expired well after the wrapped deadline");
  }
  {
    // remaining_ms across the same wrap.
    DeadmanTimer t;
    const uint32_t now = 0xFFFFFFF0u;
    t.arm(now, 100);
    check(t.remaining_ms(now) == 100, "remaining_ms at arm time, spanning the wrap");
    check(t.remaining_ms(0xFFFFFFFFu) == 85, "remaining_ms counts down correctly right before the wrap");
    check(t.remaining_ms(50) == 34, "remaining_ms continues to count down correctly just after the wrap");
    check(t.remaining_ms(84) == 0, "remaining_ms floors at 0 at the wrapped deadline");
  }
  {
    // Re-arming replaces the previous deadline outright (matches adjtime()'s "replaces, does
    // not accumulate" convention elsewhere in this codebase).
    DeadmanTimer t;
    t.arm(0, 1000);
    t.arm(0, 5000);
    check(t.remaining_ms(0) == 5000, "re-arming replaces the previous deadline");
  }

  // int_level_still_short(): the INTLEVEL re-check guard used by design A''. A pure predicate
  // deliberately kept trivial -- the value that matters is that it is actually called from
  // loop() on a schedule, which this file cannot exercise, only the logic itself.
  check(int_level_still_short(true, 0x0FFF), "int_level_still_short: read ok, still the short value -> true");
  check(!int_level_still_short(true, 0xFFFF), "int_level_still_short: read ok, back to the driver default -> false");
  check(!int_level_still_short(false, 0x0FFF), "int_level_still_short: read failed -> false regardless of value");

  std::printf("%s: %d failure(s)\n", g_failures ? "FAILED" : "OK", g_failures);
  return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
