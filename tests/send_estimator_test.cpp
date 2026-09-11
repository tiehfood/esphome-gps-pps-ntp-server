// Host unit tests for ntp_server's T3 send-time estimator.
//
//   clang++ -std=c++17 -Wall -Wextra -Werror -I components/ntp_server \
//       tests/send_estimator_test.cpp -o /tmp/send_estimator_test && /tmp/send_estimator_test
//
// The estimator predicts how long after t0 the W5500 is told to transmit, so T3 can be written
// before the packet leaves. These tests pin the behaviour that went wrong on the live server on
// 2026-09-10: under outbound network load a ~1 s stall produced four consecutive slow sends,
// the estimate re-seeded milliseconds high, and the next replies carried a late T3.

#include "send_estimator.h"

#include <cstdio>
#include <cstdlib>

using esphome::ntp_server::SendEstimator;
using Result = SendEstimator::Result;

static int g_failures = 0;

static void check(bool ok, const char *name) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok)
    g_failures++;
}

static constexpr int64_t MS = 1000;
static constexpr int64_t S = 1000000;

// A warmed-up estimator sitting at ~800 us, as the live server does.
static SendEstimator settled_at_800() {
  SendEstimator e;
  int64_t now = 0;
  e.learn(800, now);
  for (int i = 0; i < 40; i++) {
    now += 250 * MS;
    e.learn(800, now);
  }
  return e;
}

int main() {
  {
    SendEstimator e;
    check(e.estimate() == 0, "starts unseeded");
    check(e.learn(640, 0) == Result::SEEDED && e.estimate() == 640, "cold start seeds from the first real sample");
  }
  {
    SendEstimator e;
    e.learn(1000, 0);
    check(e.learn(1080, 250 * MS) == Result::LEARNED && e.estimate() == 1010, "small innovation moves the estimate by 1/8");
  }
  {
    SendEstimator e;
    check(e.learn(50, 0) == Result::IGNORED && e.learn(5000, 0) == Result::IGNORED && e.estimate() == 0,
          "samples at or beyond SEND_US_MIN / SEND_US_MAX are ignored");
  }
  {
    SendEstimator e = settled_at_800();
    const int32_t before = e.estimate();
    check(e.learn(3000, 20 * S) == Result::REJECTED && e.estimate() == before, "an isolated outlier is rejected");
    e.learn(800, 20 * S + 250 * MS);
    check(e.estimate() == before, "and does not move the estimate afterwards");
  }
  {
    // The measured failure: four consecutive, mutually consistent slow sends within one second.
    SendEstimator e = settled_at_800();
    const int32_t before = e.estimate();
    int64_t now = 20 * S;
    bool reseeded = false;
    for (int32_t v : {2210, 2230, 2190, 2205}) {
      reseeded |= e.learn(v, now) == Result::RESEEDED;
      now += 250 * MS;
    }
    check(!reseeded && e.estimate() == before, "a consistent 1 s stall burst no longer re-seeds");
  }
  {
    // A stall whose sends disagree with each other.
    SendEstimator e = settled_at_800();
    const int32_t before = e.estimate();
    int64_t now = 20 * S;
    bool reseeded = false;
    for (int i = 0; i < 40; i++) {
      reseeded |= e.learn((i % 2) ? 1400 : 2900, now) == Result::RESEEDED;
      now += 250 * MS;
    }
    check(!reseeded && e.estimate() == before, "a long stall with inconsistent sends never re-seeds");
  }
  {
    // A genuine change in the send path: consistently ~1300 us, sustained.
    SendEstimator e = settled_at_800();
    int64_t now = 20 * S;
    int reseeds = 0;
    int64_t reseeded_at = -1;
    for (int i = 0; i < 20; i++) {
      if (e.learn(1290 + (i % 3) * 10, now) == Result::RESEEDED) {
        reseeds++;
        if (reseeded_at < 0)
          reseeded_at = now;
      }
      now += 250 * MS;
    }
    check(reseeds == 1, "a sustained, consistent change re-seeds exactly once");
    check(reseeded_at >= 23 * S, "but only after the run spans at least 3 s");
    check(e.estimate() >= 1290 && e.estimate() <= 1310, "to the centre of the run");
  }
  {
    // Sparse clients: one request a minute still adapts once the run is long enough.
    SendEstimator e = settled_at_800();
    int64_t now = 60 * S;
    bool reseeded = false;
    for (int i = 0; i < 4; i++) {
      reseeded |= e.learn(1500, now) == Result::RESEEDED;
      now += 60 * S;
    }
    check(reseeded && e.estimate() == 1500, "sparse clients re-seed after SEND_RESEED_AFTER rejects");
  }
  {
    // A learned sample in the middle of a run of rejects ends the run.
    SendEstimator e = settled_at_800();
    int64_t now = 20 * S;
    for (int i = 0; i < 3; i++) {
      e.learn(1500, now);
      now += 2 * S;
    }
    e.learn(800, now);
    now += 2 * S;
    check(e.learn(1500, now) == Result::REJECTED, "a normal sample resets the reject run");
  }
  {
    SendEstimator e = settled_at_800();
    e.learn_fallback(1600);
    check(e.estimate() == 900, "fallback learns sendto() duration at 1/8");
    const int32_t before = e.estimate();
    e.learn_fallback(9000);
    check(e.estimate() == before, "fallback ignores out-of-range durations");
  }

  std::printf("%s: %d failure(s)\n", g_failures ? "FAILED" : "OK", g_failures);
  return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
