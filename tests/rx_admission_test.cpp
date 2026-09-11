// Host unit tests for ntp_server's receive-admission rule.
//
//   clang++ -std=c++17 -Wall -Wextra -Werror -I components/ntp_server \
//       tests/rx_admission_test.cpp -o /tmp/rx_admission_test && /tmp/rx_admission_test
//
// evaluate_rx_edge() turns one INTn edge + burst-start pair into usable/lead; rx_admission()
// turns that (plus the live strict switch) into ACCEPT/OLD_EDGE/NO_EDGE. Split so the verdict
// can be re-evaluated against the CURRENT switch state without re-measuring the edge.

#include "rx_admission.h"

#include <cstdio>
#include <cstdlib>

using esphome::ntp_server::evaluate_rx_edge;
using esphome::ntp_server::queued_behind_bytes;
using esphome::ntp_server::rx_admission;
using esphome::ntp_server::RxEdgeEval;
using esphome::ntp_server::RxVerdict;

static int g_failures = 0;

static void check(bool ok, const char *name) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok)
    g_failures++;
}

// Every case below the design A block passes short_wait_active=false, queued=-1, rx_gap=-1 --
// i.e. the short-wait rule is not in play, so this reproduces the pre-design-A behaviour exactly.
static const bool kNoShortWait = false;
static const int32_t kNoQueued = -1;
static const int32_t kNoGap = -1;
// A gap comfortably above RX_FRESH_BURST_MIN_GAP_US (800), for short-wait ACCEPT cases that must
// keep their meaning under design A'' (fresh-burst admission).
static const int32_t kFreshGap = 1000;

int main() {
  // ---- Threshold boundaries, both regimes ----
  check(rx_admission(true, true, 199, false, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::ACCEPT,
        "legacy: 199us lead accepted");
  check(rx_admission(true, true, 999, false, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::ACCEPT,
        "legacy: 999us lead accepted");
  check(rx_admission(true, true, 1000, false, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::OLD_EDGE,
        "legacy: 1000us lead is OLD_EDGE");
  check(rx_admission(true, true, 200, false, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::ACCEPT,
        "legacy: 200us lead accepted (only 1000us matters)");

  check(rx_admission(true, true, 199, true, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::ACCEPT,
        "strict: 199us lead accepted");
  check(rx_admission(true, true, 200, true, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::OLD_EDGE,
        "strict: 200us lead is OLD_EDGE");
  check(rx_admission(true, true, 999, true, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::OLD_EDGE,
        "strict: 999us lead is OLD_EDGE");
  check(rx_admission(true, true, 1000, true, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::OLD_EDGE,
        "strict: 1000us lead is OLD_EDGE");

  // ---- No usable edge ----
  check(rx_admission(true, false, 0, true, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::NO_EDGE,
        "strict + armed + no edge -> NO_EDGE");
  check(rx_admission(true, false, 0, false, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::ACCEPT,
        "legacy never reports NO_EDGE");
  check(rx_admission(false, false, 0, true, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::ACCEPT,
        "capture never armed -> never NO_EDGE, even strict");
  check(rx_admission(false, false, 0, false, kNoShortWait, kNoQueued, kNoGap) == RxVerdict::ACCEPT,
        "capture never armed, legacy -> ACCEPT");

  // ---- evaluate_rx_edge(): boot state and wrap-safety ----
  {
    RxEdgeEval e = evaluate_rx_edge(0, 12345, 12400);
    check(!e.edge_usable, "edge_seq == 0 (never captured) is never usable");
  }
  {
    RxEdgeEval e = evaluate_rx_edge(1, 12345, 0);
    check(!e.edge_usable, "burst_start_us == 0 (no burst yet) is never usable");
  }
  {
    // Normal case: burst started 10us after the edge.
    RxEdgeEval e = evaluate_rx_edge(1, 1000, 1010);
    check(e.edge_usable && e.lead_us == 10, "normal small lead is usable");
  }
  {
    // Edge AFTER burst start (negative lead) -- not usable.
    RxEdgeEval e = evaluate_rx_edge(1, 2000, 1000);
    check(!e.edge_usable && e.lead_us <= 0, "an edge after the burst start is not usable");
  }
  {
    // Edge older than RX_STALL_EDGE_MAX_AGE_US -- not trusted.
    RxEdgeEval e = evaluate_rx_edge(1, 0, 3000000);
    check(!e.edge_usable, "a lead at/beyond the max trusted age is not usable");
  }
  {
    // 32-bit micros() wrap: edge just before wrap, burst just after -- lead must still compute
    // as a small positive number via unsigned wraparound subtraction, not a huge one.
    const uint32_t edge = 0xFFFFFFF0u;  // 16 before wrap
    const uint32_t burst = 0x00000010u;  // 16 after wrap
    RxEdgeEval e = evaluate_rx_edge(1, edge, burst);
    check(e.edge_usable && e.lead_us == 32, "lead computes correctly across a 32-bit micros() wrap");
  }
  {
    RxVerdict v = rx_admission(true, true, 32, true, kNoShortWait, kNoQueued, kNoGap);
    check(v == RxVerdict::ACCEPT, "a wrap-safe small lead is accepted under the strict rule too");
  }

  // ---- Design A ("W5500 Short Interrupt Wait" live): edge-aware admission ----
  // These are the "no usable edge" ACCEPT cases -- design A'' additionally requires a fresh-burst
  // rx_gap, so they now pass kFreshGap to keep their original meaning (see the fresh-burst table
  // below for the rx_gap-driven cases).
  check(rx_admission(true, false, 0, true, true, 0, kFreshGap) == RxVerdict::ACCEPT,
        "short wait: no usable edge, nothing queued behind, fresh burst -> ACCEPT");
  check(rx_admission(true, false, 0, true, true, 59, kFreshGap) == RxVerdict::ACCEPT,
        "short wait: no usable edge, 59 bytes queued behind, fresh burst -> ACCEPT");
  check(rx_admission(true, false, 0, true, true, 60, kFreshGap) == RxVerdict::NO_EDGE,
        "short wait: no usable edge, 60 bytes queued behind -> NO_EDGE");
  check(rx_admission(true, false, 0, true, true, -1, kFreshGap) == RxVerdict::NO_EDGE,
        "short wait: no usable edge, queued unknown (-1) -> NO_EDGE (treated as queued)");
  check(rx_admission(true, true, 199, true, true, 0, kNoGap) == RxVerdict::ACCEPT,
        "short wait: usable edge, 199us lead -> ACCEPT (rx_gap irrelevant)");
  check(rx_admission(true, true, 200, true, true, 0, kNoGap) == RxVerdict::OLD_EDGE,
        "short wait: usable edge, 200us lead -> OLD_EDGE (unchanged threshold)");
  check(rx_admission(false, false, 0, true, true, 0, kNoGap) == RxVerdict::ACCEPT,
        "short wait: capture not armed, no edge -> ACCEPT (rx_gap irrelevant)");

  // Strict off entirely overrides short_wait_active -- legacy behaviour, regardless of
  // queued/rx_gap.
  check(rx_admission(true, false, 0, false, true, 60, kNoGap) == RxVerdict::ACCEPT,
        "short wait active but strict off -> legacy behaviour (no NO_EDGE at all)");
  check(rx_admission(true, true, 1000, false, true, 60, kNoGap) == RxVerdict::OLD_EDGE,
        "short wait active but strict off -> legacy OLD_EDGE threshold (1ms) still applies");

  // ---- Design A'' (fresh-burst admission): rx_gap_us discriminates on-time from late ----
  // strict, short-wait active, capture armed, no usable edge, nothing queued behind (queued=0):
  check(rx_admission(true, false, 0, true, true, 0, 799) == RxVerdict::NO_EDGE,
        "fresh-burst: 799us gap (just under the floor) -> NO_EDGE");
  check(rx_admission(true, false, 0, true, true, 0, 800) == RxVerdict::ACCEPT,
        "fresh-burst: 800us gap (exactly the floor) -> ACCEPT");
  check(rx_admission(true, false, 0, true, true, 0, 1100) == RxVerdict::ACCEPT,
        "fresh-burst: 1100us gap -> ACCEPT");
  check(rx_admission(true, false, 0, true, true, 0, -1) == RxVerdict::NO_EDGE,
        "fresh-burst: rx_gap unknown (-1) -> NO_EDGE (treated conservatively)");
  check(rx_admission(true, false, 0, true, true, 60, 1100) == RxVerdict::NO_EDGE,
        "fresh-burst: queued=60 (already refused on queued) + fresh gap -> still NO_EDGE");
  check(rx_admission(true, false, 0, true, true, -1, 1100) == RxVerdict::NO_EDGE,
        "fresh-burst: queued unknown (-1) + fresh gap -> still NO_EDGE");

  // Short wait inactive: rx_gap plays no role at all -- still today's strict behaviour.
  check(rx_admission(true, false, 0, true, false, 0, 500) == RxVerdict::NO_EDGE,
        "short wait inactive: no usable edge -> NO_EDGE regardless of rx_gap");

  // ---- Mutation check: if rx_gap_us were ignored (design A's queued-only rule applied
  // instead), exactly the rx_gap-driven NO_EDGE cases above must fail. Restored immediately
  // after -- this does not change the shipped function, only this check's local stand-in for
  // it. ----
  {
    auto rx_admission_ignoring_rx_gap = [](bool capture_armed, bool edge_usable, int32_t lead_us,
                                            bool strict, bool short_wait_active, int32_t queued_bytes) {
      const int32_t threshold = strict ? 200 : 1000;
      if (edge_usable && lead_us >= threshold)
        return RxVerdict::OLD_EDGE;
      if (!strict || !capture_armed || edge_usable)
        return RxVerdict::ACCEPT;
      if (!short_wait_active)
        return RxVerdict::NO_EDGE;
      if (queued_bytes < 0 || queued_bytes >= 60)
        return RxVerdict::NO_EDGE;
      return RxVerdict::ACCEPT;
    };
    // The rx_gap-based refusals: queued=0 but the gap is too short (799) or unknown (-1). Ignoring
    // rx_gap collapses both to "queued=0 -> ACCEPT", flipping them away from NO_EDGE.
    const bool gap799_would_flip =
        rx_admission_ignoring_rx_gap(true, false, 0, true, true, 0) != RxVerdict::NO_EDGE;
    const bool gapUnknown_would_flip =
        rx_admission_ignoring_rx_gap(true, false, 0, true, true, 0) != RxVerdict::NO_EDGE;
    check(gap799_would_flip, "mutation: ignoring rx_gap_us flips the 799us-gap NO_EDGE case");
    check(gapUnknown_would_flip, "mutation: ignoring rx_gap_us flips the unknown-gap NO_EDGE case");
    // Cases NOT expected to flip by this mutation: fresh-gap ACCEPT (queued=0, gap>=800 -- the
    // queued-only rule already accepts it), and the queued-based NO_EDGE cases (queued>=60 or
    // unknown -- refused on queued alone, independent of rx_gap).
    check(rx_admission_ignoring_rx_gap(true, false, 0, true, true, 0) == RxVerdict::ACCEPT,
          "mutation: queued=0 ACCEPT case (ignoring rx_gap) is unaffected");
    check(rx_admission_ignoring_rx_gap(true, false, 0, true, true, 60) == RxVerdict::NO_EDGE,
          "mutation: queued=60 NO_EDGE case is unaffected by ignoring rx_gap");
    check(rx_admission_ignoring_rx_gap(true, false, 0, true, true, -1) == RxVerdict::NO_EDGE,
          "mutation: queued unknown NO_EDGE case is unaffected by ignoring rx_gap");
  }

  // ---- queued_behind_bytes() ----
  check(queued_behind_bytes(92, 90) == 0, "queued_behind_bytes: normal exchange (92/90) -> 0");
  check(queued_behind_bytes(154, 90) == 62, "queued_behind_bytes: 154/90 -> 62");
  check(queued_behind_bytes(-1, 90) == -1, "queued_behind_bytes: rx_rsr unknown -> -1");
  check(queued_behind_bytes(92, -1) == -1, "queued_behind_bytes: frame_len unknown -> -1");
  check(queued_behind_bytes(-1, -1) == -1, "queued_behind_bytes: both unknown -> -1");

  std::printf("%s: %d failure(s)\n", g_failures ? "FAILED" : "OK", g_failures);
  return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
