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

// Every case below the design A' block passes short_wait_active=false, queued=-1 -- i.e. the
// short-wait rule is not in play, so this reproduces the pre-A' behaviour exactly.
static const bool kNoShortWait = false;
static const int32_t kNoQueued = -1;

int main() {
  // ---- Threshold boundaries, both regimes ----
  check(rx_admission(true, true, 199, false, kNoShortWait, kNoQueued) == RxVerdict::ACCEPT,
        "legacy: 199us lead accepted");
  check(rx_admission(true, true, 999, false, kNoShortWait, kNoQueued) == RxVerdict::ACCEPT,
        "legacy: 999us lead accepted");
  check(rx_admission(true, true, 1000, false, kNoShortWait, kNoQueued) == RxVerdict::OLD_EDGE,
        "legacy: 1000us lead is OLD_EDGE");
  check(rx_admission(true, true, 200, false, kNoShortWait, kNoQueued) == RxVerdict::ACCEPT,
        "legacy: 200us lead accepted (only 1000us matters)");

  check(rx_admission(true, true, 199, true, kNoShortWait, kNoQueued) == RxVerdict::ACCEPT,
        "strict: 199us lead accepted");
  check(rx_admission(true, true, 200, true, kNoShortWait, kNoQueued) == RxVerdict::OLD_EDGE,
        "strict: 200us lead is OLD_EDGE");
  check(rx_admission(true, true, 999, true, kNoShortWait, kNoQueued) == RxVerdict::OLD_EDGE,
        "strict: 999us lead is OLD_EDGE");
  check(rx_admission(true, true, 1000, true, kNoShortWait, kNoQueued) == RxVerdict::OLD_EDGE,
        "strict: 1000us lead is OLD_EDGE");

  // ---- No usable edge ----
  check(rx_admission(true, false, 0, true, kNoShortWait, kNoQueued) == RxVerdict::NO_EDGE,
        "strict + armed + no edge -> NO_EDGE");
  check(rx_admission(true, false, 0, false, kNoShortWait, kNoQueued) == RxVerdict::ACCEPT,
        "legacy never reports NO_EDGE");
  check(rx_admission(false, false, 0, true, kNoShortWait, kNoQueued) == RxVerdict::ACCEPT,
        "capture never armed -> never NO_EDGE, even strict");
  check(rx_admission(false, false, 0, false, kNoShortWait, kNoQueued) == RxVerdict::ACCEPT,
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
    RxVerdict v = rx_admission(true, true, 32, true, kNoShortWait, kNoQueued);
    check(v == RxVerdict::ACCEPT, "a wrap-safe small lead is accepted under the strict rule too");
  }

  // ---- Design A' ("W5500 Short Interrupt Wait" live): edge-aware admission ----
  check(rx_admission(true, false, 0, true, true, 0) == RxVerdict::ACCEPT,
        "short wait: no usable edge, nothing queued behind -> ACCEPT");
  check(rx_admission(true, false, 0, true, true, 59) == RxVerdict::ACCEPT,
        "short wait: no usable edge, 59 bytes queued behind -> ACCEPT");
  check(rx_admission(true, false, 0, true, true, 60) == RxVerdict::NO_EDGE,
        "short wait: no usable edge, 60 bytes queued behind -> NO_EDGE");
  check(rx_admission(true, false, 0, true, true, -1) == RxVerdict::NO_EDGE,
        "short wait: no usable edge, queued unknown (-1) -> NO_EDGE (treated as queued)");
  check(rx_admission(true, true, 199, true, true, 0) == RxVerdict::ACCEPT,
        "short wait: usable edge, 199us lead -> ACCEPT");
  check(rx_admission(true, true, 200, true, true, 0) == RxVerdict::OLD_EDGE,
        "short wait: usable edge, 200us lead -> OLD_EDGE (unchanged threshold)");
  check(rx_admission(false, false, 0, true, true, 0) == RxVerdict::ACCEPT,
        "short wait: capture not armed, no edge -> ACCEPT");

  // Strict off entirely overrides short_wait_active -- legacy behaviour, regardless of queued.
  check(rx_admission(true, false, 0, false, true, 60) == RxVerdict::ACCEPT,
        "short wait active but strict off -> legacy behaviour (no NO_EDGE at all)");
  check(rx_admission(true, true, 1000, false, true, 60) == RxVerdict::OLD_EDGE,
        "short wait active but strict off -> legacy OLD_EDGE threshold (1ms) still applies");

  // ---- Mutation check: if short_wait_active were ignored (today's strict rule applied
  // unconditionally), exactly the two short-wait-only ACCEPT cases above must fail. Restored
  // immediately after -- this does not change the shipped function, only this check's local
  // stand-in for it. ----
  {
    auto rx_admission_ignoring_short_wait = [](bool capture_armed, bool edge_usable, int32_t lead_us,
                                                bool strict) {
      const int32_t threshold = strict ? 200 : 1000;
      if (edge_usable && lead_us >= threshold)
        return RxVerdict::OLD_EDGE;
      if (strict && capture_armed && !edge_usable)
        return RxVerdict::NO_EDGE;
      return RxVerdict::ACCEPT;
    };
    const bool queued0_would_fail = rx_admission_ignoring_short_wait(true, false, 0, true) != RxVerdict::ACCEPT;
    const bool queued59_would_fail = rx_admission_ignoring_short_wait(true, false, 0, true) != RxVerdict::ACCEPT;
    // (queued 0 and queued 59 collapse to the same call once short_wait_active/queued_bytes are
    // dropped -- both are "armed, no edge", which is exactly the case the mutation must flip.)
    check(queued0_would_fail, "mutation: ignoring short_wait_active flips the queued=0 ACCEPT case");
    check(queued59_would_fail, "mutation: ignoring short_wait_active flips the queued=59 ACCEPT case");
    // Cases NOT expected to flip: edge lead 199 (usable edge -> ACCEPT regardless), capture not
    // armed (-> ACCEPT regardless).
    check(rx_admission_ignoring_short_wait(true, true, 199, true) == RxVerdict::ACCEPT,
          "mutation: usable-edge ACCEPT case is unaffected");
    check(rx_admission_ignoring_short_wait(false, false, 0, true) == RxVerdict::ACCEPT,
          "mutation: capture-not-armed ACCEPT case is unaffected");
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
