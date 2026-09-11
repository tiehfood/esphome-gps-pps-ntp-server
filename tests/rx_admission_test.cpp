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
using esphome::ntp_server::rx_admission;
using esphome::ntp_server::RxEdgeEval;
using esphome::ntp_server::RxVerdict;

static int g_failures = 0;

static void check(bool ok, const char *name) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok)
    g_failures++;
}

int main() {
  // ---- Threshold boundaries, both regimes ----
  check(rx_admission(true, true, 199, false) == RxVerdict::ACCEPT, "legacy: 199us lead accepted");
  check(rx_admission(true, true, 999, false) == RxVerdict::ACCEPT, "legacy: 999us lead accepted");
  check(rx_admission(true, true, 1000, false) == RxVerdict::OLD_EDGE, "legacy: 1000us lead is OLD_EDGE");
  check(rx_admission(true, true, 200, false) == RxVerdict::ACCEPT, "legacy: 200us lead accepted (only 1000us matters)");

  check(rx_admission(true, true, 199, true) == RxVerdict::ACCEPT, "strict: 199us lead accepted");
  check(rx_admission(true, true, 200, true) == RxVerdict::OLD_EDGE, "strict: 200us lead is OLD_EDGE");
  check(rx_admission(true, true, 999, true) == RxVerdict::OLD_EDGE, "strict: 999us lead is OLD_EDGE");
  check(rx_admission(true, true, 1000, true) == RxVerdict::OLD_EDGE, "strict: 1000us lead is OLD_EDGE");

  // ---- No usable edge ----
  check(rx_admission(true, false, 0, true) == RxVerdict::NO_EDGE, "strict + armed + no edge -> NO_EDGE");
  check(rx_admission(true, false, 0, false) == RxVerdict::ACCEPT, "legacy never reports NO_EDGE");
  check(rx_admission(false, false, 0, true) == RxVerdict::ACCEPT, "capture never armed -> never NO_EDGE, even strict");
  check(rx_admission(false, false, 0, false) == RxVerdict::ACCEPT, "capture never armed, legacy -> ACCEPT");

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
    RxVerdict v = rx_admission(true, true, 32, true);
    check(v == RxVerdict::ACCEPT, "a wrap-safe small lead is accepted under the strict rule too");
  }

  std::printf("%s: %d failure(s)\n", g_failures ? "FAILED" : "OK", g_failures);
  return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
