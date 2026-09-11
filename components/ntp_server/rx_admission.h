#pragma once

#include <cstdint>

namespace esphome {
namespace ntp_server {

/// An INTn edge older than this relative to the burst it is meant to explain is not trusted as
/// the arrival reference at all (carried over from the original int_lead check).
static constexpr int32_t RX_STALL_EDGE_MAX_AGE_US = 2000000;
/// Legacy threshold: an INTn-to-burst lead this large means the receive path stalled.
static constexpr int32_t RX_STALL_MAX_US = 1000;
/// Strict threshold: normal lead is 3-25 us, so 200 us is still ~10x that with margin, but far
/// tighter than the legacy 1 ms -- see docs/superpowers/plans/2026-09-09-p4-ntp-probe.md.
static constexpr int32_t RX_EDGE_LEAD_STRICT_US = 200;

struct RxEdgeEval {
  bool edge_usable;
  int32_t lead_us;
};

/// Pure, 32-bit-micros-wrap-safe evaluation of one INTn edge against the burst it is meant to
/// explain. edge_seq/burst_start_us are 0 before anything has ever happened -- e.g. at boot,
/// before the first frame -- and must not be reported as a bad edge.
inline RxEdgeEval evaluate_rx_edge(uint32_t edge_seq, uint32_t edge_us, uint32_t burst_start_us) {
  if (edge_seq == 0 || burst_start_us == 0)
    return {false, 0};
  const int32_t lead = static_cast<int32_t>(burst_start_us - edge_us);
  const bool usable = lead > 0 && lead < RX_STALL_EDGE_MAX_AGE_US;
  return {usable, lead};
}

enum class RxVerdict : uint8_t { ACCEPT, OLD_EDGE, NO_EDGE };

/// Pure admission decision, evaluated at request time against the CURRENT `strict` switch so
/// flipping it takes effect immediately (the edge measurement itself is frozen per-request, the
/// verdict is not). `capture_armed` says whether an INTn edge has EVER been captured -- before
/// that (early boot), "no edge yet" is expected, not a fault, so it must never cause a refusal.
///
/// Legacy (strict=false): only OLD_EDGE, at the wide 1 ms threshold; a missing/unusable edge is
/// never itself a reason to refuse (matches the behaviour before this admission rule existed).
/// Strict: OLD_EDGE at the far tighter 200 us threshold, and additionally NO_EDGE once capture
/// is armed and this request had no usable edge at all.
inline RxVerdict rx_admission(bool capture_armed, bool edge_usable, int32_t lead_us, bool strict) {
  const int32_t threshold = strict ? RX_EDGE_LEAD_STRICT_US : RX_STALL_MAX_US;
  if (edge_usable && lead_us >= threshold)
    return RxVerdict::OLD_EDGE;
  if (strict && capture_armed && !edge_usable)
    return RxVerdict::NO_EDGE;
  return RxVerdict::ACCEPT;
}

}  // namespace ntp_server
}  // namespace esphome
