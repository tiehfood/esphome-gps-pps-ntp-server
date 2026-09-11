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
/// Design A' (short interrupt wait): a frame with at least this many bytes already queued
/// behind ours in the W5500 RX buffer at arrival means the receive path was busy regardless of
/// whether our own INTn edge shows up -- so a missing edge there is still suspicious. Below it,
/// with the short 109us INTLEVEL active, "no edge yet" is expected: the chip's own re-assertion
/// window bounds how late a real edge can be. Verified: in normal exchanges
/// queued_behind_bytes() is 0 in 6,302 of 6,313 -- see docs/superpowers/plans/2026-09-09-p4-ntp-probe.md.
static constexpr int32_t RX_QUEUED_BEHIND_MIN_BYTES = 60;

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

/// Returns how many bytes, beyond our own frame, Sn_RX_RSR reported already queued in the
/// W5500's RX buffer at arrival -- i.e. rx_rsr minus (frame_len + the 2-byte MACRAW length
/// header). -1 (unknown) when either input was not measured. Pure; see HookInfo::rx_rsr /
/// frame_len for where the inputs come from.
inline int32_t queued_behind_bytes(int32_t rx_rsr, int32_t frame_len) {
  if (rx_rsr < 0 || frame_len < 0)
    return -1;
  return rx_rsr - (frame_len + 2);
}

/// Pure admission decision, evaluated at request time against the CURRENT `strict` switch so
/// flipping it takes effect immediately (the edge measurement itself is frozen per-request, the
/// verdict is not). `capture_armed` says whether an INTn edge has EVER been captured -- before
/// that (early boot), "no edge yet" is expected, not a fault, so it must never cause a refusal.
///
/// Legacy (strict=false): only OLD_EDGE, at the wide 1 ms threshold; a missing/unusable edge is
/// never itself a reason to refuse (matches the behaviour before this admission rule existed).
/// `short_wait_active`/`queued_bytes` are ignored in this regime.
///
/// Strict, short_wait_active=false: OLD_EDGE at the far tighter 200 us threshold, and
/// additionally NO_EDGE once capture is armed and this request had no usable edge at all --
/// unchanged from before design A' existed.
///
/// Strict, short_wait_active=true (design A', "W5500 Short Interrupt Wait" is live): OLD_EDGE
/// unchanged. But with the chip's own INTLEVEL re-assertion window shortened to ~109 us, a
/// missing edge is no longer necessarily a stall -- the real edge can simply postdate the burst
/// it explains (measured: strict refusals with nothing queued behind rose 4 -> 34 per 3,840 when
/// this switch went live, zero of them losses). So NO_EDGE only fires when something else was
/// ALSO queued behind this frame (queued_bytes >= RX_QUEUED_BEHIND_MIN_BYTES, or unknown -- -1
/// is treated conservatively as queued, i.e. still refused); otherwise the 109 us bound is
/// trusted and the request is accepted.
inline RxVerdict rx_admission(bool capture_armed, bool edge_usable, int32_t lead_us, bool strict,
                               bool short_wait_active, int32_t queued_bytes) {
  const int32_t threshold = strict ? RX_EDGE_LEAD_STRICT_US : RX_STALL_MAX_US;
  if (edge_usable && lead_us >= threshold)
    return RxVerdict::OLD_EDGE;
  if (!strict || !capture_armed || edge_usable)
    return RxVerdict::ACCEPT;
  // strict, capture armed, no usable edge.
  if (!short_wait_active)
    return RxVerdict::NO_EDGE;
  if (queued_bytes < 0 || queued_bytes >= RX_QUEUED_BEHIND_MIN_BYTES)
    return RxVerdict::NO_EDGE;
  return RxVerdict::ACCEPT;
}

}  // namespace ntp_server
}  // namespace esphome
