#pragma once

#include <cstdint>

namespace esphome {
namespace ntp_server {

/// Reverts an unattended register write after a bounded time, regardless of whether the owner
/// ever calls disarm() again. See .claude/CLAUDE.md: "Any experiment that writes W5500
/// registers needs a dead-man timer that reverts it unattended" -- without one, a wedged or
/// forgotten register write on this hardware has cost USB recoveries before.
///
/// Dependency-free on purpose, so the millis()-wrap arithmetic is host-testable
/// (tests/deadman_test.cpp) independent of the register write it guards.
class DeadmanTimer {
 public:
  void arm(uint32_t now_ms, uint32_t duration_ms) {
    this->armed_ = true;
    this->deadline_ms_ = now_ms + duration_ms;
  }

  void disarm() { this->armed_ = false; }

  bool armed() const { return this->armed_; }

  /// True once now_ms has reached or passed the deadline. Wrap-safe: casting the difference to
  /// int32_t turns "now is at or after the deadline" into a sign test, the same trick millis()
  /// wrap handling uses elsewhere in this codebase (e.g. the ARP refresh timer).
  bool expired(uint32_t now_ms) const {
    if (!this->armed_)
      return false;
    return static_cast<int32_t>(now_ms - this->deadline_ms_) >= 0;
  }

  /// 0 once disarmed or expired.
  uint32_t remaining_ms(uint32_t now_ms) const {
    if (!this->armed_)
      return 0;
    const int32_t rem = static_cast<int32_t>(this->deadline_ms_ - now_ms);
    return rem > 0 ? static_cast<uint32_t>(rem) : 0;
  }

 private:
  bool armed_{false};
  uint32_t deadline_ms_{0};
};

/// Guard for design A ("W5500 Short Interrupt Wait"): the ESP-IDF driver can re-initialise the
/// W5500 (e.g. after a recovered link event) and silently restore INTLEVEL to its own 0xFFFF
/// default, while short_wait_effective_ -- set once at the original write -- would otherwise
/// stay true forever. Pure so it is host-testable independent of the register read it guards.
inline bool int_level_still_short(bool read_ok, uint16_t readback) {
  return read_ok && readback == 0x0FFF;
}

}  // namespace ntp_server
}  // namespace esphome
