#pragma once

// RESEARCH SPIKE (throwaway) -- NOT production code.
//
// Answers: with socket 0 in MACRAW mode (esp_eth's W5500 driver, presented to lwIP as
// the netif) and socket 1 opened separately in UDP mode on port 123, does an incoming
// NTP packet land in socket 1, socket 0's MACRAW buffer, or both? See
// docs/superpowers/plans/2026-09-03-ntp-serving-latency.md for the question this feeds,
// and .claude/tracker.md for the register-level facts this was built against.
//
// Register access goes through the ethernet driver's OWN spi_device_handle_t, shared
// via ethernet::w5500_shared_spi() (components/ethernet/), never a second
// spi_bus_add_device() -- an earlier revision of this file did that with the driver's
// own spics_io_num, which re-routes the CS pad and took the device off the network
// (cost one USB recovery). There must be exactly one SPI device for the W5500.
//
// Everything here is triggered by a button/service, never at boot, so a plain reboot
// always yields a clean, working ethernet device -- see the `recover()` action for the
// non-reboot path back.

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/ethernet/ethernet_component.h"

#ifdef USE_ESP32

namespace esphome {
namespace w5500_probe {

class W5500Probe : public Component {
 public:
  void set_ethernet(ethernet::EthernetComponent *ethernet) { this->ethernet_ = ethernet; }
  void set_probe_button(button::Button *b) { this->probe_button_ = b; }
  void set_recover_button(button::Button *b) { this->recover_button_ = b; }
  void set_alt_port_button(button::Button *b) { this->alt_port_button_ = b; }
  void set_poll_only_button(button::Button *b) { this->poll_only_button_ = b; }
  void set_dump_config_button(button::Button *b) { this->dump_config_button_ = b; }
  void set_simr_button(button::Button *b) { this->simr_button_ = b; }
  void set_rx_bytes_sensor(sensor::Sensor *s) { this->rx_bytes_sensor_ = s; }
  void set_socket_status_sensor(text_sensor::TextSensor *s) { this->socket_status_sensor_ = s; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  // No SPI bus/device work happens in setup() anymore (the shared handle is the
  // ethernet driver's own), but keep this after connection so dump_config()/logs are
  // meaningful only once ethernet is actually up.
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  /// Button-triggered: stop -> rewrite buffer split -> start -> open socket 1 UDP:123.
  void run_probe_sequence() { this->run_probe_sequence(123, true); }
  /// port: UDP port to bind socket 1 to. open_socket: false runs the SPI polling
  /// loop WITHOUT opening socket 1 at all -- the control for whether the probe's own
  /// interleaved SPI traffic is what disturbs the driver.
  void run_probe_sequence(uint16_t port, bool open_socket) {
    this->run_probe_sequence(port, open_socket, false);
  }
  /// set_simr: also unmask socket 1's interrupt in SIMR. The W5500's INTn is level
  /// triggered and ESP-IDF's emac_w5500 only ever clears socket 0's flags, so socket 1's
  /// Sn_IR could pin INTn low and wedge the driver. Never actually verified -- the run
  /// that "showed" it was confounded by a spurious web failure.
  void run_probe_sequence(uint16_t port, bool open_socket, bool set_simr);
  /// Read-only dump of the common register block. Answers whether the MACRAW driver ever
  /// programs the W5500's own IP identity, which hardware UDP transmit would need.
  void dump_w5500_config();
  /// Button-triggered: undo run_probe_sequence() without a power cycle.
  void recover();

 protected:
  // W5500 SPI framing (datasheet s4): 16-bit address phase + 8-bit control phase
  // (block-select + R/W + operation mode), followed by data bytes. Sent through the
  // ethernet driver's shared spi_device_handle_t, which is already configured with
  // command_bits=16 / address_bits=8 for exactly this framing (see
  // components/ethernet/ethernet_component.cpp). Operation mode is always "variable
  // length" (OM=00): the frame just ends when CS goes high.
  void spi_write_reg_(uint8_t block, uint16_t addr, uint8_t value);
  void spi_write_reg16_(uint8_t block, uint16_t addr, uint16_t value);
  uint8_t spi_read_reg_(uint8_t block, uint16_t addr);
  // Sn_RX_RSR can tick over between the two bytes of a naive 16-bit read (W5500
  // datasheet errata) -- retry until two consecutive reads agree.
  uint16_t spi_read_reg16_stable_(uint8_t block, uint16_t addr);

  void poll_socket1_();
  void publish_status_(const char *status);

  ethernet::EthernetComponent *ethernet_{nullptr};
  button::Button *probe_button_{nullptr};
  button::Button *recover_button_{nullptr};
  button::Button *alt_port_button_{nullptr};
  button::Button *poll_only_button_{nullptr};
  bool socket_opened_{false};
  button::Button *dump_config_button_{nullptr};
  button::Button *simr_button_{nullptr};
  sensor::Sensor *rx_bytes_sensor_{nullptr};
  text_sensor::TextSensor *socket_status_sensor_{nullptr};

  bool probing_active_{false};
  uint32_t last_poll_ms_{0};
  /// Dead-man timer. If opening socket 1 diverts UDP/123 away from MACRAW, lwIP stops
  /// seeing NTP -- and possibly the API with it, leaving no way to press Recover. The
  /// probe therefore closes socket 1 by itself after this long, no matter what.
  uint32_t probe_started_ms_{0};
  static const uint32_t PROBE_AUTO_RECOVER_MS = 60000;
};

class ProbeButton : public button::Button, public Parented<W5500Probe> {
 protected:
  void press_action() override { this->parent_->run_probe_sequence(123, true); }
};

/// CONTROL 1: bind socket 1 to a port nothing else uses. If MACRAW survives this but dies
/// on 123, the interference is port-specific rather than "any open socket".
/// Read-only: does the MACRAW driver set SIPR/GAR/SUBR at all?
class DumpConfigButton : public button::Button, public Parented<W5500Probe> {
 protected:
  void press_action() override { this->parent_->dump_w5500_config(); }
};

/// Socket 1 on 123 WITH its interrupt unmasked -- the properly-controlled interrupt test.
class ProbeSimrButton : public button::Button, public Parented<W5500Probe> {
 protected:
  void press_action() override { this->parent_->run_probe_sequence(123, true, true); }
};

class ProbeAltPortButton : public button::Button, public Parented<W5500Probe> {
 protected:
  void press_action() override { this->parent_->run_probe_sequence(12345, true); }
};

/// CONTROL 2: run only the 1 Hz register polling, never opening socket 1. If MACRAW dies
/// here too, the probe's own SPI traffic is the cause and the socket is exonerated.
class ProbePollOnlyButton : public button::Button, public Parented<W5500Probe> {
 protected:
  void press_action() override { this->parent_->run_probe_sequence(0, false); }
};

class RecoverButton : public button::Button, public Parented<W5500Probe> {
 protected:
  void press_action() override { this->parent_->recover(); }
};

}  // namespace w5500_probe
}  // namespace esphome

#endif  // USE_ESP32
