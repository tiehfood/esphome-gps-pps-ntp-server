// RESEARCH SPIKE (throwaway) -- see w5500_probe.h.
#include "w5500_probe.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

#include "esp_eth.h"
#include "esp_err.h"

namespace esphome {
namespace w5500_probe {

static const char *const TAG = "w5500_probe";

// Block select bits (BSB), W5500 datasheet s4.1 "Address Phase". Common register
// block is BSB=0; socket n's own register block is (n*4)+1 -- TX/RX buffer blocks
// (n*4+2 / n*4+3) are not used here, only the registers that describe/control them.
static const uint8_t BLOCK_COMMON = 0x00;
static inline uint8_t socket_reg_block(uint8_t n) { return (n * 4) + 1; }

// Register offsets (subset of the full map) used by this spike.
static const uint16_t REG_SIMR = 0x0018;         // common: socket interrupt mask
static const uint16_t REG_Sn_MR = 0x0000;        // socket mode
static const uint16_t REG_Sn_CR = 0x0001;        // socket command
static const uint16_t REG_Sn_SR = 0x0003;        // socket status
static const uint16_t REG_Sn_PORT = 0x0004;      // socket source port, 2 bytes
static const uint16_t REG_Sn_RXBUF_SIZE = 0x001E;
static const uint16_t REG_Sn_TXBUF_SIZE = 0x001F;
static const uint16_t REG_Sn_RX_RSR = 0x0026;    // received size waiting, 2 bytes

static const uint8_t Sn_MR_UDP = 0x02;
static const uint8_t Sn_CR_OPEN = 0x01;
static const uint8_t SOCK_UDP = 0x22;

void W5500Probe::setup() {
  if (this->ethernet_ == nullptr) {
    ESP_LOGE(TAG, "no ethernet component set");
    this->mark_failed();
    return;
  }

  spi_device_interface_config_t devcfg = {};
  devcfg.command_bits = 0;
  devcfg.address_bits = 0;
  devcfg.dummy_bits = 0;
  devcfg.mode = 0;
  devcfg.clock_speed_hz = this->ethernet_->get_spi_clock_speed();
  devcfg.spics_io_num = this->ethernet_->get_spi_cs_pin();
  devcfg.queue_size = 1;  // we only ever use spi_device_polling_transmit, one at a time

  // Second device handle on the ethernet driver's own bus + CS. ESP-IDF's SPI master
  // serializes access across device handles sharing a bus, so this is safe as long as
  // we stick to spi_device_polling_transmit() (blocking, bus-locking) rather than the
  // queued/async API.
  esp_err_t err = spi_bus_add_device(this->ethernet_->get_spi_host(), &devcfg, &this->spi_dev_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "w5500_probe ready -- RESEARCH SPIKE, trigger via button, nothing runs at boot");
}

void W5500Probe::loop() {
  if (!this->probing_active_)
    return;
  uint32_t now = millis();
  if (now - this->last_poll_ms_ < 1000)
    return;
  this->last_poll_ms_ = now;
  this->poll_socket1_();
}

void W5500Probe::dump_config() {
  ESP_LOGCONFIG(TAG, "W5500 Probe:");
  ESP_LOGCONFIG(TAG, "  ** RESEARCH SPIKE ** -- throwaway, not production code");
  ESP_LOGCONFIG(TAG, "  Nothing runs automatically at boot; trigger via the probe button");
}

void W5500Probe::spi_write_reg_(uint8_t block, uint16_t addr, uint8_t value) {
  uint8_t ctrl = static_cast<uint8_t>((block << 3) | 0x04);  // OM=00 variable length, RWB=1 write
  uint8_t tx[4] = {static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF), ctrl, value};
  spi_transaction_t t = {};
  t.length = sizeof(tx) * 8;
  t.tx_buffer = tx;
  spi_device_polling_transmit(this->spi_dev_, &t);
}

void W5500Probe::spi_write_reg16_(uint8_t block, uint16_t addr, uint16_t value) {
  uint8_t ctrl = static_cast<uint8_t>((block << 3) | 0x04);  // OM=00 variable length, RWB=1 write
  uint8_t tx[5] = {static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF), ctrl,
                    static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
  spi_transaction_t t = {};
  t.length = sizeof(tx) * 8;
  t.tx_buffer = tx;
  spi_device_polling_transmit(this->spi_dev_, &t);
}

uint8_t W5500Probe::spi_read_reg_(uint8_t block, uint16_t addr) {
  uint8_t ctrl = static_cast<uint8_t>(block << 3);  // OM=00 variable length, RWB=0 read
  uint8_t tx[4] = {static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF), ctrl, 0x00};
  uint8_t rx[4] = {0, 0, 0, 0};
  spi_transaction_t t = {};
  t.length = sizeof(tx) * 8;
  t.tx_buffer = tx;
  t.rx_buffer = rx;
  spi_device_polling_transmit(this->spi_dev_, &t);
  return rx[3];
}

uint16_t W5500Probe::spi_read_reg16_stable_(uint8_t block, uint16_t addr) {
  uint16_t prev =
      static_cast<uint16_t>((static_cast<uint16_t>(this->spi_read_reg_(block, addr)) << 8) |
                             this->spi_read_reg_(block, addr + 1));
  for (int i = 0; i < 5; i++) {
    uint16_t cur =
        static_cast<uint16_t>((static_cast<uint16_t>(this->spi_read_reg_(block, addr)) << 8) |
                               this->spi_read_reg_(block, addr + 1));
    if (cur == prev)
      return cur;
    prev = cur;
  }
  return prev;
}

void W5500Probe::run_probe_sequence() {
  if (this->ethernet_ == nullptr || this->spi_dev_ == nullptr) {
    ESP_LOGE(TAG, "not set up, aborting probe");
    return;
  }
  ESP_LOGI(TAG, "=== w5500_probe: starting probe sequence (RESEARCH SPIKE) ===");
  esp_eth_handle_t eth = this->ethernet_->get_eth_handle();

  esp_err_t err = esp_eth_stop(eth);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_eth_stop failed: %s", esp_err_to_name(err));
    return;
  }

  // Buffer split: socket 0 keeps 14KB (down from the driver's default 16KB), socket 1
  // gets the 2KB it needs for a UDP:123 listener. Values as specified by the spike --
  // do not "round" these to power-of-two KB splits without checking the datasheet.
  this->spi_write_reg_(socket_reg_block(0), REG_Sn_RXBUF_SIZE, 14);
  this->spi_write_reg_(socket_reg_block(0), REG_Sn_TXBUF_SIZE, 14);
  this->spi_write_reg_(socket_reg_block(1), REG_Sn_RXBUF_SIZE, 2);
  this->spi_write_reg_(socket_reg_block(1), REG_Sn_TXBUF_SIZE, 2);

  // esp_eth_driver_install() (not esp_eth_start()) is what calls w5500_setup_default(),
  // so stop()/start() here preserves the split we just wrote -- no driver patch needed.
  err = esp_eth_start(eth);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_eth_start failed: %s", esp_err_to_name(err));
    return;
  }

  this->spi_write_reg_(socket_reg_block(1), REG_Sn_MR, Sn_MR_UDP);
  this->spi_write_reg16_(socket_reg_block(1), REG_Sn_PORT, 123);
  this->spi_write_reg_(socket_reg_block(1), REG_Sn_CR, Sn_CR_OPEN);

  uint32_t start = millis();
  uint8_t sr = 0;
  while (millis() - start < 100) {
    sr = this->spi_read_reg_(socket_reg_block(1), REG_Sn_SR);
    if (sr == SOCK_UDP)
      break;
    delay(1);
  }
  if (sr != SOCK_UDP) {
    ESP_LOGE(TAG, "socket 1 did not reach SOCK_UDP (0x22); read 0x%02X -- press recover", sr);
    this->publish_status_("open failed");
    return;
  }

  // emac_w5500_start() (called from esp_eth_start() above) already wrote SIMR=0x01
  // (socket 0 only). Widen it to both sockets so lwIP keeps getting interrupts for
  // socket 0's MACRAW traffic throughout the test.
  this->spi_write_reg_(BLOCK_COMMON, REG_SIMR, 0x03);

  this->probing_active_ = true;
  this->last_poll_ms_ = 0;
  ESP_LOGI(TAG, "socket 1 open, UDP:123, SIMR=0x03 -- watch the rx_bytes / socket_status "
                "entities now; send NTP requests to this device");
  this->publish_status_("open, watching");
}

void W5500Probe::poll_socket1_() {
  uint16_t rsr = this->spi_read_reg16_stable_(socket_reg_block(1), REG_Sn_RX_RSR);
  uint8_t sr = this->spi_read_reg_(socket_reg_block(1), REG_Sn_SR);
  ESP_LOGD(TAG, "socket 1: RX_RSR=%u SR=0x%02X", rsr, sr);

  if (this->rx_bytes_sensor_ != nullptr)
    this->rx_bytes_sensor_->publish_state(rsr);

  if (this->socket_status_sensor_ != nullptr) {
    char buf[24];
    snprintf(buf, sizeof(buf), "SR=0x%02X RSR=%u", sr, rsr);
    this->socket_status_sensor_->publish_state(buf);
  }

  if (rsr > 0) {
    ESP_LOGW(TAG, ">>> socket 1 has %u bytes waiting -- an NTP request reached the UDP socket <<<", rsr);
  }
}

void W5500Probe::publish_status_(const char *status) {
  if (this->socket_status_sensor_ != nullptr)
    this->socket_status_sensor_->publish_state(status);
}

void W5500Probe::recover() {
  ESP_LOGI(TAG, "=== w5500_probe: recovering (no power cycle) ===");
  this->probing_active_ = false;

  if (this->ethernet_ == nullptr || this->spi_dev_ == nullptr) {
    ESP_LOGE(TAG, "not set up -- power-cycle the device to recover");
    return;
  }
  esp_eth_handle_t eth = this->ethernet_->get_eth_handle();

  esp_err_t err = esp_eth_stop(eth);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_eth_stop failed: %s", esp_err_to_name(err));
  }

  this->spi_write_reg_(socket_reg_block(0), REG_Sn_RXBUF_SIZE, 16);
  this->spi_write_reg_(socket_reg_block(0), REG_Sn_TXBUF_SIZE, 16);
  this->spi_write_reg_(socket_reg_block(1), REG_Sn_RXBUF_SIZE, 0);
  this->spi_write_reg_(socket_reg_block(1), REG_Sn_TXBUF_SIZE, 0);
  // Belt-and-braces: esp_eth_start() below already restores SIMR=0x01 on its own
  // (emac_w5500_start() writes it unconditionally), this just makes the state correct
  // immediately rather than relying on that.
  this->spi_write_reg_(BLOCK_COMMON, REG_SIMR, 0x01);

  err = esp_eth_start(eth);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_eth_start failed: %s -- power-cycle the device", esp_err_to_name(err));
    return;
  }
  ESP_LOGI(TAG, "recovered: socket 0 back to 16KB/16KB, socket 1 cleared");
  this->publish_status_("recovered");
}

}  // namespace w5500_probe
}  // namespace esphome

#endif  // USE_ESP32
