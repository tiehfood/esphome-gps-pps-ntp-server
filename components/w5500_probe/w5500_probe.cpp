// RESEARCH SPIKE (throwaway) -- see w5500_probe.h.
#include "w5500_probe.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

#include "esp_eth.h"
#include "esp_err.h"
#include <driver/spi_master.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace esphome {
namespace w5500_probe {

static const char *const TAG = "w5500_probe";

namespace {
// Runs one transaction on the ethernet driver's shared spi_device_handle_t, holding
// its shared mutex for the duration -- the same lock the driver's own custom_spi_driver
// read()/write() hold (components/ethernet/ethernet_component.cpp). This is the only
// place in this file allowed to call spi_device_polling_transmit(); every register
// access goes through here.
bool w5500_shared_transact(spi_transaction_t *t) {
  ethernet::W5500SharedSpi shared = ethernet::w5500_shared_spi();
  if (shared.hdl == nullptr || shared.lock == nullptr) {
    ESP_LOGE(TAG, "shared W5500 SPI handle not available -- ethernet not set up yet?");
    return false;
  }
  bool ok = true;
  if (xSemaphoreTake(shared.lock, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (spi_device_polling_transmit(shared.hdl, t) != ESP_OK) {
      ESP_LOGE(TAG, "SPI transmit failed");
      ok = false;
    }
    xSemaphoreGive(shared.lock);
  } else {
    ESP_LOGE(TAG, "failed to acquire shared W5500 SPI lock");
    ok = false;
  }
  return ok;
}
}  // namespace

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
static const uint8_t Sn_CR_CLOSE = 0x10;
static const uint8_t SOCK_UDP = 0x22;

void W5500Probe::setup() {
  if (this->ethernet_ == nullptr) {
    ESP_LOGE(TAG, "no ethernet component set");
    this->mark_failed();
    return;
  }
  // No SPI device is created here -- register access happens through the ethernet
  // driver's own shared spi_device_handle_t (ethernet::w5500_shared_spi()), which only
  // exists once EthernetComponent::setup() has run.
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

// cmd = the 16-bit register address (ESP-IDF's naming is inverted relative to the
// W5500 datasheet -- see esp_eth_mac_w5500.c), addr = the 8-bit control byte. This
// matches devcfg.command_bits=16 / address_bits=8, which the shared handle was created
// with (components/ethernet/ethernet_component.cpp).
void W5500Probe::spi_write_reg_(uint8_t block, uint16_t addr, uint8_t value) {
  uint8_t ctrl = static_cast<uint8_t>((block << 3) | 0x04);  // OM=00 variable length, RWB=1 write
  spi_transaction_t t = {};
  t.cmd = addr;
  t.addr = ctrl;
  t.length = 8;
  t.tx_buffer = &value;
  w5500_shared_transact(&t);
}

void W5500Probe::spi_write_reg16_(uint8_t block, uint16_t addr, uint16_t value) {
  uint8_t ctrl = static_cast<uint8_t>((block << 3) | 0x04);  // OM=00 variable length, RWB=1 write
  uint8_t tx[2] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
  spi_transaction_t t = {};
  t.cmd = addr;
  t.addr = ctrl;
  t.length = sizeof(tx) * 8;
  t.tx_buffer = tx;
  w5500_shared_transact(&t);
}

uint8_t W5500Probe::spi_read_reg_(uint8_t block, uint16_t addr) {
  uint8_t ctrl = static_cast<uint8_t>(block << 3);  // OM=00 variable length, RWB=0 read
  spi_transaction_t t = {};
  t.flags = SPI_TRANS_USE_RXDATA;
  t.cmd = addr;
  t.addr = ctrl;
  t.length = 8;
  if (!w5500_shared_transact(&t))
    return 0;
  return t.rx_data[0];
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
  if (this->ethernet_ == nullptr || ethernet::w5500_shared_spi().hdl == nullptr) {
    ESP_LOGE(TAG, "not set up, aborting probe");
    return;
  }
  ESP_LOGI(TAG, "=== w5500_probe: opening socket 1 (RESEARCH SPIKE) ===");

  // NO esp_eth_stop()/esp_eth_start() here, deliberately. An earlier revision did that
  // and the device never came back -- and because stop() drops the network first, we
  // lost the API connection and every log line at exactly the moment it failed, so the
  // cause was unobservable. The 8KB/2KB buffer split now happens once at boot in
  // EthernetComponent::setup(), between esp_eth_driver_install() and esp_eth_start(),
  // which is the only point where W5500 buffer sizes may legally change. Everything
  // below only opens socket 1 and unmasks its interrupt; the network stays up.
  uint8_t rxbuf1 = this->spi_read_reg_(socket_reg_block(1), REG_Sn_RXBUF_SIZE);
  if (rxbuf1 == 0) {
    ESP_LOGE(TAG, "socket 1 has no RX buffer (Sn_RXBUF_SIZE=0) -- the boot-time split in "
                  "the ethernet component did not run; aborting so we do not read a false negative");
    this->publish_status_("no buffer");
    return;
  }
  ESP_LOGI(TAG, "socket 1 RX buffer = %uKB", rxbuf1);

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

  if (this->ethernet_ == nullptr || ethernet::w5500_shared_spi().hdl == nullptr) {
    ESP_LOGE(TAG, "not set up -- power-cycle the device to recover");
    return;
  }
  // Close socket 1 and re-mask its interrupt. Deliberately does NOT touch the buffer
  // split or restart ethernet: buffer sizes may only change while sockets are closed,
  // which is true exactly once, at boot. Socket 0 keeps its 8KB either way -- ample for
  // MACRAW -- and a reboot restores the driver's defaults regardless, since every W5500
  // register is volatile. Restarting ethernet from here is what previously bricked the
  // network with no telemetry to explain it.
  this->spi_write_reg_(socket_reg_block(1), REG_Sn_CR, Sn_CR_CLOSE);
  this->spi_write_reg_(BLOCK_COMMON, REG_SIMR, 0x01);

  ESP_LOGI(TAG, "recovered: socket 1 closed, SIMR back to socket 0 only; network untouched");
  this->publish_status_("recovered");
}

}  // namespace w5500_probe
}  // namespace esphome

#endif  // USE_ESP32
