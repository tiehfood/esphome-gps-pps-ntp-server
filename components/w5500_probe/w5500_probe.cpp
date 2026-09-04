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
static const uint8_t Sn_MR_TCP = 0x01;
static const uint8_t Sn_CR_LISTEN = 0x02;
static const uint8_t SOCK_INIT = 0x13;
static const uint8_t SOCK_LISTEN = 0x14;

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

  // Dead-man timer first, before anything that could fail.
  if (now - this->probe_started_ms_ > this->probe_window_ms_) {
    ESP_LOGW(TAG, "probe window elapsed (%us) -- auto-recovering", this->probe_window_ms_ / 1000);
    this->recover();
    return;
  }

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

void W5500Probe::run_probe_sequence(uint16_t port, bool open_socket, bool set_simr, bool tcp_mode,
                                    uint32_t window_ms) {
  if (this->ethernet_ == nullptr || ethernet::w5500_shared_spi().hdl == nullptr) {
    ESP_LOGE(TAG, "not set up, aborting probe");
    return;
  }
  ESP_LOGI(TAG, "=== w5500_probe: port=%u open_socket=%s (RESEARCH SPIKE) ===", port, YESNO(open_socket));

  // NO esp_eth_stop()/esp_eth_start() here, deliberately. An earlier revision did that
  // and the device never came back -- and because stop() drops the network first, we
  // lost the API connection and every log line at exactly the moment it failed, so the
  // cause was unobservable. The 8KB/2KB buffer split now happens once at boot in
  // EthernetComponent::setup(), between esp_eth_driver_install() and esp_eth_start(),
  // which is the only point where W5500 buffer sizes may legally change. Everything
  // below only opens socket 1 and unmasks its interrupt; the network stays up.
  this->socket_opened_ = open_socket;
  this->probe_window_ms_ = window_ms;
  if (!open_socket) {
    // CONTROL: poll only. Socket 1 is never opened; the loop still does its 1 Hz register
    // reads over the shared SPI bus. If networking dies anyway, the probe's own SPI
    // traffic is the culprit and the socket is exonerated.
    ESP_LOGI(TAG, "CONTROL: polling only, socket 1 NOT opened");
    this->probing_active_ = true;
    this->last_poll_ms_ = 0;
    this->probe_started_ms_ = millis();
    this->publish_status_("poll-only control");
    return;
  }

  uint8_t rxbuf1 = this->spi_read_reg_(socket_reg_block(1), REG_Sn_RXBUF_SIZE);
  if (rxbuf1 == 0) {
    ESP_LOGE(TAG, "socket 1 has no RX buffer (Sn_RXBUF_SIZE=0) -- the boot-time split in "
                  "the ethernet component did not run; aborting so we do not read a false negative");
    this->publish_status_("no buffer");
    return;
  }
  ESP_LOGI(TAG, "socket 1 RX buffer = %uKB", rxbuf1);

  this->spi_write_reg_(socket_reg_block(1), REG_Sn_MR, tcp_mode ? Sn_MR_TCP : Sn_MR_UDP);
  this->spi_write_reg16_(socket_reg_block(1), REG_Sn_PORT, port);
  this->spi_write_reg_(socket_reg_block(1), REG_Sn_CR, Sn_CR_OPEN);

  const uint8_t want = tcp_mode ? SOCK_INIT : SOCK_UDP;
  uint32_t start = millis();
  uint8_t sr = 0;
  while (millis() - start < 100) {
    sr = this->spi_read_reg_(socket_reg_block(1), REG_Sn_SR);
    if (sr == want)
      break;
    delay(1);
  }
  if (tcp_mode && sr == SOCK_INIT) {
    // A TCP server socket only becomes interesting once it is listening.
    this->spi_write_reg_(socket_reg_block(1), REG_Sn_CR, Sn_CR_LISTEN);
    delay(5);
    sr = this->spi_read_reg_(socket_reg_block(1), REG_Sn_SR);
    ESP_LOGI(TAG, "TCP socket 1 after LISTEN: SR=0x%02X (0x14 = SOCK_LISTEN)", sr);
  }
  if (sr != want && !(tcp_mode && sr == SOCK_LISTEN)) {
    ESP_LOGE(TAG, "socket 1 did not reach 0x%02X; read 0x%02X -- press recover", want, sr);
    this->publish_status_("open failed");
    return;
  }

  // SIMR is deliberately LEFT AT 0x01 (socket 0 only).
  //
  // An earlier run unmasked socket 1 (SIMR=0x03) and ALL networking stopped -- not just
  // NTP: TCP/80 died too. W5500 INTn is level-triggered and stays asserted until the
  // raising flag is cleared, and ESP-IDF's emac_w5500 only ever clears socket 0's. So
  // socket 1's Sn_IR.RECV pins INTn low forever and the driver's RX path wedges.
  //
  // Reception does not need the interrupt: SIMR gates only INTn assertion, so socket 1
  // still fills its RX buffer and Sn_RX_RSR still grows. Leaving SIMR alone separates
  // "diverted away from MACRAW" from "interrupt wedge".

  if (set_simr) {
    this->spi_write_reg_(BLOCK_COMMON, REG_SIMR, 0x03);
    ESP_LOGW(TAG, "SIMR=0x03: socket 1 interrupt UNMASKED -- watch whether MACRAW survives");
  }

  this->probing_active_ = true;
  this->last_poll_ms_ = 0;
  this->probe_started_ms_ = millis();
  ESP_LOGI(TAG, "socket 1 open, UDP:%u, SIMR=%s -- watch rx_bytes", port, set_simr ? "0x03" : "0x01");
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
  if (this->socket_opened_)
    this->spi_write_reg_(socket_reg_block(1), REG_Sn_CR, Sn_CR_CLOSE);
  this->spi_write_reg_(BLOCK_COMMON, REG_SIMR, 0x01);

  ESP_LOGI(TAG, "recovered: socket 1 closed, SIMR back to socket 0 only; network untouched");
  this->publish_status_("recovered");
}

void W5500Probe::dump_w5500_config() {
  if (ethernet::w5500_shared_spi().hdl == nullptr) {
    ESP_LOGE(TAG, "not set up");
    return;
  }
  // Common register block (BSB=0): MR 0x0000, GAR 0x0001-04, SUBR 0x0005-08,
  // SHAR 0x0009-0E, SIPR 0x000F-12, SIMR 0x0018.
  uint8_t gar[4], subr[4], shar[6], sipr[4];
  for (int i = 0; i < 4; i++) gar[i] = this->spi_read_reg_(BLOCK_COMMON, 0x0001 + i);
  for (int i = 0; i < 4; i++) subr[i] = this->spi_read_reg_(BLOCK_COMMON, 0x0005 + i);
  for (int i = 0; i < 6; i++) shar[i] = this->spi_read_reg_(BLOCK_COMMON, 0x0009 + i);
  for (int i = 0; i < 4; i++) sipr[i] = this->spi_read_reg_(BLOCK_COMMON, 0x000F + i);
  uint8_t mr = this->spi_read_reg_(BLOCK_COMMON, 0x0000);
  uint8_t simr = this->spi_read_reg_(BLOCK_COMMON, REG_SIMR);

  ESP_LOGI(TAG, "=== W5500 common registers ===");
  ESP_LOGI(TAG, "  MR   = 0x%02X   SIMR = 0x%02X", mr, simr);
  ESP_LOGI(TAG, "  SHAR = %02X:%02X:%02X:%02X:%02X:%02X", shar[0], shar[1], shar[2], shar[3], shar[4],
           shar[5]);
  ESP_LOGI(TAG, "  SIPR = %u.%u.%u.%u   <- 0.0.0.0 means the MACRAW driver never set an IP,"
                " so hardware UDP TX would source from 0.0.0.0", sipr[0], sipr[1], sipr[2], sipr[3]);
  ESP_LOGI(TAG, "  GAR  = %u.%u.%u.%u", gar[0], gar[1], gar[2], gar[3]);
  ESP_LOGI(TAG, "  SUBR = %u.%u.%u.%u", subr[0], subr[1], subr[2], subr[3]);
}

}  // namespace w5500_probe
}  // namespace esphome

#endif  // USE_ESP32
