#include "w5500_custom_spi.h"

#if defined(USE_ESP32) && defined(USE_ETHERNET_W5500)

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#if defined(CONFIG_SOC_MCPWM_SUPPORTED)
#include <driver/mcpwm_cap.h>
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <atomic>
#include <cstring>
#include <new>

namespace esphome::ethernet {

namespace {

// Per-device context returned by init() and handed back to read/write/deinit.
struct W5500CustomSpiContext {
  spi_device_handle_t handle;
  SemaphoreHandle_t lock;
};

// Transfers up to the ESP32 SPI hardware FIFO size (64 bytes) stay on the polling path; larger
// transfers (the frame payloads) use the blocking, DMA-backed transmit.
constexpr uint32_t W5500_SPI_BULK_THRESHOLD = 64;
constexpr uint32_t W5500_SPI_LOCK_TIMEOUT_MS = 50;

// LOCAL DELTA: NTP timestamping taps. See w5500_custom_spi.h for why these exist.
//
// W5500 addressing: the control byte carries the block-select bits, so socket 0's register
// block (BSB=1) reads as 0x08 / writes as 0x0C, and its RX buffer block (BSB=3) reads as
// 0x18. Sn_RX_RSR is register 0x0026, Sn_CR is 0x0001, and SEND is command 0x20.
constexpr uint8_t W5500_CTRL_S0_REG_READ = 0x08;
constexpr uint8_t W5500_CTRL_S0_REG_WRITE = 0x0C;
constexpr uint8_t W5500_CTRL_S0_RXBUF_READ = 0x18;
constexpr uint16_t W5500_REG_SN_RX_RSR = 0x0026;
constexpr uint16_t W5500_REG_SN_CR = 0x0001;
constexpr uint8_t W5500_CMD_SEND = 0x20;
/// Socket 0 TX buffer (BSB=2) write: every transmitted frame is one write with this control byte.
constexpr uint8_t W5500_CTRL_S0_TXBUF_WRITE = 0x14;
/// SPI silence longer than this means the previous burst ended; intra-burst spacing is a
/// few microseconds, and realistic NTP gaps are milliseconds.
constexpr uint32_t W5500_BURST_GAP_US = 500;

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
volatile uint32_t g_rx_size_read_us = 0;
volatile uint32_t g_rx_payload_us = 0;
volatile uint32_t g_rx_payloads_since_size_read = 0;
volatile uint32_t g_burst_start_us = 0;
volatile uint32_t g_last_txn_us = 0;
volatile uint32_t g_send_cmd_us = 0;
volatile uint32_t g_send_cmd_seq = 0;
volatile uint32_t g_int_edge_us = 0;
volatile uint32_t g_int_edge_seq = 0;
// Sn_CR handshake tracking. The in-flight fields are written by whichever task issues a
// command (the driver's RX task for RECV, the transmitting task for SEND); a race between them
// is itself what `overlaps` exists to show, so these are diagnostic-grade, not exact.
volatile uint32_t g_cr_write_us = 0;
volatile uint32_t g_cr_polls = 0;
volatile bool g_cr_pending = false;
std::atomic<uint32_t> g_cr_commands{0};
std::atomic<uint32_t> g_cr_retried{0};
std::atomic<uint32_t> g_cr_max_us{0};
std::atomic<uint32_t> g_cr_overlaps{0};
// Network activity recorder (see w5500_custom_spi.h).
volatile bool g_net_enabled = false;
W5500NetBin *g_net_bins = nullptr;
uint16_t g_net_cur = 0;
uint32_t g_net_cur_start_us = 0;
portMUX_TYPE g_net_mux = portMUX_INITIALIZER_UNLOCKED;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

/// Marks the start of a receive burst. READS ONLY -- deliberately.
///
/// Transmit writes must not touch this. If they do, the writes of one reply land within
/// W5500_BURST_GAP_US of the next request's first read, that read stops looking like a new
/// burst, and g_burst_start_us stays pointing at the previous exchange's TX. T2 is then
/// back-dated to a stale timestamp: offset goes negative, rx_stamp_gap inflates, and the
/// packet-size sweep slope rises because the error tracks how long the exchange took.
inline void note_read_transaction() {
  const uint32_t now_us = micros();
  if (now_us - g_last_txn_us > W5500_BURST_GAP_US) {
    g_burst_start_us = now_us;
  }
  g_last_txn_us = now_us;
}

// ---- LOCAL DELTA: network activity recorder ----

/// Coarse class of an Ethernet frame by EtherType and, for IPv4, protocol and either port.
uint8_t classify_frame(const uint8_t *f, uint32_t len) {
  if (f == nullptr || len < 14)
    return W5500_FC_OTHER;
  const uint16_t ethertype = static_cast<uint16_t>((f[12] << 8) | f[13]);
  if (ethertype == 0x0806)
    return W5500_FC_ARP;
  if (ethertype != 0x0800 || len < 14 + 20)
    return W5500_FC_OTHER;
  const uint32_t ihl = static_cast<uint32_t>(f[14] & 0x0F) * 4;
  if (ihl < 20 || len < 14 + ihl + 4)
    return W5500_FC_OTHER;
  const uint8_t proto = f[14 + 9];
  const uint16_t src = static_cast<uint16_t>((f[14 + ihl] << 8) | f[14 + ihl + 1]);
  const uint16_t dst = static_cast<uint16_t>((f[14 + ihl + 2] << 8) | f[14 + ihl + 3]);
  const auto uses = [src, dst](uint16_t port) { return src == port || dst == port; };
  if (proto == 6)
    return uses(6053) ? W5500_FC_API : uses(80) ? W5500_FC_HTTP : W5500_FC_TCP;
  if (proto == 17)
    return uses(123) ? W5500_FC_NTP : uses(5353) ? W5500_FC_MDNS : W5500_FC_UDP;
  return W5500_FC_OTHER;
}

/// The bin covering now_us, rolling the ring over any bins that elapsed. Call with g_net_mux held.
W5500NetBin *net_bin_now(uint32_t now_us) {
  if (g_net_cur_start_us == 0 || now_us - g_net_cur_start_us >= W5500_NET_BIN_US * W5500_NET_BINS) {
    memset(g_net_bins, 0, sizeof(W5500NetBin) * W5500_NET_BINS);
    g_net_cur = 0;
    g_net_cur_start_us = now_us;
    g_net_bins[0].start_us = now_us;
    return &g_net_bins[0];
  }
  while (now_us - g_net_cur_start_us >= W5500_NET_BIN_US) {
    g_net_cur = static_cast<uint16_t>((g_net_cur + 1) % W5500_NET_BINS);
    g_net_cur_start_us += W5500_NET_BIN_US;
    memset(&g_net_bins[g_net_cur], 0, sizeof(W5500NetBin));
    g_net_bins[g_net_cur].start_us = g_net_cur_start_us;
  }
  return &g_net_bins[g_net_cur];
}

void net_record_frame(bool tx, const uint8_t *f, uint32_t len) {
  if (!g_net_enabled || g_net_bins == nullptr)
    return;
  const uint8_t cls = classify_frame(f, len);
  const uint32_t now_us = micros();
  portENTER_CRITICAL(&g_net_mux);
  W5500NetBin *bin = net_bin_now(now_us);
  uint16_t &count = tx ? bin->tx[cls] : bin->rx[cls];
  if (count < UINT16_MAX)
    count++;
  (tx ? bin->tx_bytes : bin->rx_bytes) += len;
  portEXIT_CRITICAL(&g_net_mux);
}

void net_record_lock_wait(uint32_t wait_us) {
  if (!g_net_enabled || g_net_bins == nullptr)
    return;
  const uint32_t now_us = micros();
  portENTER_CRITICAL(&g_net_mux);
  W5500NetBin *bin = net_bin_now(now_us);
  if (wait_us > bin->lock_wait_max_us)
    bin->lock_wait_max_us = wait_us;
  portEXIT_CRITICAL(&g_net_mux);
}

void net_record_cr_retry() {
  if (!g_net_enabled || g_net_bins == nullptr)
    return;
  const uint32_t now_us = micros();
  portENTER_CRITICAL(&g_net_mux);
  W5500NetBin *bin = net_bin_now(now_us);
  if (bin->cr_retried < UINT16_MAX)
    bin->cr_retried++;
  portEXIT_CRITICAL(&g_net_mux);
}

void *w5500_custom_spi_init(const void *spi_config) {
  const auto *config = static_cast<const eth_w5500_config_t *>(spi_config);
  auto *ctx = new (std::nothrow) W5500CustomSpiContext{};
  if (ctx == nullptr) {
    return nullptr;
  }
  // The W5500 SPI frame carries the 16-bit address in the command phase and the 8-bit control
  // byte in the address phase; mirror what the stock driver configures.
  spi_device_interface_config_t devcfg = *config->spi_devcfg;
  devcfg.command_bits = 16;
  devcfg.address_bits = 8;
  if (spi_bus_add_device(config->spi_host_id, &devcfg, &ctx->handle) != ESP_OK) {
    delete ctx;
    return nullptr;
  }
  ctx->lock = xSemaphoreCreateMutex();
  if (ctx->lock == nullptr) {
    spi_bus_remove_device(ctx->handle);
    delete ctx;
    return nullptr;
  }
  return ctx;
}

esp_err_t w5500_custom_spi_deinit(void *spi_ctx) {
  auto *ctx = static_cast<W5500CustomSpiContext *>(spi_ctx);
  spi_bus_remove_device(ctx->handle);
  vSemaphoreDelete(ctx->lock);
  delete ctx;
  return ESP_OK;
}

// Runs one transaction under the device lock, choosing the polling vs blocking transmit by size.
// Bulk payloads (> FIFO size) block so the calling task sleeps while DMA runs; small register
// accesses stay on the cheaper polling path. Used by both read and write.
esp_err_t w5500_custom_spi_transfer(W5500CustomSpiContext *ctx, spi_transaction_t *trans, uint32_t len) {
  const uint32_t lock_start_us = g_net_enabled ? micros() : 0;
  if (xSemaphoreTake(ctx->lock, pdMS_TO_TICKS(W5500_SPI_LOCK_TIMEOUT_MS)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  if (g_net_enabled) {
    net_record_lock_wait(micros() - lock_start_us);
  }
  esp_err_t ret;
  if (len > W5500_SPI_BULK_THRESHOLD) {
    ret = spi_device_transmit(ctx->handle, trans);
  } else {
    ret = spi_device_polling_transmit(ctx->handle, trans);
  }
  xSemaphoreGive(ctx->lock);
  return ret;
}

esp_err_t w5500_custom_spi_write(void *spi_ctx, uint32_t cmd, uint32_t addr, const void *data, uint32_t len) {
  auto *ctx = static_cast<W5500CustomSpiContext *>(spi_ctx);
  if (addr == W5500_CTRL_S0_REG_WRITE && cmd == W5500_REG_SN_CR && len == 1 && data != nullptr) {
    const uint32_t now_us = micros();
    // LOCAL DELTA: NTP T3. Stamp the instant the chip is told to transmit -- strictly earlier
    // than sendto() returns, which is what the estimate used to learn from and why T3 ran late.
    if (*static_cast<const uint8_t *>(data) == W5500_CMD_SEND) {
      g_send_cmd_us = now_us;
      g_send_cmd_seq++;
    }
    // LOCAL DELTA: command handshake stats (see w5500_take_cmd_stats()).
    if (g_cr_pending) {
      g_cr_overlaps.fetch_add(1, std::memory_order_relaxed);
    }
    g_cr_write_us = now_us;
    g_cr_polls = 0;
    g_cr_pending = true;
  }
  // LOCAL DELTA: network recorder. The stock driver writes each transmitted frame into the socket 0
  // TX buffer in a single write.
  if (addr == W5500_CTRL_S0_TXBUF_WRITE && len >= 14) {
    net_record_frame(true, static_cast<const uint8_t *>(data), len);
  }
  spi_transaction_t trans = {};
  trans.cmd = static_cast<uint16_t>(cmd);
  trans.addr = addr;
  trans.length = 8 * len;
  trans.tx_buffer = data;
  return w5500_custom_spi_transfer(ctx, &trans, len);
}

esp_err_t w5500_custom_spi_read(void *spi_ctx, uint32_t cmd, uint32_t addr, void *data, uint32_t len) {
  auto *ctx = static_cast<W5500CustomSpiContext *>(spi_ctx);
  note_read_transaction();
  // LOCAL DELTA: NTP T2. The driver asks Sn_RX_RSR how much is waiting, then clocks the
  // payload out; stamping the former keeps the SPI transfer out of T2.
  if (addr == W5500_CTRL_S0_REG_READ && cmd == W5500_REG_SN_RX_RSR) {
    g_rx_size_read_us = micros();
    g_rx_payloads_since_size_read = 0;
  } else if (addr == W5500_CTRL_S0_RXBUF_READ && len > 4) {
    g_rx_payload_us = micros();
    g_rx_payloads_since_size_read++;
  }
  spi_transaction_t trans = {};
  // Reads of <= 4 bytes use the transaction's inline RX buffer to avoid 4-byte boundary
  // overwrites of adjacent registers (same guard the stock driver uses).
  const bool use_rxdata = len <= 4;
  trans.flags = use_rxdata ? SPI_TRANS_USE_RXDATA : 0;
  trans.cmd = static_cast<uint16_t>(cmd);
  trans.addr = addr;
  trans.length = 8 * len;
  trans.rx_buffer = data;
  esp_err_t ret = w5500_custom_spi_transfer(ctx, &trans, len);
  if (use_rxdata && (ret == ESP_OK)) {
    memcpy(data, trans.rx_data, len);
  }
  // LOCAL DELTA: the driver polls Sn_CR until the chip clears the command it just wrote.
  if (ret == ESP_OK && g_cr_pending && addr == W5500_CTRL_S0_REG_READ && cmd == W5500_REG_SN_CR && len == 1) {
    g_cr_polls++;
    if (static_cast<const uint8_t *>(data)[0] == 0) {
      const uint32_t took_us = micros() - g_cr_write_us;
      g_cr_pending = false;
      g_cr_commands.fetch_add(1, std::memory_order_relaxed);
      if (g_cr_polls > 1) {
        g_cr_retried.fetch_add(1, std::memory_order_relaxed);
        net_record_cr_retry();
      }
      uint32_t cur = g_cr_max_us.load(std::memory_order_relaxed);
      while (took_us > cur && !g_cr_max_us.compare_exchange_weak(cur, took_us, std::memory_order_relaxed)) {
      }
    }
  }
  // LOCAL DELTA: network recorder. A received frame's payload read starts at its Ethernet header.
  if (ret == ESP_OK && addr == W5500_CTRL_S0_RXBUF_READ && len > 4) {
    net_record_frame(false, static_cast<const uint8_t *>(data), len);
  }
  return ret;
}

#if defined(CONFIG_SOC_MCPWM_SUPPORTED)
static const char *const TAG = "w5500_spi";

/// ISR context: micros() maps to esp_timer_get_time(), which is IRAM_ATTR and lock-free.
/// Nothing else is safe to call here.
bool IRAM_ATTR int_capture_cb(mcpwm_cap_channel_handle_t chan, const mcpwm_capture_event_data_t *edata,
                              void *user) {
  g_int_edge_us = micros();
  g_int_edge_seq++;
  return false;
}
#endif

}  // namespace

W5500RxStamps w5500_rx_stamps() {
  return {g_rx_size_read_us, g_rx_payload_us, g_rx_payloads_since_size_read, g_burst_start_us};
}

W5500SendStamp w5500_send_stamp() { return {g_send_cmd_us, g_send_cmd_seq}; }

W5500IntStamp w5500_int_stamp() { return {g_int_edge_us, g_int_edge_seq}; }

void w5500_set_net_recorder(bool enable) {
  if (enable && g_net_bins == nullptr) {
    auto *bins = static_cast<W5500NetBin *>(heap_caps_calloc(W5500_NET_BINS, sizeof(W5500NetBin), MALLOC_CAP_SPIRAM));
    if (bins == nullptr) {
      bins = static_cast<W5500NetBin *>(heap_caps_calloc(W5500_NET_BINS, sizeof(W5500NetBin), MALLOC_CAP_8BIT));
    }
    if (bins == nullptr) {
      ESP_LOGW("w5500_spi", "network recorder: cannot allocate %u bins", (unsigned) W5500_NET_BINS);
      return;
    }
    portENTER_CRITICAL(&g_net_mux);
    g_net_bins = bins;
    g_net_cur = 0;
    g_net_cur_start_us = 0;
    portEXIT_CRITICAL(&g_net_mux);
  }
  g_net_enabled = enable && g_net_bins != nullptr;
}

bool w5500_net_recorder_enabled() { return g_net_enabled; }

bool w5500_net_bin(uint16_t i, W5500NetBin &out) {
  if (g_net_bins == nullptr || i >= W5500_NET_BINS)
    return false;
  portENTER_CRITICAL(&g_net_mux);
  out = g_net_bins[(g_net_cur + 1 + i) % W5500_NET_BINS];
  portEXIT_CRITICAL(&g_net_mux);
  return out.start_us != 0;
}

W5500CmdStats w5500_take_cmd_stats() {
  return {g_cr_commands.exchange(0, std::memory_order_relaxed), g_cr_retried.exchange(0, std::memory_order_relaxed),
          g_cr_max_us.exchange(0, std::memory_order_relaxed), g_cr_overlaps.exchange(0, std::memory_order_relaxed)};
}

void w5500_start_int_capture(int gpio_num) {
#if defined(CONFIG_SOC_MCPWM_SUPPORTED)
  if (gpio_num < 0)
    return;
  // Safe to share the pin the ethernet driver already interrupts on: mcpwm_new_capture_channel()
  // only calls gpio_func_sel(), gpio_input_enable() and esp_rom_gpio_connect_in_signal(), never
  // gpio_config() and never intr_type, and one pad can drive several peripheral inputs.
  mcpwm_cap_timer_handle_t timer = nullptr;
  mcpwm_capture_timer_config_t tcfg = {};
  tcfg.group_id = 0;
  tcfg.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  if (mcpwm_new_capture_timer(&tcfg, &timer) != ESP_OK)
    return;
  mcpwm_cap_channel_handle_t chan = nullptr;
  mcpwm_capture_channel_config_t ccfg = {};
  ccfg.gpio_num = gpio_num;
  ccfg.prescale = 1;
  ccfg.flags.neg_edge = true;  // INTn is active low
  ccfg.flags.pull_up = true;
  if (mcpwm_new_capture_channel(timer, &ccfg, &chan) != ESP_OK)
    return;
  mcpwm_capture_event_callbacks_t cbs = {};
  cbs.on_cap = int_capture_cb;
  mcpwm_capture_channel_register_event_callbacks(chan, &cbs, nullptr);
  mcpwm_capture_channel_enable(chan);
  mcpwm_capture_timer_enable(timer);
  mcpwm_capture_timer_start(timer);
  ESP_LOGI(TAG, "hardware INT capture armed on GPIO%d", gpio_num);
#endif
}

void install_w5500_async_spi(eth_w5500_config_t &config) {
  // Point the custom driver's config at the W5500 config itself; init() reads spi_host_id and
  // spi_devcfg back out of it. The self-reference is valid because both the config and the
  // spi_devcfg it points at outlive the esp_eth_mac_new_w5500() call that runs init().
  config.custom_spi_driver.config = &config;
  config.custom_spi_driver.init = w5500_custom_spi_init;
  config.custom_spi_driver.deinit = w5500_custom_spi_deinit;
  config.custom_spi_driver.read = w5500_custom_spi_read;
  config.custom_spi_driver.write = w5500_custom_spi_write;
}

}  // namespace esphome::ethernet

#endif  // USE_ESP32 && USE_ETHERNET_W5500
