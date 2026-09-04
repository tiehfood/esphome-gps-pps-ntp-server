#pragma once

// Local copy of ESPHome 2025.12.7's built-in `ethernet` component, overriding it via
// external_components (same component name shadows the core one). Pinned so the
// w5500_probe research spike (components/w5500_probe/) can reach the driver's
// esp_eth_handle_t and the SPI bus/CS it was configured with. The only changes vs.
// upstream: a public get_eth_handle() getter and, under USE_ETHERNET_SPI, getters for
// the SPI host/CS pin/clock speed used to build the driver's device. Do not add
// anything else here — keep diffs against upstream reviewable.

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/components/network/ip_address.h"

#ifdef USE_ESP32

#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_idf_version.h"
#ifdef USE_ETHERNET_SPI
#include <driver/spi_master.h>
#endif

namespace esphome {
namespace ethernet {

enum EthernetType : uint8_t {
  ETHERNET_TYPE_UNKNOWN = 0,
  ETHERNET_TYPE_LAN8720,
  ETHERNET_TYPE_RTL8201,
  ETHERNET_TYPE_DP83848,
  ETHERNET_TYPE_IP101,
  ETHERNET_TYPE_JL1101,
  ETHERNET_TYPE_KSZ8081,
  ETHERNET_TYPE_KSZ8081RNA,
  ETHERNET_TYPE_W5500,
  ETHERNET_TYPE_OPENETH,
  ETHERNET_TYPE_DM9051,
  ETHERNET_TYPE_LAN8670,
};

struct ManualIP {
  network::IPAddress static_ip;
  network::IPAddress gateway;
  network::IPAddress subnet;
  network::IPAddress dns1;  ///< The first DNS server. 0.0.0.0 for default.
  network::IPAddress dns2;  ///< The second DNS server. 0.0.0.0 for default.
};

struct PHYRegister {
  uint32_t address;
  uint32_t value;
  uint32_t page;
};

enum class EthernetComponentState : uint8_t {
  STOPPED,
  CONNECTING,
  CONNECTED,
};

class EthernetComponent : public Component {
 public:
  EthernetComponent();
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void on_powerdown() override { powerdown(); }
  bool is_connected();

#ifdef USE_ETHERNET_SPI
  void set_clk_pin(uint8_t clk_pin);
  void set_miso_pin(uint8_t miso_pin);
  void set_mosi_pin(uint8_t mosi_pin);
  void set_cs_pin(uint8_t cs_pin);
  void set_interrupt_pin(uint8_t interrupt_pin);
  void set_reset_pin(uint8_t reset_pin);
  void set_clock_speed(int clock_speed);
#ifdef USE_ETHERNET_SPI_POLLING_SUPPORT
  void set_polling_interval(uint32_t polling_interval);
#endif
#else
  void set_phy_addr(uint8_t phy_addr);
  void set_power_pin(int power_pin);
  void set_mdc_pin(uint8_t mdc_pin);
  void set_mdio_pin(uint8_t mdio_pin);
  void set_clk_pin(uint8_t clk_pin);
  void set_clk_mode(emac_rmii_clock_mode_t clk_mode);
  void add_phy_register(PHYRegister register_value);
#endif
  void set_type(EthernetType type);
#ifdef USE_ETHERNET_MANUAL_IP
  void set_manual_ip(const ManualIP &manual_ip);
#endif
  void set_fixed_mac(const std::array<uint8_t, 6> &mac) { this->fixed_mac_ = mac; }

  network::IPAddresses get_ip_addresses();
  network::IPAddress get_dns_address(uint8_t num);
  const char *get_use_address() const;
  void set_use_address(const char *use_address);
  void get_eth_mac_address_raw(uint8_t *mac);
  std::string get_eth_mac_address_pretty();
  eth_duplex_t get_duplex_mode();
  eth_speed_t get_link_speed();
  bool powerdown();

  /// RESEARCH SPIKE hook (w5500_probe): the driver's handle, needed to esp_eth_stop()/
  /// esp_eth_start() around a register rewrite. Do not call esp_eth_* teardown on this
  /// from anywhere except the probe's own button-triggered flow.
  esp_eth_handle_t get_eth_handle() const { return this->eth_handle_; }
  /// NTP input-path hook: the esp_netif every received frame must be forwarded to via
  /// esp_netif_receive() by any replacement stack_input callback (esp_eth_update_input_path()).
  /// Null until setup() runs. Callers MUST check for null and skip installing their hook
  /// rather than ever calling esp_netif_receive(nullptr, ...).
  esp_netif_t *get_eth_netif() const { return this->eth_netif_; }
#ifdef USE_ETHERNET_SPI
  /// RESEARCH SPIKE hook: SPI host/CS/clock the driver's own device was created with,
  /// so the probe can open a second spi_device_handle_t on the same bus and CS.
  spi_host_device_t get_spi_host() const { return this->spi_host_; }
  uint8_t get_spi_cs_pin() const { return this->cs_pin_; }
  int get_spi_clock_speed() const { return this->clock_speed_; }
#endif

 protected:
  static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
  static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
#if LWIP_IPV6
  static void got_ip6_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
#endif /* LWIP_IPV6 */

  void start_connect_();
  void finish_connect_();
  void dump_connect_params_();
  void log_error_and_mark_failed_(esp_err_t err, const char *message);
#ifdef USE_ETHERNET_KSZ8081
  /// @brief Set `RMII Reference Clock Select` bit for KSZ8081.
  void ksz8081_set_clock_reference_(esp_eth_mac_t *mac);
#endif
  /// @brief Set arbitratry PHY registers from config.
  void write_phy_register_(esp_eth_mac_t *mac, PHYRegister register_data);

#ifdef USE_ETHERNET_SPI
  uint8_t clk_pin_;
  uint8_t miso_pin_;
  uint8_t mosi_pin_;
  uint8_t cs_pin_;
  int interrupt_pin_{-1};
  int reset_pin_{-1};
  int phy_addr_spi_{-1};
  int clock_speed_;
  // RESEARCH SPIKE: which SPI host setup() picked (SPI2_HOST on S3, SPI3_HOST
  // elsewhere) — upstream keeps this as a local in setup(), we store it so
  // w5500_probe can put a second device on the same bus.
  spi_host_device_t spi_host_{SPI2_HOST};
#ifdef USE_ETHERNET_SPI_POLLING_SUPPORT
  uint32_t polling_interval_{0};
#endif
#else
  // Group all 32-bit members first
  int power_pin_{-1};
  emac_rmii_clock_mode_t clk_mode_{EMAC_CLK_EXT_IN};
  std::vector<PHYRegister> phy_registers_{};

  // Group all 8-bit members together
  uint8_t clk_pin_{0};
  uint8_t phy_addr_{0};
  uint8_t mdc_pin_{23};
  uint8_t mdio_pin_{18};
#endif
#ifdef USE_ETHERNET_MANUAL_IP
  optional<ManualIP> manual_ip_{};
#endif
  uint32_t connect_begin_;

  // Group all uint8_t types together (enums and bools)
  EthernetType type_{ETHERNET_TYPE_UNKNOWN};
  EthernetComponentState state_{EthernetComponentState::STOPPED};
  bool started_{false};
  bool connected_{false};
  bool got_ipv4_address_{false};
#if LWIP_IPV6
  uint8_t ipv6_count_{0};
  bool ipv6_setup_done_{false};
#endif /* LWIP_IPV6 */

  // Pointers at the end (naturally aligned)
  esp_netif_t *eth_netif_{nullptr};
  esp_eth_handle_t eth_handle_;
  esp_eth_phy_t *phy_{nullptr};
  optional<std::array<uint8_t, 6>> fixed_mac_;

 private:
  // Stores a pointer to a string literal (static storage duration).
  // ONLY set from Python-generated code with string literals - never dynamic strings.
  const char *use_address_{""};
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern EthernetComponent *global_eth_component;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 4, 2)
extern "C" esp_eth_phy_t *esp_eth_phy_new_jl1101(const eth_phy_config_t *config);
#endif

}  // namespace ethernet
}  // namespace esphome

#endif  // USE_ESP32
