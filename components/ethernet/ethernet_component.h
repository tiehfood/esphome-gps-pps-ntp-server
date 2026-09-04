#pragma once

// Local copy of ESPHome 2025.12.7's built-in `ethernet` component, overriding it via
// external_components (same component name shadows the core one). Pinned so the
// w5500_probe research spike (components/w5500_probe/) can reach the driver's
// esp_eth_handle_t, and so it can perform its own register-level SPI access through
// the SAME spi_device_handle_t the driver uses (see w5500_shared_spi() below) rather
// than creating a second device. The only changes vs. upstream: a public
// get_eth_handle() getter and, under USE_ETHERNET_SPI + CONFIG_ETH_SPI_ETHERNET_W5500,
// a custom_spi_driver hook (in ethernet_component.cpp) that stashes the resulting SPI
// handle + mutex for w5500_shared_spi() to return. Do not add anything else here —
// keep diffs against upstream reviewable.
//
// IMPORTANT: there must be exactly one spi_device_handle_t for the W5500 in the whole
// tree. A second spi_bus_add_device() call with the driver's own spics_io_num
// re-routes the CS pad to a different hardware CS signal and takes the W5500 off the
// network (cost one USB recovery during development — see w5500_probe.cpp).

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
#if CONFIG_ETH_SPI_ETHERNET_W5500
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif
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
  // Which SPI host setup() picked (SPI2_HOST on S3, SPI3_HOST elsewhere) — upstream
  // keeps this as a local in setup(); kept as a member only because it's convenient,
  // not exposed outside this file (see w5500_shared_spi() for how the probe reaches
  // the driver's SPI device instead).
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

#if defined(USE_ETHERNET_SPI) && CONFIG_ETH_SPI_ETHERNET_W5500
/// The W5500 driver's own SPI device handle + its access mutex, populated by the
/// custom_spi_driver hook in ethernet_component.cpp's setup(). This is how other
/// components (w5500_probe) perform register-level access on the SAME spi_device_handle_t
/// the driver uses, instead of calling spi_bus_add_device() a second time — see the
/// warning at the top of this file for why that is unsafe.
/// Both fields are null until EthernetComponent::setup() has run.
/// micros() stamps of the two earliest points in a W5500 frame receive: when the driver
/// read Sn_RX_RSR, and when it began clocking the payload. Their difference is the SPI
/// transfer cost that currently inflates NTP T2.
struct W5500RxStamps {
  uint32_t size_read_us;
  uint32_t payload_us;
};
W5500RxStamps w5500_rx_stamps();

struct W5500SharedSpi {
  spi_device_handle_t hdl{nullptr};
  SemaphoreHandle_t lock{nullptr};
};
W5500SharedSpi w5500_shared_spi();
#endif

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern EthernetComponent *global_eth_component;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 4, 2)
extern "C" esp_eth_phy_t *esp_eth_phy_new_jl1101(const eth_phy_config_t *config);
#endif

}  // namespace ethernet
}  // namespace esphome

#endif  // USE_ESP32
