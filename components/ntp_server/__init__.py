import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, time as time_
from esphome.const import CONF_ID, CONF_PORT, STATE_CLASS_MEASUREMENT

# "ethernet" isn't touched through codegen (the hook reaches the driver via the C++
# global esphome::ethernet::global_eth_component), but this component's code must be
# compiled in for that symbol to exist at link time -- the project never builds
# without it anyway (W5500-only, no Wi-Fi).
DEPENDENCIES = ["network", "ethernet"]
AUTO_LOAD = ["sensor"]

CONF_TIME_ID = "time_id"
CONF_HOOK_LATENCY = "hook_latency"
CONF_RX_STAMP_GAP = "rx_stamp_gap"
CONF_ARP_PRIMES = "arp_primes"
CONF_RX_BURST_LEAD = "rx_burst_lead"

gps_pps_time_ns = cg.esphome_ns.namespace("gps_pps_time")
GPSPPSTime = gps_pps_time_ns.class_("GPSPPSTime", time_.RealTimeClock)

ntp_server_ns = cg.esphome_ns.namespace("ntp_server")
NTPServer = ntp_server_ns.class_("NTPServer", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NTPServer),
        cv.Optional(CONF_PORT, default=123): cv.port,
        cv.Required(CONF_TIME_ID): cv.use_id(GPSPPSTime),
        # Diagnostic: microseconds between the input-path hook's T2 stamp (frame
        # delivered by the driver, before lwIP) and recv_task_()'s own T2 stamp on the
        # same request. Only published on requests where the hook actually saw the
        # frame -- see docs/superpowers/plans/2026-09-03-ntp-serving-latency.md.
        cv.Optional(CONF_HOOK_LATENCY): sensor.sensor_schema(
            unit_of_measurement="µs",
            icon="mdi:timer-sand",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_RX_STAMP_GAP): sensor.sensor_schema(
            unit_of_measurement="µs",
            icon="mdi:timer-sand",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_RX_BURST_LEAD): sensor.sensor_schema(
            unit_of_measurement="µs",
            icon="mdi:timer-sand",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ARP_PRIMES): sensor.sensor_schema(
            unit_of_measurement="µs",
            icon="mdi:timer-sand",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))

    if time_id := config.get(CONF_TIME_ID):
        time_source = await cg.get_variable(time_id)
        cg.add(var.set_time_source(time_source))

    if hook_latency_config := config.get(CONF_HOOK_LATENCY):
        sens = await sensor.new_sensor(hook_latency_config)
        cg.add(var.set_hook_latency_sensor(sens))

    if rx_stamp_gap_config := config.get(CONF_RX_STAMP_GAP):
        sens = await sensor.new_sensor(rx_stamp_gap_config)
        cg.add(var.set_rx_stamp_gap_sensor(sens))

    if rx_burst_lead_config := config.get(CONF_RX_BURST_LEAD):
        sens = await sensor.new_sensor(rx_burst_lead_config)
        cg.add(var.set_rx_burst_lead_sensor(sens))

    if arp_primes_config := config.get(CONF_ARP_PRIMES):
        sens = await sensor.new_sensor(arp_primes_config)
        cg.add(var.set_arp_primes_sensor(sens))
