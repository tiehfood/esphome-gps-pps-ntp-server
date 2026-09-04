# RESEARCH SPIKE (throwaway) -- see docs/superpowers/plans/2026-09-03-ntp-serving-latency.md
# and .claude/tracker.md. Answers one question: with the W5500's socket 0 in MACRAW mode
# (esp_eth's netif) and socket 1 opened in UDP mode on port 123, where does an incoming
# NTP packet land? Not meant to survive past that answer -- do not build Phase 2/3 on it.
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button, ethernet, sensor, text_sensor
from esphome.const import CONF_ID, STATE_CLASS_MEASUREMENT

DEPENDENCIES = ["ethernet"]
AUTO_LOAD = ["sensor", "text_sensor", "button"]

CONF_ETHERNET_ID = "ethernet_id"
CONF_PROBE_BUTTON = "probe_button"
CONF_RECOVER_BUTTON = "recover_button"
CONF_ALT_PORT_BUTTON = "alt_port_button"
CONF_POLL_ONLY_BUTTON = "poll_only_button"
CONF_DUMP_CONFIG_BUTTON = "dump_config_button"
CONF_SIMR_BUTTON = "simr_button"
CONF_TCP_BUTTON = "tcp_button"
CONF_SUSTAINED_BUTTON = "sustained_button"
CONF_RX_BYTES = "rx_bytes"
CONF_SOCKET_STATUS = "socket_status"

w5500_probe_ns = cg.esphome_ns.namespace("w5500_probe")
W5500Probe = w5500_probe_ns.class_("W5500Probe", cg.Component)
ProbeButton = w5500_probe_ns.class_(
    "ProbeButton", button.Button, cg.Parented.template(W5500Probe)
)
RecoverButton = w5500_probe_ns.class_(
    "RecoverButton", button.Button, cg.Parented.template(W5500Probe)
)
ProbeAltPortButton = w5500_probe_ns.class_(
    "ProbeAltPortButton", button.Button, cg.Parented.template(W5500Probe)
)
ProbePollOnlyButton = w5500_probe_ns.class_(
    "ProbePollOnlyButton", button.Button, cg.Parented.template(W5500Probe)
)
DumpConfigButton = w5500_probe_ns.class_(
    "DumpConfigButton", button.Button, cg.Parented.template(W5500Probe)
)
ProbeSimrButton = w5500_probe_ns.class_(
    "ProbeSimrButton", button.Button, cg.Parented.template(W5500Probe)
)
ProbeTcpButton = w5500_probe_ns.class_(
    "ProbeTcpButton", button.Button, cg.Parented.template(W5500Probe)
)
ProbeSustainedButton = w5500_probe_ns.class_(
    "ProbeSustainedButton", button.Button, cg.Parented.template(W5500Probe)
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(W5500Probe),
        cv.GenerateID(CONF_ETHERNET_ID): cv.use_id(ethernet.EthernetComponent),
        cv.Optional(CONF_PROBE_BUTTON): button.button_schema(
            ProbeButton,
            icon="mdi:flask-outline",
        ),
        cv.Optional(CONF_RECOVER_BUTTON): button.button_schema(
            RecoverButton,
            icon="mdi:backup-restore",
        ),
        cv.Optional(CONF_ALT_PORT_BUTTON): button.button_schema(
            ProbeAltPortButton,
            icon="mdi:backup-restore",
        ),
        cv.Optional(CONF_POLL_ONLY_BUTTON): button.button_schema(
            ProbePollOnlyButton,
            icon="mdi:backup-restore",
        ),
        cv.Optional(CONF_DUMP_CONFIG_BUTTON): button.button_schema(
            DumpConfigButton,
            icon="mdi:backup-restore",
        ),
        cv.Optional(CONF_SIMR_BUTTON): button.button_schema(
            ProbeSimrButton,
            icon="mdi:backup-restore",
        ),
        cv.Optional(CONF_TCP_BUTTON): button.button_schema(
            ProbeTcpButton,
            icon="mdi:backup-restore",
        ),
        cv.Optional(CONF_SUSTAINED_BUTTON): button.button_schema(
            ProbeSustainedButton,
            icon="mdi:backup-restore",
        ),
        cv.Optional(CONF_RX_BYTES): sensor.sensor_schema(
            unit_of_measurement="B",
            icon="mdi:download-network-outline",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SOCKET_STATUS): text_sensor.text_sensor_schema(
            icon="mdi:lan-connect",
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # Enables the socket-buffer split in the ethernet component. Only defined when this
    # probe is configured, so a build without the spike never writes those registers.
    cg.add_define("USE_W5500_SOCKET_SPLIT")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    eth = await cg.get_variable(config[CONF_ETHERNET_ID])
    cg.add(var.set_ethernet(eth))

    if probe_conf := config.get(CONF_PROBE_BUTTON):
        b = await button.new_button(probe_conf)
        await cg.register_parented(b, config[CONF_ID])
        cg.add(var.set_probe_button(b))

    if alt_conf := config.get(CONF_ALT_PORT_BUTTON):
        b = await button.new_button(alt_conf)
        await cg.register_parented(b, config[CONF_ID])
        cg.add(var.set_alt_port_button(b))

    if poll_conf := config.get(CONF_POLL_ONLY_BUTTON):
        b = await button.new_button(poll_conf)
        await cg.register_parented(b, config[CONF_ID])
        cg.add(var.set_poll_only_button(b))

    if dump_conf := config.get(CONF_DUMP_CONFIG_BUTTON):
        b = await button.new_button(dump_conf)
        await cg.register_parented(b, config[CONF_ID])
        cg.add(var.set_dump_config_button(b))

    if simr_conf := config.get(CONF_SIMR_BUTTON):
        b = await button.new_button(simr_conf)
        await cg.register_parented(b, config[CONF_ID])
        cg.add(var.set_simr_button(b))

    if tcp_conf := config.get(CONF_TCP_BUTTON):
        b = await button.new_button(tcp_conf)
        await cg.register_parented(b, config[CONF_ID])
        cg.add(var.set_tcp_button(b))

    if sus_conf := config.get(CONF_SUSTAINED_BUTTON):
        b = await button.new_button(sus_conf)
        await cg.register_parented(b, config[CONF_ID])
        cg.add(var.set_sustained_button(b))

    if recover_conf := config.get(CONF_RECOVER_BUTTON):
        b = await button.new_button(recover_conf)
        await cg.register_parented(b, config[CONF_ID])
        cg.add(var.set_recover_button(b))

    if rx_conf := config.get(CONF_RX_BYTES):
        sens = await sensor.new_sensor(rx_conf)
        cg.add(var.set_rx_bytes_sensor(sens))

    if status_conf := config.get(CONF_SOCKET_STATUS):
        sens = await text_sensor.new_text_sensor(status_conf)
        cg.add(var.set_socket_status_sensor(sens))
