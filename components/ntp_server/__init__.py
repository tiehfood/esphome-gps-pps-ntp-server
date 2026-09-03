import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import time as time_
from esphome.const import CONF_ID, CONF_PORT

DEPENDENCIES = ["network"]

CONF_TIME_ID = "time_id"

gps_pps_time_ns = cg.esphome_ns.namespace("gps_pps_time")
GPSPPSTime = gps_pps_time_ns.class_("GPSPPSTime", time_.RealTimeClock)

ntp_server_ns = cg.esphome_ns.namespace("ntp_server")
NTPServer = ntp_server_ns.class_("NTPServer", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NTPServer),
        cv.Optional(CONF_PORT, default=123): cv.port,
        cv.Required(CONF_TIME_ID): cv.use_id(GPSPPSTime),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))

    if time_id := config.get(CONF_TIME_ID):
        time_source = await cg.get_variable(time_id)
        cg.add(var.set_time_source(time_source))
