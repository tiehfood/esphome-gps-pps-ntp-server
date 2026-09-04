import esphome.codegen as cg
from esphome.components import gps, sensor, text_sensor, time as time_
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_SATELLITES,
    STATE_CLASS_MEASUREMENT,
)

CONF_GPS_ID = "gps_id"
CONF_PPS_PIN = "pps_pin"
CONF_CLOCK_OFFSET = "clock_offset"
CONF_PPS_DRIFT = "pps_drift"
CONF_GPS_TIME = "gps_time"
CONF_GPS_SATELLITES = "gps_satellites"
CONF_GLONASS_SATELLITES = "glonass_satellites"
CONF_GALILEO_SATELLITES = "galileo_satellites"
CONF_CRASH_INFO = "crash_info"
CONF_NMEA_CLOCK_DELTA = "nmea_clock_delta"

DEPENDENCIES = ["gps"]
AUTO_LOAD = ["sensor", "text_sensor"]

gps_pps_time_ns = cg.esphome_ns.namespace("gps_pps_time")
GPSPPSTime = gps_pps_time_ns.class_(
    "GPSPPSTime", time_.RealTimeClock, gps.GPSListener
)

CONFIG_SCHEMA = time_.TIME_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(GPSPPSTime),
        cv.GenerateID(CONF_GPS_ID): cv.use_id(gps.GPS),
        cv.Required(CONF_PPS_PIN): pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_SATELLITES): sensor.sensor_schema(
            icon="mdi:satellite-variant",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CLOCK_OFFSET): sensor.sensor_schema(
            unit_of_measurement="µs",
            icon="mdi:clock-check-outline",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PPS_DRIFT): sensor.sensor_schema(
            unit_of_measurement="µs",
            icon="mdi:clock-alert-outline",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_GPS_TIME): text_sensor.text_sensor_schema(
            icon="mdi:clock-outline",
        ),
        cv.Optional(CONF_GPS_SATELLITES): sensor.sensor_schema(
            icon="mdi:satellite-variant",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_GLONASS_SATELLITES): sensor.sensor_schema(
            icon="mdi:satellite-variant",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_NMEA_CLOCK_DELTA): sensor.sensor_schema(
            icon="mdi:clock-alert",
            unit_of_measurement="ms",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_GALILEO_SATELLITES): sensor.sensor_schema(
            icon="mdi:satellite-variant",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CRASH_INFO): text_sensor.text_sensor_schema(
            icon="mdi:alert-circle-outline",
        ),
    }
).extend(cv.polling_component_schema("60s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await time_.register_time(var, config)
    await cg.register_component(var, config)

    paren = await cg.get_variable(config[CONF_GPS_ID])
    cg.add(paren.register_listener(var))

    pps_pin = await cg.gpio_pin_expression(config[CONF_PPS_PIN])
    cg.add(var.set_pps_pin(pps_pin))

    if satellites_config := config.get(CONF_SATELLITES):
        sens = await sensor.new_sensor(satellites_config)
        cg.add(var.set_satellites_sensor(sens))

    if offset_config := config.get(CONF_CLOCK_OFFSET):
        sens = await sensor.new_sensor(offset_config)
        cg.add(var.set_clock_offset_sensor(sens))

    if drift_config := config.get(CONF_PPS_DRIFT):
        sens = await sensor.new_sensor(drift_config)
        cg.add(var.set_pps_drift_sensor(sens))

    if gps_time_config := config.get(CONF_GPS_TIME):
        sens = await text_sensor.new_text_sensor(gps_time_config)
        cg.add(var.set_gps_time_sensor(sens))

    if gps_sat_config := config.get(CONF_GPS_SATELLITES):
        sens = await sensor.new_sensor(gps_sat_config)
        cg.add(var.set_gps_satellites_sensor(sens))

    if glonass_sat_config := config.get(CONF_GLONASS_SATELLITES):
        sens = await sensor.new_sensor(glonass_sat_config)
        cg.add(var.set_glonass_satellites_sensor(sens))

    if galileo_sat_config := config.get(CONF_GALILEO_SATELLITES):
        sens = await sensor.new_sensor(galileo_sat_config)
        cg.add(var.set_galileo_satellites_sensor(sens))

    if crash_info_config := config.get(CONF_CRASH_INFO):
        sens = await text_sensor.new_text_sensor(crash_info_config)
        cg.add(var.set_crash_info_sensor(sens))

    if nmea_delta_config := config.get(CONF_NMEA_CLOCK_DELTA):
        sens = await sensor.new_sensor(nmea_delta_config)
        cg.add(var.set_nmea_clock_delta_sensor(sens))
