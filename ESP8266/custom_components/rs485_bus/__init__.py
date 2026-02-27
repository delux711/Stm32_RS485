import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, uart
from esphome.const import CONF_ID, CONF_UART_ID
from esphome import pins

CONF_POLL_INTERVAL = "poll_interval"
CONF_PONG_STATUS = "pong_status"

AUTO_LOAD = ["sensor", "binary_sensor"]
DEPENDENCIES = ["uart"]

rs485_bus_ns = cg.esphome_ns.namespace("rs485_bus")
RS485Bus = rs485_bus_ns.class_("RS485Bus", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RS485Bus),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required("de_pin"): pins.gpio_output_pin_schema,
        cv.Optional(CONF_POLL_INTERVAL, default="1s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_PONG_STATUS): binary_sensor.binary_sensor_schema(),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    uart_ = await cg.get_variable(config[CONF_UART_ID])
    de_pin = await cg.gpio_pin_expression(config["de_pin"])
    var = cg.new_Pvariable(config[CONF_ID], uart_, de_pin)
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))

    if CONF_PONG_STATUS in config:
        pong_status = await binary_sensor.new_binary_sensor(config[CONF_PONG_STATUS])
        cg.add(var.set_pong_status_sensor(pong_status))

    await cg.register_component(var, config)

__all__ = ["RS485Bus", "rs485_bus_ns"]
