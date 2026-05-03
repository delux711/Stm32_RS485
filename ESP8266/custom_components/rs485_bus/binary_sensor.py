import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import RS485Bus

CONF_ADDRESS = "address"
CONF_BUS_ID = "bus_id"

CONFIG_SCHEMA = (
    binary_sensor.binary_sensor_schema()
    .extend(
        {
            cv.GenerateID(): cv.declare_id(binary_sensor.BinarySensor),
            cv.Required(CONF_BUS_ID): cv.use_id(RS485Bus),
            cv.Required(CONF_ADDRESS): cv.uint8_t,
        }
    )
)

async def to_code(config):
    sens = await binary_sensor.new_binary_sensor(config)
    bus = await cg.get_variable(config[CONF_BUS_ID])
    cg.add(bus.register_pir_sensor(config[CONF_ADDRESS], sens))
