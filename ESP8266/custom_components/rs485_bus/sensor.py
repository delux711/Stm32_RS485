import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

from . import RS485Bus

CONF_ADDRESS = "address"
CONF_BUS_ID = "bus_id"

CONFIG_SCHEMA = (
    sensor.sensor_schema()
    .extend(
        {
            cv.GenerateID(): cv.declare_id(sensor.Sensor),
            cv.Required(CONF_BUS_ID): cv.use_id(RS485Bus),
            cv.Required(CONF_ADDRESS): cv.uint8_t,
        }
    )
)

async def to_code(config):
    sens = await sensor.new_sensor(config)
    bus = await cg.get_variable(config[CONF_BUS_ID])
    cg.add(bus.register_temperature_sensor(config[CONF_ADDRESS], sens))
