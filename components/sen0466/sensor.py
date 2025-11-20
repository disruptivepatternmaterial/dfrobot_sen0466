import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, number, sensor
from esphome.const import (
    CONF_ENTITY_CATEGORY,
    CONF_ID,
    CONF_INITIAL_VALUE,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_NAME,
    CONF_RESTORE_VALUE,
    CONF_SET_ACTION,
    CONF_STEP,
    CONF_TEMPERATURE,
    DEVICE_CLASS_CARBON_MONOXIDE,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_CONFIG,
    ICON_CHEMICAL_WEAPON,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PARTS_PER_MILLION,
)

AUTO_LOAD = ["number"]
CODEOWNERS = ["@jfurtner"]
DEPENDENCIES = ["i2c"]

sen0466_sensor_ns = cg.esphome_ns.namespace("sen0466_sensor")
Sen0466SensorClassId = sen0466_sensor_ns.class_(
    "Sen0466Sensor", cg.PollingComponent, i2c.I2CDevice
)

TemperatureOffsetClassId = sen0466_sensor_ns.class_(
    "temperature_offset", number.Number)

CONF_CARBON_MONOXIDE = "carbon_monoxide"
CONF_TEMPERATURE_OFFSET = "temperature_offset"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Sen0466SensorClassId),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CARBON_MONOXIDE): sensor.sensor_schema(
                unit_of_measurement=UNIT_PARTS_PER_MILLION,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_CARBON_MONOXIDE,
                state_class=STATE_CLASS_MEASUREMENT,
                icon=ICON_CHEMICAL_WEAPON,
            ),
            cv.Optional(CONF_TEMPERATURE_OFFSET): cv.maybe_simple_value(
                number._NUMBER_SCHEMA.extend(
                    {
                        cv.GenerateID(): cv.declare_id(TemperatureOffsetClassId),
                        cv.Optional(CONF_INITIAL_VALUE, default=0.0): cv.float_range(-10, 10),
                        cv.Optional(CONF_MAX_VALUE, default=+10): cv.float_range(min=-10, max=10),
                        cv.Optional(CONF_MIN_VALUE, default=-10): cv.float_range(min=-10, max=10),
                        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
                        cv.Optional(CONF_STEP, default=0.1): cv.float_range(0.0001, 1.0),
                        cv.Optional(CONF_NAME, default="Temperature offset"): cv.string_strict("Temperature offset")
                    }
                ).extend(cv.COMPONENT_SCHEMA),
                key=CONF_INITIAL_VALUE,
            ),
        },
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x74))
)

async def to_code(config):
    main_sensor_pvariable = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(main_sensor_pvariable, config)
    await i2c.register_i2c_device(main_sensor_pvariable, config)

    if temperature_config := config.get(CONF_TEMPERATURE):
        sens = await sensor.new_sensor(temperature_config)
        cg.add(main_sensor_pvariable.set_temperature_sensor(sens))

    if carbon_monoxide_config := config.get(CONF_CARBON_MONOXIDE):
        sens = await sensor.new_sensor(carbon_monoxide_config)
        cg.add(main_sensor_pvariable.set_carbon_monoxide_sensor(sens))
    
    if temperature_offset_config := config.get(CONF_TEMPERATURE_OFFSET):
        sens = await number.new_number(temperature_offset_config, min_value=-10, max_value=10, step=0.1, )
        await cg.register_parented(sens, main_sensor_pvariable)
