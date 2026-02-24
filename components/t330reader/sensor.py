import esphome.codegen as cg
from esphome.components import sensor, text_sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLUME,
    DEVICE_CLASS_VOLUME_FLOW_RATE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_CUBIC_METER,
    UNIT_CUBIC_METER_PER_HOUR,
    UNIT_HOUR,
    UNIT_KILOWATT,
)

DEPENDENCIES = ["api", "uart"]
AUTO_LOAD = ["text_sensor"]

t330reader_ns = cg.esphome_ns.namespace("t330reader")
T330Reader = t330reader_ns.class_("T330Reader", cg.PollingComponent, uart.UARTDevice)

CONF_UART_OUT_ID = "uart_out_id"

CONF_ENERGY_KWH = "energy_kwh"
CONF_VOLUME_M3 = "volume_m3"
CONF_POWER_KW = "power_kw"
CONF_FLOW_RATE_M3H = "flow_rate_m3h"
CONF_FLOW_TEMPERATURE_C = "flow_temperature_c"
CONF_RETURN_TEMPERATURE_C = "return_temperature_c"
CONF_TEMPERATURE_DIFFERENCE_K = "temperature_difference_k"
CONF_ON_TIME_H = "on_time_h"
CONF_OPERATING_TIME_H = "operating_time_h"
CONF_ACTIVITY_DURATION_S = "activity_duration_s"
CONF_AVERAGING_DURATION_S = "averaging_duration_s"
CONF_ACCESS_NUMBER = "access_number"
CONF_STATUS = "status"
CONF_ERROR_STATUS = "error_status"
CONF_MAX_POWER_KW = "max_power_kw"
CONF_MAX_FLOW_RATE_M3H = "max_flow_rate_m3h"
CONF_MAX_FLOW_TEMPERATURE_C = "max_flow_temperature_c"
CONF_MAX_RETURN_TEMPERATURE_C = "max_return_temperature_c"
CONF_PREVIOUS_ENERGY_KWH = "previous_energy_kwh"
CONF_PREVIOUS_VOLUME_M3 = "previous_volume_m3"
CONF_PREVIOUS_ON_TIME_H = "previous_on_time_h"
CONF_PREVIOUS_OPERATING_TIME_H = "previous_operating_time_h"

CONF_VERSION_STRING = "version_string"
CONF_MBUS_ADDRESS = "mbus_address"
CONF_FABRICATION_NUMBER = "fabrication_number"
CONF_METER_DATETIME = "meter_datetime"
CONF_MAX_POWER_DATETIME = "max_power_datetime"
CONF_MAX_FLOW_RATE_DATETIME = "max_flow_rate_datetime"
CONF_MAX_FLOW_TEMPERATURE_DATETIME = "max_flow_temperature_datetime"
CONF_MAX_RETURN_TEMPERATURE_DATETIME = "max_return_temperature_datetime"

NUMERIC_SENSORS = {
    CONF_ENERGY_KWH: sensor.sensor_schema(
        unit_of_measurement="kWh",
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    CONF_VOLUME_M3: sensor.sensor_schema(
        unit_of_measurement=UNIT_CUBIC_METER,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLUME,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    CONF_POWER_KW: sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_FLOW_RATE_M3H: sensor.sensor_schema(
        unit_of_measurement=UNIT_CUBIC_METER_PER_HOUR,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLUME_FLOW_RATE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_FLOW_TEMPERATURE_C: sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_RETURN_TEMPERATURE_C: sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_TEMPERATURE_DIFFERENCE_K: sensor.sensor_schema(
        unit_of_measurement="K",
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_ON_TIME_H: sensor.sensor_schema(
        unit_of_measurement=UNIT_HOUR,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DURATION,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_OPERATING_TIME_H: sensor.sensor_schema(
        unit_of_measurement=UNIT_HOUR,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DURATION,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_ACTIVITY_DURATION_S: sensor.sensor_schema(
        unit_of_measurement="s",
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DURATION,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_AVERAGING_DURATION_S: sensor.sensor_schema(
        unit_of_measurement="s",
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DURATION,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_ACCESS_NUMBER: sensor.sensor_schema(
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    CONF_STATUS: sensor.sensor_schema(
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    CONF_ERROR_STATUS: sensor.sensor_schema(
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    CONF_MAX_POWER_KW: sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_MAX_FLOW_RATE_M3H: sensor.sensor_schema(
        unit_of_measurement=UNIT_CUBIC_METER_PER_HOUR,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLUME_FLOW_RATE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_MAX_FLOW_TEMPERATURE_C: sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_MAX_RETURN_TEMPERATURE_C: sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_PREVIOUS_ENERGY_KWH: sensor.sensor_schema(
        unit_of_measurement="kWh",
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_PREVIOUS_VOLUME_M3: sensor.sensor_schema(
        unit_of_measurement=UNIT_CUBIC_METER,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLUME,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_PREVIOUS_ON_TIME_H: sensor.sensor_schema(
        unit_of_measurement=UNIT_HOUR,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DURATION,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_PREVIOUS_OPERATING_TIME_H: sensor.sensor_schema(
        unit_of_measurement=UNIT_HOUR,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DURATION,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}

TEXT_SENSORS = {
    CONF_VERSION_STRING: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    CONF_MBUS_ADDRESS: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    CONF_FABRICATION_NUMBER: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    CONF_METER_DATETIME: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    CONF_MAX_POWER_DATETIME: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    CONF_MAX_FLOW_RATE_DATETIME: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    CONF_MAX_FLOW_TEMPERATURE_DATETIME: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    CONF_MAX_RETURN_TEMPERATURE_DATETIME: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(T330Reader),
            cv.GenerateID(CONF_UART_OUT_ID): cv.use_id(uart.UARTComponent),
            **{cv.Optional(key): schema for key, schema in NUMERIC_SENSORS.items()},
            **{cv.Optional(key): schema for key, schema in TEXT_SENSORS.items()},
        }
    )
    .extend(cv.polling_component_schema("30m"))
    .extend(uart.UART_DEVICE_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = cv.All(
    uart.final_validate_device_schema(
        "t330reader",
        baud_rate=2400,
        require_rx=True,
        data_bits=8,
        parity="EVEN",
        stop_bits=1,
    ),
    uart.final_validate_device_schema(
        "t330reader",
        uart_bus=CONF_UART_OUT_ID,
        baud_rate=2400,
        require_tx=True,
        data_bits=8,
        parity="EVEN",
        stop_bits=1,
    ),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    uart_out = await cg.get_variable(config[CONF_UART_OUT_ID])
    cg.add(var.set_uart_out(uart_out))

    for key in NUMERIC_SENSORS:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_sensor")(sens))

    for key in TEXT_SENSORS:
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_sensor")(sens))
