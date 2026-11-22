import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (
    binary_sensor,
    button,
    fan,
    globals as globals_component,
    switch,
    text_sensor,
)
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32"]

CONF_SCK_PIN = "sck_pin"
CONF_MISO_PIN = "miso_pin"
CONF_MOSI_PIN = "mosi_pin"
CONF_CS_PIN = "cs_pin"
CONF_GDO0_PIN = "gdo0_pin"
CONF_GDO2_PIN = "gdo2_pin"
CONF_FAN_ADDRESSES = "fan_addresses"
CONF_FAN_1_ID = "fan_1_id"
CONF_FAN_2_ID = "fan_2_id"
CONF_LIGHT_1_ID = "light_1_id"
CONF_LIGHT_2_ID = "light_2_id"
CONF_LAST_REMOTE_TEXT_ID = "last_remote_text_id"
CONF_LAST_COMMAND_TEXT_ID = "last_command_text_id"
CONF_DISCOVERED_ADDRESS_TEXT_ID = "discovered_address_text_id"
CONF_DISCOVERY_SWITCH_ID = "discovery_switch_id"
CONF_FAN_1_BUTTONS = "fan_1_buttons"
CONF_FAN_2_BUTTONS = "fan_2_buttons"
CONF_PERSISTED_VALUES = "persisted_values"

cc1101_ns = cg.esphome_ns.namespace("cc1101_fan_controller")
CC1101FanController = cc1101_ns.class_("CC1101FanController", cg.Component)


def _fixed_list(item_schema, length):
    return cv.All(cv.ensure_list(item_schema), cv.Length(min=length, max=length))


def _fan_address(value):
    # Accept both YAML integers and quoted decimal/hex substitutions, such as
    # "0x94120". Quoting the substitution keeps ESPHome from parsing it as a
    # string that the regular decimal int_range validator cannot convert.
    if isinstance(value, str):
        try:
            value = int(value, 0)
        except ValueError as err:
            raise cv.Invalid("fan address must be a decimal or 0x-prefixed integer") from err
    return cv.int_range(min=1, max=0xFFFFF)(value)


def _validate_config(config):
    addresses = config[CONF_FAN_ADDRESSES]
    if addresses[0] == addresses[1]:
        raise cv.Invalid("fan_addresses must contain two different addresses")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CC1101FanController),
            cv.Required(CONF_SCK_PIN): cv.int_range(min=0, max=48),
            cv.Required(CONF_MISO_PIN): cv.int_range(min=0, max=48),
            cv.Required(CONF_MOSI_PIN): cv.int_range(min=0, max=48),
            cv.Required(CONF_CS_PIN): cv.int_range(min=0, max=48),
            cv.Required(CONF_GDO0_PIN): cv.int_range(min=0, max=48),
            cv.Required(CONF_GDO2_PIN): cv.int_range(min=0, max=48),
            cv.Required(CONF_FAN_ADDRESSES): _fixed_list(
                _fan_address, 2
            ),
            cv.Required(CONF_FAN_1_ID): cv.use_id(fan.Fan),
            cv.Required(CONF_FAN_2_ID): cv.use_id(fan.Fan),
            cv.Required(CONF_LIGHT_1_ID): cv.use_id(binary_sensor.BinarySensor),
            cv.Required(CONF_LIGHT_2_ID): cv.use_id(binary_sensor.BinarySensor),
            cv.Required(CONF_LAST_REMOTE_TEXT_ID): cv.use_id(text_sensor.TextSensor),
            cv.Required(CONF_LAST_COMMAND_TEXT_ID): cv.use_id(text_sensor.TextSensor),
            cv.Required(CONF_DISCOVERED_ADDRESS_TEXT_ID): cv.use_id(text_sensor.TextSensor),
            cv.Required(CONF_DISCOVERY_SWITCH_ID): cv.use_id(switch.Switch),
            cv.Required(CONF_FAN_1_BUTTONS): _fixed_list(cv.use_id(button.Button), 7),
            cv.Required(CONF_FAN_2_BUTTONS): _fixed_list(cv.use_id(button.Button), 7),
            cv.Required(CONF_PERSISTED_VALUES): _fixed_list(
                cv.use_id(globals_component.GlobalsComponent), 22
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_config,
)


async def _get_variables(ids):
    return [await cg.get_variable(entity_id) for entity_id in ids]


async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_SCK_PIN],
        config[CONF_MISO_PIN],
        config[CONF_MOSI_PIN],
        config[CONF_CS_PIN],
        config[CONF_GDO0_PIN],
        config[CONF_GDO2_PIN],
    )
    await cg.register_component(var, config)
    cg.add_library(
        "CC1101-ESP-Arduino",
        None,
        "https://github.com/jacekplonka/CC1101-ESP-Arduino.git",
    )

    cg.add(var.set_fan_addresses(*config[CONF_FAN_ADDRESSES]))

    fans = await _get_variables([config[CONF_FAN_1_ID], config[CONF_FAN_2_ID]])
    lights = await _get_variables([config[CONF_LIGHT_1_ID], config[CONF_LIGHT_2_ID]])
    diagnostics = await _get_variables(
        [
            config[CONF_LAST_REMOTE_TEXT_ID],
            config[CONF_LAST_COMMAND_TEXT_ID],
            config[CONF_DISCOVERED_ADDRESS_TEXT_ID],
            config[CONF_DISCOVERY_SWITCH_ID],
        ]
    )
    cg.add(var.bind_entities(*fans, *lights, *diagnostics))

    for fan_index, button_key in enumerate((CONF_FAN_1_BUTTONS, CONF_FAN_2_BUTTONS)):
        for action_index, button_var in enumerate(await _get_variables(config[button_key])):
            cg.add(var.bind_button(fan_index, action_index, button_var))

    global_ids = config[CONF_PERSISTED_VALUES]
    value_pointers = cg.RawExpression(
        "{" + ", ".join(f"&id({global_id.id})" for global_id in global_ids) + "}"
    )
    cg.add(var.bind_persistent_values(value_pointers))
