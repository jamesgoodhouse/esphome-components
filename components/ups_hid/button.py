import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from . import ups_hid_ns, CONF_UPS_HID_ID, UpsHidComponent

DEPENDENCIES = ["ups_hid"]

UpsHidButton = ups_hid_ns.class_("UpsHidButton", button.Button, cg.Component)

CONF_BEEPER_ACTION = "beeper_action"
CONF_TEST_ACTION = "test_action"
CONF_DEBUG_ACTION = "debug_action"

BEEPER_ACTIONS = {
    "enable": "enable",
    "disable": "disable",
    "mute": "mute",
    "test": "test"
}

TEST_ACTIONS = {
    "battery_quick": "battery_quick",
    "battery_deep": "battery_deep",
    "battery_stop": "battery_stop",
    "ups_test": "ups_test",
    "ups_stop": "ups_stop"
}

DEBUG_ACTIONS = {
    "clear_event_log": "clear_event_log",
}

def validate_button_config(config):
    actions = [
        config.get(CONF_BEEPER_ACTION),
        config.get(CONF_TEST_ACTION),
        config.get(CONF_DEBUG_ACTION),
    ]
    set_count = sum(1 for a in actions if a is not None)

    if set_count > 1:
        raise cv.Invalid("Specify only one of 'beeper_action', 'test_action', or 'debug_action'")

    if set_count == 0:
        raise cv.Invalid("Must specify one of 'beeper_action', 'test_action', or 'debug_action'")

    return config

CONFIG_SCHEMA = cv.All(
    button.button_schema(UpsHidButton).extend(
        {
            cv.GenerateID(CONF_UPS_HID_ID): cv.use_id(
                UpsHidComponent
            ),
            cv.Optional(CONF_BEEPER_ACTION): cv.enum(
                BEEPER_ACTIONS, lower=True
            ),
            cv.Optional(CONF_TEST_ACTION): cv.enum(
                TEST_ACTIONS, lower=True
            ),
            cv.Optional(CONF_DEBUG_ACTION): cv.enum(
                DEBUG_ACTIONS, lower=True
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_button_config,
)


async def to_code(config):
    var = await button.new_button(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_UPS_HID_ID])
    cg.add(var.set_ups_hid_parent(parent))

    if CONF_BEEPER_ACTION in config:
        cg.add(var.set_beeper_action(config[CONF_BEEPER_ACTION]))
    elif CONF_TEST_ACTION in config:
        cg.add(var.set_test_action(config[CONF_TEST_ACTION]))
    elif CONF_DEBUG_ACTION in config:
        cg.add(var.set_debug_action(config[CONF_DEBUG_ACTION]))
