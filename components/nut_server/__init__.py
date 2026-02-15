"""NUT Server Component for ESPHome - Configuration and code generation."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_PORT,
    CONF_USERNAME,
    CONF_PASSWORD,
)

DEPENDENCIES = ["esp32", "network", "ups_hid"]
AUTO_LOAD = []
MULTI_CONF = False

CONF_UPS_HID_ID = "ups_hid_id"
CONF_MAX_CLIENTS = "max_clients"
CONF_UPS_NAME = "ups_name"
CONF_USERS = "users"

nut_server_ns = cg.esphome_ns.namespace("nut_server")
NutServerComponent = nut_server_ns.class_("NutServerComponent", cg.Component)

# Import ups_hid namespace
ups_hid_ns = cg.esphome_ns.namespace("ups_hid")
UpsHidComponent = ups_hid_ns.class_("UpsHidComponent", cg.PollingComponent)

USER_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_USERNAME): cv.string_strict,
        cv.Required(CONF_PASSWORD): cv.string_strict,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NutServerComponent),
        cv.Required(CONF_UPS_HID_ID): cv.use_id(UpsHidComponent),
        cv.Optional(CONF_PORT, default=3493): cv.port,
        cv.Optional(CONF_USERS): cv.ensure_list(USER_SCHEMA),
        # Legacy single-user keys (backward compatible)
        cv.Optional(CONF_USERNAME): cv.string,
        cv.Optional(CONF_PASSWORD): cv.string,
        cv.Optional(CONF_MAX_CLIENTS, default=8): cv.int_range(min=1, max=20),
        cv.Optional(CONF_UPS_NAME): cv.string,
    }
).extend(cv.COMPONENT_SCHEMA)


def validate_config(config):
    """Validate NUT server configuration."""
    has_users = CONF_USERS in config
    has_legacy_user = CONF_USERNAME in config
    has_legacy_pass = CONF_PASSWORD in config

    if has_users and (has_legacy_user or has_legacy_pass):
        raise cv.Invalid(
            "Cannot use both 'users' and legacy 'username'/'password' keys. "
            "Use 'users' for multi-user or legacy keys for single-user."
        )

    if has_legacy_pass and not has_legacy_user:
        raise cv.Invalid("Username is required when password is set")

    return config


CONFIG_SCHEMA = CONFIG_SCHEMA.add_extra(validate_config)


async def to_code(config):
    """Generate C++ code for NUT server component."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Get reference to UPS HID component
    ups_hid = await cg.get_variable(config[CONF_UPS_HID_ID])
    cg.add(var.set_ups_hid(ups_hid))

    # Set port
    cg.add(var.set_port(config[CONF_PORT]))

    # Set authentication -- multi-user list takes priority
    if CONF_USERS in config:
        for user in config[CONF_USERS]:
            cg.add(var.add_user(user[CONF_USERNAME], user[CONF_PASSWORD]))
    elif CONF_USERNAME in config:
        password = config.get(CONF_PASSWORD, "")
        cg.add(var.add_user(config[CONF_USERNAME], password))

    # Set max clients
    cg.add(var.set_max_clients(config[CONF_MAX_CLIENTS]))

    # Set UPS name - use custom name if provided, otherwise use component ID
    if CONF_UPS_NAME in config:
        ups_name = config[CONF_UPS_NAME]
    else:
        ups_name = str(config[CONF_UPS_HID_ID])
    cg.add(var.set_ups_name(ups_name))
