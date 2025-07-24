import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.automation import maybe_simple_id
from esphome.components import cover, uart
from esphome.const import (
    CONF_ID,
    CONF_TRIGGER_ID,
    CONF_ACCELERATION,
    CONF_ADDRESS,
    CONF_VALUE,
)

CODEOWNERS = ["@strange-v"]
DEPENDENCIES = ["uart"]

mature_blinds_ns = cg.esphome_ns.namespace("mature_blinds")
MatureBlinds = mature_blinds_ns.class_(
    "MatureBlinds", cg.Component, cover.Cover, uart.UARTDevice
)

# Triggers
BeforeStartTrigger = mature_blinds_ns.class_("BeforeStartTrigger", automation.Trigger)
AfterStopTrigger = mature_blinds_ns.class_("AfterStopTrigger", automation.Trigger)

# Actions
HomeAction = mature_blinds_ns.class_("HomeAction", automation.Action)
MoveDownAction = mature_blinds_ns.class_("MoveDownAction", automation.Action)


CONF_EN_PIN = "en_pin"
CONF_DIR_PIN = "dir_pin"
CONF_STEP_PIN = "step_pin"
CONF_DIAG_PIN = "diag_pin"
CONF_R_SENSE = "r_sense"
CONF_RMS_CURRENT = "rms_current"
CONF_STALL_VALUE = "stall_value"
CONF_SPEED_IN_US = "speed_in_us"
CONF_INVERT_ROTATION = "invert_rotation"
CONF_FULL_DISTANCE = "full_distance_in_steps"
CONF_HOMING_GAP = "homing_gap_in_steps"

CONF_ON_BEFORE_START = "on_before_start"
CONF_ON_AFTER_STOP = "on_after_stop"

CONFIG_SCHEMA = cv.All(
    cover.COVER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(MatureBlinds),
            cv.Optional(CONF_EN_PIN): cv.int_,
            cv.Required(CONF_DIR_PIN): cv.int_,
            cv.Required(CONF_STEP_PIN): cv.int_,
            cv.Required(CONF_DIAG_PIN): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_R_SENSE): cv.float_,
            cv.Required(CONF_ADDRESS): cv.int_,
            cv.Required(CONF_RMS_CURRENT): cv.int_,
            cv.Required(CONF_STALL_VALUE): cv.int_,
            cv.Required(CONF_SPEED_IN_US): cv.int_,
            cv.Required(CONF_ACCELERATION): cv.int_,
            cv.Required(CONF_FULL_DISTANCE): cv.int_,
            cv.Required(CONF_HOMING_GAP): cv.int_,
            cv.Optional(CONF_INVERT_ROTATION): cv.boolean,
            cv.Optional(CONF_ON_BEFORE_START): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(BeforeStartTrigger),
                }
            ),
            cv.Optional(CONF_ON_AFTER_STOP): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(AfterStopTrigger),
                }
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    cv.only_with_arduino,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_r_sense(config[CONF_R_SENSE]))
    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(var.set_dir_pin(config[CONF_DIR_PIN]))
    cg.add(var.set_step_pin(config[CONF_STEP_PIN]))
    diag_pin = (await cg.gpio_pin_expression(config[CONF_DIAG_PIN]),)
    cg.add(var.set_diag_pin(diag_pin))
    cg.add(var.set_rms_current(config[CONF_RMS_CURRENT]))
    cg.add(var.set_stall_value(config[CONF_STALL_VALUE]))
    cg.add(var.set_speed(config[CONF_SPEED_IN_US]))
    cg.add(var.set_acceleration(config[CONF_ACCELERATION]))
    cg.add(var.set_full_distance(config[CONF_FULL_DISTANCE]))
    cg.add(var.set_homing_gap(config[CONF_HOMING_GAP]))
    if CONF_EN_PIN in config:
        cg.add(var.set_en_pin(config[CONF_EN_PIN]))
    if CONF_INVERT_ROTATION in config:
        cg.add(var.set_invert_rotation(config[CONF_INVERT_ROTATION]))

    cg.add_platformio_option("build_flags", "CONFIG_IDF_TARGET_ESP32", "1")
    cg.add_library("SPI", None)
    cg.add_library("teemuatlut/TMCStepper", "^0.7.3")
    cg.add_library("gin66/FastAccelStepper", "^0.33.4")

    for conf in config.get(CONF_ON_BEFORE_START, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    for conf in config.get(CONF_ON_AFTER_STOP, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)


SIMPLE_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(MatureBlinds),
    }
)
MOVE_DOWN_ACTION_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(MatureBlinds),
        cv.Required(CONF_VALUE): cv.templatable(cv.int_),
    }
)


@automation.register_action("cover.home", HomeAction, SIMPLE_ACTION_SCHEMA)
async def home_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("cover.move_down", MoveDownAction, MOVE_DOWN_ACTION_SCHEMA)
async def move_down_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_VALUE], args, cg.int_)
    cg.add(var.set_value(template_))
    return var
