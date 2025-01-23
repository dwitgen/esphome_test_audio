import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@your-github-username"]  # Replace with your GitHub username
DEPENDENCIES = ["esp32"]  # If it depends on other components

esp_peripherals_ns = cg.esphome_ns.namespace("esp_peripherals")
ESPPeripherals = esp_peripherals_ns.class_("ESPPeripherals", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ESPPeripherals),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
