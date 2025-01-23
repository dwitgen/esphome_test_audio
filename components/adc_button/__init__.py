import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
from esphome.const import CONF_ID

CODEOWNERS = ["@your-github-username"]  # Replace with your GitHub username
DEPENDENCIES = ["esp32"]

adc_button_ns = cg.esphome_ns.namespace("adc_button")
ADCButton = adc_button_ns.class_("ADCButton", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ADCButton),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add_define("USE_ADC_BUTTON")  # Define for C/C++ code integration

