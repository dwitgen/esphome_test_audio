import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID , CONF_NAME, CONF_DISABLED_BY_DEFAULT

from .. import (
    CONF_ESP_ADF_ID,
    ESPADF,
    ESPADFPipeline,
    esp_adf_ns,
    final_validate_usable_board,
)

AUTO_LOAD = ["esp_adf"]
DEPENDENCIES = ["esp32"]

ESPADFButton = esp_adf_ns.class_("ESPADFButton",  cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ESPADFButton),
            cv.GenerateID(CONF_ESP_ADF_ID): cv.use_id(ESPADF),
            cv.Optional(CONF_NAME): cv.string,
            cv.Optional(CONF_DISABLED_BY_DEFAULT, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_with_esp_idf,
)

FINAL_VALIDATE_SCHEMA = final_validate_usable_board("button")


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    #await cg.register_parented(var, config[CONF_ESP_ADF_ID])

    await button.register_button(var, config)
    
