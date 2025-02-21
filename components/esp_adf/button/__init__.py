import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor #, button
from esphome.const import CONF_ID

from .. import (
    CONF_ESP_ADF_ID,
    ESPADF,
    esp_adf_ns,
    final_validate_usable_board,
)

AUTO_LOAD = ["esp_adf"]
DEPENDENCIES = ["esp32"]

ESPADFButton = esp_adf_ns.class_("ESPADFButton",  cg.Component) # button.Button, cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ESPADFButton),
            cv.GenerateID(CONF_ESP_ADF_ID): cv.use_id(ESPADF),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_with_esp_idf,
)

FINAL_VALIDATE_SCHEMA = final_validate_usable_board("button")


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # ✅ Define buttons as binary sensors
    buttons = {
        "btn_vol_up": "Volume Up",
        "btn_vol_down": "Volume Down",
        "btn_set": "Set",
        "btn_play": "Play",
        "btn_mode": "Mode",
        "btn_record": "Record",
    }

    for button_id, button_name in buttons.items():
        sensor_config = {
            CONF_ID: f"{config[CONF_ID]}_{button_id}", 
            "name": button_name,
        }
        sensor = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(getattr(var, button_id), sensor) 
    #for button_id, button_name in buttons.items():
    #    btn = await binary_sensor.new_binary_sensor(
    #        {
    #            "id": button_id,
    #            "name": f"{button_name}",
    #            "internal": False,  # Ensure it is visible in Home Assistant
    #        }
    #    )
    #    cg.add(getattr(var, button_id), btn)
    