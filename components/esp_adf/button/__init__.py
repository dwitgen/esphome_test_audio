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
        # ✅ Step 1: Create the binary sensor instance
        sensor = cg.new_Pvariable(cg.new_id(), binary_sensor.BinarySensor.new())

        # ✅ Step 2: Register the binary sensor with ESPHome
        await binary_sensor.register_binary_sensor(sensor, {})

        # ✅ Step 3: Set the name and link to our component
        cg.add(sensor.set_name(button_name))
        setattr(var, button_id, sensor) 
    #for button_id, button_name in buttons.items():
    #    btn = await binary_sensor.new_binary_sensor(
    #        {
    #            "id": button_id,
    #            "name": f"{button_name}",
    #            "internal": False,  # Ensure it is visible in Home Assistant
    #        }
    #    )
    #    cg.add(getattr(var, button_id), btn)
    