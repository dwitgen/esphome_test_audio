import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, sensor
from esphome.const import CONF_ID, CONF_NAME, CONF_DISABLED_BY_DEFAULT,  CONF_UNIT_OF_MEASUREMENT, CONF_ICON

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
    # Register the main component
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Link to the ESPADF instance
    esp_adf = await cg.get_variable(config[CONF_ESP_ADF_ID])
    cg.add(var.set_esp_adf(esp_adf))

    # ✅ Define buttons as binary sensors
    buttons = {
        "btn_vol_up": "Volume Up",
        "btn_vol_down": "Volume Down",
        "btn_set": "Set",
        "btn_play": "Play",
        "btn_mode": "Mode",
        "btn_record": "Record",
    }

     # Create and register binary sensors for each button
    for button_id, button_name in buttons.items():
        # Step 1: Create a unique ID for the binary sensor
        sensor_id = cv.declare_id(binary_sensor.BinarySensor)(f"{config[CONF_ID]}_{button_id}")

        # Step 2: Configure and create the binary sensor
        sensor_config = {
            CONF_ID: sensor_id,
            CONF_NAME: f"{button_name}",
            CONF_DISABLED_BY_DEFAULT: False
           # "internal": False,  # Make it visible in Home Assistant
        }
        sensor = await binary_sensor.new_binary_sensor(sensor_config)

        # Step 3: Store the sensor in the component (setter method)
        cg.add(getattr(var, f"set_{button_id}")(sensor))
    
    
    # Create and configure the volume sensor
    volume_sensor_id = cv.declare_id(sensor.Sensor)(f"{config[CONF_ID]}_volume_level")
    volume_sensor = cg.new_Pvariable(volume_sensor_id)
    cg.add(volume_sensor.set_name("Volume Level"))
    cg.add(volume_sensor.set_unit_of_measurement("%"))
    cg.add(volume_sensor.set_icon("mdi:volume-high"))
    cg.add(volume_sensor.set_disabled_by_default(False))
    cg.add(var.set_volume_sensor(volume_sensor))
    