import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, sensor
from esphome.const import CONF_ID, CONF_NAME, CONF_DISABLED_BY_DEFAULT, CONF_UNIT_OF_MEASUREMENT, CONF_ICON, CONF_FORCE_UPDATE, CONF_ON_PRESS, CONF_TRIGGER_ID
from esphome import automation

from .. import (
    CONF_ESP_ADF_ID,
    ESPADF,
    esp_adf_ns,
    final_validate_usable_board,
)

AUTO_LOAD = ["esp_adf"]
DEPENDENCIES = ["esp32"]

ESPADFButton = esp_adf_ns.class_("ESPADFButton", cg.Component)

BUTTON_SCHEMA = cv.Schema(
    {
        
        cv.Optional(CONF_ON_PRESS): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(binary_sensor.PressTrigger),
                }
            ),
        #cv.Optional("on_press"): automation.validate_automation(
        #    {
        #        cv.GenerateID(): cv.declare_id(binary_sensor.PressTrigger),
        #        cv.Optional("on_press"): automation.validate_automation(single=True),
        #    }
        #),
        cv.Optional("on_release"): automation.validate_automation(
            {
                cv.GenerateID(): cv.declare_id(binary_sensor.ReleaseTrigger),
            }
        ),
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ESPADFButton),
            cv.GenerateID(CONF_ESP_ADF_ID): cv.use_id(ESPADF),
            cv.Optional("btn_vol_up"): BUTTON_SCHEMA,
            cv.Optional("btn_vol_down"): BUTTON_SCHEMA,
            cv.Optional("btn_set"): BUTTON_SCHEMA,
            cv.Optional("btn_play"): BUTTON_SCHEMA,
            cv.Optional("btn_mode"): BUTTON_SCHEMA,
            cv.Optional("btn_record"): BUTTON_SCHEMA,
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

    # Create and configure the volume sensor
    volume_sensor_id = cv.declare_id(sensor.Sensor)(f"{config[CONF_ID]}_volume_level")
    volume_sensor_config = {
        CONF_ID: volume_sensor_id,
        CONF_NAME: "Volume Level",  # Matches your working binary sensor style
        CONF_UNIT_OF_MEASUREMENT: "%",
        CONF_ICON: "mdi:volume-high",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_FORCE_UPDATE: False,
        "internal": False,  # Ensure visibility in Home Assistant
    }
    volume_sensor = await sensor.new_sensor(volume_sensor_config)
    cg.add(var.set_volume_sensor(volume_sensor))

    # Create and configure the volume sensor
    #volume_sensor_id = cv.declare_id(sensor.Sensor)(f"{config[CONF_ID]}_volume_level")
    #volume_sensor = cg.new_Pvariable(volume_sensor_id, sensor.Sensor())
    #cg.add(volume_sensor.set_name("Volume Level"))
    #cg.add(volume_sensor.set_unit_of_measurement("%"))
    #cg.add(volume_sensor.set_icon("mdi:volume-high"))
    #cg.add(volume_sensor.set_disabled_by_default(False))
    #cg.add(var.set_volume_sensor(volume_sensor))

    # Define buttons as binary sensors
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
        sensor_id = cv.declare_id(binary_sensor.BinarySensor)(f"{config[CONF_ID]}_{button_id}")
        sensor_config = {
            CONF_ID: sensor_id,
            CONF_NAME: f"{button_name}",
            CONF_DISABLED_BY_DEFAULT: False,
            # "internal": False,  # Commented out, defaults to False
        }
        sensor_obj = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(getattr(var, f"set_{button_id}")(sensor_obj))
        
        # Handle actions (on_press / on_release)
        if "on_press" in sensor_config:
            automation_var = await automation.build_automation(
                sensor_obj, sensor_config["on_press"], cg.RawExpression("sensor_obj->state")
            )
            cg.add(sensor_obj.add_on_state_callback(automation_var))

        if "on_release" in sensor_config:
            await automation.build_automation(
                sensor_obj, sensor_config["on_release"], cg.RawExpression("false")
            )
        #if "on_press" in sensor_config:
        #        await automation.build_automation(
        #            sensor_obj.add_on_state_callback, sensor_config["on_press"]
        #        )

        #if "on_release" in sensor_config:
        #    await automation.build_automation(
        #        sensor_obj.add_on_state_callback, sensor_config["on_release"]
        #    )