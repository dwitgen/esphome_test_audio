import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
from esphome.const import CONF_ID

CODEOWNERS = [""]
DEPENDENCIES = ["esp32"]

jsmn_ns = cg.esphome_ns.namespace("jsmn")
JSMNComponent = jsmn_ns.class_("JSMNComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(JSMNComponent),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    esp32.add_idf_component(
        name="jsmn",
        repo="https://github.com/dwitgen/jsmn",
        path=".",
        components=["*"],
    )
