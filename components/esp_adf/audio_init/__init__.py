import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

# Define namespace
esp_adf_ns = cg.esphome_ns.namespace("esp_adf")
AudioInit = esp_adf_ns.class_("AudioInit", cg.Component)

# Configuration schema
CONF_SAMPLE_RATE = "sample_rate"
CONF_BITS_PER_SAMPLE = "bits_per_sample"
CONF_CHANNELS = "channels"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AudioInit),
        cv.Optional(CONF_SAMPLE_RATE, default=44100): cv.positive_int,
        cv.Optional(CONF_BITS_PER_SAMPLE, default=16): cv.positive_int,
        cv.Optional(CONF_CHANNELS, default=2): cv.one_of(1, 2, int=True),
    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_bits_per_sample(config[CONF_BITS_PER_SAMPLE]))
    cg.add(var.set_channels(config[CONF_CHANNELS]))
