import os
from os.path import join, isfile
import subprocess

import esphome.config_validation as cv
import esphome.codegen as cg
import esphome.final_validate as fv

from esphome.components import esp32

from esphome.const import CONF_ID, CONF_BOARD

CODEOWNERS = [""]
DEPENDENCIES = ["esp32"]

CONF_ESP_ADF_ID = "esp_adf_id"
CONF_ESP_ADF = "esp_adf"

esp_adf_ns = cg.esphome_ns.namespace("esp_adf")
ESPADF = esp_adf_ns.class_("ESPADF", cg.Component)
ESPADFPipeline = esp_adf_ns.class_("ESPADFPipeline", cg.Parented.template(ESPADF))

SUPPORTED_BOARDS = {
    "esp32s3box": "CONFIG_ESP32_S3_BOX_BOARD",
    "esp32s3boxlite": "CONFIG_ESP32_S3_BOX_LITE_BOARD",
    "esp32s3box3": "CONFIG_ESP32_S3_BOX_3_BOARD",
    "esp32s3korvo1": "CONFIG_ESP32_S3_KORVO1_BOARD",
    "esp32korvo1": "CONFIG_ESP32_KORVO1_BOARD"
}

def get_idf_version():
    
    # Try to get the framework directory from the environment
    framework_dir = os.getenv("PIO_FRAMEWORK_ESP_IDF_DIR")

    # Fallback: Specify a hardcoded path (useful for debugging)
    if not framework_dir:
        framework_dir = "/data/cache/platformio/packages/framework-espidf"  # Replace with your actual path

    if not framework_dir or not os.path.isdir(framework_dir):
        print("[WARNING] ESP-IDF framework directory not found! Ensure PlatformIO is properly configured.")
        print("[INFO] Skipping IDF version detection. Defaulting to unknown version.")
        return "unknown"  # Return a default value or handle as necessary

    # Path to the version.txt file
    version_file = os.path.join(framework_dir, "version.txt")

    # Ensure the version.txt file exists
    if not os.path.isfile(version_file):
        print(f"[WARNING] version.txt not found in {framework_dir}. Defaulting to unknown version.")
        return "unknown"  # Return a default value or handle as necessary

    # Read the version from the version.txt file
    with open(version_file, "r") as vf:
        idf_version = vf.read().strip()

    return idf_version

    
def _default_board(config):
    config = config.copy()
    if board := config.get(CONF_BOARD) is None:
        board = esp32.get_board()
        if board in SUPPORTED_BOARDS:
            config[CONF_BOARD] = board
    return config


def final_validate_usable_board(platform: str):
    def _validate(adf_config):
        board = adf_config.get(CONF_BOARD)
        if board not in SUPPORTED_BOARDS:
            raise cv.Invalid(f"Board {board} is not supported by esp-adf {platform}")
        return adf_config

    return cv.Schema(
        {cv.Required(CONF_ESP_ADF_ID): fv.id_declaration_match_schema(_validate)},
        extra=cv.ALLOW_EXTRA,
    )


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ESPADF),
            cv.Optional(CONF_BOARD): cv.string_strict,
        }
    ),
    _default_board,
    cv.only_with_esp_idf,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add_define("USE_ESP_ADF")

    cg.add_platformio_option("build_unflags", "-Wl,--end-group")

    esp32.add_idf_component(
        name="esp-adf",
        repo="https://github.com/espressif/esp-adf",
        path="components",
        ref="v2.5",
        components=["*"],
        submodules=["components/esp-sr", "components/esp-adf-libs", "components/audio_hal"],
    )

    esp32.add_idf_component(
        name="esp-dsp",
        repo="https://github.com/espressif/esp-dsp",
        ref="v1.2.0",
    )

    cg.add_platformio_option(
        "board_build.embed_txtfiles", "components/dueros_service/duer_profile"
    )

    if board := config.get(CONF_BOARD):
        cg.add_define("USE_ESP_ADF_BOARD")
    
        esp32.add_idf_sdkconfig_option(SUPPORTED_BOARDS[board], True)
        
        esp32.add_extra_build_file(
            "esp_adf_patches/esp_adf_patch.diff",
            "https://github.com/dwitgen/esphome_test_audio/raw/main/components/esp_adf/esp_adf_patch.diff"
        )
        
        esp32.add_extra_script(
            "pre",
            "apply_custom_patch.py",
            os.path.join(os.path.dirname(__file__), "apply_custom_patch.py.script"),
        )
        # Detect ESP-IDF version
        idf_version = get_idf_version()
    
        if idf_version.startswith("4.4"):
            # Apply ESP-IDF 4.4-specific patch
            esp32.add_extra_build_file(
                "esp_adf_patches/idf_v4.4_freertos.patch",
                "https://github.com/espressif/esp-adf/raw/v2.5/idf_patches/idf_v4.4_freertos.patch",
            )
        elif idf_version.startswith("5.0"):
            # Apply ESP-IDF 5.0-specific patch
            esp32.add_extra_build_file(
                "esp_adf_patches/idf_v5.0_freertos.patch",
                "https://github.com/espressif/esp-adf/raw/main/idf_patches/idf_v5.0_freertos.patch",
            )
        elif idf_version.startswith("5.1"):
            # Apply ESP-IDF 5.1-specific patch
            esp32.add_extra_build_file(
                "esp_adf_patches/idf_v5.1_freertos.patch",
                "https://github.com/espressif/esp-adf/raw/main/idf_patches/idf_v5.1_freertos.patch",
            )
        elif idf_version.startswith("5.2"):
            # Apply ESP-IDF 5.2-specific patch
            esp32.add_extra_build_file(
                "esp_adf_patches/idf_v5.2_freertos.patch",
                "https://github.com/espressif/esp-adf/raw/main/idf_patches/idf_v5.2_freertos.patch",
            )
        elif idf_version.startswith("5.3"):
            # Apply ESP-IDF 5.3-specific patch
            esp32.add_extra_build_file(
                "esp_adf_patches/idf_v5.3_freertos.patch",
                "https://github.com/espressif/esp-adf/raw/main/idf_patches/idf_v5.3_freertos.patch",
            )
        else:
            raise ValueError(f"Unsupported ESP-IDF version: {idf_version}")
        
        esp32.add_extra_script(
            "pre",
            "apply_adf_patches.py",
            os.path.join(os.path.dirname(__file__), "apply_adf_patches.py.script"),
        )

    #if board := config.get(CONF_BOARD):
    #    cg.add_define("USE_ESP_ADF_BOARD")

    #    esp32.add_idf_sdkconfig_option(SUPPORTED_BOARDS[board], True)

    #    esp32.add_extra_build_file(
    #        "esp_adf_patches/esp_adf_patch.diff",
    #        "https://github.com/dwitgen/esphome_test_audio/raw/main/components/esp_adf/esp_adf_patch.diff"
    #    )
        
    #    esp32.add_extra_script(
    #        "pre",
    #        "apply_custom_patch.py",
    #        os.path.join(os.path.dirname(__file__), "apply_custom_patch.py.script"),
    #    )
        
    #    esp32.add_extra_script(
    #        "pre",
    #        "apply_adf_patches.py",
    #        os.path.join(os.path.dirname(__file__), "apply_adf_patches.py.script"),
    #    )
        
    #    esp32.add_extra_build_file(
    #        "esp_adf_patches/idf_v4.4_freertos.patch",
    #        "https://github.com/espressif/esp-adf/raw/v2.5/idf_patches/idf_v4.4_freertos.patch",
    #    )
