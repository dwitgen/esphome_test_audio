#include "esp_adf_button.h"
#include "esphome/core/log.h"
#include "esp_peripherals.h"
#include "periph_adc_button.h"
#include "input_key_service.h"
#include "board.h"

namespace esphome {
namespace esp_adf {

static const char *TAG = "ESPADFButton";
static periph_service_handle_t input_ser;

void ESPADFButton::setup() {
    ESP_LOGE(TAG, "Setting up ESP-ADF Button Component...");

    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    input_cfg.based_cfg.task_stack = 4 * 1024;

    input_ser = input_key_service_create(&input_cfg);
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);

    periph_service_set_callback(input_ser, input_key_service_cb, this);
    periph_service_start(input_ser);

    ESP_LOGE(TAG, "ESP-ADF Button Component Initialized Successfully!");
}

esp_err_t ESPADFButton::input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx) {
    if (!ctx) {
        ESP_LOGE(TAG, "ERROR: Context is NULL in input_key_service_cb!");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "[ * ] Button Event: ID=%d, Type=%d", (int)evt->data, evt->type);

    const char *key_types[INPUT_KEY_SERVICE_ACTION_PRESS_RELEASE + 1] = {
        "UNKNOWN", "CLICKED", "CLICK RELEASED", "PRESSED", "PRESS RELEASED"
    };
    if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK || evt->type == INPUT_KEY_SERVICE_ACTION_PRESS) {
        switch ((int)evt->data) {
            case INPUT_KEY_USER_ID_REC:
                ESP_LOGE(TAG, "[ * ] [Rec] KEY %s", key_types[evt->type]);
                break;
            case INPUT_KEY_USER_ID_SET:
                ESP_LOGE(TAG, "[ * ] [SET] KEY %s", key_types[evt->type]);
                break;
            case INPUT_KEY_USER_ID_PLAY:
                ESP_LOGE(TAG, "[ * ] [Play] KEY %s", key_types[evt->type]);
                break;
            case INPUT_KEY_USER_ID_MODE:
                ESP_LOGE(TAG, "[ * ] [MODE] KEY %s", key_types[evt->type]);
                break;
            case INPUT_KEY_USER_ID_VOLDOWN:
                ESP_LOGE(TAG, "[ * ] [Vol-] KEY %s", key_types[evt->type]);
                //speaker->volume_down();
                break;
            case INPUT_KEY_USER_ID_VOLUP:
                ESP_LOGE(TAG, "[ * ] [Vol+] KEY %s", key_types[evt->type]);
                //speaker->volume_up();
                break;
            default:
                ESP_LOGE(TAG, "User Key ID[%d] does not support", (int)evt->data);
                break;
        }
    } else {
        ESP_LOGE(TAG, "Not supported action");
    }

    return ESP_OK;
}

}  // namespace esp_adf
}  // namespace esphome
