#include "esp_adf_button.h"

#ifdef USE_ESP_IDF

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_peripherals.h"
#include "periph_adc_button.h"
#include "input_key_service.h"
#include <audio_hal.h>
#include "board.h"

#ifdef __cplusplus
}
#endif

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esp_adf {

static const char *TAG = "ESPADFButton";
static periph_service_handle_t input_ser;



void ESPADFButton::setup() {
    ESP_LOGE(TAG, "Setting up ESP-ADF Button Component...");

    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    periph_cfg.task_core = 1;
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    audio_board_key_init(set);

    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    //input_cfg.based_cfg.task_stack = 4 * 1024;
    input_cfg.based_cfg.task_core = 1;

    input_ser = input_key_service_create(&input_cfg);
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);

    periph_service_set_callback(input_ser, input_key_service_cb, this);
    periph_service_start(input_ser);

    ESP_LOGE(TAG, "ESP-ADF Button Component Initialized Successfully!");

    // Adding initial setup for volume controls for the speaker
    // Find the key for the generic volume sensor
    uint32_t volume_sensor_key = 0;
    for (auto *sensor : App.get_sensors()) {
        if (sensor->get_name() == "generic_volume_sensor") {
            volume_sensor_key = sensor->get_object_id_hash();
            break;
        }
    }

    // Use the key to get the sensor
    if (volume_sensor_key != 0) {
        this->volume_sensor = App.get_sensor_by_key(volume_sensor_key, true);
        ESP_LOGI(TAG, "Internal generic volume sensor initialized successfully: %s", this->volume_sensor->get_name().c_str());
    } else {
        ESP_LOGE(TAG, "Failed to find key for internal generic volume sensor");
    }


     // Link YAML binary sensors
    // Find the key for the button sensors
    uint32_t volup_sensor_key = 0;
    uint32_t voldown_sensor_key = 0;
    uint32_t set_sensor_key = 0;
    uint32_t play_sensor_key = 0;
    uint32_t mode_sensor_key = 0;
    uint32_t record_sensor_key = 0;
    for (auto *binary_sensor : App.get_binary_sensors()) {
        if (binary_sensor->get_name() == "internal_btn_vol_up") {
            volup_sensor_key = binary_sensor->get_object_id_hash();
        } else if (binary_sensor->get_name() == "internal_btn_vol_down") {
            voldown_sensor_key = binary_sensor->get_object_id_hash();
        } else if (binary_sensor->get_name() == "internal_btn_set") {
            set_sensor_key = binary_sensor->get_object_id_hash();
        } else if (binary_sensor->get_name() == "internal_btn_play") {
            play_sensor_key = binary_sensor->get_object_id_hash();
        } else if (binary_sensor->get_name() == "internal_btn_mode") {
            mode_sensor_key = binary_sensor->get_object_id_hash();
        } else if (binary_sensor->get_name() == "internal_btn_record") {
            record_sensor_key = binary_sensor->get_object_id_hash();
        }
    }

    if (volup_sensor_key != 0) {
        this->internal_btn_vol_up = App.get_binary_sensor_by_key(volup_sensor_key, true);
        ESP_LOGI(TAG, "Binary sensor for volume up initialized successfully: %s", this->internal_btn_vol_up->get_name().c_str());
    } else {
        ESP_LOGE(TAG, "Failed to find key for binary sensor volume up");
    }
    if (voldown_sensor_key != 0) {
        this->internal_btn_vol_down = App.get_binary_sensor_by_key(voldown_sensor_key, true);
        ESP_LOGI(TAG, "Binary sensor for volume down initialized successfully: %s", this->internal_btn_vol_down->get_name().c_str());
    } else {
        ESP_LOGE(TAG, "Failed to find key for binary sensor volume down");
    }
    if (set_sensor_key != 0) {
        this->internal_btn_set = App.get_binary_sensor_by_key(set_sensor_key, true);
        ESP_LOGI(TAG, "Binary sensor for set initialized successfully: %s", this->internal_btn_set->get_name().c_str());
    } else {
        ESP_LOGE(TAG, "Failed to find key for binary sensor set");
    }
    if (play_sensor_key != 0) {
        this->internal_btn_play = App.get_binary_sensor_by_key(play_sensor_key, true);
        ESP_LOGI(TAG, "Binary sensor for play initialized successfully: %s", this->internal_btn_play->get_name().c_str());
    } else {
        ESP_LOGE(TAG, "Failed to find key for binary sensor play");
    }
    if (mode_sensor_key != 0) {
        this->internal_btn_mode = App.get_binary_sensor_by_key(mode_sensor_key, true);
        ESP_LOGI(TAG, "Binary sensor for mode initialized successfully: %s", this->internal_btn_mode->get_name().c_str());
    } else {
        ESP_LOGE(TAG, "Failed to find key for binary sensor mode");
    }
    if (record_sensor_key != 0) {
        this->internal_btn_record = App.get_binary_sensor_by_key(record_sensor_key, true);
        ESP_LOGI(TAG, "Binary sensor for record initialized successfully: %s", this->internal_btn_record->get_name().c_str());
    } else {
        ESP_LOGE(TAG, "Failed to find key for binary sensor record");
    }

    // Initialize the audio board and store the handle
    this->board_handle_ = audio_board_init();
    if (this->board_handle_ == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize audio board");
        this->mark_failed();
        return;
    }

    // Set initial volume
    this->set_volume(volume_); // Set initial volume to 50%

    // Read and set initial volume
    int initial_volume = this->get_current_volume();
    this->set_volume(initial_volume);

}

//Volume controls for buttons again speaker may mot be the correct location for this
void ESPADFButton::set_volume(int volume) {
    ESP_LOGI(TAG, "Setting volume to %d", volume);
    
    // Ensure the volume is within the range 0-100
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    this->volume_ = volume;

    // Set volume using HAL
    
    //audio_board_handle_t board_handle = audio_board_init();
    esp_err_t err = audio_hal_set_volume(board_handle_->audio_hal, volume);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error setting volume: %s", esp_err_to_name(err));
    }

    // Update the volume sensor
    if (this->volume_sensor != nullptr) {
      this->volume_sensor->publish_state(this->volume_);
    } else {
      ESP_LOGE(TAG, "Volume sensor is not initialized");
    }

}
int ESPADFButton::get_current_volume() {
  
  int current_volume = 0;
  esp_err_t read_err = audio_hal_get_volume(board_handle_->audio_hal, &current_volume);
  if (read_err == ESP_OK) {
    ESP_LOGI(TAG, "Current device volume: %d", current_volume);
  } else {
    ESP_LOGE(TAG, "Error reading current volume: %s", esp_err_to_name(read_err));
  }

  return current_volume;
}
void ESPADFButton::volume_up() {
    ESP_LOGI(TAG, "Volume up button pressed");
    int current_volume = this->get_current_volume();
    this->set_volume(current_volume + 10);
}

void ESPADFButton::volume_down() {
    ESP_LOGI(TAG, "Volume down button pressed");
    int current_volume = this->get_current_volume();
    this->set_volume(current_volume - 10);
}

esp_err_t ESPADFButton::input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx) {
    ESPADFButton *button = static_cast<ESPADFButton *>(ctx);
    ESP_LOGD(TAG, "[ * ] input key id is %d, %d", (int)evt->data, evt->type);
    const char *key_types[INPUT_KEY_SERVICE_ACTION_PRESS_RELEASE + 1] = {
      "UNKNOWN", "CLICKED", "CLICK RELEASED", "PRESSED", "PRESS RELEASED"
    };
  
    // Determine the sensor state: true for press events, false otherwise.
    bool is_pressed = (evt->type == INPUT_KEY_SERVICE_ACTION_PRESS || evt->type == INPUT_KEY_SERVICE_ACTION_CLICK);
  
    switch ((int)evt->data) {
      case INPUT_KEY_USER_ID_REC:
        ESP_LOGI(TAG, "[ * ] [Rec] KEY %s", key_types[evt->type]);
        button->internal_btn_record->publish_state(is_pressed);
        break;
        
      case INPUT_KEY_USER_ID_SET:
        ESP_LOGI(TAG, "[ * ] [SET] KEY %s", key_types[evt->type]);
        button->internal_btn_set->publish_state(is_pressed);
        break;
        
      case INPUT_KEY_USER_ID_PLAY:
        ESP_LOGI(TAG, "[ * ] [Play] KEY %s", key_types[evt->type]);
        button->internal_btn_play->publish_state(is_pressed);
        break;
        
      case INPUT_KEY_USER_ID_MODE:
        ESP_LOGI(TAG, "[ * ] [MODE] KEY %s", key_types[evt->type]);
        button->internal_btn_mode->publish_state(is_pressed);
        break;
        
      case INPUT_KEY_USER_ID_VOLDOWN:
        ESP_LOGI(TAG, "[ * ] [Vol-] KEY %s", key_types[evt->type]);
        button->internal_btn_vol_down->publish_state(is_pressed);
        // Only trigger on press (true) to avoid double actions
        if (is_pressed) {
          button->volume_down();
        }
        break;
        
      case INPUT_KEY_USER_ID_VOLUP:
        ESP_LOGI(TAG, "[ * ] [Vol+] KEY %s", key_types[evt->type]);
        button->internal_btn_vol_up->publish_state(is_pressed);
        // Only trigger on press (true)
        if (is_pressed) {
          button->volume_up();
        }
        break;
        
      default:
        ESP_LOGE(TAG, "User Key ID[%d] does not support", (int)evt->data);
        break;
    }
    return ESP_OK;
 }

}  // namespace esp_adf
}  // namespace esphome

#endif  // USE_ESP_IDF
