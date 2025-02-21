#pragma once

#include "../esp_adf.h"

#ifdef USE_ESP_IDF

#include "../esp_adf.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <audio_hal.h>
#include "esp_peripherals.h"
#include "periph_adc_button.h"
#include "input_key_service.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#ifdef __cplusplus
}
#endif


#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace esp_adf {

class ESPADFButton : public Component {
 public:
  void setup() override;
  static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx);

  // Declare methods for volume control
  void set_volume(int volume);
  void volume_up();
  void volume_down();
  // Declare a method to get the current volume from the device
  int get_current_volume();

  binary_sensor::BinarySensor *btn_vol_up;
  binary_sensor::BinarySensor *btn_vol_down;
  binary_sensor::BinarySensor *btn_set;
  binary_sensor::BinarySensor *btn_play;
  binary_sensor::BinarySensor *btn_mode;
  binary_sensor::BinarySensor *btn_record;
  sensor::Sensor *volume_sensor = nullptr;

  

  //void set_speaker(ESPADFSpeaker *speaker) { this->speaker_ = speaker; }  // ✅ Store speaker reference
  
 private:
  int volume_ = 50;
  //ESPADFSpeaker *speaker_ = nullptr; 

 protected:
  audio_board_handle_t board_handle_ = nullptr;
};

}  // namespace esp_adf
}  // namespace esphome

#endif  // USE_ESP_IDF