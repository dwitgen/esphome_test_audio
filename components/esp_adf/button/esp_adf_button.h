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

  // Setter methods for binary sensors
  void set_btn_vol_up(binary_sensor::BinarySensor *sensor) { btn_vol_up_ = sensor; }
  void set_btn_vol_down(binary_sensor::BinarySensor *sensor) { btn_vol_down_ = sensor; }
  void set_btn_set(binary_sensor::BinarySensor *sensor) { btn_set_ = sensor; }
  void set_btn_play(binary_sensor::BinarySensor *sensor) { btn_play_ = sensor; }
  void set_btn_mode(binary_sensor::BinarySensor *sensor) { btn_mode_ = sensor; }
  void set_btn_record(binary_sensor::BinarySensor *sensor) { btn_record_ = sensor; }

 // Public binary sensor pointers
 binary_sensor::BinarySensor *btn_vol_up_{nullptr};
 binary_sensor::BinarySensor *btn_vol_down_{nullptr};
 binary_sensor::BinarySensor *btn_set_{nullptr};
 binary_sensor::BinarySensor *btn_play_{nullptr};
 binary_sensor::BinarySensor *btn_mode_{nullptr};
 binary_sensor::BinarySensor *btn_record_{nullptr};

 // Volume sensor
 sensor::Sensor *volume_sensor{nullptr};

 // Setter for ESPADF
 void set_esp_adf(ESPADF *adf) { esp_adf_ = adf; }

protected:
 audio_board_handle_t board_handle_{nullptr};
 ESPADF *esp_adf_{nullptr};
 int volume_{50};  // Moved from private to protected to match .cpp

  
};

}  // namespace esp_adf
}  // namespace esphome

#endif  // USE_ESP_IDF