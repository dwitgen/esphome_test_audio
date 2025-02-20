#pragma once

#include "../esp_adf.h"

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "esp_peripherals.h"
#include "periph_adc_button.h"
#include "input_key_service.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
//#include "../speaker/esp_adf_speaker.h"


#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
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

  //void set_speaker(ESPADFSpeaker *speaker) { this->speaker_ = speaker; }  // ✅ Store speaker reference
  
 //private:
  //ESPADFSpeaker *speaker_ = nullptr; 
};

}  // namespace esp_adf
}  // namespace esphome
