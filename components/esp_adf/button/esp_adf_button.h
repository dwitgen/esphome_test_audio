#pragma once

#include "esphome/core/component.h"
#include "esp_peripherals.h"
#include "periph_adc_button.h"
#include "input_key_service.h"

namespace esphome {
namespace esp_adf {

class ESPADFButton : public Component {
 public:
  void setup() override;
  static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx);
};

}  // namespace esp_adf
}  // namespace esphome
