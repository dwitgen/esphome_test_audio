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

     // Setter methods for buttons (updated from BinarySensor to Button)
    void set_btn_vol_up(button::Button *button) { btn_vol_up_ = button; }
    void set_btn_vol_down(button::Button *button) { btn_vol_down_ = button; }
    void set_btn_set(button::Button *button) { btn_set_ = button; }
    void set_btn_play(button::Button *button) { btn_play_ = button; }
    void set_btn_mode(button::Button *button) { btn_mode_ = button; }
    void set_btn_record(button::Button *button) { btn_record_ = button; }
    void set_volume_sensor(sensor::Sensor *sensor) { volume_sensor_ = sensor; }
    void set_esp_adf(esp_adf::ESPADF *adf) { esp_adf_ = adf; }

    //void add_on_press_btn_play(std::function<void()>&& callback) { btn_play_->add_on_press(std::move(callback)); }

    // Public binary sensor pointers
    //binary_sensor::BinarySensor *btn_vol_up_{nullptr};
    //binary_sensor::BinarySensor *btn_vol_down_{nullptr};
    //binary_sensor::BinarySensor *btn_set_{nullptr};
    //binary_sensor::BinarySensor *btn_play_{nullptr};
    //binary_sensor::BinarySensor *btn_mode_{nullptr};
    //binary_sensor::BinarySensor *btn_record_{nullptr};

    // Volume sensor pointer (renamed for consistency)
    //sensor::Sensor *volume_sensor_{nullptr};

    

  protected:
    audio_board_handle_t board_handle_{nullptr};
    ESPADF *esp_adf_{nullptr};
    int volume_{50};  // Moved from private to protected to match .cpp
   
    button::Button *btn_vol_up_{nullptr};
    button::Button *btn_vol_down_{nullptr};
    button::Button *btn_set_{nullptr};
    button::Button *btn_play_{nullptr};
    button::Button *btn_mode_{nullptr};
    button::Button *btn_record_{nullptr};
    sensor::Sensor *volume_sensor_{nullptr};
    

  
};

}  // namespace esp_adf
}  // namespace esphome

#endif  // USE_ESP_IDF