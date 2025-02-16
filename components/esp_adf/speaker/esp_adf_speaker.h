#pragma once

#ifdef USE_ESP_IDF

#include "../esp_adf.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <audio_element.h>
#include <audio_pipeline.h>
#include <audio_hal.h>
#include "esp_peripherals.h"
#include <board.h>
#include <http_stream.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <esp_event.h>

#ifdef __cplusplus
}
#endif

#include "esphome/components/speaker/speaker.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace esp_adf {

class ESPADFSpeaker : public ESPADFPipeline, public speaker::Speaker, public Component {
 public:
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }

  void setup() override;
  void loop() override;

  void start() override;
  void stop() override;

  size_t play(const uint8_t *data, size_t length) override;

  bool has_buffered_data() const override;

  // Declare methods for volume control
  void set_volume(int volume);
  void volume_up();
  void volume_down();
  // Declare a method to get the current volume from the device
  int get_current_volume();
  //void update_playback_state(int state);
  void update_playback_state(const char *state);

  binary_sensor::BinarySensor *internal_btn_vol_up;
  binary_sensor::BinarySensor *internal_btn_vol_down;
  binary_sensor::BinarySensor *internal_btn_set;
  binary_sensor::BinarySensor *internal_btn_play;
  binary_sensor::BinarySensor *internal_btn_mode;
  binary_sensor::BinarySensor *internal_btn_record;

  static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
  void handle_set_button();
  void handle_play_button();
  void handle_mode_button();
  void handle_rec_button();
  void handle_adc_button(int adc_value);
  void play_url(const std::string &url); 
  void set_and_play_url(const std::string &url);
  void set_dynamic_url(const std::string &url);

  void setup_adc();
  void handle_buttons();
  void process_button(int adc_value, int low_thresh, int high_thresh, const char* button_name, std::function<void()> on_press);
  void read_adc();
  bool setup_adc_calibration(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
  void init_adc_buttons();

  struct TaskParams {
    ESPADFSpeaker *speaker;  // Pointer to the ESPADFSpeaker instance
    std::string url;         // Optional HTTP URL for streaming
  };

  // Declare a sensor for volume level
  sensor::Sensor *volume_sensor = nullptr;
  sensor::Sensor *playback_sensor = nullptr;
  text_sensor::TextSensor *playback_state_text_sensor = nullptr;

  // Method to initialize pipeline and cleanup
  audio_pipeline_handle_t initialize_audio_pipeline(bool is_http_stream);
  void cleanup_audio_pipeline();
  static void log_forwarding_task(void *params);
  
 protected:
  void start_();
  void watch_();
  
  static void player_task(void *params);
  audio_board_handle_t board_handle_ = nullptr;
  static void button_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
  void handle_button_event(int32_t id, int32_t event_type);

  TaskHandle_t player_task_handle_{nullptr};
   struct {
     QueueHandle_t handle;
     uint8_t *storage;
   } buffer_queue_;
   QueueHandle_t event_queue_;

  adc_oneshot_unit_handle_t adc1_handle;
  adc_cali_handle_t adc1_cali_handle;
  bool adc_calibrated = false;

  
 private:
   int volume_ = 50;  // Default volume level
   bool is_http_stream_;
   audio_pipeline_handle_t pipeline_;
   audio_element_handle_t i2s_stream_writer_;
   audio_element_handle_t i2s_stream_writer_http_;
   audio_element_handle_t i2s_stream_writer_raw_;
   audio_element_handle_t filter_;
   audio_element_handle_t http_filter_;
   audio_element_handle_t raw_write_;
   audio_element_handle_t http_stream_reader_;
   std::string url_;
   bool i2s_installed_ = false;
   static int http_stream_event_handler(http_stream_event_msg_t *msg);
   int detected_sample_rate_ = 44100;  // Default sample rate
   int detected_channels_ = 2;         // Default channel count
   audio_element_handle_t mp3_decoder_ = nullptr; // MP3 decoder handle

   // New private helper methods for modularization
   esp_err_t configure_i2s_stream(audio_element_handle_t *i2s_stream, int sample_rate);
   esp_err_t configure_resample_filter(audio_element_handle_t *filter, int src_rate, int dest_rate, int dest_ch);
   esp_err_t configure_http_stream_reader(audio_element_handle_t *reader);
   bool check_heap_memory(uint32_t threshold);
   bool init_pipeline(size_t rb_size);
   bool register_pipeline_elements(const std::vector<std::pair<std::string, audio_element_handle_t>> &elements);
   bool link_pipeline_elements(const std::vector<std::string> &link_tags);
   void start_pipeline();
};


}  // namespace esp_adf
}  // namespace esphome

#endif  // USE_ESP_IDF
