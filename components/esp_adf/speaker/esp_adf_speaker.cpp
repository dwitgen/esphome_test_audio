#include "esp_adf_speaker.h"

#ifdef USE_ESP_IDF

#include <driver/i2s.h>
// Added includes for button controls
#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <audio_hal.h>
#include <filter_resample.h>
#include <i2s_stream.h>
#include <raw_stream.h>

// Added include for board config to be used with button and other controls
#ifdef USE_ESP_ADF_BOARD
#include <board.h>
#endif

namespace esphome {
namespace esp_adf {

static const size_t BUFFER_COUNT = 50;
static const char *const TAG = "esp_adf.speaker";

// Define ADC configuration added for button controls, maybe not correct to have in speaker config
#define ADC_WIDTH_BIT    ADC_WIDTH_BIT_12
#define ADC_ATTEN        ADC_ATTEN_DB_12

//Volume controls for buttons again speaker may mot be the correct location for this
void ESPADFSpeaker::set_volume(int volume) {
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
int ESPADFSpeaker::get_current_volume() {
  
  int current_volume = 0;
  esp_err_t read_err = audio_hal_get_volume(board_handle_->audio_hal, &current_volume);
  if (read_err == ESP_OK) {
    ESP_LOGI(TAG, "Current device volume: %d", current_volume);
  } else {
    ESP_LOGE(TAG, "Error reading current volume: %s", esp_err_to_name(read_err));
  }

  return current_volume;
}
void ESPADFSpeaker::volume_up() {
    ESP_LOGI(TAG, "Volume up button pressed");
    int current_volume = this->get_current_volume();
    this->set_volume(current_volume + 10);
}

void ESPADFSpeaker::volume_down() {
    ESP_LOGI(TAG, "Volume down button pressed");
    int current_volume = this->get_current_volume();
    this->set_volume(current_volume - 10);
}

void ESPADFSpeaker::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ESP ADF Speaker...");

  #ifdef USE_ESP_ADF_BOARD
  // Use the PA enable pin from board.h configuration trying to stop speaker popping with control of the PA during speaker operations
  gpio_num_t pa_enable_gpio = static_cast<gpio_num_t>(get_pa_enable_gpio());
  int but_channel = INPUT_BUTOP_ID;
  #endif

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << PA_ENABLE_GPIO);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);
  gpio_set_level(PA_ENABLE_GPIO, 0);  // Ensure PA is off initially

  ExternalRAMAllocator<uint8_t> allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);

  this->buffer_queue_.storage = allocator.allocate(sizeof(StaticQueue_t) + (BUFFER_COUNT * sizeof(DataEvent)));
  if (this->buffer_queue_.storage == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate buffer queue!");
    this->mark_failed();
    return;
  }

  this->buffer_queue_.handle =
      xQueueCreateStatic(BUFFER_COUNT, sizeof(DataEvent), this->buffer_queue_.storage + sizeof(StaticQueue_t),
                         (StaticQueue_t *) (this->buffer_queue_.storage));

  this->event_queue_ = xQueueCreate(20, sizeof(TaskEvent));
  if (this->event_queue_ == nullptr) {
    ESP_LOGW(TAG, "Could not allocate event queue.");
    this->mark_failed();
    return;
  }

 //Adding intial setup for volume controls for the speaker
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

  if (this->volume_sensor == nullptr) {
    ESP_LOGE(TAG, "Failed to get internal generic volume sensor component");
  } else {
    ESP_LOGI(TAG, "Internal generic volume sensor initialized correctly");
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
  
  // Configure ADC for volume control
  adc1_config_width(ADC_WIDTH_BIT);
  adc1_config_channel_atten((adc1_channel_t)but_channel, ADC_ATTEN);
   
}

void ESPADFSpeaker::start() { this->state_ = speaker::STATE_STARTING; }
void ESPADFSpeaker::start_() {
  if (!this->parent_->try_lock()) {
    return;  // Waiting for another i2s component to return lock
  }
  xTaskCreate(ESPADFSpeaker::player_task, "speaker_task", 8192, (void *) this, 0, &this->player_task_handle_);
}

void ESPADFSpeaker::player_task(void *params) {
  ESPADFSpeaker *this_speaker = (ESPADFSpeaker *) params;

  TaskEvent event;
  event.type = TaskEventType::STARTING;
  xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);
  //#pragma GCC diagnostic push
  //#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  i2s_driver_config_t i2s_config = {
      .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM,
      .dma_buf_count = 8,
      .dma_buf_len = 1024,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
      .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT
  };
  //#pragma GCC diagnostic pop
  audio_pipeline_cfg_t pipeline_cfg = {
      .rb_size = 8 * 1024,
  };
  audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipeline_cfg);

  i2s_stream_cfg_t i2s_cfg = {
      .type = AUDIO_STREAM_WRITER,
      .i2s_config = i2s_config,
      .i2s_port = I2S_NUM_0,
      .use_alc = false,
      .volume = 0,
      .out_rb_size = I2S_STREAM_RINGBUFFER_SIZE,
      .task_stack = I2S_STREAM_TASK_STACK,
      .task_core = I2S_STREAM_TASK_CORE,
      .task_prio = I2S_STREAM_TASK_PRIO,
      .stack_in_ext = false,
      .multi_out_num = 0,
      .uninstall_drv = true,
      .need_expand = false,
      .expand_src_bits = I2S_BITS_PER_SAMPLE_16BIT,
  };
  audio_element_handle_t i2s_stream_writer = i2s_stream_init(&i2s_cfg);

  rsp_filter_cfg_t rsp_cfg = {
      .src_rate = 16000,
      .src_ch = 1,
      .dest_rate = 16000,
      .dest_bits = 16,
      .dest_ch = 2,
      .src_bits = 16,
      .mode = RESAMPLE_DECODE_MODE,
      .max_indata_bytes = RSP_FILTER_BUFFER_BYTE,
      .out_len_bytes = RSP_FILTER_BUFFER_BYTE,
      .type = ESP_RESAMPLE_TYPE_AUTO,
      .complexity = 2,
      .down_ch_idx = 0,
      .prefer_flag = ESP_RSP_PREFER_TYPE_SPEED,
      .out_rb_size = RSP_FILTER_RINGBUFFER_SIZE,
      .task_stack = RSP_FILTER_TASK_STACK,
      .task_core = RSP_FILTER_TASK_CORE,
      .task_prio = RSP_FILTER_TASK_PRIO,
      .stack_in_ext = true,
  };
  audio_element_handle_t filter = rsp_filter_init(&rsp_cfg);

  raw_stream_cfg_t raw_cfg = {
      .type = AUDIO_STREAM_WRITER,
      .out_rb_size = 8 * 1024,
  };
  audio_element_handle_t raw_write = raw_stream_init(&raw_cfg);

  audio_pipeline_register(pipeline, raw_write, "raw");
  audio_pipeline_register(pipeline, filter, "filter");
  audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

  const char *link_tag[3] = {
      "raw",
      // "filter",
      "i2s",
  };
  audio_pipeline_link(pipeline, &link_tag[0], 2);

  audio_pipeline_run(pipeline);

  DataEvent data_event;

  event.type = TaskEventType::STARTED;
  xQueueSend(this_speaker->event_queue_, &event, 0);
  // Enable PA just after started event to try and stop popping
  gpio_set_level(PA_ENABLE_GPIO, 1);  

  uint32_t last_received = millis();

  while (true) {
    if (xQueueReceive(this_speaker->buffer_queue_.handle, &data_event, 0) != pdTRUE) {
      if (millis() - last_received > 500) {
        // No audio for 500ms, stop
        break;
      } else {
        continue;
      }
    }
    if (data_event.stop) {
      // Stop signal from main thread
      while (xQueueReceive(this_speaker->buffer_queue_.handle, &data_event, 0) == pdTRUE) {
        // Flush queue
      }
      break;
    }

    size_t remaining = data_event.len;
    size_t current = 0;
    if (remaining > 0)
      last_received = millis();

    while (remaining > 0) {
      int bytes_written = raw_stream_write(raw_write, (char *) data_event.data + current, remaining);
      if (bytes_written == ESP_FAIL) {
        event = {.type = TaskEventType::WARNING, .err = ESP_FAIL};
        xQueueSend(this_speaker->event_queue_, &event, 0);
        continue;
      }

      remaining -= bytes_written;
      current += bytes_written;
    }

    event.type = TaskEventType::RUNNING;
    xQueueSend(this_speaker->event_queue_, &event, 0);
  }

  audio_pipeline_stop(pipeline);
  audio_pipeline_wait_for_stop(pipeline);
  audio_pipeline_terminate(pipeline);

  event.type = TaskEventType::STOPPING;
  xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);

  audio_pipeline_unregister(pipeline, i2s_stream_writer);
  audio_pipeline_unregister(pipeline, filter);
  audio_pipeline_unregister(pipeline, raw_write);

  audio_pipeline_deinit(pipeline);
  audio_element_deinit(i2s_stream_writer);
  audio_element_deinit(filter);
  audio_element_deinit(raw_write);

  event.type = TaskEventType::STOPPED;
  xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);
  // Disable PA just after the stopped event, doing this to prevent popping in speaker
  gpio_set_level(PA_ENABLE_GPIO, 0);  

  while (true) {
    delay(10);
  }
}

void ESPADFSpeaker::stop() {
  if (this->state_ == speaker::STATE_STOPPED)
    return;
  if (this->state_ == speaker::STATE_STARTING) {
    this->state_ = speaker::STATE_STOPPED;
    return;
  }
  this->state_ = speaker::STATE_STOPPING;
  DataEvent data;
  data.stop = true;
  xQueueSendToFront(this->buffer_queue_.handle, &data, portMAX_DELAY);
}

void ESPADFSpeaker::watch_() {
  TaskEvent event;
  if (xQueueReceive(this->event_queue_, &event, 0) == pdTRUE) {
    switch (event.type) {
      case TaskEventType::STARTING:
      case TaskEventType::STOPPING:
        break;
      case TaskEventType::STARTED:
        this->state_ = speaker::STATE_RUNNING;
        break;
      case TaskEventType::RUNNING:
        this->status_clear_warning();
        break;
      case TaskEventType::STOPPED:
        this->parent_->unlock();
        this->state_ = speaker::STATE_STOPPED;
        vTaskDelete(this->player_task_handle_);
        this->player_task_handle_ = nullptr;
        break;
      case TaskEventType::WARNING:
        ESP_LOGW(TAG, "Error writing to pipeline: %s", esp_err_to_name(event.err));
        this->status_set_warning();
        break;
    }
  }
}

void ESPADFSpeaker::loop() {
  this->watch_();
  switch (this->state_) {
    case speaker::STATE_STARTING:
      this->start_();
      break;
    case speaker::STATE_RUNNING:
    case speaker::STATE_STOPPING:
    case speaker::STATE_STOPPED:
      break;
  }
}

size_t ESPADFSpeaker::play(const uint8_t *data, size_t length) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Failed to play audio, speaker is in failed state.");
    return 0;
  }
  if (this->state_ != speaker::STATE_RUNNING && this->state_ != speaker::STATE_STARTING) {
    this->start();
  }
  size_t remaining = length;
  size_t index = 0;
  while (remaining > 0) {
    DataEvent event;
    event.stop = false;
    size_t to_send_length = std::min(remaining, BUFFER_SIZE);
    event.len = to_send_length;
    memcpy(event.data, data + index, to_send_length);
    if (xQueueSend(this->buffer_queue_.handle, &event, 0) != pdTRUE) {
      return index;  // Queue full
    }
    remaining -= to_send_length;
    index += to_send_length;
  }
  return index;
}

bool ESPADFSpeaker::has_buffered_data() const { return uxQueueMessagesWaiting(this->buffer_queue_.handle) > 0; }

}  // namespace esp_adf
}  // namespace esphome

#endif  // USE_ESP_IDF
