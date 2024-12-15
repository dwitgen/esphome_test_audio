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
#include "esp_http_client.h"
#include "http_stream.h"
#include "audio_pipeline.h"
#include "mp3_decoder.h"

// Added include for board config to be used with button and other controls
#ifdef USE_ESP_ADF_BOARD
#include <board.h>
#endif

namespace esphome {
namespace esp_adf {

static const size_t BUFFER_COUNT = 50;
static const char *const TAG = "esp_adf.speaker";

esp_err_t configure_i2s_stream_writer_http(audio_element_handle_t *i2s_stream_writer) {
    i2s_driver_config_t i2s_config = {
        .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100,
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
        .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
    };

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

    *i2s_stream_writer = i2s_stream_init(&i2s_cfg);
    if (*i2s_stream_writer == NULL) {
        ESP_LOGE(TAG, "Failed to initialize I2S stream writer for HTTP");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "I2S stream writer for HTTP initialized");
    return ESP_OK;
}

esp_err_t configure_i2s_stream_writer_raw(audio_element_handle_t *i2s_stream_writer) {
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
        .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
    };

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

    *i2s_stream_writer = i2s_stream_init(&i2s_cfg);
    if (*i2s_stream_writer == NULL) {
        ESP_LOGE(TAG, "Failed to initialize I2S stream writer for raw");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "I2S stream writer for raw initialized");
    return ESP_OK;
}

// Function to configure resample filter
esp_err_t configure_resample_filter(audio_element_handle_t *filter) {
    rsp_filter_cfg_t rsp_cfg = {
        .src_rate = 44100,
        .src_ch = 2,
        .dest_rate = 44100,
        .dest_bits = 16,
        .dest_ch = 1,
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

    *filter = rsp_filter_init(&rsp_cfg);
    if (*filter == NULL) {
        ESP_LOGE(TAG, "Failed to initialize resample filter");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Resample filter initialized");
    return ESP_OK;
}

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

void ESPADFSpeaker::handle_mode_button() {
    ESP_LOGI(TAG, "Mde button action");
    // Add code for mode
}

void ESPADFSpeaker::handle_play_button() {
    switch (this->state_) {
        case speaker::STATE_RUNNING:
            ESP_LOGI(TAG, "Stopping playback...");
            this->cleanup_audio_pipeline();
            this->state_ = speaker::STATE_STOPPED;
            break;

        case speaker::STATE_STOPPED:
            if (this->url_.empty()) {
                ESP_LOGE(TAG, "No URL set to play!");
                return;
            }
            ESP_LOGI(TAG, "Starting playback...");
            ESP_LOGI(TAG, "Playing URL: %s", this->url_.c_str());
            this->play_url(this->url_);
            this->state_ = speaker::STATE_STARTING;
            break;

        default:
            ESP_LOGW(TAG, "Unexpected state: %d. Resetting to STOPPED.", this->state_);
            this->cleanup_audio_pipeline();
            this->state_ = speaker::STATE_STOPPED;
            break;
    }
}

void ESPADFSpeaker::handle_set_button() {
    ESP_LOGI(TAG, "Set button action");
    // Add code to handle set action
}

void ESPADFSpeaker::handle_rec_button() {
    ESP_LOGI(TAG, "Record button action");
    // Add code to start recording
}

void ESPADFSpeaker::initialize_audio_pipeline() {
    esp_err_t ret;

    ret = configure_resample_filter(&this->http_filter_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing resample filter: %s", esp_err_to_name(ret));
        return;
    }

    ret = configure_i2s_stream_writer_http(&this->i2s_stream_writer_http_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing I2S stream writer for HTTP: %s", esp_err_to_name(ret));
        return;
    }

    ret = configure_i2s_stream_writer_raw(&this->i2s_stream_writer_raw_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing I2S stream writer for raw: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Audio pipeline and elements initialized successfully");
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

  this->initialize_audio_pipeline();   
}

void ESPADFSpeaker::set_and_play_url(const std::string &url) {
    ESP_LOGI(TAG, "Received URL to play: %s", url.c_str());
    this->play_url(url);  // Reuse existing playback logic
}

void ESPADFSpeaker::set_dynamic_url(const std::string &url) {
    this->url_ = url;
    ESP_LOGI(TAG, "Updated URL: %s", url.c_str());
}

/*void ESPADFSpeaker::play_url(const std::string &url) {
 if (this->state_ == speaker::STATE_RUNNING || this->state_ == speaker::STATE_STARTING) {
     ESP_LOGI(TAG, "Audio stream is already running, ignoring play request");
     return;
 }*/

void ESPADFSpeaker::play_url(const std::string &url) {
    if (this->state_ == speaker::STATE_RUNNING || this->state_ == speaker::STATE_STARTING) {
        ESP_LOGI(TAG, "Audio stream is already running, ignoring play request");
        return;
    }
    ESP_LOGI(TAG, "Attempting to play URL: %s", url.c_str());

    // Cleanup any existing pipeline
    this->cleanup_audio_pipeline();

    #ifdef HTTP_STREAM_RINGBUFFER_SIZE
    #undef HTTP_STREAM_RINGBUFFER_SIZE
    #endif
    #define HTTP_STREAM_RINGBUFFER_SIZE (12 * 1024)

    // Ensure enough heap is available before proceeding
    uint32_t heap_before = esp_get_free_heap_size();
    if (heap_before < 50 * 1024) {  // Example threshold
        ESP_LOGE(TAG, "Insufficient heap memory: %u bytes available", heap_before);
        return;
    }

    // Configure HTTP stream
    ESP_LOGI(TAG, "Configuring HTTP stream");
    //http_stream_cfg_t http_cfg = HTTP_STREAM_CFG_DEFAULT();
   //http_cfg.cert_pem = NULL;  // Disable server certificate verification for testing
    http_stream_cfg_t http_cfg = {
        .type = AUDIO_STREAM_READER,
        .out_rb_size = HTTP_STREAM_RINGBUFFER_SIZE,  // Ring buffer size
        .task_stack = HTTP_STREAM_TASK_STACK,
        .task_core = HTTP_STREAM_TASK_CORE,
        .task_prio = HTTP_STREAM_TASK_PRIO,
        .stack_in_ext = false,
        .event_handle = NULL,
        .user_data = NULL,
        .auto_connect_next_track = false,
        .enable_playlist_parser = false,
        //.cert_pem = NULL,  // Disable certificate verification
        //.crt_bundle_attach = NULL,  // Do not use certificate bundle
       
    };

    this->http_stream_reader_ = http_stream_init(&http_cfg);
    if (this->http_stream_reader_ == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP stream reader");
        return;
    }
    audio_element_set_uri(this->http_stream_reader_, url.c_str());

    // Initialize MP3 decoder
    ESP_LOGI(TAG, "Initializing MP3 decoder");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    audio_element_handle_t mp3_decoder = mp3_decoder_init(&mp3_cfg);
    if (mp3_decoder == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MP3 decoder");
        return;
    }

    ESP_LOGI(TAG, "Heap before registering elements: %u bytes", esp_get_free_heap_size());
    // Initialize audio pipeline
    ESP_LOGI(TAG, "Initializing audio pipeline");
    audio_pipeline_cfg_t pipeline_cfg = {.rb_size = 8 * 1024};
    this->pipeline_ = audio_pipeline_init(&pipeline_cfg);
    if (this->pipeline_ == NULL) {
        ESP_LOGE(TAG, "Failed to initialize audio pipeline");
        return;
    }
    ESP_LOGI(TAG, "Heap before registering elements: %u bytes", esp_get_free_heap_size());
    // Register elements to pipeline
    /*if (audio_pipeline_register(this->pipeline_, this->http_stream_reader_, "http") != ESP_OK ||
        audio_pipeline_register(this->pipeline_, mp3_decoder, "mp3") != ESP_OK ||
        audio_pipeline_register(this->pipeline_, this->http_filter_, "filter") != ESP_OK ||
        audio_pipeline_register(this->pipeline_, this->i2s_stream_writer_http_, "i2s") != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register elements to pipeline");
        audio_pipeline_deinit(this->pipeline_);
        this->pipeline_ = nullptr;
        return;
    }*/
    if (audio_pipeline_register(this->pipeline_, this->http_stream_reader_, "http") != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register HTTP stream reader");
    return;
    }
    ESP_LOGI(TAG, "Heap after registering HTTP stream reader: %u bytes", esp_get_free_heap_size());
    
    if (audio_pipeline_register(this->pipeline_, mp3_decoder, "mp3") != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MP3 decoder");
        return;
    }
    ESP_LOGI(TAG, "Heap after registering MP3 decoder: %u bytes", esp_get_free_heap_size());
    
    if (audio_pipeline_register(this->pipeline_, this->http_filter_, "filter") != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register HTTP filter");
        return;
    }
    ESP_LOGI(TAG, "Heap after registering HTTP filter: %u bytes", esp_get_free_heap_size());
    
    if (audio_pipeline_register(this->pipeline_, this->i2s_stream_writer_http_, "i2s") != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register I2S stream writer");
        return;
    }
    ESP_LOGI(TAG, "Heap after registering I2S stream writer: %u bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Heap before registering elements: %u bytes", esp_get_free_heap_size());
    // Link elements in pipeline
    ESP_LOGI(TAG, "Linking pipeline elements");
    const char *link_tag[4] = {"http", "mp3", "filter", "i2s"};
    if (audio_pipeline_link(this->pipeline_, &link_tag[0], 4) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to link pipeline elements");
        audio_pipeline_deinit(this->pipeline_);
        this->pipeline_ = nullptr;
        return;
    }

    // Start the pipeline
    ESP_LOGI(TAG, "Starting the audio pipeline");
    gpio_set_level(PA_ENABLE_GPIO, 1);  // Enable amplifier
    if (audio_pipeline_run(this->pipeline_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start audio pipeline");
        audio_pipeline_deinit(this->pipeline_);
        this->pipeline_ = nullptr;
        return;
    }

    // Update state and log
    ESP_LOGI(TAG, "Audio pipeline started successfully for URL: %s", url.c_str());
    this->state_ = speaker::STATE_RUNNING;
}

void ESPADFSpeaker::cleanup_audio_pipeline() {
    TaskEvent event;
    event.type = TaskEventType::STOPPING;
    xQueueSend(this->event_queue_, &event, portMAX_DELAY);

    ESP_LOGI(TAG, "Transitioning state to STOPPED (state = %d)", this->state_);
    this->state_ = speaker::STATE_STOPPED;
    ESP_LOGI(TAG, "State updated to STOPPED (state = %d)", this->state_);

    event.type = TaskEventType::STOPPED;
    xQueueSend(this->event_queue_, &event, portMAX_DELAY);

    if (this->pipeline_ != nullptr) {
        ESP_LOGI(TAG, "Stopping current audio pipeline");

        esp_err_t err;

        ESP_LOGI(TAG, "Calling audio_pipeline_stop()");
        err = audio_pipeline_stop(this->pipeline_);
        ESP_LOGI(TAG, "audio_pipeline_stop() returned: %s", esp_err_to_name(err));

        ESP_LOGI(TAG, "Calling audio_pipeline_wait_for_stop()");
        err = audio_pipeline_wait_for_stop(this->pipeline_);
        ESP_LOGI(TAG, "audio_pipeline_wait_for_stop() returned: %s", esp_err_to_name(err));

        ESP_LOGI(TAG, "Calling audio_pipeline_terminate()");
        err = audio_pipeline_terminate(this->pipeline_);
        ESP_LOGI(TAG, "audio_pipeline_terminate() returned: %s", esp_err_to_name(err));

        ESP_LOGI(TAG, "Calling audio_pipeline_unregister() for i2s_stream_writer_http_");
        err = audio_pipeline_unregister(this->pipeline_, this->i2s_stream_writer_http_);
        ESP_LOGI(TAG, "audio_pipeline_unregister() for i2s_stream_writer_http_ returned: %s", esp_err_to_name(err));

        ESP_LOGI(TAG, "Calling audio_pipeline_unregister() for http_filter_");
        err = audio_pipeline_unregister(this->pipeline_, this->http_filter_);
        ESP_LOGI(TAG, "audio_pipeline_unregister() for http_filter_ returned: %s", esp_err_to_name(err));

        ESP_LOGI(TAG, "Calling audio_pipeline_unregister() for http_stream_reader_");
        err = audio_pipeline_unregister(this->pipeline_, this->http_stream_reader_);
        ESP_LOGI(TAG, "audio_pipeline_unregister() for http_stream_reader_ returned: %s", esp_err_to_name(err));

        ESP_LOGI(TAG, "Calling audio_pipeline_deinit()");
        err = audio_pipeline_deinit(this->pipeline_);
        ESP_LOGI(TAG, "audio_pipeline_deinit() returned: %s", esp_err_to_name(err));

        this->pipeline_ = nullptr;
    } else {
        ESP_LOGI(TAG, "No active pipeline to clean up");
    }

    ESP_LOGI(TAG, "Transitioning state to STOPPED (state = %d)", this->state_);
    this->state_ = speaker::STATE_STOPPED;
    ESP_LOGI(TAG, "State updated to STOPPED (state = %d)", this->state_);

    event.type = TaskEventType::STOPPED;
    xQueueSend(this->event_queue_, &event, portMAX_DELAY);
}


void ESPADFSpeaker::start() { this->state_ = speaker::STATE_STARTING; }
void ESPADFSpeaker::start_() {
  if (!this->parent_->try_lock()) {
    return;  // Waiting for another i2s component to return lock
  }
  xTaskCreate(ESPADFSpeaker::player_task, "speaker_task", 8192, (void *) this, 0, &this->player_task_handle_);
}
bool ESPADFSpeaker::is_running() const {
    // Check if the pipeline is valid and running
    if (this->pipeline_ != nullptr && audio_pipeline_state(this->pipeline_) == AUDIO_PIPELINE_STATE_RUNNING) {
        return true;
    }
    ESP_LOGI("ESPADFSpeaker", "Pipeline is not running; state might be STOPPED or STOPPING.");
    return false;
}

void ESPADFSpeaker::player_task(void *params) {
    TaskParams *task_params = static_cast<TaskParams *>(params);
    ESPADFSpeaker *this_speaker = task_params->speaker;
    const std::string &url = task_params->url;

    TaskEvent event;
    event.type = TaskEventType::STARTING;
    xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);

    // Configure I2S stream
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
        .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
    };

    // Initialize audio pipeline
    audio_pipeline_cfg_t pipeline_cfg = {.rb_size = 8 * 1024};
    this_speaker->pipeline_ = audio_pipeline_init(&pipeline_cfg);

    // Configure I2S stream writer
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
    };
    this_speaker->i2s_stream_writer_ = i2s_stream_init(&i2s_cfg);
    if (this_speaker->i2s_stream_writer_ == nullptr) {
        ESP_LOGE("ESPADFSpeaker", "Failed to initialize I2S stream writer");
        event.type = TaskEventType::WARNING;
        xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);
        return;
    }

    // Handle HTTP streaming if a URL is provided
    if (!url.empty()) {
        ESP_LOGI("ESPADFSpeaker", "Starting HTTP stream for URL: %s", url.c_str());

        // HTTP Stream configuration
        http_stream_cfg_t http_cfg = HTTP_STREAM_CFG_DEFAULT();
        http_cfg.type = AUDIO_STREAM_READER;
        this_speaker->http_stream_reader_ = http_stream_init(&http_cfg);

        if (this_speaker->http_stream_reader_ == nullptr) {
            ESP_LOGE("ESPADFSpeaker", "Failed to initialize HTTP stream reader");
            event.type = TaskEventType::WARNING;
            xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);
            return;
        }

        // Resample filter configuration
        rsp_filter_cfg_t rsp_cfg = {
            .src_rate = 44100,
            .src_ch = 2,
            .dest_rate = 16000,
            .dest_bits = 16,
            .dest_ch = 1,
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
        this_speaker->filter_ = rsp_filter_init(&rsp_cfg);

        if (this_speaker->filter_ == nullptr) {
            ESP_LOGE("ESPADFSpeaker", "Failed to initialize resample filter");
            event.type = TaskEventType::WARNING;
            xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);
            return;
        }

        // Register pipeline elements for HTTP stream
        audio_pipeline_register(this_speaker->pipeline_, this_speaker->http_stream_reader_, "http");
        audio_pipeline_register(this_speaker->pipeline_, this_speaker->filter_, "filter");
        audio_pipeline_register(this_speaker->pipeline_, this_speaker->i2s_stream_writer_, "i2s");

        const char *link_tag_http[3] = {"http", "filter", "i2s"};
        audio_pipeline_link(this_speaker->pipeline_, &link_tag_http[0], 3);
    } else {
        // Handle raw audio stream
        raw_stream_cfg_t raw_cfg = {.type = AUDIO_STREAM_WRITER, .out_rb_size = 8 * 1024};
        this_speaker->raw_write_ = raw_stream_init(&raw_cfg);

        if (this_speaker->raw_write_ == nullptr) {
            ESP_LOGE("ESPADFSpeaker", "Failed to initialize raw stream writer");
            event.type = TaskEventType::WARNING;
            xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);
            return;
        }

        // Register pipeline elements for raw stream
        audio_pipeline_register(this_speaker->pipeline_, this_speaker->raw_write_, "raw");
        audio_pipeline_register(this_speaker->pipeline_, this_speaker->i2s_stream_writer_, "i2s");

        const char *link_tag_raw[2] = {"raw", "i2s"};
        audio_pipeline_link(this_speaker->pipeline_, &link_tag_raw[0], 2);
    }

    audio_pipeline_run(this_speaker->pipeline_);

    // Notify task started
    event.type = TaskEventType::STARTED;
    xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);

    // Simulated processing loop
    while (true) {
        // Placeholder for stream management logic
        delay(10);
    }

    // Cleanup pipeline
    audio_pipeline_stop(this_speaker->pipeline_);
    audio_pipeline_wait_for_stop(this_speaker->pipeline_);
    audio_pipeline_terminate(this_speaker->pipeline_);
    audio_pipeline_deinit(this_speaker->pipeline_);

    if (!url.empty()) {
        // Deinitialize HTTP elements
        audio_element_deinit(this_speaker->http_stream_reader_);
        audio_element_deinit(this_speaker->filter_);
    } else {
        // Deinitialize raw elements
        audio_element_deinit(this_speaker->raw_write_);
    }
    audio_element_deinit(this_speaker->i2s_stream_writer_);

    event.type = TaskEventType::STOPPED;
    xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);
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
        ESP_LOGI(TAG, "Transitioning to STATE_STARTING");
        break;
      case TaskEventType::STARTED:
        ESP_LOGI(TAG, "Transitioning to STATE_RUNNING");
        this->state_ = speaker::STATE_RUNNING;
        break;
      case TaskEventType::RUNNING:
        this->status_clear_warning();
        break;
      case TaskEventType::STOPPING:
        ESP_LOGI(TAG, "Transitioning to STATE_STOPPING");
        break;
      case TaskEventType::STOPPED:
        ESP_LOGI(TAG, "Transitioning to STATE_STOPPED");
        this->state_ = speaker::STATE_STOPPED;
        this->parent_->unlock();
        vTaskDelete(this->player_task_handle_);
        this->player_task_handle_ = nullptr;
        break;
      case TaskEventType::WARNING:
        ESP_LOGW(TAG, "Pipeline warning: %s", esp_err_to_name(event.err));
        this->status_set_warning();
        break;
      default:
        ESP_LOGW(TAG, "Unknown task event type");
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
