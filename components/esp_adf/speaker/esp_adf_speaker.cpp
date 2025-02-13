#include "esp_adf_speaker.h"

#ifdef USE_ESP_IDF

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include "soc/soc_caps.h"
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <audio_hal.h>
#include <filter_resample.h>
#include <i2s_stream.h>
#include <raw_stream.h>
#include <esp_http_client.h>
#include <http_stream.h>
#include <audio_pipeline.h>
#include <mp3_decoder.h>
#include "esp_peripherals.h"
#include "periph_adc_button.h"
#include "input_key_service.h"
#ifdef __cplusplus
}
#endif

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

// Added include for board config to be used with button and other controls
#ifdef USE_ESP_ADF_BOARD
#include <board.h>
#endif

namespace esphome {
namespace esp_adf {

static const size_t BUFFER_COUNT = 50;
static const char *const TAG = "esp_adf.speaker";


bool ESPADFSpeaker::setup_adc_calibration(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = nullptr;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "ADC Calibration Successful (Curve Fitting)");
            calibrated = true;
        } else {
            ESP_LOGW(TAG, "ADC Calibration Not Supported or Skipped");
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "ADC Calibration Successful (Line Fitting)");
            calibrated = true;
        } else {
            ESP_LOGW(TAG, "ADC Calibration Not Supported or Skipped");
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC Calibration Scheme Created Successfully");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "ADC Calibration Scheme Creation Failed: %s (%d)", esp_err_to_name(ret), ret);
    }
    return calibrated;
}


void ESPADFSpeaker::process_button(int adc_value, int low_thresh, int high_thresh, const char* button_name, std::function<void()> on_press) {
    static std::map<std::string, bool> button_states;  // Track button states
    bool is_pressed = (adc_value >= low_thresh && adc_value <= high_thresh);
    bool was_pressed = button_states[button_name];

    
    if (is_pressed != was_pressed) {
        button_states[button_name] = is_pressed;
        ESP_LOGI(TAG, "Button %s %s", button_name, is_pressed ? "PRESSED" : "RELEASED");
        
        // Publish the state to Home Assistant via ESPHome
        if (strcmp(button_name, "VOL_UP") == 0 && internal_btn_vol_up)
            internal_btn_vol_up->publish_state(is_pressed);

        if (strcmp(button_name, "VOL_DOWN") == 0 && internal_btn_vol_down)
            internal_btn_vol_down->publish_state(is_pressed);

        if (strcmp(button_name, "SET") == 0 && internal_btn_set)
            internal_btn_set->publish_state(is_pressed);

        if (strcmp(button_name, "PLAY") == 0 && internal_btn_play)
            internal_btn_play->publish_state(is_pressed);

        if (strcmp(button_name, "MODE") == 0 && internal_btn_mode)
            internal_btn_mode->publish_state(is_pressed);

        if (strcmp(button_name, "REC") == 0 && internal_btn_record)
            internal_btn_record->publish_state(is_pressed);

        if (is_pressed && on_press) {
            on_press();  // Trigger action on button press
        }

        
    }
}

void ESPADFSpeaker::handle_buttons() {
    static uint32_t last_check_time = 0;
    const uint32_t debounce_interval = 50;  // 100 ms debounce interval

    // Check if debounce interval has passed
    if (millis() - last_check_time >= debounce_interval) {
        last_check_time = millis();  // Update the timestamp

        int adc_raw = 0;
        esp_err_t ret = adc_oneshot_read(adc1_handle, INPUT_BUTOP_ID, &adc_raw);
        
        if (ret == ESP_OK) {
            ESP_LOGV(TAG, "ADC Raw Value: %d", adc_raw);

            process_button(adc_raw, VOL_UP_THRESHOLD_LOW, VOL_UP_THRESHOLD_HIGH, "VOL_UP", [this]() { this->volume_up(); });
            process_button(adc_raw, VOL_DOWN_THRESHOLD_LOW, VOL_DOWN_THRESHOLD_HIGH, "VOL_DOWN", [this]() { this->volume_down(); });
            process_button(adc_raw, SET_THRESHOLD_LOW, SET_THRESHOLD_HIGH, "SET", []() { ESP_LOGI(TAG, "SET Button Pressed"); });
            process_button(adc_raw, PLAY_THRESHOLD_LOW, PLAY_THRESHOLD_HIGH, "PLAY", []() { ESP_LOGI(TAG, "PLAY Button Pressed"); });
            process_button(adc_raw, MODE_THRESHOLD_LOW, MODE_THRESHOLD_HIGH, "MODE", []() { ESP_LOGI(TAG, "MODE Button Pressed"); });
            process_button(adc_raw, REC_THRESHOLD_LOW, REC_THRESHOLD_HIGH, "REC", []() { ESP_LOGI(TAG, "REC Button Pressed"); });
        } else {
            ESP_LOGE(TAG, "ADC Read Failed: %s", esp_err_to_name(ret));
        }
    }
}


// Helper to configure I2S stream with dynamic sample rate
esp_err_t ESPADFSpeaker::configure_i2s_stream(audio_element_handle_t *i2s_stream, int sample_rate) {
    // Step 1: Configure the I2S driver
    i2s_stream_cfg_t i2s_cfg = {
        .type = AUDIO_STREAM_WRITER,               // Stream type: writer
        .transmit_mode = I2S_COMM_MODE_STD,        // Standard I2S mode
        .chan_cfg = {                              // Channel configuration
            .id = I2S_NUM_0,                       // I2S port
            .role = I2S_ROLE_MASTER,               // Master role
            .dma_desc_num = 8,                     // Number of DMA descriptors
            .dma_frame_num = 1024,                  // Number of frames per DMA descriptor
            .auto_clear = true,                    // Auto-clear DMA buffer
        },
        .std_cfg = {                               // Standard I2S configuration
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(static_cast<uint32_t>(sample_rate)),
            .slot_cfg = {                          // Slot configuration
                .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT, // 16-bit data width
                .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,  // Auto slot width
                .slot_mode = I2S_SLOT_MODE_MONO,            // Mono mode
                .slot_mask = I2S_STD_SLOT_LEFT,             // Use left slot
                .ws_width = 16,                             // WS width
                .ws_pol = false,                            // WS polarity
                .bit_shift = true,                          // MSB first
                //.msb_right = true,                          // MSB alignment
            },
        },
        .use_alc = false,                          // Automatic Level Control
        .volume = 9,                               // Initial volume
        .out_rb_size = I2S_STREAM_RINGBUFFER_SIZE, // Ring buffer size
        .task_stack = I2S_STREAM_TASK_STACK,       // Task stack size
        .task_core = 1,                            // Run on core 1
        .task_prio = I2S_STREAM_TASK_PRIO,         // Task priority
        .stack_in_ext = false,                     // Do not allocate stack in external memory
        .multi_out_num = 0,                        // Single output
        .uninstall_drv = true,                     // Uninstall driver on destruction
        .need_expand = false,                      // No data expansion needed
        .buffer_len = I2S_STREAM_BUF_SIZE,
    };
    
    // Step 2: Initialize the I2S stream
    *i2s_stream = i2s_stream_init(&i2s_cfg);
    if (*i2s_stream == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize I2S stream (Sample Rate: %d)", sample_rate);
        return ESP_FAIL;
    } 

    // Step 3: Log success and return
    ESP_LOGI(TAG, "I2S stream initialized (Sample Rate: %d)", sample_rate);
    this->i2s_installed_ = true;  // Mark I2S as installed
    return ESP_OK;
}

int ESPADFSpeaker::http_stream_event_handler(http_stream_event_msg_t *msg) {
    ESP_LOGI("HTTP_EVENT", "Event received: event_id=%d", msg->event_id);

    switch (msg->event_id) {
        case HTTP_STREAM_PRE_REQUEST:
            ESP_LOGI("HTTP_EVENT", "Preparing HTTP request...");
            break;
        case HTTP_STREAM_ON_REQUEST:
            ESP_LOGI("HTTP_EVENT", "Sending HTTP request...");
            break;
        case HTTP_STREAM_ON_RESPONSE:
            ESP_LOGI("HTTP_EVENT", "HTTP response received");
            break;
        case HTTP_STREAM_FINISH_TRACK:
            ESP_LOGI("HTTP_EVENT", "Finished track playback");
            break;
        default:
            ESP_LOGW("HTTP_EVENT", "Unhandled event: %d", msg->event_id);
            break;
    }
    return ESP_OK;
}

// Helper to configure http stream reader
esp_err_t ESPADFSpeaker::configure_http_stream_reader(audio_element_handle_t *reader) {

    #ifdef HTTP_STREAM_RINGBUFFER_SIZE
    #undef HTTP_STREAM_RINGBUFFER_SIZE
    #endif
    #define HTTP_STREAM_RINGBUFFER_SIZE (16 * 1024)

    // Configure HTTP stream reader
    http_stream_cfg_t http_cfg = {
        .type = AUDIO_STREAM_READER,
        .out_rb_size = HTTP_STREAM_RINGBUFFER_SIZE,  // Ring buffer size
        .task_stack = HTTP_STREAM_TASK_STACK,
        .task_core = HTTP_STREAM_TASK_CORE,
        .task_prio = HTTP_STREAM_TASK_PRIO,
        .stack_in_ext = false,
        .event_handle = NULL, //ESPADFSpeaker::http_stream_event_handler,
        .user_data = NULL,
        .auto_connect_next_track = false,
        .enable_playlist_parser = false,
        .cert_pem = NULL,  // Disable certificate verification
        .crt_bundle_attach = NULL,  // Do not use certificate bundle
       
    };

    *reader = http_stream_init(&http_cfg);
    if (*reader == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize HTTP stream reader");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "HTTP reader initialized");
    return ESP_OK;
}

// Helper to configure resample filter with dynamic rates
esp_err_t ESPADFSpeaker::configure_resample_filter(audio_element_handle_t *filter, int src_rate, int dest_rate, int dest_ch) {
    rsp_filter_cfg_t rsp_cfg = {
        .src_rate = src_rate,
        .src_ch = 2,
        .dest_rate = dest_rate,
        .dest_bits = 16,
        .dest_ch = dest_ch,
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
    if (*filter == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize resample filter (src_rate: %d, dest_rate: %d)", src_rate, dest_rate);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Resample filter initialized (src_rate: %d, dest_rate: %d)", src_rate, dest_rate);
    return ESP_OK;
}


// Define ADC configuration added for button controls, maybe not correct to have in speaker config
//#define ADC_WIDTH_BIT    ADC_WIDTH_BIT_12
//#define ADC_ATTEN        ADC_ATTEN_DB_12

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

audio_pipeline_handle_t ESPADFSpeaker::initialize_audio_pipeline(bool is_http_stream) {
    esp_err_t ret;

    // Configure resample filter
    int src_rate = is_http_stream ? 44100 : 44100;
    int dest_rate = is_http_stream ? 44100 : 16000;
    int dest_ch = 1;

    ret = configure_resample_filter(&this->http_filter_, src_rate, dest_rate, dest_ch);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing resample filter: %s", esp_err_to_name(ret));
        return nullptr;
    }

    // Configure I2S stream writer
    if (is_http_stream) {
        ret = configure_i2s_stream(&this->i2s_stream_writer_http_, 44100);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error initializing I2S stream writer for HTTP: %s", esp_err_to_name(ret));
            return nullptr;
        }
    } else {
        ret = configure_i2s_stream(&this->i2s_stream_writer_raw_, 16000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error initializing I2S stream writer for RAW: %s", esp_err_to_name(ret));
            return nullptr;
        }

        raw_stream_cfg_t raw_cfg = {
            .type = AUDIO_STREAM_WRITER,
            .out_rb_size = 8 * 1024,
        };
        this->raw_write_ = raw_stream_init(&raw_cfg);
        if (this->raw_write_ == nullptr) {
            ESP_LOGE(TAG, "Failed to initialize RAW write stream");
            return nullptr;
        }

    }

    // Initialize audio pipeline
    ESP_LOGD(TAG, "Initializing audio pipeline");
    audio_pipeline_cfg_t pipeline_cfg = {.rb_size = 8 * 1024};

    this->pipeline_ = audio_pipeline_init(&pipeline_cfg);
    if (this->pipeline_ == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize audio pipeline");
        return nullptr;
    }

    // Register components
    if (is_http_stream) {
        // Initialize MP3 decoder
        //ESP_LOGD(TAG, "Initializing MP3 decoder");
        mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
        audio_element_handle_t mp3_decoder = mp3_decoder_init(&mp3_cfg);
        if (mp3_decoder == nullptr) {
            ESP_LOGE(TAG, "Failed to initialize MP3 decoder");
            return nullptr;
        }
        if (audio_pipeline_register(this->pipeline_, this->http_stream_reader_, "http") != ESP_OK ||
            audio_pipeline_register(this->pipeline_, mp3_decoder, "mp3") != ESP_OK ||
            audio_pipeline_register(this->pipeline_, this->http_filter_, "filter") != ESP_OK ||
            audio_pipeline_register(this->pipeline_, this->i2s_stream_writer_http_, "i2s") != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register HTTP pipeline components");
            return nullptr;
        }
         // Link components
        const char *link_tag[4] = {"http", "mp3", "filter", "i2s"};
        if (audio_pipeline_link(this->pipeline_, link_tag, 4) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to link HTTP pipeline components");
            return nullptr;
        }
   } else {
        if (audio_pipeline_register(this->pipeline_, this->raw_write_, "raw") != ESP_OK ||
            audio_pipeline_register(this->pipeline_, this->i2s_stream_writer_raw_, "i2s") != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register RAW pipeline components");
            return nullptr;
        }
        // Link components
        const char *link_tag[2] = {"raw", "i2s"};
        if (audio_pipeline_link(this->pipeline_, link_tag, 2) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to link RAW pipeline components");
            return nullptr;
        }
    }


    ESP_LOGI(TAG, "Audio pipeline initialized successfully for %s stream",
             is_http_stream ? "HTTP" : "RAW");
    return this->pipeline_;
}

void ESPADFSpeaker::setup() {
    //esp_log_set_vprintf(vprintf);
    ESP_LOGCONFIG(TAG, "Setting up ESP ADF Speaker...");

    #ifdef USE_ESP_ADF_BOARD
    // Use the PA enable pin from board.h configuration trying to stop speaker popping with control of the PA during speaker operations
    gpio_num_t pa_enable_gpio = static_cast<gpio_num_t>(get_pa_enable_gpio());
    //int but_channel = INPUT_BUTOP_ID;
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

    //esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    //esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    // ✅ Let audio_board_key_init handle everything
    //esp_err_t ret = audio_board_key_init(set);
    //if (ret != ESP_OK) {
    //    ESP_LOGE(TAG, "Failed to initialize audio board keys");
    //    this->mark_failed();
    //    return;
    //} else {
    //    ESP_LOGE(TAG, "Audio board keys initialized successfully");
    //}

    //init_adc_buttons();
    ESP_LOGI(TAG, "Initializing ADC Buttons...");

    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = {
        .based_cfg = {
            .task_stack = ADC_BUTTON_STACK_SIZE, //4 * 1024, // INPUT_KEY_SERVICE_TASK_STACK_SIZE,
            .task_prio = ADC_BUTTON_TASK_PRIORITY, //10, //INPUT_KEY_SERVICE_TASK_PRIORITY,
            .task_core = ADC_BUTTON_TASK_CORE_ID, //INPUT_KEY_SERVICE_TASK_ON_CORE,
            .task_func = nullptr,
            .extern_stack = false,
            .service_start = nullptr,
            .service_stop = nullptr,
            .service_destroy = nullptr,
            .service_ioctl = nullptr,
            .service_name = nullptr,
            .user_data = nullptr
        },
        .handle = set
    };

    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);
    if (input_ser == NULL) {
        ESP_LOGE(TAG, "Failed to create Input Key Service");
    } else {
        ESP_LOGE(TAG, "Input Key Service created successfully");
    }
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);

    // Set the callback
    esp_err_t cb_status = periph_service_set_callback(input_ser, ESPADFSpeaker::input_key_service_cb, this);
    if (cb_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set input key callback");
    } else {
        ESP_LOGE(TAG, "Input Key callback registered successfully");
    }

    
    // ✅ Let audio_board_key_init handle everything
    esp_err_t ret = audio_board_key_init(set);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio board keys");
        this->mark_failed();
        return;
    } else {
        ESP_LOGE(TAG, "Audio board keys initialized successfully");
    }

    // Set initial volume
    this->set_volume(volume_); // Set initial volume to 50%

    // Read and set initial volume
    int initial_volume = this->get_current_volume();
    this->set_volume(initial_volume);

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

    //init_adc_buttons();
    //xTaskCreate(log_forwarding_task, "log_forwarding", 2048, NULL, 5, NULL);
}

//static void log_forwarding_task(void *params) {
//    while (1) {
//        ESP_LOGI("LOG_FORWARD", "🔄 Forwarding ADC logs to console...");
//        vTaskDelay(pdMS_TO_TICKS(2000));  // Log every 2 seconds
//    }
//}


void ESPADFSpeaker::set_and_play_url(const std::string &url) {
    ESP_LOGI(TAG, "Received URL to play: %s", url.c_str());
    this->play_url(url);  // Reuse existing playback logic
}

void ESPADFSpeaker::play_url(const std::string &url) {
    if (this->state_ == speaker::STATE_RUNNING || this->state_ == speaker::STATE_STARTING) {
        ESP_LOGI(TAG, "Audio stream is already running, ignoring play request");
        return;
    }

    ESP_LOGI(TAG, "Attempting to play URL: %s", url.c_str());
    TaskEvent event;
    event.type = TaskEventType::STARTING;
    xQueueSend(this->event_queue_, &event, portMAX_DELAY);

    // Cleanup any existing pipeline
    this->cleanup_audio_pipeline();

    esp_err_t ret = configure_http_stream_reader(&this->http_stream_reader_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing HTTP stream reader: %s", esp_err_to_name(ret));
        return;
    }
    audio_element_set_uri(this->http_stream_reader_, url.c_str());

    // Initialize new audio pipeline
    this->initialize_audio_pipeline(true); 
    
    // Start the pipeline
    ESP_LOGI(TAG, "Starting the audio pipeline");
    gpio_set_level(PA_ENABLE_GPIO, 1);  // Enable amplifier
    if (audio_pipeline_run(this->pipeline_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start audio pipeline");
        audio_pipeline_deinit(this->pipeline_);
        this->pipeline_ = nullptr;
        return;
    }
    ESP_LOGE(TAG, "Audio Task running on core: %d", xPortGetCoreID());
    // Update state and log
    ESP_LOGI(TAG, "Audio pipeline started successfully for URL: %s", url.c_str());
    //this->state_ = speaker::STATE_RUNNING;
    event.type = TaskEventType::STARTED;
    xQueueSend(this->event_queue_, &event, portMAX_DELAY);
}

esp_err_t ESPADFSpeaker::input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx) {
    const char *key_types[] = {"UNKNOWN", "CLICKED", "CLICK RELEASED", "PRESSED", "PRESS RELEASED"};
    ESP_LOGE("ADC_BTN", "Button Event - ID: %d, Type: %s", (int)evt->data, key_types[evt->type]);
    ESPADFSpeaker *speaker = static_cast<ESPADFSpeaker *>(ctx); // Ensure we have speaker instance
    if (!speaker) {
        ESP_LOGE("ADC_BTN", "Speaker context is NULL");
        return ESP_FAIL;
    }
    // ✅ Perform actions based on button pressed
    switch ((int)evt->data) {
        case INPUT_KEY_USER_ID_VOLUP:
            ESP_LOGE("ADC_BTN", "[Vol+] Button Pressed");
            speaker->volume_up();
            break;
        case INPUT_KEY_USER_ID_VOLDOWN:
            ESP_LOGE("ADC_BTN", "[Vol-] Button Pressed");
            speaker->volume_down();
            break;
        case INPUT_KEY_USER_ID_PLAY:
            ESP_LOGE("ADC_BTN", "[Play] Button Pressed");
            break;
        default:
            ESP_LOGE("ADC_BTN", "Unknown Button ID: %d", (int)evt->data);
            break;
    }

    return ESP_OK;
}



void ESPADFSpeaker::init_adc_buttons() {
    ESP_LOGI(TAG, "Initializing ADC Buttons...");

    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    input_cfg.based_cfg.task_stack = 4 * 1024;

    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);
    if (input_ser == NULL) {
        ESP_LOGE(TAG, "Failed to create Input Key Service");
    } else {
        ESP_LOGE(TAG, "Input Key Service created successfully");
    }
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);

    // Set the callback
    esp_err_t cb_status = periph_service_set_callback(input_ser, input_key_service_cb, this);
    if (cb_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set input key callback");
    } else {
        ESP_LOGE(TAG, "Input Key callback registered successfully");
    }

    
    // ✅ Let audio_board_key_init handle everything
    esp_err_t ret = audio_board_key_init(set);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio board keys");
        this->mark_failed();
        return;
    } else {
        ESP_LOGE(TAG, "Audio board keys initialized successfully");
    }

    ESP_LOGI(TAG, "Checking ADC button states...");
    adc_btn_list *btn_list = adc_btn_create_list(NULL, 0);  // Assuming we have button config elsewhere
    if (btn_list != NULL) {
        adc_btn_list *node = btn_list;
        while (node) {
            int voltage = adc_read(static_cast<adc_channel_t>(node->adc_info.adc_ch));

            ESP_LOGI(TAG, "Button Channel %d Voltage: %d", node->adc_info.adc_ch, voltage);
            node = node->next;
        }
        adc_btn_destroy_list(btn_list);
    } else {
        ESP_LOGW(TAG, "No ADC button list available.");
    }

    
}


void ESPADFSpeaker::cleanup_audio_pipeline() {
    if (this->pipeline_ != nullptr) {
        ESP_LOGI(TAG, "Stopping current audio pipeline");
        //init_adc_buttons();
        // Stop and terminate the pipeline
        audio_pipeline_stop(this->pipeline_);
        audio_pipeline_wait_for_stop(this->pipeline_);
        audio_pipeline_terminate(this->pipeline_);
        
        // Reset the ringbuffers
        ESP_LOGI(TAG, "Resetting pipeline ringbuffers and states");
        audio_pipeline_reset_ringbuffer(this->pipeline_);
        audio_pipeline_reset_items_state(this->pipeline_);

        // Unregister and deinitialize elements
        if (this->i2s_stream_writer_http_ != nullptr) {
            audio_pipeline_unregister(this->pipeline_, this->i2s_stream_writer_http_);
            audio_element_deinit(this->i2s_stream_writer_http_);
            this->i2s_stream_writer_http_ = nullptr;
            ESP_LOGI(TAG, "Unregistered and deinitialized HTTP I2S stream writer");
        }

        if (this->i2s_stream_writer_raw_ != nullptr) {
            audio_pipeline_unregister(this->pipeline_, this->i2s_stream_writer_raw_);
            audio_element_deinit(this->i2s_stream_writer_raw_);
            this->i2s_stream_writer_raw_ = nullptr;
            ESP_LOGI(TAG, "Unregistered and deinitialized RAW I2S stream writer");
        }

        if (this->raw_write_ != nullptr) {
            audio_pipeline_unregister(this->pipeline_, this->raw_write_);
            audio_element_deinit(this->raw_write_);
            this->raw_write_ = nullptr;
            ESP_LOGI(TAG, "Unregistered and deinitialized RAW stream writer");
        }

        if (this->http_filter_ != nullptr) {
            audio_pipeline_unregister(this->pipeline_, this->http_filter_);
            audio_element_deinit(this->http_filter_);
            this->http_filter_ = nullptr;
            ESP_LOGI(TAG, "Unregistered and deinitialized HTTP filter");
        }

        if (this->http_stream_reader_ != nullptr) {
            audio_pipeline_unregister(this->pipeline_, this->http_stream_reader_);
            audio_element_deinit(this->http_stream_reader_);
            this->http_stream_reader_ = nullptr;
            ESP_LOGI(TAG, "Unregistered and deinitialized HTTP stream reader");
        }

        // Deinitialize the audio pipeline
        audio_pipeline_deinit(this->pipeline_);
        this->pipeline_ = nullptr;
      
        ESP_LOGI(TAG, "Audio pipeline cleanup completed successfully");
    } else {
        ESP_LOGI(TAG, "Audio pipeline is already cleaned up");
    }
    this->state_ = speaker::STATE_STOPPED;
    int pa_state = gpio_get_level(PA_ENABLE_GPIO);
    if (pa_state == 1) {
        ESP_LOGI(TAG, "Disabling PA...");
        gpio_reset_pin(PA_ENABLE_GPIO);
        gpio_set_level(PA_ENABLE_GPIO, 0);  // Set GPIO LOW to disable PA
        ESP_LOGI(TAG, "PA disabled successfully");
    } else {
        ESP_LOGI(TAG, "PA was already disabled");
    }
}

void ESPADFSpeaker::start() { this->state_ = speaker::STATE_STARTING; }
void ESPADFSpeaker::start_() {
  if (!this->parent_->try_lock()) {
    return;  // Waiting for another i2s component to return lock
  }
  xTaskCreate(ESPADFSpeaker::player_task, "speaker_task", 8192, (void *) this, 0, &this->player_task_handle_);
  ESP_LOGE(TAG, "Audio Task running on core: %d", xPortGetCoreID());
}

void ESPADFSpeaker::player_task(void *params) {
    ESPADFSpeaker *this_speaker = (ESPADFSpeaker *)params;

    // Ensure enough heap is available before proceeding
    uint32_t heap_before = esp_get_free_heap_size();
    if (heap_before < 50 * 1024) {  // Example threshold
        ESP_LOGE(TAG, "Insufficient heap memory: %u bytes available", heap_before);
        return;
    }

    TaskEvent event;
    event.type = TaskEventType::STARTING;
    xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);
    ESP_LOGI(TAG, "Heap before initialize_audio_pipeline: %u bytes", esp_get_free_heap_size());
    // Initialize the audio pipeline for RAW stream
    this_speaker->initialize_audio_pipeline(false); // RAW stream initialization

    gpio_reset_pin(PA_ENABLE_GPIO);
    if (audio_pipeline_run(this_speaker->pipeline_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start audio pipeline");
        return;
    }
   
    DataEvent data_event;

    event.type = TaskEventType::STARTED;
    xQueueSend(this_speaker->event_queue_, &event, 0);
    gpio_set_level(PA_ENABLE_GPIO, 1);

    uint32_t last_received = millis();

    while (true) {
        if (xQueueReceive(this_speaker->buffer_queue_.handle, &data_event, 0) != pdTRUE) {
            if (millis() - last_received > 500) {
                break;
            } else {
                continue;
            }
        }
        if (data_event.stop) {
            while (xQueueReceive(this_speaker->buffer_queue_.handle, &data_event, 0) == pdTRUE) {
                break;
            }
        }

        size_t remaining = data_event.len;
        size_t current = 0;
        if (remaining > 0)
            last_received = millis();

        while (remaining > 0) {
            int bytes_written = raw_stream_write(this_speaker->raw_write_, (char *) data_event.data + current, remaining);
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

    // Signal that cleanup is starting
    event.type = TaskEventType::STOPPING;
    xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);
    ESP_LOGI(TAG, "Cleaning up audio pipeline after player_task...");
    this_speaker->cleanup_audio_pipeline();

    event.type = TaskEventType::STOPPED;
    xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);
    ESP_LOGI(TAG, "Player task cleanup completed.");
    
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
  //handle_buttons();  // Handle button inputs

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
