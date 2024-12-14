#include "audio_init.h"
#include "esp_log.h"
#include "i2s_stream.h"
#include "filter_resample.h"

static const char *TAG = "audio_init";

// Function to configure I2S stream writer for HTTP
esp_err_t AudioInit::configure_i2s_stream_writer_http(audio_element_handle_t *i2s_stream_writer) {
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

// Function to configure I2S stream writer for raw
esp_err_t AudioInit::configure_i2s_stream_writer_raw(audio_element_handle_t *i2s_stream_writer) {
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
esp_err_t AudioInit::configure_resample_filter(audio_element_handle_t *filter) {
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
