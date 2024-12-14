#include "audio_init.h"
#include "esp_log.h"
#include "i2s_stream.h"
#include "filter_resample.h"

static const char *TAG = "audio_init";

esp_err_t audio_init_pipeline(audio_pipeline_handle_t *pipeline) {
    audio_pipeline_cfg_t cfg = AUDIO_PIPELINE_CONFIG_DEFAULT();
    *pipeline = audio_pipeline_init(&cfg);
    if (*pipeline == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize audio pipeline");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Audio pipeline initialized");
    return ESP_OK;
}

esp_err_t audio_init_i2s_writer(audio_element_handle_t *i2s_writer) {
    i2s_stream_cfg_t i2s_cfg = {
        .type = AUDIO_STREAM_WRITER,
        .i2s_config = {
            .mode = I2S_MODE_MASTER | I2S_MODE_TX,
            .sample_rate = 44100,
            .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
            .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
            .communication_format = I2S_COMM_FORMAT_STAND_I2S,
            .dma_buf_count = 8,
            .dma_buf_len = 1024,
        },
        .i2s_port = I2S_NUM_0,
    };
    *i2s_writer = i2s_stream_init(&i2s_cfg);
    if (*i2s_writer == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize I2S writer");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "I2S writer initialized");
    return ESP_OK;
}

esp_err_t audio_init_resample_filter(audio_element_handle_t *filter) {
    rsp_filter_cfg_t rsp_cfg = {
        .src_rate = 44100,
        .src_ch = 2,
        .dest_rate = 16000,
        .dest_ch = 1,
        .mode = RESAMPLE_DECODE_MODE,
        .max_indata_bytes = 2048,
        .out_len_bytes = 2048,
    };
    *filter = rsp_filter_init(&rsp_cfg);
    if (*filter == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize resample filter");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Resample filter initialized");
    return ESP_OK;
}
