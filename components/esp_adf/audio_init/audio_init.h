#ifndef AUDIO_INIT_H
#define AUDIO_INIT_H

#include "audio_pipeline.h"
#include "audio_element.h"
#include "esp_err.h"

class AudioInit {
public:
    esp_err_t configure_i2s_stream_writer_http(audio_element_handle_t *i2s_stream_writer);
    esp_err_t configure_i2s_stream_writer_raw(audio_element_handle_t *i2s_stream_writer);
    esp_err_t configure_resample_filter(audio_element_handle_t *filter);
};

#endif // AUDIO_INIT_H
