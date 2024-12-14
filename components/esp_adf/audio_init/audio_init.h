#ifndef AUDIO_INIT_H
#define AUDIO_INIT_H

#include "audio_pipeline.h"
#include "audio_element.h"
#include "esp_err.h"

// Function prototypes
esp_err_t audio_init_pipeline(audio_pipeline_handle_t *pipeline);
esp_err_t audio_init_i2s_writer(audio_element_handle_t *i2s_writer);
esp_err_t audio_init_resample_filter(audio_element_handle_t *filter);

#endif // AUDIO_INIT_H
