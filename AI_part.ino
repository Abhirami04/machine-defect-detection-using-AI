#define EIDSP_QUANTIZE_FILTERBANK   0

/* Includes ---------------------------------------------------------------- */
#include <sound_detection_inferencing.h> // Replace with your Edge Impulse library
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"

/** Audio buffers, pointers and selectors */
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false; // Set to true to see features generated from raw signal
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
static bool record_status = true;

// GPIO control variables
#define FAULTY_THRESHOLD 0.9 // Threshold for "faulty" class probability (0.0 to 1.0)
#define FAULTY_MIN_HIGH_MS 5000 // Minimum time to keep D4 HIGH (5 seconds)
#define FAULTY_GPIO 2 // GPIO2 (D4 on ESP32 DOIT Kit)
static bool faulty_detected = false; // Tracks if faulty state is active
static unsigned long faulty_start_time = 0; // Timestamp when faulty state began

/**
 * @brief      Arduino setup function
 */
void setup()
{
    Serial.begin(115200);
    while (!Serial); // Wait for USB connection (optional for native USB)
    Serial.println("Edge Impulse Inferencing Demo with INMP441");

    // Initialize GPIO D4 (GPIO2) as output
    pinMode(FAULTY_GPIO, OUTPUT);
    digitalWrite(FAULTY_GPIO, LOW); // Start with GPIO LOW

    // Summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    run_classifier_init();
    ei_printf("\nStarting continuous inference in 2 seconds...\n");
    ei_sleep(2000);

    if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }

    ei_printf("Recording with INMP441...\n");
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void loop()
{
    bool m = microphone_inference_record();
    if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = &microphone_audio_signal_get_data; // Fixed typo (µ to &)
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", r);
        return;
    }

    if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)) {
        // Print predictions
        ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

        // Check for "faulty" class and control GPIO D4
        bool faulty_current = false;
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (strcmp(result.classification[ix].label, "Faulty") == 0) {
                if (result.classification[ix].value >= FAULTY_THRESHOLD) {
                    faulty_current = true;
                }
                break;
            }
        }

        unsigned long current_time = millis();

        if (faulty_current && !faulty_detected) {
            // Faulty condition detected
            faulty_detected = true;
            faulty_start_time = current_time;
            digitalWrite(FAULTY_GPIO, HIGH);
            ei_printf("Faulty detected, GPIO D4 HIGH\n");
        } else if (!faulty_current && faulty_detected) {
            // Check if minimum HIGH time has elapsed
            if (current_time - faulty_start_time >= FAULTY_MIN_HIGH_MS) {
                faulty_detected = false;
                digitalWrite(FAULTY_GPIO, LOW);
                ei_printf("Faulty cleared, GPIO D4 LOW\n");
            }
        }

        print_results = 0;
    }
}

/**
 * @brief      Callback for audio buffer processing
 */
static void audio_inference_callback(uint32_t n_bytes)
{
    for (int i = 0; i < n_bytes >> 1; i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];
        if (inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

/**
 * @brief      Task to capture I2S samples from INMP441
 */
static void capture_samples(void* arg)
{
    const int32_t i2s_bytes_to_read = (uint32_t)arg;
    size_t bytes_read = i2s_bytes_to_read;

    while (record_status) {
        /* Read data from I2S */
        i2s_read(I2S_NUM_1, (void*)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);

        if (bytes_read <= 0) {
            ei_printf("Error in I2S read: %d\n", bytes_read);
        } else {
            if (bytes_read < i2s_bytes_to_read) {
                ei_printf("Partial I2S read\n");
            }

            // Scale data for better audio amplitude
            for (int x = 0; x < i2s_bytes_to_read / 2; x++) {
                sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 8;
            }

            if (record_status) {
                audio_inference_callback(i2s_bytes_to_read);
            } else {
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief      Initialize inferencing struct and start I2S
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));
    if (inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));
    if (inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    if (i2s_init(EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start I2S!\n");
        return false;
    }

    ei_sleep(100);
    record_status = true;

    xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, (void*)sample_buffer_size, 10, NULL);
    return true;
}

/**
 * @brief      Wait for new data
 */
static bool microphone_inference_record(void)
{
    if (inference.buf_ready == 1) {
        ei_printf("Error: Sample buffer overrun. Decrease EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW\n");
        return false;
    }

    while (inference.buf_ready == 0) {
        delay(1);
    }

    inference.buf_ready = 0;
    return true;
}

/**
 * @brief      Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    return 0;
}

/**
 * @brief      Stop I2S and release buffers
 */
static void microphone_inference_end(void)
{
    i2s_deinit();
    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
}

/**
 * @brief      Initialize I2S for INMP441
 */
static int i2s_init(uint32_t sampling_rate)
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = sampling_rate, // Typically 16000 Hz for Edge Impulse
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // INMP441 L/R tied to GND
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 512,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = -1,
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = 26,    // SCK (Bit Clock)
        .ws_io_num = 32,     // WS (Word Select)
        .data_out_num = -1,  // Not used (no TX)
        .data_in_num = 33,   // SD (Data Input)
    };

    esp_err_t ret = 0;
    ret = i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);
    if (ret != ESP_OK) {
        ei_printf("Error in i2s_driver_install: %d\n", ret);
        return ret;
    }

    ret = i2s_set_pin(I2S_NUM_1, &pin_config);
    if (ret != ESP_OK) {
        ei_printf("Error in i2s_set_pin: %d\n", ret);
        return ret;
    }

    ret = i2s_zero_dma_buffer(I2S_NUM_1);
    if (ret != ESP_OK) {
        ei_printf("Error in initializing DMA buffer: %d\n", ret);
        return ret;
    }

    return ret;
}

/**
 * @brief      Deinitialize I2S
 */
static int i2s_deinit(void)
{
    i2s_driver_uninstall(I2S_NUM_1);
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif