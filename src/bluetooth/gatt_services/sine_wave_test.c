#include "sine_wave_test.h"
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>
#include <data_fifo.h>
#include "arm_math.h"

#include "../../SensorManager/SensorManager.h"
#include "audio_datapath.h"
#include "decimation_filter.h"
LOG_MODULE_REGISTER(sine_wave_test, CONFIG_LOG_DEFAULT_LEVEL);

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Configuration - exactly matching seal check service
#define SINE_WAVE_SAMPLE_RATE 4000          // 4kHz (decimator output at rate 12)
#define SINE_WAVE_DURATION_MS 512           // Duration in milliseconds
#define NUM_SINE_WAVE_SAMPLES 2048          // (SINE_WAVE_SAMPLE_RATE * SINE_WAVE_DURATION_MS / 1000)
                                            // = 4000 * 512 / 1000 = 2048 samples
                                            // Actual duration: 512ms (0.512 seconds)
#define INITIAL_SINE_WAVE_DROP 128          // Same as INITIAL_SEAL_CHECK_DROP
#define SINE_WAVE_FREQUENCY 1000.0f         // 1kHz sine wave
#define SINE_WAVE_AMPLITUDE 0.9f            // 50% volume

// Buffers - same size as seal check service
static int16_t sine_wave_buffer[NUM_SINE_WAVE_SAMPLES];      // Generation buffer
static int16_t sine_wave_recording[NUM_SINE_WAVE_SAMPLES];   // Recording buffer

// Phase tracking
static float generated_phase_degrees = 0.0f;  // Phase used during generation
static float detected_phase_degrees = 0.0f;   // Phase detected in recording

// External references
extern struct data_fifo fifo_rx;
extern int hw_codec_volume_set(uint8_t volume);

// Forward declaration of callback
static void sine_wave_recording_callback(void);

// Work item for sine wave test completion
static struct k_work sine_wave_complete_work;

/**
 * @brief Generates a 1kHz sine wave buffer with specified initial phase
 * 
 * This function creates a buffer filled with samples representing
 * a 1kHz sine wave at 50% of maximum amplitude with a given starting phase.
 * 
 * @param initial_phase_degrees Initial phase in degrees (0-360)
 */
static void generate_1khz_sine_wave(float initial_phase_degrees)
{
    const float two_pi = 2.0f * M_PI;
    const float freq_normalized = SINE_WAVE_FREQUENCY / (float)SINE_WAVE_SAMPLE_RATE;
    const float initial_phase_rad = initial_phase_degrees * M_PI / 180.0f;
    
    // Store the phase for later comparison
    generated_phase_degrees = initial_phase_degrees;
    
    for (int i = 0; i < NUM_SINE_WAVE_SAMPLES; i++) {
        // Generate sine wave: sin(2π * f * t + φ₀)
        float phase = two_pi * freq_normalized * (float)i + initial_phase_rad;
        float sample = SINE_WAVE_AMPLITUDE * sinf(phase);
        
        // Convert to Q15 format (16-bit signed integer, range: -32768 to 32767)
        sine_wave_buffer[i] = (int16_t)(sample * 32767.0f);
    }
    
    // LOG_INF("Generated 1kHz sine wave: %d samples at %.1f%% amplitude", 
    //         NUM_SINE_WAVE_SAMPLES, (double)(SINE_WAVE_AMPLITUDE * 100.0f));
    // LOG_INF("Duration: %d ms (%.3f seconds)", 
    //         SINE_WAVE_DURATION_MS, (double)(SINE_WAVE_DURATION_MS / 1000.0f));
    // LOG_INF("Initial Phase: %.2f degrees (%.4f radians)", 
    //         (double)initial_phase_degrees, (double)initial_phase_rad);
    // LOG_INF("First 5 samples: %d, %d, %d, %d, %d",
    //         sine_wave_buffer[0], sine_wave_buffer[1], sine_wave_buffer[2],
    //         sine_wave_buffer[3], sine_wave_buffer[4]);
}

/**
 * @brief Callback function called when sine wave recording is complete
 */
static void sine_wave_recording_callback(void)
{
    k_work_submit(&sine_wave_complete_work);
}

/**
 * @brief Detect the phase of the recorded sine wave
 * 
 * Uses cross-correlation with reference sine/cosine to determine phase.
 * The detected phase represents the initial phase of the recorded signal.
 * 
 * @return Detected phase in degrees (0-360)
 */
static float detect_phase_from_recording(void)
{
    const float two_pi = 2.0f * M_PI;
    const float freq_normalized = SINE_WAVE_FREQUENCY / (float)SINE_WAVE_SAMPLE_RATE;
    
    // Use first few cycles for phase detection (avoid edge effects)
    const int samples_to_use = 400;  // About 100ms = 100 cycles at 1kHz
    
    // Calculate correlation with sine and cosine references
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    
    for (int i = 0; i < samples_to_use && i < NUM_SINE_WAVE_SAMPLES; i++) {
        float phase = two_pi * freq_normalized * (float)i;
        float normalized_sample = (float)sine_wave_recording[i] / 32767.0f;
        
        // Correlate with sin and cos references
        sum_sin += normalized_sample * sinf(phase);
        sum_cos += normalized_sample * cosf(phase);
    }
    
    // Phase is arctan2(sin_correlation, cos_correlation)
    float phase_rad = atan2f(sum_sin, sum_cos);
    float phase_deg = phase_rad * 180.0f / M_PI;
    
    // Normalize to 0-360 range
    if (phase_deg < 0) {
        phase_deg += 360.0f;
    }
    
    // Store detected phase
    detected_phase_degrees = phase_deg;
    
    // LOG_INF("Phase Detection Results:");
    // LOG_INF("  - Sum(sin correlation): %.4f", (double)sum_sin);
    // LOG_INF("  - Sum(cos correlation): %.4f", (double)sum_cos);
    LOG_INF("  - Detected Phase: %.2f degrees ", (double)phase_deg);
    LOG_INF("  - Generated Phase: %.2f degrees", (double)generated_phase_degrees);
    if(180.0f-phase_deg<30.0f)
    {
        LOG_INF("The speaker's polarity is inverted.");
    }
    else
    {
        LOG_INF("The speaker's polarity is non-inverted.");
    }
    return phase_deg;
}

/**
 * @brief Work handler for sine wave test completion
 */
static void sine_wave_complete_work_handler(struct k_work *work)
{
    LOG_INF("=== Sine wave test completed ===");
    
    // Stop audio playback
    audio_datapath_buffer_stop();
    
    // Release audio datapath
    audio_datapath_release();
    
    // Print first few recorded samples for debugging
    // LOG_INF("First 10 recorded samples:");
    // LOG_INF("  [0-4]:  %d, %d, %d, %d, %d",
    //         sine_wave_recording[0], sine_wave_recording[1], sine_wave_recording[2],
    //         sine_wave_recording[3], sine_wave_recording[4]);
    // LOG_INF("  [5-9]:  %d, %d, %d, %d, %d",
    //         sine_wave_recording[5], sine_wave_recording[6], sine_wave_recording[7],
    //         sine_wave_recording[8], sine_wave_recording[9]);
    
    // Calculate RMS to verify recording level
    float sum_squares = 0.0f;
    for (int i = 0; i < NUM_SINE_WAVE_SAMPLES; i++) {
        float normalized = (float)sine_wave_recording[i] / 32767.0f;
        sum_squares += normalized * normalized;
    }
    float rms = sqrtf(sum_squares / NUM_SINE_WAVE_SAMPLES);
    
    // LOG_INF("Recorded signal RMS: %.4f (expected ~%.4f for 50%% sine)", 
    //         (double)rms, (double)(SINE_WAVE_AMPLITUDE / sqrtf(2.0f)));
    
    // Detect phase from recorded signal
    detect_phase_from_recording();
    
    LOG_INF("=== Test Analysis Complete ===");
}

/**
 * @brief Main function to generate and record a 1kHz sine wave at 50% volume
 * 
 * This function:
 * 1. Initializes the audio FIFO if needed
 * 2. Sets volume to 50% (0x80 out of 0xFF)
 * 3. Generates a 1kHz sine wave at 50% amplitude with specified phase
 * 4. Starts playback of the sine wave
 * 5. Records the output simultaneously
 * 6. Detects phase after recording
 * 
 * Follows the exact same pattern as seal_check_service.c
 * 
 * @param initial_phase_degrees Initial phase for sine wave generation (0-360 degrees)
 * @return 0 on success, negative error code on failure
 */
int sine_wave_test_start_with_phase(float initial_phase_degrees)
{
    int ret;
    
    LOG_INF("=== Starting 1kHz sine wave test ===");
    // LOG_INF("Volume: 50%%, Phase: %.2f degrees", (double)initial_phase_degrees);
    
    // Step 1: Initialize FIFO if not already initialized (same as seal check)
    if (!fifo_rx.initialized) {
        ret = data_fifo_init(&fifo_rx);
        if (ret) {
            LOG_ERR("Failed to initialize rx FIFO: %d", ret);
            return ret;
        }
    }
    
    // Step 2: Set volume to 50% (0x80 is 50% of 0xFF max)
    hw_codec_volume_set(0xE6);  // 50% volume (seal check uses 0xB0)
    
    // Step 3: Generate the 1kHz sine wave with specified phase
    generate_1khz_sine_wave(initial_phase_degrees);
    
    // Step 4: Initialize audio decimator to 4kHz (same as seal check: rate 12 = 4kHz)
    audio_datapath_decimator_init(12);
    
    // Step 5: Acquire audio datapath (same as seal check)
    audio_datapath_aquire(&fifo_rx);
    
    // Step 6: Start playback of the sine wave (same pattern as seal check multitone)
    ret = audio_datapath_buffer_play(sine_wave_buffer, 
                                     NUM_SINE_WAVE_SAMPLES, 
                                     false,  // no loop
                                     1.0f,   // amplitude multiplier
                                     NULL);  // no callback
    
    if (ret != 0) {
        LOG_ERR("Failed to start sine wave playback: %d", ret);
        audio_datapath_release();
        return ret;
    }
    
    // Step 7: Start recording simultaneously (same pattern as seal check)
    record_to_buffer(sine_wave_recording, 
                    NUM_SINE_WAVE_SAMPLES, 
                    INITIAL_SINE_WAVE_DROP,  // Drop initial samples (same as seal check)
                    false,  // not for seal check
                    true,   // enable recording
                    sine_wave_recording_callback);
    
    LOG_INF("Sine wave playback and recording started successfully");
    return 0;
}

/**
 * @brief Convenience function to start sine wave test with 0 degree phase
 * 
 * @return 0 on success, negative error code on failure
 */
int sine_wave_test_start(void)
{
    return sine_wave_test_start_with_phase(0.0f);
}

/**
 * @brief Initialize the sine wave test module
 * 
 * @return 0 on success
 */
int init_sine_wave_test(void)
{
    // Initialize work item
    k_work_init(&sine_wave_complete_work, sine_wave_complete_work_handler);
    
    LOG_INF("Sine wave test module initialized");
    return 0;
}

/**
 * @brief Shell command to start sine wave test
 */
static int cmd_sine_wave_test(const struct shell *shell, size_t argc, char **argv)
{
    float phase_degrees = 0.0f;  // Default phase
    
    // Parse optional phase argument
    if (argc >= 2) {
        phase_degrees = atof(argv[1]);
        
        // Normalize to 0-360 range
        while (phase_degrees < 0) phase_degrees += 360.0f;
        while (phase_degrees >= 360.0f) phase_degrees -= 360.0f;
        
        shell_print(shell, "Starting 1kHz sine wave test with phase: %.2f degrees", 
                   (double)phase_degrees);
    } else {
        shell_print(shell, "Starting 1kHz sine wave test with default phase (0 degrees)");
    }
    
    int ret = sine_wave_test_start_with_phase(phase_degrees);
    if (ret == 0) {
        shell_print(shell, "Sine wave test started successfully");
    } else {
        shell_error(shell, "Failed to start sine wave test: %d", ret);
    }
    
    return ret;
}

SHELL_CMD_REGISTER(sine_test, NULL, 
                  "Start 1kHz sine wave test. Usage: sine_test [phase_degrees]", 
                  cmd_sine_wave_test);
