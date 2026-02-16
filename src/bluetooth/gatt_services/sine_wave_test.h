#ifndef _SINE_WAVE_TEST_H_
#define _SINE_WAVE_TEST_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start 1kHz sine wave generation and recording test with default phase
 * 
 * This function generates a 1kHz sine wave at 50% volume with 0 degree phase 
 * and records it. The test automatically:
 * - Initializes audio FIFO if needed
 * - Sets volume to 50%
 * - Generates and plays a 1kHz sine wave
 * - Records the output
 * - Detects and compares phase after recording
 * 
 * @return 0 on success, negative error code on failure
 */
int sine_wave_test_start(void);

/**
 * @brief Start 1kHz sine wave test with specified initial phase
 * 
 * Same as sine_wave_test_start() but allows specifying the initial phase
 * of the generated sine wave. Useful for testing phase detection accuracy.
 * 
 * @param initial_phase_degrees Initial phase in degrees (0-360)
 * @return 0 on success, negative error code on failure
 */
int sine_wave_test_start_with_phase(float initial_phase_degrees);

/**
 * @brief Initialize the sine wave test module
 * 
 * Call this function during system initialization.
 * 
 * @return 0 on success
 */
int init_sine_wave_test(void);

#ifdef __cplusplus
}
#endif

#endif /* _SINE_WAVE_TEST_H_ */
