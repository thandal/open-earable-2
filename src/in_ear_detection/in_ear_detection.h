#ifndef _IN_EAR_DETECTION_H_
#define _IN_EAR_DETECTION_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief High-level wearing state reported by the in-ear detection framework.
 *
 * The state model is intentionally small and stable so higher-level features can
 * depend on it without being coupled to a specific detection algorithm.
 */
enum in_ear_state {
	/** Detection has not produced a usable decision yet. */
	IN_EAR_STATE_UNKNOWN = 0,
	/** The device is currently considered not worn. */
	IN_EAR_STATE_NOT_WORN,
	/** The device is currently considered worn. */
	IN_EAR_STATE_WORN,
};

/**
 * @brief Identifies which subsystem produced a state update.
 *
 * New sources can be added without changing the consumer-facing state model.
 */
enum in_ear_update_source {
	/** Source was not specified by the publisher. */
	IN_EAR_UPDATE_SOURCE_UNSPECIFIED = 0,
	/** Update originated from a sensor-processing pipeline. */
	IN_EAR_UPDATE_SOURCE_PIPELINE,
	/** Update was issued manually by control code or tests. */
	IN_EAR_UPDATE_SOURCE_MANUAL,
};

/**
 * @brief Complete in-ear status message published on zbus.
 *
 * This structure is the canonical payload for the module's zbus channel.
 * It carries the current state plus enough metadata for consumers that need
 * ordering, provenance, or future extension points.
 */
struct in_ear_status_msg {
	/** Newly active in-ear state. */
	enum in_ear_state state;
	/** State that was active immediately before @ref state. */
	enum in_ear_state previous_state;
	/** Monotonic counter incremented on every accepted state change. */
	uint32_t change_counter;
	/** Timestamp in microseconds when the state change was published. */
	uint64_t timestamp_us;
	/** Update producer identifier from @ref in_ear_update_source. */
	uint16_t source;
	/** Reserved for algorithm-specific annotations and future extensions. */
	uint16_t flags;
};

/**
 * @brief Initialize the in-ear detection framework.
 *
 * The framework currently uses static storage only, so initialization is cheap
 * and idempotent. The function exists to keep the module's lifecycle explicit
 * and to provide a stable place for future setup work.
 *
 * @retval 0 Initialization succeeded.
 */
int in_ear_detection_init(void);

/**
 * @brief Get the current global wearing state.
 *
 * @return The last published state, or @ref IN_EAR_STATE_UNKNOWN if no state is
 *         available.
 */
enum in_ear_state in_ear_detection_state(void);

/**
 * @brief Check whether the device is currently considered worn.
 *
 * This is the convenience function intended for most call sites in the code
 * base that only need a simple yes/no decision.
 *
 * @retval true The current state is @ref IN_EAR_STATE_WORN.
 * @retval false The current state is anything else.
 */
bool in_ear_detection_is_worn(void);

/**
 * @brief Read the complete current status snapshot.
 *
 * @param[out] status Destination for the latest status message.
 *
 * @retval 0 Status was copied successfully.
 * @retval -EINVAL @p status was NULL.
 * @retval Negative error code propagated from zbus.
 */
int in_ear_detection_get(struct in_ear_status_msg *status);

/**
 * @brief Publish a new high-level wearing state.
 *
 * The framework updates the transition metadata automatically. If the requested
 * state already matches the current one, no new zbus event is emitted.
 *
 * This is the preferred entry point for detection logic implemented in sensor
 * processing pipelines or other producers.
 *
 * @param state New wearing state to activate.
 * @param source Producer identifier from @ref in_ear_update_source.
 * @param flags Optional producer-specific flags.
 *
 * @retval 0 State handling completed successfully.
 * @retval Negative error code if the current state could not be read or the new
 *         status could not be published.
 */
int in_ear_detection_set_state(enum in_ear_state state, uint16_t source, uint16_t flags);

/**
 * @brief Publish a fully prepared status message.
 *
 * Use this when a producer needs explicit control over every field in the
 * message payload. Most producers should call @ref in_ear_detection_set_state
 * instead.
 *
 * @param status Fully populated status message to publish.
 *
 * @retval 0 Message was published successfully.
 * @retval -EINVAL @p status was NULL.
 * @retval Negative error code propagated from zbus.
 */
int in_ear_detection_publish(const struct in_ear_status_msg *status);

/**
 * @brief Subscribe a zbus observer to wearing state changes.
 *
 * Consumers can also declare the channel directly and attach observers
 * themselves. This helper keeps typical usage centralized and discoverable.
 *
 * @param observer Observer or subscriber to register.
 * @param timeout Timeout used for zbus observer registration.
 *
 * @retval 0 Observer was added successfully.
 * @retval -EINVAL @p observer was NULL.
 * @retval Negative error code propagated from zbus.
 */
int in_ear_detection_subscribe(const struct zbus_observer *observer, k_timeout_t timeout);

/**
 * @brief Get the backing zbus channel for in-ear status updates.
 *
 * This allows advanced consumers to integrate with the channel directly.
 *
 * @return Pointer to the module's zbus channel.
 */
const struct zbus_channel *in_ear_detection_channel(void);

/** Public declaration of the in-ear detection zbus channel. */
ZBUS_CHAN_DECLARE(in_ear_detection_chan);

#ifdef __cplusplus
}
#endif

#endif /* _IN_EAR_DETECTION_H_ */
