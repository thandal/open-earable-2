#include "gesture_detection_pipeline.h"

#include <memory>

#include "../SensorManager/SensorManager.h"
#include "BiQuadFilter.h"
#include "absolute_stage.h"
#include "biquad_filter_stage.h"
#include "cooldown_gate_stage.h"
#include "gesture_detection.h"
#include "local_peak_stage.h"
#include "media_control.h"
#include "processing_pipeline.h"
#include "range_gate_stage.h"
#include "sensor_component_extractor.h"
#include "sensor_processing_consumer.h"
#include "sensor_source_stage.h"
#include "sink_stage.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gesture_detection_pipeline, LOG_LEVEL_DBG);

namespace {

constexpr const char *kPipelineName = "gesture_detection";

constexpr uint8_t kImuSampleRateIdx = 1; /* 50 Hz */
constexpr uint32_t kTapCooldownUs = 150000U;
constexpr float kTapThresholdLow = 0.8f;
constexpr float kTapThresholdHigh = 15.0f;

static float kGestureBandpassCoeffs[2][5] = {
	{ 0.9565432255568768f, -1.9130864511137535f, 0.9565432255568768f,
	  -1.9111970674260732f, 0.9149758348014339f },
	{ 0.20657208382614792f, 0.41314416765229584f, 0.20657208382614792f,
	  -0.3695273773512413f, 0.19581571265583303f },
};

constexpr k_timeout_t kResolveWindow = K_MSEC(CONFIG_BUTTON_GESTURE_MULTIPRESS_WINDOW_MS);

struct GestureDetectionContext {
	k_mutex lock;
	k_work_delayable resolve_work;
	bool initialized;
	uint8_t tap_count;
};

static GestureDetectionContext gesture_detection_ctx;

static void apply_processing_consumer(bool enabled)
{
	LOG_INF("Gesture pipeline %s IMU processing", enabled ? "enabling" : "disabling");

	int ret = update_sensor_consumer_state(ID_IMU, SENSOR_CONSUMER_PROCESSING, enabled,
					       kImuSampleRateIdx);
	if (ret != 0) {
		LOG_WRN("Failed to update IMU processing consumer: %d", ret);
	}
}

static enum media_control_gesture gesture_from_count(uint8_t count)
{
	switch (count) {
	case 2:
		return MEDIA_CTRL_GESTURE_DOUBLE_TAP;
	case 3:
		return MEDIA_CTRL_GESTURE_TRIPLE_TAP;
	default:
		return static_cast<enum media_control_gesture>(0);
	}
}

static void resolve_taps_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (k_mutex_lock(&gesture_detection_ctx.lock, K_FOREVER) != 0) {
		return;
	}

	const uint8_t tap_count = gesture_detection_ctx.tap_count;
	gesture_detection_ctx.tap_count = 0U;
	k_mutex_unlock(&gesture_detection_ctx.lock);

	if (tap_count == 0U) {
		LOG_DBG("Resolve window expired without taps");
		return;
	}

	LOG_INF("Resolving %u tap(s) after window %u ms", tap_count,
		(unsigned int)CONFIG_BUTTON_GESTURE_MULTIPRESS_WINDOW_MS);

	const enum media_control_gesture gesture = gesture_from_count(tap_count);
	if (gesture == 0) {
		LOG_DBG("Ignoring unsupported tap count %u", tap_count);
		return;
	}

	int ret = gesture_detection_publish_event(tap_count, GESTURE_UPDATE_SOURCE_PIPELINE, 0U);
	if (ret != 0) {
		LOG_WRN("Failed to publish gesture event %u: %d", tap_count, ret);
	}

	ret = media_control_handle_gesture(gesture);
	if (ret != 0) {
		LOG_WRN("Failed to handle media gesture %d: %d", gesture, ret);
	}
}

static void on_tap_detected(const struct sensor_data *sample, void *user_data)
{
	ARG_UNUSED(sample);
	ARG_UNUSED(user_data);

	if (!gesture_detection_is_enabled()) {
		LOG_INF("Tap candidate ignored because gesture detection is disabled");
		return;
	}

	if (k_mutex_lock(&gesture_detection_ctx.lock, K_FOREVER) != 0) {
		return;
	}

	if (gesture_detection_ctx.tap_count < UINT8_MAX) {
		gesture_detection_ctx.tap_count++;
	}

	LOG_INF("Tap accepted at %llu us, pending tap count=%u",
		(unsigned long long)(sample != nullptr ? sample->time : 0ULL),
		gesture_detection_ctx.tap_count);

	(void)k_work_reschedule(&gesture_detection_ctx.resolve_work, kResolveWindow);
	k_mutex_unlock(&gesture_detection_ctx.lock);
}

static std::unique_ptr<ProcessingPipeline> create_pipeline()
{
	auto pipeline = std::make_unique<ProcessingPipeline>();

	pipeline->add_source("imu_source", std::make_unique<SensorSourceStage>(ID_IMU));
	pipeline->add_stage("accel_z",
			    std::make_unique<SensorComponentExtractor>(2 * sizeof(float), PARSE_TYPE_FLOAT));
	pipeline->add_stage("bandpass",
			    std::make_unique<BiQuadFilterStage>(PARSE_TYPE_FLOAT, 2,
								kGestureBandpassCoeffs));
	pipeline->add_stage("magnitude", std::make_unique<AbsoluteStage>(PARSE_TYPE_FLOAT));
	pipeline->add_stage("peak_search", std::make_unique<LocalPeakStage>());
	pipeline->add_stage("threshold_gate",
			    std::make_unique<RangeGateStage>(PARSE_TYPE_FLOAT, kTapThresholdLow,
							      kTapThresholdHigh));
	pipeline->add_stage("cooldown_gate", std::make_unique<CooldownGateStage>(kTapCooldownUs));
	pipeline->add_stage("tap_sink", std::make_unique<SinkStage>(on_tap_detected, nullptr));

	pipeline->connect("imu_source", "accel_z");
	pipeline->connect("accel_z", "bandpass");
	pipeline->connect("bandpass", "magnitude");
	pipeline->connect("magnitude", "peak_search");
	pipeline->connect("peak_search", "threshold_gate");
	pipeline->connect("threshold_gate", "cooldown_gate");
	pipeline->connect("cooldown_gate", "tap_sink");

	return pipeline;
}

} /* namespace */

int gesture_detection_pipeline_init(void)
{
	if (!gesture_detection_ctx.initialized) {
		k_mutex_init(&gesture_detection_ctx.lock);
		k_work_init_delayable(&gesture_detection_ctx.resolve_work, resolve_taps_work_handler);
		gesture_detection_ctx.tap_count = 0U;
		gesture_detection_ctx.initialized = true;
		LOG_INF("Gesture detection pipeline context initialized");
	}

	LOG_INF("Installing gesture detection pipeline");
	set_processing_pipeline(kPipelineName, create_pipeline());
	gesture_detection_pipeline_on_enabled_changed(gesture_detection_is_enabled());
	return 0;
}

void gesture_detection_pipeline_on_enabled_changed(bool enabled)
{
	if (!gesture_detection_ctx.initialized) {
		return;
	}

	apply_processing_consumer(enabled);

	if (k_mutex_lock(&gesture_detection_ctx.lock, K_FOREVER) != 0) {
		return;
	}

	if (!enabled) {
		gesture_detection_ctx.tap_count = 0U;
		(void)k_work_cancel_delayable(&gesture_detection_ctx.resolve_work);
		LOG_INF("Gesture detection disabled, cleared pending taps");
	}

	k_mutex_unlock(&gesture_detection_ctx.lock);
}
