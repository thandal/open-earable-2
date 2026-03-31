#include "wear_detection_pipeline.h"

#include <cmath>
#include <cstring>
#include <errno.h>
#include <memory>

#include "../../SensorManager/SensorManager.h"
#include "media_control.h"
#include "in_ear_detection.h"
#include "processing_pipeline.h"
#include "sensor_processing_consumer.h"
#include "sensor_source_stage.h"
#include "trigger_action_stage.h"
#include "sensor_component_extractor.h"
#include "derivative_stage.h"
#include "absolute_stage.h"
#include "switch_stage.h"
#include "sink_stage.h"
#include "biquad_filter_stage.h"
#include "if_stage.h"
#include "division_stage.h"
#include "and_stage.h"
#include "not_stage.h"

#include "BiQuadFilter.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(wear_detection_pipeline, LOG_LEVEL_DBG);

namespace {

constexpr const char *kPipelineName = "wear_detection";

constexpr uint8_t kImuSampleRateIdx = 1;   // 50 Hz
constexpr uint8_t kBaroSampleRateIdx = 14; // 25 Hz
constexpr uint8_t kPpgSampleRateIdx = 0;   // 25 Hz

constexpr float kImuAccelTriggerThreshold = 4.0f;       // m/s^2 delta from gravity
constexpr float kImuGyroTriggerThreshold = 150.0f;      // dps
constexpr float kBaroPressureRateThreshold = 150.0f;     // Pa/s

constexpr uint32_t kPpgCaptureVotesRequired = 3U;
constexpr uint32_t kPpgCaptureMaxSamples = 4U;
constexpr k_timeout_t kPpgCaptureWindow = K_MSEC(200);

constexpr float kAmbientFractionOfGreenMax = 0.12f;
constexpr float kAmbientFractionOfRedMax = 0.25f;
constexpr float kAmbientFractionOfIrMax = 0.30f;
constexpr float kGreenOverRedMin = 1.10f;
constexpr float kRedOverIrMin = 1.05f;
constexpr float kIrOverAmbientMin = 1.50f;

constexpr float kPpgAmbientThreshhold = 5000.0f;
constexpr float kPpgGreenThreshhold = 30000.0f;
constexpr float kPpgGreenRedRatioThreshhold = 1.15f;

constexpr float kBaroLowPassCoeffs[5] = {
	0.09131040226436236,
	0.18262080452872473,
	0.09131040226436236,
	-0.9823573998046842,
	0.3475990088621337
};

BiQuadFilter baroLowPassFilter(1);

struct WearDetectionContext {
	k_mutex lock;
	k_work_delayable ppg_stop_work;
	bool initialized;
	bool capture_armed;
	uint32_t capture_samples_seen;
	uint32_t capture_positive_votes;
};

static WearDetectionContext wear_detection_ctx;

static void apply_processing_consumer(uint8_t sensor_id, bool enabled, uint8_t sample_rate_idx)
{
	int ret = update_sensor_consumer_state(sensor_id, SENSOR_CONSUMER_PROCESSING, enabled,
					       sample_rate_idx);
	if (ret != 0) {
		LOG_WRN("Failed to update processing consumer for sensor %u: %d", sensor_id, ret);
	}
}

static void stop_ppg_capture_locked()
{
	wear_detection_ctx.capture_armed = false;
	wear_detection_ctx.capture_samples_seen = 0U;
	wear_detection_ctx.capture_positive_votes = 0U;
	(void)k_work_cancel_delayable(&wear_detection_ctx.ppg_stop_work);
	apply_processing_consumer(ID_PPG, false, kPpgSampleRateIdx);
}

static void ppg_stop_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (k_mutex_lock(&wear_detection_ctx.lock, K_FOREVER) != 0) {
		return;
	}

	stop_ppg_capture_locked();
	k_mutex_unlock(&wear_detection_ctx.lock);
}

static void arm_ppg_capture_locked()
{
	wear_detection_ctx.capture_armed = true;
	wear_detection_ctx.capture_samples_seen = 0U;
	wear_detection_ctx.capture_positive_votes = 0U;

	apply_processing_consumer(ID_PPG, true, kPpgSampleRateIdx);
	(void)k_work_reschedule(&wear_detection_ctx.ppg_stop_work, kPpgCaptureWindow);
}

static void trigger_ppg_capture(const struct sensor_data *trigger, void *user_data)
{
	ARG_UNUSED(trigger);
	ARG_UNUSED(user_data);

	LOG_DBG("Triggering PPG capture");

	if (!in_ear_detection_is_enabled()) {
		return;
	}

	if (k_mutex_lock(&wear_detection_ctx.lock, K_FOREVER) != 0) {
		return;
	}

	arm_ppg_capture_locked();
	k_mutex_unlock(&wear_detection_ctx.lock);
}

static void apply_in_ear_state_decision(const struct sensor_data *trigger, void *user_data)
{
	ARG_UNUSED(user_data);

	LOG_DBG("Applying in-ear state decision");

	if (trigger == nullptr || trigger->size < sizeof(uint8_t)) {
		return;
	}

	const enum in_ear_state decided_state =
		static_cast<enum in_ear_state>(trigger->data[0]);

	LOG_DBG("Decided in-ear state: %d", decided_state);

	if (decided_state != IN_EAR_STATE_WORN &&
	    decided_state != IN_EAR_STATE_NOT_WORN) {
		return;
	}

	(void)media_control_handle_gesture(decided_state == IN_EAR_STATE_WORN
					 ? MEDIA_CTRL_GESTURE_IN_EAR
					 : MEDIA_CTRL_GESTURE_OUT_OF_EAR);
	(void)in_ear_detection_set_state(decided_state, IN_EAR_UPDATE_SOURCE_PIPELINE, 0U);
}

class ImuMovementTriggerStage : public SensorProcessingStage {
public:
	ImuMovementTriggerStage() : SensorProcessingStage(1) {}

	int process(const struct sensor_data *const inputs[], struct sensor_data *output) override
	{
		if (inputs == nullptr || inputs[0] == nullptr || output == nullptr) {
			return -EINVAL;
		}

		float components[9];
		std::memcpy(components, inputs[0]->data, sizeof(components));

		const float accel_mag = std::sqrt(components[0] * components[0] +
					       components[1] * components[1] +
					       components[2] * components[2]);
		const float gyro_mag = std::sqrt(components[3] * components[3] +
					      components[4] * components[4] +
					      components[5] * components[5]);

		const float accel_delta = std::fabs(accel_mag - 9.81f);
		const float triggered =
			(accel_delta >= kImuAccelTriggerThreshold || gyro_mag >= kImuGyroTriggerThreshold)
				? 1.0f
				: 0.0f;
		
		if (triggered > 0.0f) {
			LOG_DBG("IMU triggered movement: accel_delta=%.2f, gyro_mag=%.2f", accel_delta, gyro_mag);
		}

		*output = *inputs[0];
		output->size = sizeof(float);
		std::memcpy(output->data, &triggered, sizeof(triggered));
		return 0;
	}
};

void print_stage_callback(const struct sensor_data *input, void *user_data)
{
	ARG_UNUSED(user_data);

	if (input == nullptr || input->size < sizeof(uint32_t)) {
		return;
	}

	uint32_t value;
	std::memcpy(&value, input->data, sizeof(uint32_t));
	LOG_DBG("Print stage output: %u", value);
}

class PpgWearClassifierStage : public SensorProcessingStage {
public:
	PpgWearClassifierStage() : SensorProcessingStage(1) {}

	int process(const struct sensor_data *const inputs[], struct sensor_data *output) override
	{
		if (inputs == nullptr || inputs[0] == nullptr || output == nullptr) {
			return -EINVAL;
		}

		if (k_mutex_lock(&wear_detection_ctx.lock, K_FOREVER) != 0) {
			return -EAGAIN;
		}

		if (!wear_detection_ctx.capture_armed) {
			k_mutex_unlock(&wear_detection_ctx.lock);
			return 1;
		}

		uint32_t red = 0U;
		uint32_t ir = 0U;
		uint32_t green = 0U;
		uint32_t ambient = 0U;
		std::memcpy(&red, inputs[0]->data + 0 * sizeof(uint32_t), sizeof(uint32_t));
		std::memcpy(&ir, inputs[0]->data + 1 * sizeof(uint32_t), sizeof(uint32_t));
		std::memcpy(&green, inputs[0]->data + 2 * sizeof(uint32_t), sizeof(uint32_t));
		std::memcpy(&ambient, inputs[0]->data + 3 * sizeof(uint32_t), sizeof(uint32_t));

		const bool ambient_low =
			(ambient <= green * kAmbientFractionOfGreenMax) &&
			(ambient <= red * kAmbientFractionOfRedMax) &&
			(ambient <= ir * kAmbientFractionOfIrMax);
		const bool spectrum_matches =
			ambient_low &&
			(green >= red * kGreenOverRedMin) &&
			(red >= ir * kRedOverIrMin) &&
			(ir >= ambient * kIrOverAmbientMin);

		wear_detection_ctx.capture_samples_seen++;
		if (spectrum_matches) {
			wear_detection_ctx.capture_positive_votes++;
		}

		const bool should_decide =
			(wear_detection_ctx.capture_positive_votes >= kPpgCaptureVotesRequired) ||
			(wear_detection_ctx.capture_samples_seen >= kPpgCaptureMaxSamples);

		if (should_decide) {
			const enum in_ear_state decided_state =
				(wear_detection_ctx.capture_positive_votes >= kPpgCaptureVotesRequired)
					? IN_EAR_STATE_WORN
					: IN_EAR_STATE_NOT_WORN;
			stop_ppg_capture_locked();
			k_mutex_unlock(&wear_detection_ctx.lock);

			*output = *inputs[0];
			output->size = sizeof(uint8_t);
			output->data[0] = static_cast<uint8_t>(decided_state);
			return 0;
		}

		k_mutex_unlock(&wear_detection_ctx.lock);
		return 1;
	}
};

static std::unique_ptr<ProcessingPipeline> create_pipeline()
{
	auto pipeline = std::make_unique<ProcessingPipeline>();

	// pipeline->add_source("imu_source", std::make_unique<SensorSourceStage>(ID_IMU));
	pipeline->add_source("baro_source", std::make_unique<SensorSourceStage>(ID_TEMP_BARO));
	pipeline->add_source("ppg_source", std::make_unique<SensorSourceStage>(ID_PPG));

	// pipeline->add_stage("imu_trigger", std::make_unique<ImuMovementTriggerStage>());
	
	pipeline->add_stage("baro_component", std::make_unique<SensorComponentExtractor>(sizeof(float), PARSE_TYPE_FLOAT));

	baroLowPassFilter.set_coefficients(const_cast<float(*)[5]>(&kBaroLowPassCoeffs), 1);

	pipeline->add_stage("baro_lowpass", std::make_unique<BiQuadFilterStage>(PARSE_TYPE_FLOAT, baroLowPassFilter));

	pipeline->add_stage("baro_deriv", std::make_unique<DerivativeStage>(PARSE_TYPE_FLOAT, 10));
	pipeline->add_stage("abs_baro_deriv", std::make_unique<AbsoluteStage>(PARSE_TYPE_FLOAT));
	pipeline->add_stage("baro_switch", std::make_unique<SwitchStage>(PARSE_TYPE_FLOAT, kBaroPressureRateThreshold, kBaroPressureRateThreshold));
	
	pipeline->add_stage("ppg_enable", std::make_unique<TriggerActionStage>(
					  PARSE_TYPE_UINT8, trigger_ppg_capture, nullptr));

	pipeline->add_stage("print", std::make_unique<SinkStage>(print_stage_callback));

	pipeline->add_stage("ppg_ambient", std::make_unique<SensorComponentExtractor>(3 * sizeof(uint32_t), PARSE_TYPE_UINT32)); // Extract ambient component
	pipeline->add_stage("ppg_green", std::make_unique<SensorComponentExtractor>(1 * sizeof(uint32_t), PARSE_TYPE_UINT32)); // Extract green component
	pipeline->add_stage("ppg_red", std::make_unique<SensorComponentExtractor>(0, PARSE_TYPE_UINT32)); // Extract red component
	pipeline->add_stage("ppg_ir", std::make_unique<SensorComponentExtractor>(2 * sizeof(uint32_t), PARSE_TYPE_UINT32)); // Extract ir component

	pipeline->add_stage("ppg_ambient_thresh", std::make_unique<SwitchStage>(PARSE_TYPE_UINT32, kPpgAmbientThreshhold, kPpgAmbientThreshhold, true));
	pipeline->add_stage("ppg_green_thresh", std::make_unique<SwitchStage>(PARSE_TYPE_UINT32, kPpgGreenThreshhold, kPpgGreenThreshhold, true));
	pipeline->add_stage("ppg_green_red_ratio", std::make_unique<DivisionStage>(2, PARSE_TYPE_UINT32));
	pipeline->add_stage("ppg_green_red_thresh", std::make_unique<SwitchStage>(PARSE_TYPE_DOUBLE, kPpgGreenRedRatioThreshhold, kPpgGreenRedRatioThreshhold, true));
	pipeline->add_stage("ppg_ambient_thresh_state", std::make_unique<SensorComponentExtractor>(sizeof(uint32_t), PARSE_TYPE_UINT8));
	pipeline->add_stage("ppg_green_thresh_state", std::make_unique<SensorComponentExtractor>(sizeof(uint32_t), PARSE_TYPE_UINT8));
	pipeline->add_stage("ppg_green_red_thresh_state", std::make_unique<SensorComponentExtractor>(sizeof(double), PARSE_TYPE_UINT8));
	pipeline->add_stage("ppg_ambient_thresh_state_inv", std::make_unique<NotStage>());

	pipeline->add_stage("ppg_and", std::make_unique<AndStage>(3));

	// pipeline->add_stage("ppg_classifier", std::make_unique<PpgWearClassifierStage>());
	pipeline->add_stage("ppg_decision_sink", std::make_unique<TriggerActionStage>(
					 PARSE_TYPE_UINT8, apply_in_ear_state_decision, nullptr));

	// pipeline->connect("imu_source", "imu_trigger");
	// pipeline->connect("imu_trigger", "imu_action");
	pipeline->connect("baro_source", "baro_component");
	pipeline->connect("baro_component", "baro_lowpass");
	pipeline->connect("baro_lowpass", "baro_deriv");
	pipeline->connect("baro_deriv", "abs_baro_deriv");
	pipeline->connect("abs_baro_deriv", "baro_switch");
	// pipeline->connect("abs_baro_deriv", "print");
	pipeline->connect("baro_switch", "ppg_enable");

	pipeline->connect("ppg_source", "ppg_ambient");
	pipeline->connect("ppg_source", "ppg_green");
	pipeline->connect("ppg_source", "ppg_red");
	pipeline->connect("ppg_source", "ppg_ir");

	pipeline->connect("ppg_ambient", "ppg_ambient_thresh");
	pipeline->connect("ppg_green", "ppg_green_thresh");

	pipeline->connect("ppg_green", "ppg_green_red_ratio", 0);
	pipeline->connect("ppg_red", "ppg_green_red_ratio", 1);
	pipeline->connect("ppg_green_red_ratio", "ppg_green_red_thresh");
	pipeline->connect("ppg_ambient_thresh", "ppg_ambient_thresh_state");
	pipeline->connect("ppg_green_thresh", "ppg_green_thresh_state");
	pipeline->connect("ppg_green_red_thresh", "ppg_green_red_thresh_state");
	pipeline->connect("ppg_ambient_thresh_state", "ppg_ambient_thresh_state_inv");

	pipeline->connect("ppg_ambient_thresh_state_inv", "ppg_and", 0);
	pipeline->connect("ppg_green_thresh_state", "ppg_and", 1);
	pipeline->connect("ppg_green_red_thresh_state", "ppg_and", 2);

	pipeline->connect("ppg_and", "ppg_decision_sink");

	pipeline->connect("ppg_ir", "print");


	// pipeline->connect("ppg_classifier", "ppg_decision_sink");

	return pipeline;
}

} // namespace

int in_ear_detection_pipeline_init(void)
{
	if (!wear_detection_ctx.initialized) {
		k_mutex_init(&wear_detection_ctx.lock);
		k_work_init_delayable(&wear_detection_ctx.ppg_stop_work, ppg_stop_work_handler);
		wear_detection_ctx.initialized = true;
	}

	set_processing_pipeline(kPipelineName, create_pipeline());
	in_ear_detection_pipeline_on_enabled_changed(in_ear_detection_is_enabled());
	return 0;
}

void in_ear_detection_pipeline_on_enabled_changed(bool enabled)
{
	if (!wear_detection_ctx.initialized) {
		return;
	}

	apply_processing_consumer(ID_IMU, enabled, kImuSampleRateIdx);
	apply_processing_consumer(ID_TEMP_BARO, enabled, kBaroSampleRateIdx);

	if (k_mutex_lock(&wear_detection_ctx.lock, K_FOREVER) != 0) {
		return;
	}

	if (!enabled) {
		stop_ppg_capture_locked();
	}

	k_mutex_unlock(&wear_detection_ctx.lock);
}
