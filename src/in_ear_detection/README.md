# In-Ear Detection

This module provides the firmware-wide abstraction for answering one question:

`Is the OpenEarable currently worn?`

The implementation is split into two layers:

- The **framework** owns the global state, enable/disable lifecycle, and notification surfaces.
- The **pipeline** is one concrete producer that computes wearing decisions from sensor data and pushes them into the framework.

The key design goal is to keep those layers separate. Application code should depend on the framework API, not on the details of the current wear-classification algorithm.

## Directory structure

- `in_ear_detection.h`
  Public API for querying and updating in-ear state.
- `in_ear_detection.c`
  Framework implementation and zbus channel ownership.
- `wear_detection_pipeline.h`
  Public entry points for bringing up the built-in processing pipeline.
- `wear_detection_pipeline.cpp`
  Concrete sensor-processing graph and sensor-control orchestration.
- `README.md`
  This document.

## Framework overview

The framework is the stable contract for the rest of the firmware.

It is responsible for:

- maintaining a single global in-ear status
- exposing synchronous query helpers
- publishing asynchronous updates through zbus
- tracking whether wear detection is enabled
- carrying metadata such as update source, timestamps, and change counters
- allowing different producers to set the state without coupling consumers to a specific classifier

### Public state model

The high-level wearing state is intentionally small:

- `IN_EAR_STATE_UNKNOWN`
  No usable decision is available yet.
- `IN_EAR_STATE_NOT_WORN`
  The device is considered out of ear.
- `IN_EAR_STATE_WORN`
  The device is considered in ear.

This small enum is deliberate. It keeps the public contract stable even if the classifier becomes more complex later.

### Public framework API

The main API is declared in `src/in_ear_detection/in_ear_detection.h`.

#### Query helpers

- `in_ear_detection_state()`
  Returns the current global state enum.
- `in_ear_detection_is_worn()`
  Convenience boolean view of the state.
- `in_ear_detection_is_enabled()`
  Returns whether the wear-detection subsystem is currently enabled.
- `in_ear_detection_get(struct in_ear_status_msg *status)`
  Returns the full current status payload.

#### Control helpers

- `in_ear_detection_set_enabled(bool enabled, uint16_t source, uint16_t flags)`
  Turns detection on or off and publishes a framework update if the enable state changes.
- `in_ear_detection_enable(...)`
  Convenience wrapper for enabling.
- `in_ear_detection_disable(...)`
  Convenience wrapper for disabling.

#### Producer entry points

- `in_ear_detection_set_state(enum in_ear_state state, uint16_t source, uint16_t flags)`
  Preferred way for producers to publish a new wearing decision.
- `in_ear_detection_publish(const struct in_ear_status_msg *status)`
  Low-level escape hatch when a producer needs full control over the payload.

#### Subscription helpers

- `in_ear_detection_subscribe(const struct zbus_observer *observer, k_timeout_t timeout)`
  Helper for attaching a zbus observer to the in-ear channel.
- `in_ear_detection_channel()`
  Returns the backing zbus channel directly.
- `in_ear_detection_chan`
  Public zbus channel declaration.

### Framework payload

The canonical zbus payload is `struct in_ear_status_msg`.

It contains:

- `enabled`
  Whether wear detection is currently active.
- `previous_enabled`
  Previous enable state before this update.
- `state`
  Current wearing decision.
- `previous_state`
  Previous wearing decision.
- `change_counter`
  Monotonic update counter.
- `timestamp_us`
  Publish timestamp in microseconds.
- `source`
  Producer identity, for example `IN_EAR_UPDATE_SOURCE_PIPELINE`.
- `flags`
  Reserved metadata for future classifier annotations.

## Framework lifecycle

`in_ear_detection_init()` brings up the framework itself. Today it is cheap because the module uses static storage, but it still exists to make the lifecycle explicit.

The built-in pipeline is brought up separately through:

- `in_ear_detection_pipeline_init()`

This separation is important:

- the framework can exist without any specific classifier
- the pipeline can be swapped or replaced later
- tests or external code can inject decisions directly through the framework without running the built-in pipeline

## zbus integration

The framework owns one dedicated zbus channel for in-ear status.

That channel is the fan-out point for:

- internal firmware consumers
- the GATT service
- future logging, analytics, or automation modules

The framework stores the current state in the channel payload itself. This means:

- synchronous reads can fetch the current state from the channel
- asynchronous consumers can subscribe to change events
- there is a single authoritative source of truth

## Bluetooth integration

The GATT service lives in:

- `src/bluetooth/gatt_services/in_ear_detection_service.c`

It exposes three characteristics:

- detection enabled
  `read`, `write`, `notify`
- raw wear state
  `read`, `notify`
- boolean `is worn`
  `read`, `notify`

This lets external clients:

- enable or disable wear detection
- inspect the current classifier result
- subscribe to changes without having to know the internal zbus contract

## Sensor-manager integration

The built-in pipeline does not own sensors directly. Instead, it asks the sensor manager to add or remove the `SENSOR_CONSUMER_PROCESSING` bit on specific sensors.

This is done through the helper added to `SensorManager`:

- `update_sensor_consumer_state(...)`

That helper exists so the in-ear pipeline can:

- enable IMU processing
- enable barometer processing
- temporarily enable PPG processing
- remove only the processing consumer bit again later

without clobbering BLE or SD-card consumers that may also be using the same sensor.

## Built-in pipeline

The built-in classifier is implemented in:

- `src/in_ear_detection/wear_detection_pipeline.cpp`

It is a trigger-based cascaded classifier:

1. A low-cost trigger path watches always-on sensors.
2. If that path indicates a likely wear transition, the system arms a short optical sampling window.
3. PPG is enabled only for that short window.
4. A second-stage classifier evaluates the optical pattern.
5. The result is sent into the framework as a wearing decision.

### Intended high-level behavior

The intended behavior is:

- IMU motion spikes or rapid barometric pressure changes indicate that the earable may have moved into or out of the ear.
- Those events trigger a very brief PPG sampling burst.
- The optical signature is then checked:
  ambient very low, green highest, red in the middle, IR below red.
- The result becomes either `IN_EAR_STATE_WORN` or `IN_EAR_STATE_NOT_WORN`.

### Current trigger path

The current code contains two trigger mechanisms:

- an `ImuMovementTriggerStage` class
- a barometer-based trigger path that is actually wired into the graph

Important detail:

- the IMU trigger stage currently exists in the source file but is **not currently connected** in `create_pipeline()`
- the barometer trigger path **is connected**

So the current graph is primarily pressure-triggered, even though the file already contains IMU-trigger logic and the framework enables IMU processing as well.

### Current barometer path

The currently connected barometer path is:

1. `baro_source`
   Produces raw barometer sensor samples.
2. `baro_component`
   Extracts the pressure component from the barometer payload.
3. `baro_lowpass`
   Applies a biquad low-pass filter.
4. `baro_deriv`
   Computes the derivative over a sliding window.
5. `abs_baro_deriv`
   Converts the derivative magnitude to an absolute value.
6. `baro_switch`
   Thresholds the derivative magnitude.
7. `ppg_enable`
   A trigger stage that calls `trigger_ppg_capture(...)` when the threshold output is non-zero.

In short:

- raw pressure
- filtered pressure
- pressure rate of change
- absolute pressure rate
- threshold crossing
- trigger PPG capture

### PPG capture window

PPG is not left running continuously for in-ear detection.

When a trigger fires:

- `trigger_ppg_capture(...)` acquires the module lock
- the capture context is armed
- sample counters are reset
- the PPG processing consumer bit is enabled
- a delayed work item is scheduled to turn PPG off again after `kPpgCaptureWindow`

The window is intentionally short:

- default timeout in the current code: `200 ms`

If no valid decision is made before that timeout expires, the delayed work disables the PPG processing path again.

### Current optical decision path

The source file currently contains two optical-classification approaches:

- a self-contained `PpgWearClassifierStage`
- a fully wired graph built from smaller reusable stages

Important detail:

- `PpgWearClassifierStage` exists in the file but is **not currently connected** in `create_pipeline()`
- the currently active implementation is the stage graph built from extractors, switches, division, inversion, and logical AND

### Currently wired PPG graph

The active PPG graph is:

1. `ppg_source`
   Produces raw PPG frames.
2. `ppg_ambient`
   Extracts ambient.
3. `ppg_green`
   Extracts green.
4. `ppg_red`
   Extracts red.
5. `ppg_ir`
   Extracts IR.
6. `ppg_ambient_thresh`
   Checks ambient against a threshold.
7. `ppg_green_thresh`
   Checks green against a threshold.
8. `ppg_green_red_ratio`
   Computes `green / red`.
9. `ppg_green_red_thresh`
   Thresholds the `green / red` ratio.
10. `ppg_ambient_thresh_state`
    Extracts the state byte from the switch output.
11. `ppg_green_thresh_state`
    Extracts the state byte from the green-threshold output.
12. `ppg_green_red_thresh_state`
    Extracts the state byte from the ratio-threshold output.
13. `ppg_ambient_thresh_state_inv`
    Inverts the ambient threshold state.
14. `ppg_and`
    Computes the logical AND of:
    ambient below threshold, green above threshold, green/red ratio above threshold.
15. `ppg_decision_sink`
    Consumes the final decision and applies it to the framework.

There is also a `print` sink stage used for debug logging on the IR path.

### What the currently wired optical rule actually checks

The currently wired stage graph uses these heuristics:

- ambient must be below `kPpgAmbientThreshhold`
- green must be above `kPpgGreenThreshhold`
- green/red ratio must be above `kPpgGreenRedRatioThreshhold`

That is slightly different from the more complete rule expressed elsewhere in the file and in the original intent.

Specifically:

- the active graph does **not currently enforce** the full `red > ir > ambient` style ordering
- the active graph does **not currently use** the `PpgWearClassifierStage` voting logic
- the active graph produces a boolean decision through threshold composition rather than the standalone classifier class

This README states that explicitly because it is important for future tuning work.

## Decision sink behavior

The last step of the current pipeline is a sink/action stage that turns the stage-graph output into real firmware behavior.

`apply_in_ear_state_decision(...)` currently does two things:

1. Dispatches a media-control gesture:
   - `MEDIA_CTRL_GESTURE_IN_EAR` for `IN_EAR_STATE_WORN`
   - `MEDIA_CTRL_GESTURE_OUT_OF_EAR` for `IN_EAR_STATE_NOT_WORN`
2. Calls `in_ear_detection_set_state(...)`

That means a wear decision affects both:

- the framework state
- the play/pause media path

## Enable and disable behavior

When the in-ear framework is enabled:

- IMU processing is enabled
- barometer processing is enabled
- PPG stays off until a trigger arms the capture window

When the framework is disabled:

- IMU processing is disabled
- barometer processing is disabled
- any active PPG capture is cancelled
- the PPG processing consumer bit is removed

The disable path is intentionally aggressive so the module stops consuming sensor power when it is not needed.

## Notes on current implementation status

The source tree currently contains both:

- a more explicit standalone classifier class
- a more compositional stage graph

The active graph is the compositional one.

The code also contains an IMU trigger stage that is not wired into the graph yet.

So the current state of the implementation is best described as:

- the **framework is complete and reusable**
- the **pipeline exists and runs as one concrete producer**
- the **barometer-trigger path is wired**
- the **IMU-trigger path is partially prepared but not currently connected**
- the **optical classification is currently implemented as a threshold graph**
- the **standalone voting classifier class is present but not currently used**

## How to evolve this module

If you want to modify the wear classifier in the future, there are three main places to work:

### 1. Framework-level changes

Edit `in_ear_detection.h` and `in_ear_detection.c` when you need to change:

- public state representation
- metadata fields
- enable/disable semantics
- update publication rules

### 2. Sensor-orchestration changes

Edit `wear_detection_pipeline.cpp` when you need to change:

- which sensors participate in wear detection
- which sample rates are used
- when PPG is armed
- how long the PPG window stays open

### 3. Classifier logic changes

Edit `wear_detection_pipeline.cpp` when you need to change:

- optical thresholds
- filtering
- derivative windows
- boolean combination rules
- whether to use the standalone `PpgWearClassifierStage` or the current stage graph

## Recommended future cleanup

The current implementation would benefit from a follow-up cleanup that:

- either wires the IMU path fully or removes the unused IMU-trigger class
- either wires the standalone `PpgWearClassifierStage` or removes it in favor of the stage graph
- consolidates the two different optical-rule representations so there is only one authoritative classifier definition
- replaces ad-hoc debug sink logging with a cleaner debug strategy when tuning is finished

## Summary

Use the framework when you need to **consume** or **publish** in-ear state.

Use the pipeline when you need to **change how the wearing decision is computed**.

Treat the current pipeline as a concrete, tunable implementation, not as the permanent public contract of the module.
