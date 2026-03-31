# In-Ear Detection Framework

This module provides the shared firmware abstraction for "is the OpenEarable currently worn?".

## Responsibilities

- Maintain one global wearing state for the whole firmware.
- Maintain one global enable/disable state for the framework.
- Publish state changes through a dedicated Zephyr `zbus` channel.
- Offer a small public API for synchronous state queries and asynchronous subscriptions.
- Stay independent from the actual detection algorithm.

## Design

The framework does not implement wearing detection itself. Producers such as sensor-processing pipelines are expected to compute a decision and then call:

- `in_ear_detection_set_state(...)` for normal state updates.
- `in_ear_detection_set_enabled(...)`, `in_ear_detection_enable(...)`, or `in_ear_detection_disable(...)` to control whether detection logic is active.
- `in_ear_detection_publish(...)` when they need full control over the status payload.

Consumers can:

- Call `in_ear_detection_is_worn()` for a simple boolean check.
- Call `in_ear_detection_is_enabled()` to check whether the framework is active.
- Call `in_ear_detection_get(...)` to inspect the full status snapshot.
- Subscribe to `in_ear_detection_chan` or use `in_ear_detection_subscribe(...)` to receive change notifications.

## Bluetooth integration

The GATT service in :file:`src/bluetooth/gatt_services/in_ear_detection_service.c` exposes three characteristics:

- Detection enabled state (`read`, `write`, `notify`)
- Current in-ear state enum (`read`, `notify`)
- Current "is worn" boolean (`read`, `notify`)

## Extension points

The `in_ear_status_msg` payload contains metadata fields that are meant to evolve:

- `source` identifies the producer of a state decision.
- `flags` can carry algorithm-specific annotations.
- `change_counter` provides monotonic ordering for consumers.
- `timestamp_us` records when the accepted state change was published.

The public `enum in_ear_state` is intentionally small so application code can stay stable even if the internal detection logic becomes more sophisticated.
