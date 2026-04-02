# OpenEarable v2 Firmware — Code Quality Assessment

## Summary

The project-specific code is functional but has significant quality issues across thread safety, error handling, memory safety, and architectural cohesion. The codebase mixes C and C++ idioms inconsistently and has substantial dead code and magic numbers. Below is a prioritized breakdown.

---

## Critical Issues

### 1. Thread Safety — Pervasive Race Conditions

Nearly every module has shared mutable state accessed from ISR callbacks, work queues, and threads without synchronization:

- **`SensorManager.cpp`** — `active_sensors` counter modified from timer callbacks and config thread with no lock.
- **`Wire.cpp`** — mutex locking is **commented out** (lines 81, 93), so I2C transactions are completely unprotected.
- **`PowerManager.cpp`** — `power_on`, `charging_disabled`, `last_charging_state` modified from ISR callbacks without atomics or mutexes.
- **`time_sync.c`** — `time_offset_us` written by BLE callback, read by `get_current_time_us()`, no synchronization.
- **All sensor classes** — `_active`, `_running`, `_ble_stream` booleans accessed from timer interrupt and main thread.
- **All sensors** set `_active = false` **before** calling `k_timer_stop()`, so the timer can fire on a deactivated sensor. The correct order is to stop the timer first.
- **`sensor_service.c`** — `connection_complete` modified in ISR context but read in thread context without synchronization.
- **`led.c`** — `on_phase` static variable in `led_blink_work_handler` and `led_units` array modified without synchronization.
- **`Button.cpp`** — `button_cb_data` is static but modified per instance, causing cross-instance ISR interference.

### 2. Silent Error Swallowing

Return codes are logged but never propagated. The system continues in degraded states without the caller knowing:

- **`BQ25120a.cpp` / `BQ27220.cpp`** — I2C read failures return stale data (e.g. `voltage()` returns 0.0 on I2C failure, which looks like a dead battery).
- **`main.cpp:73-76`** — USB init failure returns 0 (success).
- **`PowerManager.cpp:631-634`** — four consecutive `pm_device_action_run()` calls, all return values ignored.
- **`sensor_service.c`** — busy-wait loop (`while(notify_count >= MAX_NOTIFIES_IN_FLIGHT) { k_yield(); }`) instead of a semaphore.
- **`BQ25120a.cpp:106-107`** — `readReg()` returns success even when `i2c_burst_read()` fails.
- **`PowerManager.cpp:337-345`** — load switch enable errors logged as warnings, execution continues regardless.

### 3. Memory Safety

- **`uicr.c`** — **known memory collision**: `MEM_ADDR_UICR_SNR` is 8 bytes wide but `MEM_ADDR_UICR_CH` is placed only 4 bytes after it, causing overlap. There's a comment acknowledging this: `// Michael: Collision with MEM_ADDR_UICR_CH`.
- **`SensorScheme.cpp:351`** — `sensorSchemesMap[id]` uses `operator[]` which **creates** entries for missing keys; should use `.find()`.
- **`BQ27220.cpp:343`** — `k_malloc()` with no null check.
- **`streamctrl.c:576`** — `memcpy(chip_id, data, sizeof(chip_id))` assumes `data` has at least 8 bytes, no bounds check.
- **`Equalizer.cpp`** — `data[n]` loop increments by 2 with no odd-length guard.
- **`sensor_service.c:262, 303`** — `active_sensor_configs` allocated with `k_malloc()`/`k_realloc()` but never freed.
- **`BQ27220.cpp:315`** — VLA (`uint8_t buf[len]`) is not standard C++ and allocates unknown size on stack.
- **`Wire.cpp:118-119, 124`** — TX buffer writes with no overflow check; silent truncation.

### 4. Unsafe Enum/Type Casts

- **`SensorManager.cpp:188`** — raw cast `(enum sensor_id) config.sensorId` from untrusted BLE data, no bounds check.
- **`Button.cpp:33`** — `gpio_pin_get_dt()` return (int) cast directly to `button_action` enum.
- **`hw_codec_adau1860.cpp:174-182`** — `#ifdef` splits an if/else chain, creating mismatched braces and different clamping behavior between FDSP and non-FDSP builds.
- **`PowerManager.cpp:407`** — uses `abs()` on float (should be `fabs()`), undefined behavior.
- **`time_sync.c:155-167`** — integer overflow check happens **after** the overflow (undefined behavior).

---

## Architectural Issues

### 5. PowerManager is a God Class

`PowerManager.cpp` (23KB) manages charging, fuel gauge, GPIO callbacks, LED control, BLE disconnect, reboot logic, battery reporting, and sleep/wake. It should be split into focused components (charging manager, battery monitor, power controller).

### 6. No Sensor Registry — Hardcoded Sensor Lists

Every time sensors are stopped (`SensorManager.cpp:131-136`), there's a manual list:
```cpp
Baro::sensor.stop();
IMU::sensor.stop();
PPG::sensor.stop();
Temp::sensor.stop();
BoneConduction::sensor.stop();
Microphone::sensor.stop();
```
Adding a sensor requires touching multiple files. A registry pattern would eliminate this.

### 7. Hardcoded GATT Attribute Indices

Throughout the BLE services: `sensor_service.attrs[4]`, `button_service.attrs[2]`, `sensor_service.attrs[7]`. If the `BT_GATT_SERVICE_DEFINE` macro changes, these silently break.

### 8. C/C++ Style Mixing

The codebase inconsistently mixes C and C++ patterns — `memcpy` on C++ objects (`StateIndicator.cpp:107`), C-style casts, raw pointers with no ownership model, `k_malloc` without RAII. The `Wire` class wraps Zephyr's C I2C API in an Arduino-style C++ interface but leaves callbacks unimplemented.

---

## Code Quality Issues

### 9. Magic Numbers Everywhere

- **`PowerManager.cpp:177`** — `0.8 * target_current - 2 * power_manager._battery_settings.i_term` (why 0.8? why 2x?).
- **`PowerManager.h:64-69`** — battery settings hardcoded with German comments: `3.7, 4.3, 3.0, 2.5, // Spannungen [V]`.
- **`BQ27220.cpp:447-456`** — raw RAM addresses (`0x9240`, `0x9243`, `0x9282`) with no named constants.
- **`DefaultSensors.h`** — `maxBleFrequencyIndex` values (2, 1, 12, 7, 17, 6) with no explanation.
- **`SensorScheme.cpp:100`** — `&parseInfo_service.attrs[5]` for notification attribute.
- **`BQ25120a.cpp:268-270`** — `0.8f + ((status >> 2 & 0x1F)) * 0.1f` with no explanation.
- **`led.c:21`** — `BLINK_FREQ_MS 1000` used as `K_MSEC(BLINK_FREQ_MS / 2)` without explaining why half.
- **`PPG.cpp:35-39`** — GPIO pin 6 hardcoded instead of coming from device tree.

### 10. Substantial Dead/Commented-Out Code

- **`PowerManager.cpp`** — ~50 lines of commented-out timer definitions, LED toggles, reset handlers, BT disable (with note "crashes").
- **`button_pressed.cpp`** — large block of commented-out multi-button handling.
- **`Baro.cpp:98-99`** — commented-out `bmp.start()` call (is the barometer actually starting?).
- **`BoneConduction.cpp:46-48`** — empty `reset()` with comment about "pulse oximeter" (copy-paste from PPG).
- **`Wire.cpp:81, 93`** — mutex lock/unlock commented out.
- **`Button.cpp:80-83`** — `gpio_remove_callback()` commented out, leaving dangling ISR callbacks.
- **`battery_service.cpp:47-48, 141-144`** — commented-out characteristics.
- **`EdgeMlSensor.h:6-14`** — commented-out macro block.

### 11. Spelling Errors in APIs

- **`Wire.h:59`, `TWIM.h:35`** — method named `aquire()` instead of `acquire()`.
- **`DefaultSensors.h:20`** — `microComponenents` instead of `microComponents`.
- **`DefaultSensors.h:21-22`** — `"INNER"` vs `"Outer"` case inconsistency.
- **`led_service.h:19`** — header guard says `AUDIO_PLAYER_H` (copy-paste error).

### 12. Incomplete TODOs in Production Paths

- **`streamctrl.c:607`** — `"TODO: check if the device wants to pair"`.
- **`PowerManager.cpp:366`** — `"TODO: check power on condition"`.
- **`PowerManager.cpp:619`** — `"TODO: check states of load switch"`.
- **`time_sync.c:146`** — `can_sync_time()` always returns true with TODO.
- **`uicr.h:12`** — `"TODO: Discuss better alternative for UICR storage"`.
- **`hw_codec_adau1860.cpp:64`** — `"TODO: make writing to bank work"` with 200ms sleep workaround.
- **`PowerManager.cpp:329`** — `"TODO: Flash red LED once"`.
- **`SensorManager.cpp:242`** — `"TODO: if (ble_sensors.empty()) ..."`.
- **`BQ25120a.cpp:65`** — `"TODO: check value"` for unjustified 1ms sleep.

### 13. Additional Issues

- **`ParseType.h`** — `parseTypeSizes[]` array depends on exact enum ordering with no `static_assert` to verify alignment. Adding an enum entry will silently break the mapping.
- **`SensorScheme.cpp:203-206`** — `strlen()` called twice on the same string instead of caching the result.
- **`SensorScheme.cpp:300-312`** — inconsistent error codes: some functions return `-1`, others return `-ENOMEM`.
- **`SensorManager.cpp:199-202`** — `active_sensors` counter going negative indicates a deeper double-stop bug.
- **`SensorManager.cpp:27-28`** — `sensor_service.h` included twice.
- **`PowerManager.cpp:356-359`** — `while(!power_on && battery_controller.power_connected())` infinite loop with no timeout.
- **`PPG.cpp:77`** — floating-point comparison without epsilon.
- **`uicr.c:38-40`** — NVM write verification reads immediately after async write, may get stale value.
- **`uicr.c:86-100`** — `uicr_hw_revision_get()` takes `char*` with hardcoded `snprintf` size 16, no way for caller to specify buffer size.
- **`SDLogger.cpp`** — `is_open` is a public bool accessed from multiple files without synchronization; should be private with atomic access.
- **`Temp.cpp:25-46`** — `_active` set twice in same function, power management calls unbalanced on failure paths.

---

## What's Done Well

- **ZBUS adoption** for inter-module messaging is a good architectural choice.
- **Device tree integration** for hardware configuration is proper Zephyr practice.
- **Sensor abstraction** via `EdgeMlSensor` base class is a reasonable pattern (though underutilized).
- **Build configuration** split (debug/FOTA/release) is clean.
- **SD card logging** with ring buffers and background flush is a reasonable design.
- **Board definition structure** follows Zephyr conventions well.

---

## Recommended Fix Priority

1. **Fix the UICR memory collision** — this is a data corruption bug in production.
2. **Re-enable Wire mutex locking** — concurrent I2C access will corrupt transactions.
3. **Add atomic/mutex protection** to shared state in sensors, PowerManager, time_sync.
4. **Stop timer before setting flags** in all sensor `stop()` methods.
5. **Propagate errors** from I2C reads in BQ25120a/BQ27220 instead of returning stale data.
6. **Replace `sensorSchemesMap[id]` with `.find()`** to prevent silent map entry creation.
7. **Extract magic numbers** into named constants, especially battery parameters and register addresses.
8. **Split PowerManager** into focused classes with single responsibilities.
9. **Implement sensor registry** to eliminate hardcoded sensor lists.
10. **Remove dead code** and resolve or remove TODOs.
