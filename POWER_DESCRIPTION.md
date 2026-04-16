# Power Management Review — OpenEarable v2

## Context

This is a **code review / analysis**, not an implementation plan. The user asked for a
summary of the firmware's power-management strategy and any issues with it, after
reading Zephyr's PM documentation and walking through `PowerManager.cpp` and the
device/sensor drivers.

Reference reading:
- Zephyr PM: https://docs.zephyrproject.org/latest/services/pm/index.html
- Zephyr Device PM: https://docs.zephyrproject.org/latest/services/pm/device.html
- Zephyr Device Runtime PM: https://docs.zephyrproject.org/latest/services/pm/device_runtime.html

---

## 0. Target design: boot sequence and load-switch lifecycle

### Hardware power rails (from schematic)

| Rail | Device | GPIO | Powers |
|------|--------|------|--------|
| **V_LS** (`ls_1_8`) | AP22118CA4-7 | `gpio1.11` | I2C2/I2C3 pull-ups, MX25R6435F flash VCC, SD card SPI level-shifter low side, sensors (1.8 V supply via connector) |
| **LS_LDO** (`ls_3_3`) | BQ25120A internal | `gpio0.14` | KTD2026 LED controller VIN, PPG LDO, sensors (3.3 V supply via connector), audio codec |
| **V_SD** (`ls_sd`) | AP22118CA4-7 | `gpio1.12` | SD card VDD, SD card SPI level-shifter high side |

**i2c1 pull-ups are on a permanent supply** (not V_LS). This is why
`PowerManager::begin` can talk to BQ25120A / BQ27220 / KTD2026 before any load
switch is up.

### Which rails each consumer needs

| Consumer | `ls_1_8` | `ls_3_3` | `ls_sd` | Notes |
|----------|:--------:|:--------:|:-------:|-------|
| Sensors (I2C3: BMA580, BMX160, BMP388, MLX90632) | **yes** | some | — | I2C3 pull-ups on V_LS; MLX90632/BMA580 also need 3.3 V |
| PPG (MAXM86161, I2C2) | **yes** | **yes** | — | I2C2 pull-ups on V_LS; PPG LDO on 3.3 V |
| Audio codec (ADAU1860, I2C2) | **yes** | — | — | I2C2 pull-ups on V_LS; codec enable via separate GPIO |
| LED controller (KTD2026, I2C1) | — | **yes** | — | I2C1 pull-ups on permanent supply; VIN needs 3.3 V |
| SPI NOR flash (MX25R6435F) | **yes** | — | — | VCC on V_LS; DFU only in the app |
| SD card (SDHC on SPI4) | **yes** | — | **yes** | Level-shifter needs V_LS (low) + V_SD (high) |
| BQ25120A / BQ27220 (I2C1) | — | — | — | Permanent supply |

### Target idle profile

"Device on, BLE advertising, no sensors, no SD logging, no audio, no DFU" plus
a status LED flashing to indicate state:

- `ls_1_8`: **OFF** (unless an SD card is physically present → ON for level-shifter)
- `ls_3_3`: **ON** while LED is active (KTD2026 owns via `_get`/`_put`)
- `ls_sd`: **OFF** (unless an SD card is physically present → ON)

### Target boot sequence

```
T=0   Reset
      └→ mcuboot starts
         └→ SYS_INIT init_load_switch (PRE_KERNEL_2, prio 80)
            ├→ gpio1.11 = HIGH  (ls_1_8 ON — needed for SPI NOR flash)
            └→ gpio0.14 = HIGH  (ls_3_3 ON — needed for status LED)
         └→ mcuboot reads/verifies image, blinks LED
         └→ mcuboot jumps to app  (GPIO state preserved: ls_1_8 HIGH, ls_3_3 HIGH)

T=?   Application kernel init  (mcuboot is gone; no reset; GPIOs intact)
      └→ POST_KERNEL prio 80: board_init.c device init
         ├→ init_pm_device(ls_1_8): gpio → INACTIVE (HIGH→LOW transient)
         ├→ init_pm_device(ls_3_3): gpio → INACTIVE (HIGH→LOW transient)
         └→ init_pm_device(ls_sd):  gpio → INACTIVE (was already off)
         Framework state: all three SUSPENDED, refcount 0
      └→ spi_nor driver init (mx25r6435f)
         pm_device_driver_init checks pm_device_is_powered(dev):
           - power-domains = <&load_switch> → parent is SUSPENDED
           - pm_device_is_powered returns FALSE
           → pm_device_init_off(dev); return 0;
           NO SPI transactions. NO infinite-loop hang.
      └→ sdhc_spi driver init (sdhc0)
         Only checks device_is_ready(spi_dev). No SPI I/O. No hang.

T=?   main() → power_manager.begin()
      ├→ Talks to BQ25120A/BQ27220 on i2c1 (permanent supply — no load switch)
      ├→ Reads battery, reset reason, button state
      ├→ Claims ZERO load switches
      ├→ If bad battery / no button: power_down()  (rails stay off, sys_poweroff)
      └→ If powering on:
         ├→ state_indicator.init → KTD2026::begin
         │  └→ _get(ls_3_3)  → RESUME → gpio HIGH → LED on
         ├→ SDCardManager::init → reads sd_state GPIO
         │  └→ if card present: _get(ls_1_8) + _get(ls_sd)
         ├→ Sensors started later (BLE client configures)
         │  └→ sensor.init → _get(ls_1_8) [+ _get(ls_3_3) for some]
         ├→ Audio started later (streaming begins)
         │  └→ dac.begin → _get(ls_1_8)
         └→ DFU (MCUmgr upload over BLE)
            └→ hook: _get(mx25r64) → cascades via power-domains to _get(ls_1_8)
               → TURN_ON: spi_nor_configure (JEDEC ID, SFDP, mxicy config)
               → RESUME: exit DPD → flash ready
            └→ upload completes: _put(mx25r64) → SUSPEND (enter DPD)
               → cascades _put(ls_1_8) → if refcount=0, SUSPEND → gpio LOW
```

### Changes required

**DTS — `boards/openearable_v2_nrf5340_cpuapp.overlay`:**
```dts
mx25r64: mx25r6435f@1 {
    ...
    power-domains = <&load_switch>;        /* NEW: declares V_LS dependency */
    zephyr,pm-device-runtime-auto;         /* NEW: defer init until first _get */
};
```

**DTS — `openearable_v2_nrf5340_cpuapp_common.dts`** (optional cleanup):
Add `zephyr,pm-device-runtime-auto;` to all three load-switch nodes, allowing
the explicit `pm_device_runtime_enable` calls in PowerManager.cpp to be deleted.

**Kconfig — `prj.conf`:**
```
CONFIG_PM_DEVICE_POWER_DOMAIN=y            /* NEW: enables pm_device_is_powered check */
```

**`src/drivers/LED_Controller/KTD2026.cpp`:**
- Delete `pm_device_runtime_get(ls_1_8)` in `begin()` (line 38)
- Delete `pm_device_runtime_put(ls_1_8)` in `power_off()` (line 58)
- i2c1 pull-ups are on permanent supply; KTD2026 VIN is on `ls_3_3` only

**`src/Battery/PowerManager.cpp`:**
- Delete the conditional `pm_device_runtime_enable` + `_get` block inside
  `if (charging)` in `begin()`.
- Delete the unconditional enable + `_get` block after `if (!power_on) return
  power_down()` in `begin()` (all three rails).
- If `pm-device-runtime-auto` is set in DTS: delete all `pm_device_runtime_enable`
  calls; they're automatic. Otherwise keep just the three enables, no gets.

**`src/SD_Card/SD_Card_Manager/SD_Card_Manager.cpp`:**
- `aquire_ls()` / `release_ls()`: claim `ls_1_8` + `ls_sd` only (not `ls_3_3`;
  SD path doesn't need it per schematic)
- New: a card-present base claim, separate from mount/unmount. In `init()`,
  read `sd_state_pin`; if card present, acquire the two rails immediately (so
  USB MSC can see the card without a FW-side mount). On the state-change ISR
  debounce handler, add an insertion branch that acquires rails; the existing
  removal branch releases them.
- `mount()` / `unmount()` stop calling `aquire_ls()` / `release_ls()`. They
  just toggle USB MSC and fs_mount state. The card-present claim holds the rails
  for USB MSC; mount/unmount gate firmware-side filesystem access.

**MCUmgr DFU hook (new code):**
- Register via `CONFIG_MCUMGR_GRP_IMG_UPLOAD_CHECK_HOOK` (already enabled in
  prj.conf) or `CONFIG_MCUMGR_SMP_COMMAND_STATUS_HOOKS`.
- On upload start: `pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(mx25r64)))`.
  The power-domains cascade handles `ls_1_8`; the spi_nor driver handles chip init
  (TURN_ON) and DPD exit (RESUME).
- On upload end: `pm_device_runtime_put(...)`. Driver enters DPD (SUSPEND);
  if `ls_1_8` refcount drops to 0, load switch goes off.

### Why the boot hang is fixed

The Zephyr `spi_nor` driver's init function (`spi_nor.c:1602`) calls
`pm_device_driver_init(dev, spi_nor_pm_control)` at line 1635.
`pm_device_driver_init` (`subsys/pm/device.c:358-395`) immediately checks
`pm_device_is_powered(dev)`:

```c
if (!pm_device_is_powered(dev)) {
    pm_device_init_off(dev);
    return 0;       /* ← no TURN_ON, no SPI I/O, no hang */
}
```

`pm_device_is_powered` (`subsys/pm/device.c:341-356`) returns false when
`CONFIG_PM_DEVICE_POWER_DOMAIN` is enabled AND the device has a `power-domains`
parent AND that parent's state is not ACTIVE.

Today the hang happens because `mx25r64` has **no** `power-domains` property and
`CONFIG_PM_DEVICE_POWER_DOMAIN` is not set, so `pm_device_is_powered` always
returns true. The driver proceeds to `spi_nor_configure` → `spi_nor_rdsr` reads
garbage (0xFF) from the unpowered chip → `spi_nor_wait_until_ready` enters an
**unbounded `while (true)` loop** (`spi_nor.c:479`) that never exits because the
WIP bit (0x01) appears set in the garbage. Boot hangs before `main()`.

### Verification plan

1. Add the `power-domains` + `pm-device-runtime-auto` to `mx25r64` in DTS,
   `CONFIG_PM_DEVICE_POWER_DOMAIN=y` in prj.conf. Build. Boot. Confirm device
   reaches `main()` and PowerManager::begin completes (UART log shows battery
   info). Confirm `mx25r64` is in OFF state (no SPI NOR traffic on scope/LA).

2. Trigger a DFU upload. Confirm `ls_1_8` comes up (scope on gpio1.11), flash
   write succeeds, `ls_1_8` goes back down after upload completes.

3. Remove the `pm_device_runtime_get(ls_1_8)` / `pm_device_runtime_get(ls_sd)`
   from PowerManager::begin. Remove `ls_1_8` from KTD2026. Boot. Confirm:
   - gpio1.11 LOW after boot (ls_1_8 off)
   - gpio0.14 HIGH when LED is active (ls_3_3 on)
   - gpio1.12 LOW (ls_sd off if no card) or HIGH (ls_sd on if card present)

4. Insert SD card. Confirm USB MSC mounts on the host. Remove card. Confirm
   rails drop. Re-insert. Confirm rails come back and USB MSC works.

5. Start a sensor stream via BLE. Confirm ls_1_8 comes up. Stop the stream.
   Confirm ls_1_8 goes down (assuming no SD card present).

---

## 1. Zephyr PM background (one-paragraph refresher)

Zephyr separates **System PM** (CPU sleep states, set by a policy callback during the
idle thread) from **Device PM** (per-driver action callbacks: `TURN_ON`, `RESUME`,
`SUSPEND`, `TURN_OFF`). Device Runtime PM (`CONFIG_PM_DEVICE_RUNTIME`) layers a
reference counter on top: drivers call `pm_device_runtime_get()` before using a device
and `pm_device_runtime_put()` after, and the framework calls the action callback to
flip ACTIVE↔SUSPENDED automatically. Devices can be tied together via the
`power-domains` DT property, so getting a child also brings up its supply, and the
`zephyr,pm-device-runtime-auto` DT property makes the framework enable runtime PM
without an explicit `pm_device_runtime_enable()` in code.

---

## 2. What OpenEarable v2 actually does

**Configuration** (`prj.conf:125-128`):
```
CONFIG_PM=y
CONFIG_PM_DEVICE=y
CONFIG_PM_DEVICE_RUNTIME=y
CONFIG_POWEROFF=y
```
System PM is *enabled* but no PM policy hooks or sleep states are exercised — the
application never participates in system suspend. The MCU only ever runs at full
clock or fully off (`sys_poweroff()`).

**Architecture in three layers:**

1. **Application policy — `PowerManager` (`src/Battery/PowerManager.{h,cpp}`)**
   - Custom 10-state battery/charging state machine fed by BQ27220 (fuel gauge) +
     BQ25120A (charger) over I²C1, driven by two work items
     (`charge_ctrl_delayable`, `fuel_gauge_work`) and three GPIO callbacks
     (`power_good_callback`, `fuel_gauge_callback`, `battery_controller_callback`).
     Classification is factored into a pure `classify_charging()` helper (§6).
   - Owns the boot/power-on/power-off sequence: `begin()` and `power_down()`.
   - Publishes battery state on the `battery_chan` ZBUS channel.

2. **Power gating — three custom load-switch "devices"**
   `boards/teco/openearable_v2/board_init.c` defines three `DEVICE_DT_DEFINE`s
   (`ls_1_8`, `ls_3_3`, `ls_sd`) using a generic PM control callback that just
   toggles a GPIO and waits `power-delay-us`:
   ```c
   case PM_DEVICE_ACTION_SUSPEND: gpio_pin_set_dt(&data->ctrl_pin, 0); break;
   case PM_DEVICE_ACTION_RESUME : gpio_pin_set_dt(&data->ctrl_pin, 1);
                                  k_usleep(data->delay_us); break;
   ```
   The load switches use `pm_device_runtime_*` for refcounting. `ls_3_3` is the
   3.3 V LDO inside the BQ25120A (DT child of `bq25120a@6a`); `ls_1_8` and `ls_sd`
   are GPIO load switches on `gpio1.11` / `gpio1.12`. `PowerManager::begin()`
   `pm_device_runtime_enable`s all three and then `pm_device_runtime_get`s them
   — see §3.8, §3.9.

3. **Application-level sensor/peripheral wrappers**
   Every sensor driver (`IMU.cpp`, `Baro.cpp`, `BoneConduction.cpp`, `PPG.cpp`,
   `Temp.cpp`), the LED driver (`KTD2026.cpp`), the audio codec (`ADAU1860.cpp`),
   and the SD card manager (`SD_Card_Manager.cpp`) call
   `pm_device_runtime_get(ls_*)` in `init()`/`begin()` and
   `pm_device_runtime_put(ls_*)` in `stop()`/`power_off()`. None of the sensors
   themselves are real Zephyr devices — they're declared as
   `compatible = "i2c-device"` and accessed through C++ wrappers, so they have no
   PM action callbacks of their own.

**Power-down sequence (`PowerManager::power_down`):**
1. Disconnect all BLE connections (`bt_conn_foreach` → `bt_mgmt_conn_disconnect`)
2. Stop external advertising
3. `led_controller.begin()` then `power_off()`  ← see §3.3
4. `stop_sensor_manager()` (drives every sensor's `stop()` → releases load switches)
5. If not charging: arm BQ25120A and BQ27220 wake interrupts; high-impedance the charger
6. Stop the BLE watchdog, `dac.end()`, clear error LED
7. If charging: `sys_reboot()` and return
8. Otherwise: disable BQ25120A LS_LDO, then `pm_device_action_run(SUSPEND)` on
   `ls_sd`, `ls_3_3`, `ls_1_8`, `cons` (the UART console)
9. `sys_poweroff()`; fallback `k_msleep(1000); sys_reboot()`

**Sensor pattern (typical):**
```cpp
bool Sensor::init() {
    pm_device_runtime_get(ls_1_8);   // and ls_3_3 if needed
    if (!hw.begin()) { pm_device_runtime_put(ls_1_8); return false; }
    ...
}
void Sensor::stop() {
    if (!_active) return;
    _active = false; _running = false;
    k_timer_stop(...); k_work_cancel(...);
    hw.softReset()/stop()/sleepMode();   // varies — see §3.4
    pm_device_runtime_put(ls_1_8);
}
```

---

## 3. Issues

### Critical

#### 3.1 No mid-power "BLE advertising only" state
During normal "idle" operation (advertising or connected) the BLE controller
(network core + RF) is the dominant power consumer. There is no
advertising-only / sensors-off mid-power state — the device is "everything on"
or "off". The 12 s power-on delay reported in `PROBLEMS.md` may also be
downstream of BLE init being slow. The **poweroff** path is handled — see §3.1a.

#### 3.1a `bt_disable()` in `power_down()` — works
Empirically tested on-device: `bt_disable()` returns 0 in ~234 ms and the audit
reports `netcore_off=1`. Code path: `bt_disable()` → HCI driver close hook →
`bt_hci_transport_teardown()` at `zephyr/drivers/bluetooth/hci/nrf53_support.c:23`
→ `nrf53_cpunet_enable(false)` → onoff refcount → 0 → `onoff_stop` callback
writes `NETWORK.FORCEOFF = Hold` via the HAL. Upstream Zephyr, not NCS-specific.
The old "crashes / doesn't wake up" comment in the source was stale and has
been removed. A 200 ms sleep before the call lets earlier `bt_conn_disconnect`
events propagate.

#### 3.2 No system PM is actually used
`CONFIG_PM=y` is set but the application never enters a low-power sleep state.
Zephyr's tickless idle reduces wakeups, but the SoC stays in CONSTLAT, the HF clock is
likely running, and the I²C peripheral blocks (see 3.6) are powered. There is no PM
policy callback, no `pm_state_force()`, no constraint API usage. The "low power" path
is jump-straight-to-`sys_poweroff()`.

#### 3.3 `power_down` re-initialises the LED controller before turning it off
Inside `power_down()`:
```cpp
led_controller.begin();
led_controller.power_off();
```
`KTD2026::begin()` calls `pm_device_runtime_get(ls_1_8)` +
`pm_device_runtime_get(ls_3_3)`, sleeps 10 ms, talks to the chip, and resets it.
`power_off()` then writes the mute register and `_put`s both rails. So if the LED was
already off (`_active=false`), this path turns ls_3_3 on, talks to the chip, then turns
it back off — pure waste during a shutdown that's already racing the user's button.
If the LED was on, `begin()` is short-circuited by `if (_active) return;` — but the
fact that this whole block is unconditional makes it look like a copy/paste bug.
Almost certainly a regression and should just be deleted.

#### 3.4 Sensors don't actually sleep before their rail is cut
The "graceful shutdown before pm_device_runtime_put" pattern is implemented
inconsistently:

| Sensor          | Driver               | Pre-cut shutdown                          |
|-----------------|----------------------|-------------------------------------------|
| BMX160 (IMU)    | `IMU.cpp:99-112`     | `imu.softReset()` only — comment `// turn off imu (?)`. Suspend mode regs (0x10/0x14/0x18) NOT used |
| BMP388 (Baro)   | `Baro.cpp:111-121`   | **Nothing** — direct `pm_device_runtime_put` |
| BMA580 (Bone)   | `BoneConduction.cpp:140-153` | `bma580.stop()` (sets BMA5_REG_CMD_SUSPEND) ✓ |
| MAXM86161 (PPG) | `PPG.cpp:156-169`    | `ppg.stop()` (REG_SYSTEM_CONTROL shutdown bit) ✓ |
| MLX90632 (Temp) | `Temp.cpp`           | `temp.sleepMode()` ✓ |

For BMP388 and BMX160 the rail is hard-cut while the chip is still in active mode.
This is suboptimal for the IC and — more importantly — it leaves the I²C lines
floating from the device's perspective for a moment, which is a credible suspect
for the **i2c3 sensor-dropout regression** described in `PROBLEMS.md`. (BMP388,
BMA580, MLX90632 are exactly the i2c3 sensors that drop out; BMX160 is on the same
bus but is the only one mentioned as still working — which is curious.)

#### 3.5 PPG manually drives a GPIO LDO that is never released
`PPG.cpp:35-45` constructs a `gpio_dt_spec` for `gpio0.6` and asserts it as the PPG
LDO enable, then sleeps 5 ms. There is **no matching teardown** in `PPG::stop()` — the
GPIO stays HIGH for the rest of the device's lifetime, leaving an external LDO
enabled even when PPG is idle. This same block also leaks the `ls_1_8`/`ls_3_3`
refcounts on the error path: `gpio_pin_configure_dt` failing returns `false` after
the two `pm_device_runtime_get` calls, without putting either back, and without
setting `_active=true` so a later `stop()` can't recover them.

#### 3.6 I²C buses are never gated
`i2c1`, `i2c2`, `i2c3` all have `status = "okay"` and define both `default` and
`sleep` pinctrl states (`*.dts:95-183`). But nothing ever drives the bus into the
sleep state — `nordic,nrf-twim` only flips pinctrl when its own PM action callback
runs, and nobody calls `pm_device_action_run` on the I²C controllers. So the TWIM
peripheral blocks are clocked continuously.

### High

#### 3.7 No `power-domains` linkage between sensors and load switches
Sensor DT nodes (`bmp388@76`, `bma580@18`, `bmx160@68`, `mlx90632@3a`,
`maxm86161@62`) declare no `power-domains` property. Combined with the fact that
they're `compatible = "i2c-device"` and have no driver/PM callback, the load-switch
refcount and the sensor's "active" state are coordinated only by the C++ wrappers.
Easy to leak (see 3.5) and easy to introduce ordering bugs.

#### 3.8 `ls_1_8` and `ls_sd` are pinned on for the device's entire lifetime
`PowerManager::begin()` does `pm_device_runtime_get(ls_1_8)` +
`pm_device_runtime_get(ls_3_3)` + `pm_device_runtime_get(ls_sd)` at the bottom
of the init path (and again, conditionally, on the charging-at-boot branch).
The matching `_put`s only happen inside `power_down()`. So even when nothing is
streaming and no SD I/O is happening, all three rails stay on. This makes
`SDCardManager::aquire_ls`/`release_ls` (which carefully refcounts all three
rails for mount/unmount) a no-op in practice — the floor on each rail is ≥1.

#### 3.9 Begin-time load-switch enablement is duplicated and order-sensitive
`PowerManager::begin()` enables all three load switches on the non-charging
path, and separately enables them inside the `if (charging)` branch — which
runs *before* the main-path enable block. On the charging path the first
`pm_device_runtime_enable(ls_3_3)` happens before `state_indicator.init` calls
`KTD2026::begin()` (which claims ls_3_3). On the non-charging path, the
comment at the enable block notes "With `pm_device_init_suspended()` in
board_init, `_enable` is a no-op (no glitch)." — which implies there was a
glitch concern. The whole duplication goes away if `zephyr,pm-device-runtime-auto`
is added to the load-switch DTS nodes (see §0 "Changes required" and §3.13).

### Medium

#### 3.10 `power_down` ordering: sensors are stopped before BLE is disconnected
`power_down()` sequence: `bt_conn_foreach` → disconnect, `bt_mgmt_ext_adv_stop`,
LED off, `stop_sensor_manager()`. But `bt_conn_disconnect` is asynchronous — by
the time we get to `stop_sensor_manager()`, peer disconnect events may not have
completed. We then `dac.end()` and stop the watchdog while BLE is still tearing
down. Re-ordering the audio/sensor teardown to *after* BLE has fully torn down
would be safer, and would also let us actually call `bt_disable()` once the
underlying ISO/audio paths are released.

#### 3.11 The `DEBOUNCE_POWER_MS = K_MSEC(1000)` constant is unused at the place that needs it
`PowerManager.h` defines it, but `power_good_callback` schedules
`power_down_work` with `K_NO_WAIT`. So unplugging USB while powered-off triggers
an immediate `power_down`, with no debounce. Not catastrophic but the named
constant is misleading.

#### 3.12 No fault-recovery path for non-undervoltage faults
In `classify_charging`, only BAT_UVLO (bit 5) has a recovery transition (→
PRECHARGING when power is connected and current is flowing). OVP / TS / input-UV
faults fall through to `FAULT` and re-enter it on the next poll with no exit.
The work handler does re-run `setup_pmic()` on a TS fault, which might un-stick
the TS path, but everything else requires a user power-cycle.

### Low / code-quality

#### 3.13 No `zephyr,pm-device-runtime-auto` on the load-switch nodes
Adding it to `load_switch`, `load_switch_sd`, and the BQ25120A child `load-switch`
in `*.dts:26-123` would let us delete the explicit `pm_device_runtime_enable`
plumbing in `PowerManager::begin()`.

#### 3.14 Synchronous `pm_device_runtime_put` everywhere
Some shutdown paths (especially PPG, which talks to the chip) could use
`pm_device_runtime_put_async` so the suspend completes off the calling thread.
Probably doesn't matter for the rare power-down case but might smooth sensor
reconfiguration.

#### 3.15 Sensors aren't real Zephyr devices
Compatible = `"i2c-device"` means: no `init_fn`, no `pm_action_cb`, no
`PM_DEVICE_DT_INST_DEFINE`. We're paying the price (manual coordination, no domains,
no auto-runtime, can't piggy-back on system PM) without the benefit (smaller code).
This is a structural choice — converting to real drivers is a larger refactor — but
it's the root cause of half the issues above.

---

## 4. Likely connections to PROBLEMS.md regressions

`PROBLEMS.md` lists three live regressions. Possible links:

1. **i2c3 sensor dropouts (temp, baro, bone)** — best candidates are §3.4 (BMP388
   has zero pre-cut shutdown; BMA580's stop is fine; the temperature one *does* call
   `sleepMode`) and §3.9 (load-switch enable ordering). The IMU still working
   *despite* being on the same bus and rail is the most diagnostic clue: it's the
   one that calls `softReset` (which is more disruptive than nothing).

2. **12 s power-on press** — most likely a BQ25120A button-hold-time register
   change. The `Push-button Control` register is 0x08; bits `MRRESET[1:0]` set
   the MR-hold → reset time (00=5s, 01=9s default, 10=11s, 11=15s), and bit
   `MRWAKE2` sets the WAKE2 latch time (0=1000ms, 1=1500ms default). Firmware
   never writes 0x08, so timings sit at reset defaults; a previous firmware
   that felt like "~4s" almost certainly wrote `MRRESET=00`. `PowerManager`
   itself doesn't gate on time — only on the WAKE2 latch.

3. **Bone conductor data loss at 6400 Hz with SD logging** — primarily a CPU /
   bus-contention issue (i2c3 at 1 MHz vs spi4 SD at 32 MHz, both DMA but both
   draining the same RAM). Power-management adjacent only insofar as nothing is
   ever clock-gated, so we run at full power throughout.

---

## 5. Suggested next steps (in priority order)

**Phase 1 — Unblock the target design (§0)**

1. **DTS + Kconfig: wire mx25r64 into power-domains.** Add
   `power-domains = <&load_switch>;` and `zephyr,pm-device-runtime-auto;` to the
   `mx25r64` node in `boards/openearable_v2_nrf5340_cpuapp.overlay`. Add
   `CONFIG_PM_DEVICE_POWER_DOMAIN=y` to `prj.conf`. This eliminates the boot hang
   that currently forces `ls_1_8` to be always-on (see §0 "Why the boot hang is
   fixed"). **Verify on-device that the board boots to `main()` with `ls_1_8` off.**

2. **Remove KTD2026's unnecessary `ls_1_8` claim** (`KTD2026.cpp:38, 58`). LED
   controller is on i2c1 (permanent pull-ups per schematic); it only needs `ls_3_3`.

3. **PowerManager::begin claims zero load switches.** Delete both enable blocks
   in `begin()` (the conditional one inside `if (charging)` and the unconditional
   one after `if (!power_on) return power_down()`), plus their
   `pm_device_runtime_get(ls_1_8)` / `pm_device_runtime_get(ls_3_3)` /
   `pm_device_runtime_get(ls_sd)` calls. Add `zephyr,pm-device-runtime-auto;` to
   the three load-switch DTS nodes so enable is automatic. Or keep explicit
   `pm_device_runtime_enable` calls but NO `_get`.

4. **SDCardManager: presence-based rail ownership.** `init()` reads `sd_state_pin`;
   if card present, `_get(ls_1_8)` + `_get(ls_sd)`. State-change ISR debounce
   handler gains an insertion branch that `_get`s, matching the existing removal
   branch that `_put`s. `mount()`/`unmount()` stop calling `aquire_ls`/`release_ls`
   — they just toggle USB MSC and `fs_mount`. `aquire_ls` drops `ls_3_3` (SD card
   doesn't need it per schematic). **Re-test USB MSC mount/unmount with card
   insert/remove.**

5. **MCUmgr DFU hook.** Register via the existing
   `CONFIG_MCUMGR_GRP_IMG_UPLOAD_CHECK_HOOK`. On upload start:
   `pm_device_runtime_get(mx25r64_dev)` (cascades to `ls_1_8` via power-domains).
   On upload end: `pm_device_runtime_put(mx25r64_dev)`.

**Phase 2 — Fix existing issues (§3)**

6. **Delete the spurious `led_controller.begin()` call in `power_down()`** (§3.3)
   and verify shutdown still works.
7. **Add proper sleep modes to BMX160 and BMP388** (`IMU::stop()` and
   `Baro::stop()`) before `pm_device_runtime_put`. Then re-test the i2c3 dropout
   regression. (§3.4)
8. **Fix the PPG GPIO LDO leak** (release `gpio0.6` in `PPG::stop()`; fix the
   error path in `PPG::init()`). (§3.5)
9. **`git diff` `BQ25120a.cpp`** against the last known-good firmware for the
   button-hold-time register change. (§4 / 12 s power-on)

**Phase 3 — Stretch goals**

10. **Convert sensors to real Zephyr drivers** with PM action callbacks and
    `power-domains` pointing at load switches (§3.15).

---

## 6. Charging code: structure

The charging-related code lives in `src/Battery/PowerManager.{h,cpp}` and
`src/Battery/BQ25120a.{h,cpp}`. A few patterns are load-bearing; grepping for
these names will orient most changes.

### `BQ25120a::ChargePhase` — typed enum for the 2-bit charging-phase field

`enum class ChargePhase : uint8_t { Discharge, Charging, Done, Fault }`. The
register-0x00 `>> 6` shift lives in a single accessor,
`BQ25120a::read_charge_phase()`. Everything that cares about phase (the work
handler's big switch, `get_battery_status` GATT encoding, `cmd_battery_info`)
uses the enum. One diagnostic (`cmd_sensor_diag`) still prints the raw byte
inline — deliberate, because it also dumps the full register for debugging.

### `BQ25120a::ActiveScope` — RAII guard for I2C access to the charger

I2C reads/writes to the BQ25120A require the chip to be out of high-impedance
mode (CD pin low) while the transaction happens. `ActiveScope` is a refcounted
RAII guard with a `k_mutex` that brackets any such access: the 0→1 transition
calls `exit_high_impedance()`, the 1→0 transition calls `enter_high_impedance()`,
and nested/concurrent scopes compose correctly. Call-site usage:

```cpp
{
    BQ25120a::ActiveScope active(battery_controller);
    auto phase = battery_controller.read_charge_phase();
    // ...
}
```

Most sites use the scope. A few exceptions: `PowerManager::begin()` spans ~100
lines of init between a manual `exit_high_impedance()` and `enter_high_impedance()`
because early returns into `power_down()` would dismiss a scope guard at the wrong
time; and `power_down()` itself has a belt-and-suspenders standalone
`enter_high_impedance()` after `set_wakeup_int`. `BQ25120a::setup()` and
`BQ25120a::write_LDO_voltage_control()` use the scope internally.

### `charger_init_pending` — ISR → work-queue handoff for charger re-init

`power_good_callback` runs in GPIO ISR context on USB plug-in/plug-out. I2C can't
run there, so it can't call `setup_pmic()` directly. Instead, on plug-in it sets
`charger_init_pending = true` and schedules the `charge_ctrl_delayable` work
item. `charge_task` (work context) sees the flag, clears it, runs `setup_pmic()`
+ `enable_charge()`, then submits `fuel_gauge_work`. Boot doesn't need to seed
the flag — `begin()` calls `setup_pmic()` once up front.

### `PowerManager::setup_pmic()` — single entry point for charger configuration

All charger-configure paths (`begin()`, `charge_task()` on plug-in,
`fuel_gauge_work_handler` on TS-fault recovery, the `sensor_diag` shell command)
route through `setup_pmic()`. `_battery_settings` is referenced in exactly one
place outside the member declaration. `BQ25120a::setup()` holds its own
`ActiveScope` internally, so `setup_pmic()` adds no extra bracketing.

### `BQ25120a::fault_bits[]` — single source of truth for fault-register labels

A constexpr array of `{mask, name}` pairs for register 0x01 (faults). The work
handler's `LOG_WRN` loop and the shell diag's `shell_print` loop both iterate
this array. Labels follow BQ25120A datasheet Table 13 exactly:
B4=BAT_OCP (cleared on read), B5=BAT_UVLO (persistent), B6=VIN_UV (cleared on
read), B7=VIN_OV (persistent). Heads-up: upstream commit `91d4081` swapped the
B4/B7 names in 2025-05; the swap is fixed here but still lives in upstream.

### `classify_charging()` — pure state classifier

`fuel_gauge_work_handler` separates three concerns:

1. **Read** hardware state into `struct charging_snapshot` (phase, `bat_status`,
   `gauge_status`, fault byte, voltage, current, target current, `power_connected`).
2. **Classify** via file-static pure `classify_charging(snapshot, cfg) → enum
   charging_state`. Zero I/O, no logging, no side effects — unit-testable in
   isolation (no tests written yet).
3. **Side effects** on the result: logging by phase, TS-fault recovery
   (`setup_pmic()`), zbus publish, polling-interval adjustment.

The undervoltage-recovery edge case (BAT_UVLO + power connected + current
flowing → PRECHARGING) is a single branch inside the classifier rather than a
side-effecting hairball in the switch.

### `cmd_battery_info` vs. work-handler LOG_DBG dump

Both print overlapping battery snapshots in slightly different formats. Not
consolidated; a shared formatter taking a `printk`/`shell_print` callback is the
natural next step if fields need to be added.

---

## 7. Files referenced

| File | Notes |
|------|-------|
| `src/Battery/PowerManager.h` | Class definition; `DEBOUNCE_POWER_MS`; battery_settings constants |
| `src/Battery/PowerManager.cpp` | `begin`, `power_down`, work handlers, shell commands; most of §3 lives here |
| `src/Battery/BQ25120a.{cpp,h}` | Charger; HiZ + `ActiveScope`, `ChargePhase`, `fault_bits`; button-hold-time registers (0x08) are not written, see §4 / 12 s power-on |
| `src/Battery/BQ27220.{cpp,h}` | Fuel gauge; wakeup interrupts |
| `boards/teco/openearable_v2/board_init.c` | Custom load-switch PM devices |
| `boards/teco/openearable_v2/openearable_v2_nrf5340_cpuapp_common.dts` | I²C/load-switch nodes; missing pm-device-runtime-auto |
| `src/SensorManager/IMU.cpp` | softReset only; no suspend mode |
| `src/SensorManager/Baro.cpp` | No pre-cut shutdown |
| `src/SensorManager/BoneConduction.cpp` | OK; calls bma580.stop() |
| `src/SensorManager/PPG.cpp` | GPIO LDO leak; error-path refcount leak |
| `src/SensorManager/Temp.cpp` | OK; calls sleepMode() |
| `src/SensorManager/SensorManager.cpp` | `stop_sensor_manager` calls every `.stop()` |
| `src/drivers/LED_Controller/KTD2026.cpp` | begin/power_off pair, called weirdly from power_down |
| `src/drivers/ADAU1860.cpp` | Audio codec PM |
| `src/SD_Card/SD_Card_Manager/SD_Card_Manager.cpp` | aquire_ls/release_ls (defeated by §3.8) |
| `prj.conf` | CONFIG_PM*, CONFIG_POWEROFF |
| `PROBLEMS.md` | Reported regressions (sensor dropout, 12 s power-on, BC data loss) |
| `SYSTEM_DESCRIPTION.md` | I²C topology, BMP388/BMX160/MLX90632 optimisation notes |
