#include "PowerManager.h"

#include "macros_common.h"

#include <stdio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/shell/shell.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/zbus/zbus.h>

#ifdef CONFIG_BOOTLOADER_MCUBOOT
#include <zephyr/dfu/mcuboot.h>
#endif

#include <hal/nrf_ficr.h>
#include <hal/nrf_gpio.h>

#include "../drivers/LED_Controller/KTD2026.h"
#include "../drivers/ADAU1860.h"
#include "../buttons/Button.h"
#include "../SensorManager/SensorManager.h"

#include "../utils/StateIndicator.h"

#include "bt_mgmt.h"
#include "bt_mgmt_ctlr_cfg_internal.h"

#include <zephyr/logging/log_ctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(power_manager, LOG_LEVEL_DBG);

K_WORK_DELAYABLE_DEFINE(PowerManager::charge_ctrl_delayable, PowerManager::charge_ctrl_work_handler);
K_WORK_DELAYABLE_DEFINE(PowerManager::power_down_work, PowerManager::power_down_work_handler);

K_WORK_DEFINE(PowerManager::fuel_gauge_work, PowerManager::fuel_gauge_work_handler);
K_WORK_DEFINE(PowerManager::battery_controller_work, PowerManager::battery_controller_work_handler);

ZBUS_CHAN_DEFINE(battery_chan, struct battery_data, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0));

static struct battery_data msg;

void PowerManager::fuel_gauge_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    LOG_DBG("Fuel Gauge GPOUT Interrupt");
    k_work_submit(&fuel_gauge_work);
}

void PowerManager::battery_controller_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_work_submit(&battery_controller_work);
}

void PowerManager::power_good_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    bool power_good = battery_controller.power_connected();

    k_work_submit(&fuel_gauge_work);

    if (power_good) {
        power_manager.charger_init_pending = true;
        k_work_schedule(&charge_ctrl_delayable, K_NO_WAIT);
    } else {
        k_work_cancel_delayable(&charge_ctrl_delayable);
        if (!power_manager.power_on) k_work_reschedule(&power_manager.power_down_work, K_NO_WAIT);
    }
}

void PowerManager::power_down_work_handler(struct k_work * work) {
	power_manager.power_down();
}

void PowerManager::charge_ctrl_work_handler(struct k_work * work) {
	power_manager.charge_task();
    // Schedule next execution
    k_work_schedule(&charge_ctrl_delayable, power_manager.chrg_interval);
}

void PowerManager::battery_controller_work_handler(struct k_work * work) {
    button_state state;

    {
        BQ25120a::ActiveScope active(battery_controller);
        state = battery_controller.read_button_state();
    }

    if (state.wake_2) {
        power_manager.power_on = !power_manager.power_on;
        if (!power_manager.power_on) power_manager.power_down();
    }
}

struct charging_snapshot {
    BQ25120a::ChargePhase phase;
    bat_status bat;
    gauge_status gs;
    uint8_t fault;
    float voltage_v;
    float current_ma;
    float target_current_ma;
    bool power_connected;
};

// Pure classifier: hardware snapshot + config → application-level charging_state.
// No I2C, no logging, no side effects. Unit-testable by construction.
static enum charging_state classify_charging(const charging_snapshot &s,
                                             const battery_settings &cfg) {
    switch (s.phase) {
        case BQ25120a::ChargePhase::Discharge:
            if (s.gs.edv1) return BATTERY_CRITICAL;
#ifdef CONFIG_BATTERY_ENABLE_LOW_STATE
            if (s.gs.edv2) return BATTERY_LOW;
#endif
            return DISCHARGING;

        case BQ25120a::ChargePhase::Charging:
            if (s.bat.SYSDWN) return PRECHARGING;
            if (s.current_ma > 0.8f * s.target_current_ma - 2.0f * cfg.i_term) {
                return CHARGING;
            }
            if (s.voltage_v > cfg.u_term - 0.02f) {
#ifdef CONFIG_BATTERY_ENABLE_TRICKLE_CHARGE
                return TRICKLE_CHARGING;
#else
                return CHARGING;
#endif
            }
            return POWER_CONNECTED;

        case BQ25120a::ChargePhase::Done:
            return FULLY_CHARGED;

        case BQ25120a::ChargePhase::Fault:
            // Undervoltage fault with power connected + current flowing = recovery.
            if ((s.fault & (1 << 5)) && s.power_connected &&
                s.current_ma > 0.5f * cfg.i_term) {
                return PRECHARGING;
            }
            return FAULT;
    }
    return DISCHARGING;
}

void PowerManager::fuel_gauge_work_handler(struct k_work * work) {
    int ret;
    battery_level_status status;

    msg.battery_level = fuel_gauge.state_of_charge();

    power_manager.get_battery_status(status);

    charging_snapshot snap;
    snap.bat = fuel_gauge.battery_status();
    snap.gs = fuel_gauge.gauging_state();
    snap.voltage_v = fuel_gauge.voltage();
    snap.current_ma = fuel_gauge.current();
    snap.target_current_ma = fuel_gauge.charge_current();

    if (power_manager.power_on && snap.bat.SYSDWN) {
        LOG_WRN("Battery reached system down voltage.");
        k_work_reschedule(&power_manager.power_down_work, K_NO_WAIT);
    }

    if (snap.bat.CHGINH) {
        power_manager.charging_disabled = true;
        battery_controller.disable_charge();
    } else if (power_manager.charging_disabled) {
        battery_controller.enable_charge();
    }

    uint8_t ts_fault = 0;
    {
        BQ25120a::ActiveScope active(battery_controller);
        snap.phase = battery_controller.read_charge_phase();
        snap.power_connected = battery_controller.power_connected();
        if (snap.phase == BQ25120a::ChargePhase::Fault) {
            snap.fault = battery_controller.read_fault();
            ts_fault = battery_controller.read_ts_fault();
        } else {
            snap.fault = 0;
        }
    }

    msg.charging_state = classify_charging(snap, power_manager._battery_settings);

    switch (snap.phase) {
        case BQ25120a::ChargePhase::Discharge:
            LOG_INF("charging state: discharge");
            break;
        case BQ25120a::ChargePhase::Charging:
            LOG_INF("charging state: charging");
            LOG_DBG("Voltage: %.3f V", snap.voltage_v);
            LOG_DBG("Charging current: %.3f mA", snap.current_ma);
            LOG_DBG("Target current: %.3f mA", snap.target_current_ma);
            LOG_DBG("State of charge: %.3f %%", fuel_gauge.state_of_charge());
            break;
        case BQ25120a::ChargePhase::Done:
            LOG_INF("charging state: done");
            break;
        case BQ25120a::ChargePhase::Fault:
            LOG_WRN("charging state: fault");
            for (const auto &b : BQ25120a::fault_bits) {
                if (snap.fault & b.mask) LOG_WRN("%s", b.name);
            }
            if (snap.fault & (1 << 5)) {
                LOG_WRN("Battery voltage at under-voltage fault: %.3f V", snap.voltage_v);
            }
            if ((ts_fault >> 5) & 0x7) {
                LOG_WRN("TS_ENABLED: %i, TS FAULT: %i", ts_fault >> 7, (ts_fault >> 5) & 0x3);
                power_manager.setup_pmic();
            }
            LOG_DBG("------------------ Battery Info ------------------");
            LOG_DBG("Battery Status:");
            LOG_DBG("  Present: %d, Full Charge: %d, Full Discharge: %d",
                    snap.bat.BATTPRES, snap.bat.FC, snap.bat.FD);
            LOG_DBG("Basic Measurements:");
            LOG_DBG("  Voltage: %.3f V", snap.voltage_v);
            LOG_DBG("  Current: %.3f mA", snap.current_ma);
            break;
    }

    // Adjust interval based on state
    if (msg.charging_state == FAULT || msg.charging_state == POWER_CONNECTED) {
        power_manager.chrg_interval = K_SECONDS(CONFIG_BATTERY_CHARGE_CONTROLLER_FAST_INTERVAL_SECONDS);
    } else {
        power_manager.chrg_interval = K_SECONDS(CONFIG_BATTERY_CHARGE_CONTROLLER_NORMAL_INTERVAL_SECONDS);
    }

    ret = zbus_chan_pub(&battery_chan, &msg, K_FOREVER);
    if (ret) {
        LOG_WRN("power manager msg queue full");
    }
}

int PowerManager::begin() {
    earable_state oe_state;

    oe_state.charging_state = DISCHARGING;
    oe_state.pairing_state = PAIRED;

    battery_controller.begin();
    fuel_gauge.begin();
    earable_btn.begin();

    battery_controller.exit_high_impedance();

    uint8_t bat_state = battery_controller.read_charging_state();

    button_state btn = battery_controller.read_button_state();

    power_on = btn.wake_2;

    // get reset reason
    uint32_t reset_reas = NRF_RESET->RESETREAS;

    // reset the reset reason
    NRF_RESET->RESETREAS = 0xFFFFFFFF;
    
    if (reset_reas & RESET_RESETREAS_RESETPIN_Msk) {
        oe_boot_state.timer_reset = bat_state & (1 << 4);
        power_on |= oe_boot_state.timer_reset;
    }

    if (reset_reas & RESET_RESETREAS_SREQ_Msk) {
        LOG_INF("Rebooting ...");
        power_on = true;
    }

    setup_pmic();
    battery_controller.set_int_callback(battery_controller_callback);

    op_state state = fuel_gauge.operation_state();
    if (state.SEC != BQ27220::SEALED) {
        fuel_gauge.setup(_battery_settings);
    }

    bool battery_condition = check_battery();

    if (!battery_condition) LOG_WRN("Battery check failed.");

    bool charging = battery_controller.power_connected();

    if (!battery_condition) {
        power_on = false;
        if (!charging){
            //TODO: Flash red LED once
            return power_down(false);
        }
    }

    if (charging) {
        /* Enable PM runtime for load switches needed during charging.
         * ls_3_3 is claimed on demand by state_indicator. */
        int ret = pm_device_runtime_enable(ls_1_8);
        if (ret != 0) {
            LOG_WRN("Error setting up load switch 1.8V.");
        }
        /* Immediately resume — V_LS powers the SPI level shifter; even a
         * brief drop corrupts the SD card's SPI state. */
        pm_device_runtime_get(ls_1_8);

        ret = pm_device_runtime_enable(ls_3_3);
        if (ret != 0) {
            LOG_WRN("Error setting up load switch 3.3V.");
        }
        pm_device_runtime_get(ls_3_3);

        ret = pm_device_runtime_enable(ls_sd);
        if (ret != 0) {
            LOG_WRN("Error setting up load switch SD.");
        }
        pm_device_runtime_get(ls_sd);


        oe_state.charging_state = POWER_CONNECTED;

        k_work_schedule(&charge_ctrl_delayable, K_NO_WAIT);

        if (!power_on) {
            state_indicator.init(oe_state);
        }

        while(!power_on && battery_controller.power_connected()) {
            k_sleep(K_SECONDS(1));
        }
    } else {
        oe_state.charging_state = DISCHARGING;
    }

    if (!power_on) return power_down();

    battery_controller.set_power_connect_callback(power_good_callback);
    fuel_gauge.set_int_callback(fuel_gauge_callback);

    battery_controller.enter_high_impedance();

    /* Enable PM runtime for all load switches. With pm_device_init_suspended()
     * in board_init, _enable is a no-op (no glitch). */
    int ret = pm_device_runtime_enable(ls_1_8);
    if (ret != 0) {
        LOG_WRN("Error setting up load switch 1.8V.");
    }
    pm_device_runtime_get(ls_1_8);

    ret = pm_device_runtime_enable(ls_3_3);
    if (ret != 0) {
        LOG_WRN("Error setting up load switch 3.3V.");
    }
    pm_device_runtime_get(ls_3_3);

    ret = pm_device_runtime_enable(ls_sd);
    if (ret != 0) {
        LOG_WRN("Error setting up load switch SD.");
    }
    pm_device_runtime_get(ls_sd);


    ret = device_is_ready(error_led.port);
    if (!ret) {
        LOG_WRN("Error LED not ready.");
    }

    ret = gpio_pin_configure_dt(&error_led, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_INF("Failed to set Error LED as output: ERROR -%i.", ret);
    }

    float capacity = fuel_gauge.capacity();
    if (abs(capacity - _battery_settings.capacity) > 1e-4) {
        fuel_gauge.setup(_battery_settings);
        set_error_led();
    }

#ifdef CONFIG_BOOTLOADER_MCUBOOT
    bool img_confirmed = boot_is_img_confirmed();

	if (!img_confirmed) {
		ret = boot_write_img_confirmed();
		if (ret) {
			LOG_ERR("Failed to confirm image");
			// reboot and revert to last confirmed image
			sys_reboot(SYS_REBOOT_COLD);
		}
        LOG_INF("Image confirmed");
        #ifdef CONFIG_SETUP_FUEL_GAUGE
        fuel_gauge.setup(_battery_settings);
        #endif
	}
#endif

    state_indicator.init(oe_state);

    uint32_t device_id[2];

    // Read the device ID
    device_id[0] = nrf_ficr_deviceid_get(NRF_FICR, 0);
    device_id[1] = nrf_ficr_deviceid_get(NRF_FICR, 1);

    oe_boot_state.device_id = (((uint64_t) device_id[1]) << 32) | device_id[0];

    return 0;
}

void PowerManager::set_error_led(int val) {
    gpio_pin_set_dt(&error_led, val > 0 ? 1 : 0);
}

bool PowerManager::check_battery() {
    bool charging = battery_controller.power_connected();

    if (charging) {
        float voltage = fuel_gauge.voltage();

        if (voltage < _battery_settings.u_charge_prevent) {
            battery_controller.disable_charge();
            return false;
        }

        float temp = fuel_gauge.temperature();
        
        if (temp < _battery_settings.temp_min || temp > _battery_settings.temp_max) {
            battery_controller.disable_charge();
            return false;
        } else if (temp < _battery_settings.temp_fast_min || temp > _battery_settings.temp_fast_max) {
            battery_controller.write_charging_control(_battery_settings.i_charge / 2);
            battery_controller.enable_charge();
        } else {
            battery_controller.write_charging_control(_battery_settings.i_charge);
            battery_controller.enable_charge();
        }
    }

    bat_status bs = fuel_gauge.battery_status();
    if (bs.SYSDWN) return false;

    return true;
}

void PowerManager::get_battery_status(battery_level_status &status) {
    BQ25120a::ChargePhase phase;
    {
        BQ25120a::ActiveScope active(battery_controller);
        phase = battery_controller.read_charge_phase();
    }

    status.flags = 0;
    status.power_state = 0x1; // battery_present

    // charging state
    if (battery_controller.power_connected())  {
        status.power_state |= (0x1 << 1); // external source wired (wireless = 3-4),
        if (phase == BQ25120a::ChargePhase::Charging) {
            status.power_state |= (0x1 << 5); // charging
            status.power_state |= (0x1 << 9); // const current
        }
        else if (phase == BQ25120a::ChargePhase::Done) status.power_state |= (0x3 << 5); // inactive discharge
    } else {
        status.power_state |= (0x2 << 5); // active discharge
    }

    // battery level
    gauge_status gs = fuel_gauge.gauging_state();

    // charge level
    if (gs.edv1) status.power_state |= (0x3 << 7); // critical
    else if (gs.edv2) status.power_state |= (0x2 << 7); // low
    else status.power_state |= (0x1 << 7); // good
}

void PowerManager::get_energy_status(battery_energy_status &status) {
    float voltage = fuel_gauge.voltage();
    float current_mA = fuel_gauge.current();
    float capacity = fuel_gauge.capacity(); 

    status.flags = 0b00011010; // presence of fields
    status.voltage = sfloat_from_float(voltage);
    status.charge_rate = sfloat_from_float(voltage * current_mA / 1000);
    status.available_capacity = sfloat_from_float(3.7f * capacity / 1000);
}

void PowerManager::get_health_status(battery_health_status &status) {
    float state_of_health = fuel_gauge.state_of_health();
    int cycle_count = fuel_gauge.cycle_count();
    float temp = fuel_gauge.temperature(); 

    status.flags = 0b00000111; // presence of fields
    status.battery_health_summary = state_of_health;
    status.cycle_count = cycle_count;
    status.current_temperature = round(CLAMP(temp,-127,128));
}

void bt_disconnect_handler(struct bt_conn *conn, void * data) {
    int ret;
    struct bt_conn_info info;

    ret = bt_conn_get_info(conn, &info);
    if (ret != 0) return;
    
    if (info.state == BT_CONN_STATE_CONNECTED) {
        ret = bt_mgmt_conn_disconnect(conn, *((uint8_t*)data));
    }
}

void PowerManager::setup_pmic() {
    battery_controller.setup(_battery_settings);
}

void PowerManager::reboot() {
    int ret;

    // disconnect devices
    uint8_t data = BT_HCI_ERR_REMOTE_USER_TERM_CONN;
    bt_conn_foreach(BT_CONN_TYPE_ALL, bt_disconnect_handler, &data);

    ret = bt_mgmt_ext_adv_stop(0);
    if (ret) {
        LOG_WRN("Failed to stop ext adv: %d", ret);
    }

    stop_sensor_manager();

    ret = bt_mgmt_stop_watchdog();
    ERR_CHK(ret);

    dac.end();

    sys_reboot(SYS_REBOOT_COLD);
}

int PowerManager::power_down(bool fault) {
    int ret;

    uint8_t data = BT_HCI_ERR_REMOTE_USER_TERM_CONN;
    bt_conn_foreach(BT_CONN_TYPE_ALL, bt_disconnect_handler, &data);

    ret = bt_mgmt_ext_adv_stop(0);
    if (ret) {
        LOG_WRN("Failed to stop ext adv: %d", ret);
    }

    led_controller.begin();
    led_controller.power_off();

    stop_sensor_manager();

    bool charging = battery_controller.power_connected();

    if (!charging) {
        ret = battery_controller.set_wakeup_int();
        if (ret != 0) return ret;

        ret = fuel_gauge.set_wakeup_int();
        if (ret != 0) return ret;

        battery_controller.enter_high_impedance();
    }

    // TODO: bt_disable() crashes here (doesn't wake up). See POWER_DESCRIPTION.md §3.1.

    if (fault) {
        LOG_WRN("Power off due to fault");
    } else {
        LOG_INF("Power off");
    }
    LOG_PANIC();

    ret = bt_mgmt_stop_watchdog();

    dac.end();

    gpio_pin_set_dt(&error_led, 0);

    if (charging) {
        sys_reboot(SYS_REBOOT_COLD);
        return 0;
    }

    // Disable EN_LS_LDO in the BQ25120A so the 3.3V LDO is fully off
    {
        BQ25120a::ActiveScope active(battery_controller);
        battery_controller.write_LS_control(false);
    }

    ret = pm_device_action_run(ls_sd,  PM_DEVICE_ACTION_SUSPEND);
    ret = pm_device_action_run(ls_3_3, PM_DEVICE_ACTION_SUSPEND);
    ret = pm_device_action_run(ls_1_8, PM_DEVICE_ACTION_SUSPEND);
    ret = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);

    sys_poweroff();

    // safety if poweroff failed
    k_msleep(1000);
    sys_reboot(SYS_REBOOT_COLD);
}


void PowerManager::charge_task() {
    if (charger_init_pending) {
        charger_init_pending = false;
        LOG_INF("Setting up charge controller ........");
        setup_pmic();
        battery_controller.enable_charge();
    }
    k_work_submit(&fuel_gauge_work);
}

int cmd_setup_fuel_gauge(const struct shell *shell, size_t argc, const char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    fuel_gauge.setup(power_manager._battery_settings);

    power_manager.reboot();

    return 0;
}

static int cmd_battery_info(const struct shell *shell, size_t argc, const char **argv) {
    ARG_UNUSED(argc);
	ARG_UNUSED(argv);

    shell_print(shell, "------------------ Battery Info ------------------");
    // Battery fuel gauge status
    bat_status status = fuel_gauge.battery_status();
    shell_print(shell, "Battery Status:");
    shell_print(shell, "  Present: %i, Full Charge: %i, Full Discharge: %i", 
            status.BATTPRES, status.FC, status.FD);

    // Basic measurements
    shell_print(shell, "Basic Measurements:");
    shell_print(shell, "  Voltage: %.3f V", fuel_gauge.voltage());
    shell_print(shell, "  Temperature: %.1f °C", fuel_gauge.temperature());
    shell_print(shell, "  Current: %.1f mA (avg: %.1f mA)", 
            fuel_gauge.current(), fuel_gauge.average_current());
    shell_print(shell, "  State of Charge: %.1f%%", fuel_gauge.state_of_charge());

    // Capacity info
    shell_print(shell, "Capacity Information:");
    shell_print(shell, "  Design Capacity: %.1f mAh", fuel_gauge.design_cap());
    shell_print(shell, "  Full Charge Capacity: %.1f mAh", fuel_gauge.capacity());
    shell_print(shell, "  Remaining Capacity: %.1f mAh", fuel_gauge.remaining_cap());
    
    // Time estimates
    float ttf = fuel_gauge.time_to_full();
    float tte = fuel_gauge.time_to_empty();
    shell_print(shell, "Time Estimates:");
    shell_print(shell, "  Time to Full: %ih %02dmin", (int)ttf / 60, (int)ttf % 60);
    shell_print(shell, "  Time to Empty: %ih %02dmin", (int)tte / 60, (int)tte % 60);

    // Battery controller status
    {
        BQ25120a::ActiveScope active(battery_controller);

        shell_print(shell, "Charging Information:");
        BQ25120a::ChargePhase phase = battery_controller.read_charge_phase();
        shell_print(shell, "  Charging State: %i", static_cast<int>(phase));
        shell_print(shell, "  Power Good: %i", battery_controller.power_connected());

        struct chrg_state charge_ctrl = battery_controller.read_charging_control();
        shell_print(shell, "  Charge Control: enabled=%i, current=%.1f mA",
                charge_ctrl.enabled, charge_ctrl.mAh);
    }

    return 0;
}

static void i2c_manual_recover(uint32_t pin_scl, uint32_t pin_sda) {
    // Configure SCL and SDA as standard push-pull
    nrf_gpio_cfg_output(pin_scl);
    nrf_gpio_pin_set(pin_scl);
    nrf_gpio_cfg_output(pin_sda);
    nrf_gpio_pin_set(pin_sda);
    k_usleep(100);

    // Generate 9 SCL clock pulses to clock out any stuck transmitting slave
    for (int i = 0; i < 10; i++) {
        nrf_gpio_pin_clear(pin_scl);
        k_usleep(10);
        nrf_gpio_pin_set(pin_scl);
        k_usleep(10);
    }

    // Generate a STOP condition (SDA LOW to HIGH while SCL is HIGH)
    nrf_gpio_pin_clear(pin_scl);
    k_usleep(10);
    nrf_gpio_pin_clear(pin_sda); // SDA LOW
    k_usleep(10);
    nrf_gpio_pin_set(pin_scl); // SCL HIGH
    k_usleep(10);
    nrf_gpio_pin_set(pin_sda); // SDA HIGH
    k_usleep(10);

    // Restore to Input (High-Impedance)
    nrf_gpio_cfg_input(pin_scl, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(pin_sda, NRF_GPIO_PIN_NOPULL);
}

static int cmd_sensor_diag(const struct shell *shell, size_t argc, const char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell, "=== Sensor Bus Diagnostic ===");

    // 1. Load switch GPIO states (output pins need nrf_gpio_pin_out_read)
    shell_print(shell, "\n-- Load Switch GPIO States --");
    shell_print(shell, "  ls_1_8 (P1.11): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(1, 11)));
    shell_print(shell, "  ls_3_3 (P0.14): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(0, 14)));
    shell_print(shell, "  ls_sd  (P1.12): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(1, 12)));
    shell_print(shell, "  PPG LDO (P0.06): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(0, 6)));

    // 2. I2C bus line levels (HIGH = idle/OK, LOW = stuck)
    shell_print(shell, "\n-- I2C Bus Line Levels --");
    shell_print(shell, "  I2C2: SCL(P1.00)=%d  SDA(P1.15)=%d",
        nrf_gpio_pin_read(NRF_GPIO_PIN_MAP(1, 0)),
        nrf_gpio_pin_read(NRF_GPIO_PIN_MAP(1, 15)));
    shell_print(shell, "  I2C3: SCL(P1.02)=%d  SDA(P1.03)=%d",
        nrf_gpio_pin_read(NRF_GPIO_PIN_MAP(1, 2)),
        nrf_gpio_pin_read(NRF_GPIO_PIN_MAP(1, 3)));

    // 3. BQ25120a PMIC registers
    shell_print(shell, "\n-- BQ25120a PMIC --");
    {
        BQ25120a::ActiveScope active(battery_controller);

        uint8_t charging_state = battery_controller.read_charging_state();
        uint8_t fault = battery_controller.read_fault();
        uint8_t ts_fault = battery_controller.read_ts_fault();
        uint8_t ls_ldo_raw = battery_controller.read_ls_ldo_ctrl_raw();
        float ldo_voltage = battery_controller.read_ldo_voltage();

        shell_print(shell, "  Charging state reg: 0x%02x (state=%d)", charging_state, charging_state >> 6);
        shell_print(shell, "  Fault reg: 0x%02x", fault);
        for (const auto &b : BQ25120a::fault_bits) {
            if (fault & b.mask) shell_print(shell, "    %s", b.name);
        }
        shell_print(shell, "  TS fault reg: 0x%02x", ts_fault);
        shell_print(shell, "  LS_LDO_CTRL raw: 0x%02x", ls_ldo_raw);
        shell_print(shell, "    EN_LS_LDO (bit7): %d", (ls_ldo_raw >> 7) & 1);
        shell_print(shell, "    LDO voltage: %.1f V", (double)ldo_voltage);
        shell_print(shell, "  Power Good (PG): %d", battery_controller.power_connected());

        // 3b. If LDO voltage is wrong, try to fix it and report
        if ((ls_ldo_raw >> 7) == 0) {
            shell_print(shell, "\n  EN_LS_LDO is OFF — re-running PMIC setup...");
            power_manager.setup_pmic();
            uint8_t after = battery_controller.read_ls_ldo_ctrl_raw();
            shell_print(shell, "  After setup: LS_LDO_CTRL=0x%02x (EN=%d)", after, (after >> 7) & 1);
        }
    }

    // 4. I2C3 sensor probes
    shell_print(shell, "\n-- I2C3 Sensor Probes --");
    const struct device *i2c3_dev = DEVICE_DT_GET(DT_NODELABEL(i2c3));
    struct { const char *name; uint8_t addr; } sensors[] = {
        {"BMX160 (IMU)",  0x68},
        {"BMP388 (Baro)", 0x76},
        {"BMA580 (Bone)", 0x18},
        {"MLX90632 (Temp)", 0x3A},
    };
    for (int i = 0; i < 4; i++) {
        uint8_t dummy;
        int ret = i2c_burst_read(i2c3_dev, sensors[i].addr, 0x00, &dummy, 1);
        shell_print(shell, "  %s @0x%02x: %s (ret=%d)",
            sensors[i].name, sensors[i].addr,
            ret == 0 ? "OK" : "FAIL", ret);
    }

    // 5. I2C2 sensor probe
    shell_print(shell, "\n-- I2C2 Sensor Probes --");
    const struct device *i2c2_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
    {
        uint8_t dummy;
        int ret = i2c_burst_read(i2c2_dev, 0x62, 0x00, &dummy, 1);
        shell_print(shell, "  MAXM86161 (PPG) @0x62: %s (ret=%d)",
            ret == 0 ? "OK" : "FAIL", ret);
    }

    shell_print(shell, "\n=== Diagnostic Complete ===");
    return 0;
}

static int cmd_sensor_bus_reset(const struct shell *shell, size_t argc, const char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    // 1. Stop all sensors to halt ongoing I2C traffic
    shell_print(shell, "Stopping all sensors...");
    stop_sensor_manager();
    k_msleep(100);

    // 2. Disable EN_LS_LDO in PMIC register BEFORE pulling GPIOs
    //    With EN_LS_LDO=1 persisting in the PMIC, the 3.3V LDO may leak
    //    even with LSCTRL low, preventing sensors from fully de-powering.
    shell_print(shell, "Disabling EN_LS_LDO in PMIC...");
    {
        BQ25120a::ActiveScope active(battery_controller);
        battery_controller.write_LS_control(false);
    }

    // 3. Suspend I2C peripherals before GPIO takeover
    shell_print(shell, "Suspending I2C peripherals...");
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay)
    const struct device *i2c2_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
    pm_device_action_run(i2c2_dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay)
    const struct device *i2c3_dev = DEVICE_DT_GET(DT_NODELABEL(i2c3));
    pm_device_action_run(i2c3_dev, PM_DEVICE_ACTION_SUSPEND);
#endif

    // 4. Turn off ALL load switches via raw GPIO (bypass PM refcounts)
    shell_print(shell, "Turning off all load switches...");
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 11));   // ls_1_8
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 11));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 14));   // ls_3_3 / LSCTRL
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 14));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 12));   // ls_sd
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 12));

    // 5. Force I2C lines LOW to break back-powering through pull-ups/ESD diodes
    shell_print(shell, "Forcing I2C lines LOW to break back-power path...");
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 0));    // I2C2 SCL
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 0));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 15));   // I2C2 SDA
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 15));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 2));    // I2C3 SCL
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 2));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 3));    // I2C3 SDA
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 3));

    // 6. Power off so EN_LS_LDO=0 persists and sensors stay de-powered.
    //    Leave off as long as needed, then press button to restart.
    //    setup() will re-enable the LDO on boot.
    shell_print(shell, "Powering off. Press button to restart.");
    k_msleep(20);
    sys_poweroff();

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(battery_cmd,
    SHELL_COND_CMD(CONFIG_SHELL, info, NULL, "Print battery info", cmd_battery_info),
    SHELL_COND_CMD(CONFIG_SHELL, setup, NULL, "Setup fuel gauge", cmd_setup_fuel_gauge),
    SHELL_COND_CMD(CONFIG_SHELL, sensor_diag, NULL, "Diagnose sensor bus and power rail state", cmd_sensor_diag),
    SHELL_COND_CMD(CONFIG_SHELL, sensor_bus_reset, NULL, "Hard reset I2C sensor bus via GPIO sink", cmd_sensor_bus_reset),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(battery, &battery_cmd, "Power Manager Commands", NULL);

PowerManager power_manager;