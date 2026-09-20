//! Boot-time hardware self-test.
//!
//! Non-fatal: every check logs its result and boot continues regardless.
//! Intended for board bring-up / repair: find missing or unsoldered parts.
//!
//! Checks run through the existing drivers, so a pass also proves the
//! driver's I2C / GPIO path to that part works.

use std::thread;
use std::time::{Duration, Instant};

#[allow(unused_imports)]
use esp_idf_svc::hal::gpio::{InputPin, IOPin};

use super::Device;

/// Run the self-test. Never blocks boot; results go to the log.
pub fn run_tests(device: &mut Device) {
    log::info!("=== Hardware Self-Test (non-fatal) ===");
    let mut failures = 0u32;
    let mut warnings = 0u32;

    // ── Power / charging ──────────────────────────────
    {
        if device.get_vbus_pgood() {
            log::info!("[PWR] VBUS present: OK");
        } else {
            log::warn!("[PWR] VBUS not detected (fine on battery only; if USB is connected, check the USB connector and power path)");
            warnings += 1;
        }

        if device.get_battery_is_charging() {
            log::info!("[PWR] Battery charging: OK");
        } else {
            log::warn!("[PWR] Battery not charging (may be full or absent)");
            warnings += 1;
        }

        #[cfg(any(feature = "has_max17048", feature = "has_bq27427"))]
        {
            #[cfg(feature = "has_max17048")]
            let fg_desc = "MAX17048 (0x36)";
            #[cfg(feature = "has_bq27427")]
            let fg_desc = "BQ27427 (0x55)";

            match device.fuel_gauge.get_battery_voltage() {
                Ok(v) => log::info!("[PWR] Fuel gauge {fg_desc}: {v:.2} V"),
                Err(e) => {
                    log::warn!("[PWR] Fuel gauge {fg_desc} read failed: {e}. Check: chip solder joints, I2C SDA/SCL, pull-ups");
                    failures += 1;
                }
            }
        }
    }

    // ── System buttons (ESP32 GPIOs, active low) ──────
    {
        let buttons = [
            ("Home", device.button_home.pin(), device.button_home.is_high()),
            ("Vol+", device.button_vol_up.pin(), device.button_vol_up.is_high()),
            ("Vol-", device.button_vol_down.pin(), device.button_vol_down.is_high()),
            ("Power", device.button_power.pin(), device.button_power.is_high()),
        ];
        if buttons.iter().all(|(_, _, high)| *high) {
            log::info!("[BTN] Home / Vol+ / Vol- / Power all released: OK");
        } else {
            for (name, pin, high) in &buttons {
                if !*high {
                    log::warn!("[BTN] {name} (GPIO{pin}) reads LOW = pressed or stuck. Check: button solder joints, pull-up");
                }
            }
            warnings += 1;
        }
    }

    // ── IO expander (rev2 only) ───────────────────────
    #[cfg(feature = "has_io_expander")]
    {
        match device.io_expander.get_pins() {
            Ok(pins) => {
                let bits = pins
                    .iter()
                    .enumerate()
                    .fold(0u16, |acc, (i, &b)| acc | ((b as u16) << i));
                log::info!("[IOEXP] TCA9535 (0x20) read OK, inputs = 0x{bits:04X}");
            }
            Err(e) => {
                log::warn!("[IOEXP] TCA9535 (0x20) read failed: {e}. Check: chip solder joints, I2C, pull-ups");
                failures += 1;
            }
        }
    }
    #[cfg(not(feature = "has_io_expander"))]
    {
        log::info!("[IOEXP] No IO expander fitted on this hardware revision");
    }

    // ── FPGA configuration pins ───────────────────────
    {
        // After power-up, INIT_B is low during T_POR (10-35 ms), then high.
        let start = Instant::now();
        let mut init_ready = false;
        let mut read_err = false;
        while start.elapsed() < Duration::from_millis(100) {
            match device.fpga.get_init_b() {
                Ok(true) => {
                    init_ready = true;
                    break;
                }
                Ok(false) => thread::sleep(Duration::from_millis(5)),
                Err(e) => {
                    log::warn!("[FPGA] Cannot read INIT_B: {e}. Check: INIT_B pin solder joint");
                    read_err = true;
                    break;
                }
            }
        }

        if read_err {
            failures += 1;
        } else if !init_ready {
            log::warn!("[FPGA] INIT_B stayed low. Check: 3V3_FPGA / 1V0 rails, config pins (M0/M1/M2), INIT_B solder joint");
            failures += 1;
        } else {
            log::info!("[FPGA] INIT_B high (ready for configuration)");

            // Assert PROGRAM_B, then verify the device reacts.
            if let Err(e) = device.fpga.set_program_b(true) {
                log::warn!("[FPGA] Cannot drive PROGRAM_B low: {e}");
                failures += 1;
            } else {
                thread::sleep(Duration::from_millis(1));
                match device.fpga.get_init_b() {
                    Ok(false) => log::info!("[FPGA] INIT_B went low while PROGRAM_B asserted: OK"),
                    Ok(true) => {
                        log::warn!("[FPGA] INIT_B did not react to PROGRAM_B. Check: PROGRAM_B/INIT_B solder joints, pull-ups");
                        failures += 1;
                    }
                    Err(e) => {
                        log::warn!("[FPGA] Cannot read INIT_B: {e}");
                        failures += 1;
                    }
                }

                if let Err(e) = device.fpga.set_program_b(false) {
                    log::warn!("[FPGA] Cannot release PROGRAM_B: {e}");
                    failures += 1;
                } else {
                    thread::sleep(Duration::from_millis(6));
                    match device.fpga.get_init_b() {
                        Ok(true) => log::info!("[FPGA] INIT_B released high: OK"),
                        Ok(false) => {
                            log::warn!("[FPGA] INIT_B stayed low after release. Check: FPGA power rails");
                            failures += 1;
                        }
                        Err(e) => {
                            log::warn!("[FPGA] Cannot read INIT_B: {e}");
                            failures += 1;
                        }
                    }

                    match device.fpga.get_done() {
                        Ok(false) => log::info!("[FPGA] DONE low (not configured yet, expected)"),
                        Ok(true) => {
                            log::warn!("[FPGA] DONE unexpectedly high while unconfigured. Check: DONE pin");
                            warnings += 1;
                        }
                        Err(e) => {
                            log::warn!("[FPGA] Cannot read DONE: {e}");
                            warnings += 1;
                        }
                    }
                }
            }
        }
    }

    // ── SD card ───────────────────────────────────────
    {
        if device.sdcard.is_some() {
            match std::fs::read_dir("/sdcard") {
                Ok(entries) => log::info!("[SD] Mounted: OK ({} entries)", entries.count()),
                Err(e) => {
                    log::warn!("[SD] Mounted, but cannot list /sdcard: {e}");
                    warnings += 1;
                }
            }
        } else {
            log::warn!("[SD] Not mounted. Check: card inserted, SD socket solder joints, SDIO lines");
            warnings += 1;
        }
    }

    // ── RTC ───────────────────────────────────────────
    {
        match device.rtc.read_datetime() {
            Ok(Some(dt)) => log::info!(
                "[RTC] Time: {:04}-{:02}-{:02} {:02}:{:02}:{:02} (w{})",
                dt.years,
                dt.months,
                dt.days,
                dt.hours,
                dt.minutes,
                dt.seconds,
                dt.weekdays
            ),
            Ok(None) => {
                log::warn!("[RTC] Time invalid (VL flag). Normal on first boot / dead backup battery; if persistent check PCF8563 (0x51) and its crystal");
                warnings += 1;
            }
            Err(e) => {
                log::warn!("[RTC] Read failed: {e}. Check: PCF8563 (0x51), 32.768 kHz crystal, I2C");
                failures += 1;
            }
        }
    }

    // ── IMU ───────────────────────────────────────────
    {
        match device.imu.read_accel() {
            Ok(s) => log::info!("[IMU] Accel: X={:.3} Y={:.3} Z={:.3} g", s.x, s.y, s.z),
            Err(e) => {
                log::warn!("[IMU] Read failed: {e}. Check: LSM6DS3TR-C (0x6A) solder joints, I2C");
                failures += 1;
            }
        }
    }

    // ── Audio DAC ─────────────────────────────────────
    {
        match device.dac.get_interrupt_status() {
            Ok(st) => {
                log::info!(
                    "[DAC] Headset detected: {}, short-circuit L={} R={}",
                    st.headset_detected,
                    st.left_short_circuit,
                    st.right_short_circuit
                );
                if st.left_short_circuit || st.right_short_circuit {
                    log::warn!("[DAC] Short-circuit flag set. Check: speaker wiring / headphone jack");
                    warnings += 1;
                }
            }
            Err(e) => {
                log::warn!("[DAC] Status read failed: {e}. Check: TLV320DAC3101 (0x18), I2C, /RESET");
                failures += 1;
            }
        }
    }

    log::info!("=== Self-test complete: {failures} failure(s), {warnings} warning(s) — boot continues ===");
}
