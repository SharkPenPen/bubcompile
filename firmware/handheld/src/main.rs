//! Game Bub handheld firmware.
//!
//! Boot sequence (bring-up hardened):
//!
//! 1. Serial banner: version, hardware revision, serial, **reset reason**.
//! 2. Full hardware self-test (`device::test_hardware`) - logs every component.
//! 3. FPGA bitstream load. If that fails, the device does **not** die: it enters
//!    the diagnostics loop so the failure stays visible on the serial console.

use std::time::Duration;

use anyhow::Context;

use crate::{
    device::{drivers::fpga, test_hardware, Device},
    ui::UI,
};

mod bitstream;
mod cart_backup;
mod control;
mod core;
mod crash_handler;
mod device;
mod fwinfo;
mod hwinfo;
mod input;
mod kvs;
mod led;
mod power;
pub mod ui;
mod util;
mod worker;

enum StartupAction {
    MainMenu,
    RunCartridge,
}

fn get_startup_action() -> StartupAction {
    match kvs::keys::STARTUP_ACTION.get() {
        Some(1) => StartupAction::RunCartridge,
        _ => StartupAction::MainMenu,
    }
}

/// ESP32 reset reason. BROWNOUT / PANIC / *_WDT is what "it flashed but does not
/// boot" usually looks like, so print it on every boot.
fn reset_reason_str() -> &'static str {
    match unsafe { esp_idf_svc::sys::esp_reset_reason() } as i32 {
        1 => "POWERON",
        2 => "EXT_PIN",
        3 => "SW_RESTART",
        4 => "PANIC (crash)",
        5 => "INT_WDT",
        6 => "TASK_WDT",
        7 => "WDT",
        8 => "DEEPSLEEP_WAKE",
        9 => "BROWNOUT (supply dropped - check power path / battery)",
        12 => "JTAG",
        14 => "PWR_GLITCH",
        15 => "CPU_LOCKUP",
        _ => "OTHER",
    }
}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    // Make sure INFO and above are not filtered out in a release build.
    log::set_max_level(log::LevelFilter::Info);
    let _ = esp_idf_svc::log::set_target_level("gpio", log::LevelFilter::Warn);
    crash_handler::setup();

    let commit = fwinfo::GIT_COMMIT_BYTES;
    log::info!("################ Game Bub handheld ################");
    log::info!("# fw version   : {}", fwinfo::FIRMWARE_VERSION);
    log::info!("# git commit   : {:02x?}", &commit[..4]);
    log::info!("# hardware     : {}", hwinfo::get_hardware_version());
    log::info!("# serial       : {}", hwinfo::get_serial_number());
    log::info!("# reset reason : {}", reset_reason_str());
    log::info!("##################################################");

    kvs::Kvs::init().context("KVS init")?;

    // ── Revision gate ────────────────────────────────────────────────────
    // A mismatch (or a blank eFuse) warns instead of aborting: aborting made a
    // board whose eFuse was never programmed impossible to bring up.
    cfg_if::cfg_if! {
        if #[cfg(feature = "rev2")] {
            let required_revision = 2;
        } else if #[cfg(feature = "rev4")] {
            let required_revision = 4;
        } else {
            compile_error!("No board revision selected");
        }
    };
    let actual_revision = hwinfo::get_hardware_version().major;
    if actual_revision != required_revision && actual_revision != 0 && actual_revision != 255 {
        log::error!(
            "[SYS] Built for revision {required_revision} but this board reports \
             {actual_revision} - pin maps may be wrong, continuing anyway"
        );
    } else if actual_revision == 0 || actual_revision == 255 {
        log::warn!("[SYS] eFuse hardware major is {actual_revision:#04X} (unprogrammed)");
    }

    // ── Device init ──────────────────────────────────────────────────────
    let mut device = match Device::init() {
        Ok(()) => Device::lock(),
        Err(e) => {
            // Only reachable for an early SoC-level failure (peripherals / GPIO /
            // LEDC / SPI driver creation); everything else is non-fatal now.
            let reason = format!("Device::init() failed: {e:#}");
            log::error!("[SYS] {reason}");
            loop {
                log::error!("[SYS] {reason}");
                std::thread::sleep(Duration::from_secs(5));
            }
        }
    };

    // ── Hardware self-test (always, before anything can abort) ───────────
    let summary = test_hardware::run_tests(&mut device);
    if !summary.is_clean() {
        log::warn!(
            "[SYS] {summary} - the device may still work; see the lines above for what is missing"
        );
    }

    device.set_brightness(kvs::keys::BRIGHTNESS.get().unwrap());

    // ── FPGA bring-up. Only a real programming failure is fatal. ──────────
    fn program_fpga(device: &mut Device) -> anyhow::Result<()> {
        log::info!("[FPGA] Loading boot bitstream...");
        bitstream::initial_program_boot(device).context("initial boot bitstream")?;

        // The FPGA needs a moment after DONE before its MMCM is locked. Until then
        // `SpiReceiverFifo` is held in reset and drives nothing, so an immediate read
        // samples a floating bus and returns coupling garbage (0xEEEEEEEE was
        // observed). Probe with retries, and gate the writes below on the result:
        // writing the IRQ-enable register before the SPI domain is alive loses it.
        const PROBE_TIMEOUT: Duration = Duration::from_secs(2);
        const PROBE_INTERVAL: Duration = Duration::from_millis(50);
        let probe_started = std::time::Instant::now();
        let mut last_read = 0u32;
        let mut fpga_alive = false;
        while probe_started.elapsed() < PROBE_TIMEOUT {
            last_read = device
                .fpga
                .read_u32(fpga::REG_INFO_FRAMEWORK_VER)
                .unwrap_or(0);
            if last_read == 0xB000_0001 {
                log::info!(
                    "[FPGA] Framework version 0xB0000001 OK after {} ms",
                    probe_started.elapsed().as_millis()
                );
                fpga_alive = true;
                break;
            }
            std::thread::sleep(PROBE_INTERVAL);
        }
        device.fpga_ready = fpga_alive;
        if !fpga_alive {
            // Never fatal: the original firmware did not probe at all, and a blank
            // screen with a clear log beats a device that stops before the UI.
            log::error!(
                "[FPGA] No framework-version answer within {} ms (last read {last_read:#010X}, \
                 expected 0xB0000001). The bitstream may be for a different board revision. \
                 Continuing, but FPGA interrupts stay disabled - check the display.",
                PROBE_TIMEOUT.as_millis()
            );
        }

        device
            .fpga
            .enable_interrupt(fpga::Irq::Button)
            .context("enable FPGA button interrupt")?;
        device
            .lcd
            .enable_fpga_control()
            .context("hand the LCD over to the FPGA")?;
        Ok(())
    }

    if let Err(e) = program_fpga(&mut device) {
        test_hardware::hold_alive(&mut device, &format!("FPGA bring-up failed: {e:#}"));
    }

    // ── FPGA is running: start the workers and the UI ─────────────────────
    worker::start();
    power::PowerManager::start(&mut device);

    if let StartupAction::RunCartridge = get_startup_action() {
        worker::send(worker::Message::RunCartridge);
    }

    log::info!("[UI] Starting UI");
    let mut ui = UI::new(&mut device);
    std::mem::drop(device);

    if let StartupAction::MainMenu = get_startup_action() {
        led::LedController::set_behavior(led::LedBehavior::OFF);
    }
    ui.run();
}
