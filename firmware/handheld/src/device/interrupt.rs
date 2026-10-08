use std::{num::NonZeroU32, sync::Arc};

use esp_idf_svc::hal::{
    gpio::{InputMode, InterruptType, Pin, PinDriver},
    task::notification::{Notification, Notifier},
};

use crate::worker;
use crate::{device::drivers::fpga, ui};

use super::Device;

const FLAG_MCU_IRQ: NonZeroU32 = unsafe { NonZeroU32::new_unchecked(1) };
const FLAG_HOME: NonZeroU32 = unsafe { NonZeroU32::new_unchecked(2) };
const FLAG_POWER: NonZeroU32 = unsafe { NonZeroU32::new_unchecked(4) };
const FLAG_VOL_UP: NonZeroU32 = unsafe { NonZeroU32::new_unchecked(8) };
const FLAG_VOL_DOWN: NonZeroU32 = unsafe { NonZeroU32::new_unchecked(16) };
const FLAG_VBUS_PGOOD: NonZeroU32 = unsafe { NonZeroU32::new_unchecked(32) };
const FLAG_BUTTONS: u32 =
    FLAG_HOME.get() | FLAG_POWER.get() | FLAG_VOL_UP.get() | FLAG_VOL_DOWN.get();

/// Subscribe `pin` and enable its interrupt.
///
/// Every failure is logged and skipped: one unusable pin must not stop the
/// others, and a panic in this thread would abort the whole application.
fn setup_gpio_interrupt(
    pin: &mut PinDriver<'_, impl Pin, impl InputMode>,
    interrupt_type: InterruptType,
    notifier: Arc<Notifier>,
    flags: NonZeroU32,
    name: &str,
) {
    // SAFETY: only ISR-safe FreeRTOS functions will be called (task notify).
    let subscribed = unsafe {
        pin.subscribe(move || {
            notifier.notify_and_yield(flags);
        })
    };
    if let Err(e) = subscribed {
        log::error!("Cannot arm {name} interrupt (subscribe): {e}");
        return;
    }
    if let Err(e) = pin.set_interrupt_type(interrupt_type) {
        log::error!("Cannot arm {name} interrupt (set type): {e}");
        return;
    }
    if let Err(e) = pin.enable_interrupt() {
        log::error!("Cannot arm {name} interrupt (enable): {e}");
    }
}

impl Device<'_> {
    /// Setup interrupts on the Device interrupt sources:
    ///
    /// * Volume up, volume down, home, and power buttons
    /// * Shared MCU_IRQ line (FPGA + DAC + fuel gauge)
    ///
    /// The handler gates all FPGA access on `Device::fpga_ready`: the shared line
    /// floats until the FPGA is configured, and a floating input on a
    /// level-triggered interrupt used to storm and starve the whole application.
    pub(super) fn setup_interrupts() {
        std::thread::Builder::new()
            .name("Interrupt".to_string())
            .stack_size(4 * 1024)
            .spawn(|| {
                let notification = Notification::new();

                {
                    let device = &mut Device::get().lock().unwrap();
                    let notifier = notification.notifier();

                    setup_gpio_interrupt(
                        &mut device.button_home,
                        InterruptType::AnyEdge,
                        notifier.clone(),
                        FLAG_HOME,
                        "Home button",
                    );
                    setup_gpio_interrupt(
                        &mut device.button_power,
                        InterruptType::AnyEdge,
                        notifier.clone(),
                        FLAG_POWER,
                        "Power button",
                    );
                    setup_gpio_interrupt(
                        &mut device.button_vol_up,
                        InterruptType::AnyEdge,
                        notifier.clone(),
                        FLAG_VOL_UP,
                        "Vol+ button",
                    );
                    setup_gpio_interrupt(
                        &mut device.button_vol_down,
                        InterruptType::AnyEdge,
                        notifier.clone(),
                        FLAG_VOL_DOWN,
                        "Vol- button",
                    );
                    setup_gpio_interrupt(
                        &mut device.pin_vbus_pgood,
                        InterruptType::AnyEdge,
                        notifier.clone(),
                        FLAG_VBUS_PGOOD,
                        "VBUS pgood",
                    );

                    // The shared MCU_IRQ line is only driven once the FPGA is
                    // configured; `pin_irq` has an internal pull-up so a floating
                    // line cannot trigger a storm.
                    setup_gpio_interrupt(
                        &mut device.pin_irq,
                        InterruptType::LowLevel,
                        notifier,
                        FLAG_MCU_IRQ,
                        "MCU_IRQ",
                    );
                }

                #[allow(unused)]
                let mut prev_vbus_pgood: Option<bool> = None;

                loop {
                    let flags = match notification.wait(esp_idf_svc::hal::delay::BLOCK) {
                        Some(flags) => flags.get(),
                        _ => continue,
                    };

                    let mut device = Device::get().lock().unwrap();

                    // ESP32 GPIO interrupts must be re-armed after each trigger.
                    if (flags & FLAG_HOME.get()) != 0 {
                        let _ = device.button_home.enable_interrupt();
                    }
                    if (flags & FLAG_POWER.get()) != 0 {
                        let _ = device.button_power.enable_interrupt();
                    }
                    if (flags & FLAG_VOL_UP.get()) != 0 {
                        let _ = device.button_vol_up.enable_interrupt();
                    }
                    if (flags & FLAG_VOL_DOWN.get()) != 0 {
                        let _ = device.button_vol_down.enable_interrupt();
                    }
                    if (flags & FLAG_VBUS_PGOOD.get()) != 0 {
                        let _ = device.pin_vbus_pgood.enable_interrupt();
                    }
                    let mut poll_buttons = (flags & FLAG_BUTTONS) != 0;

                    // Rev 2: reading the I/O expander is what clears its IRQ.
                    // Non-fatal: an absent chip must not kill the handler.
                    #[cfg(feature = "has_io_expander")]
                    {
                        if let Err(e) = device.io_expander.get_pins() {
                            log::debug!("I/O expander read failed: {e}");
                        }
                    }

                    // Dock monitoring: on VBUS pgood falling, force undock.
                    let vbus_pgood = device.get_vbus_pgood();
                    if prev_vbus_pgood != Some(vbus_pgood) {
                        prev_vbus_pgood = Some(vbus_pgood);
                        if !vbus_pgood {
                            worker::send(worker::Message::DockEnd);
                        }
                    }

                    if (flags & FLAG_MCU_IRQ.get()) != 0 {
                        log::debug!("Interrupt: MCU_IRQ");

                        // Fuel gauge IRQs (rev2).
                        #[cfg(feature = "has_max17048")]
                        {
                            if let Ok(fuel_irq) = device.fuel_gauge.query_alerts() {
                                let _ = fuel_irq;
                            }
                        }

                        // DAC IRQs.
                        if let Ok(dac_irq) = device.dac.get_interrupt_status() {
                            if dac_irq.headset_detected {
                                if let Ok(has_headphones) = device.dac.get_headphones_detected() {
                                    worker::send(worker::Message::HeadphoneState(has_headphones));
                                }
                            }
                        }

                        // FPGA IRQs. Skipped entirely until the FPGA is known to be
                        // configured - otherwise this hammered the SPI bus (and the
                        // old code `unwrap()`-ed the result).
                        if device.fpga_ready {
                            match device.fpga.read_u32(fpga::REG_CTRL_IRQ_PENDING) {
                                Ok(fpga_irq) => {
                                    if fpga_irq != 0 {
                                        if let Err(e) = device
                                            .fpga
                                            .write_u32(fpga::REG_CTRL_IRQ_PENDING, fpga_irq)
                                        {
                                            log::warn!(
                                                "Cannot clear FPGA IRQ: {e}"
                                            );
                                        } else {
                                            worker::send(worker::Message::FpgaIrq(fpga_irq));
                                        }
                                    }
                                    if (fpga_irq & fpga::Irq::Button.as_flag()) != 0 {
                                        poll_buttons = true;
                                    }
                                }
                                Err(e) => log::warn!("FPGA IRQ read failed: {e}"),
                            }
                        }

                        let _ = device.pin_irq.enable_interrupt();
                    }

                    if poll_buttons && device.fpga_ready {
                        match device.get_input_state() {
                            Ok(input_state) => ui::send(ui::Message::InputState(input_state)),
                            Err(()) => log::debug!("Input state read failed"),
                        }
                    }
                }
            })
            .unwrap();
    }
}
