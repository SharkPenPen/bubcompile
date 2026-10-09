use super::Device;
use crate::{device::drivers::fpga, input::InputState};

impl Device<'_> {
    /// Get the current state of the internal buttons.
    ///
    /// Current bitstreams report every game button through the framework's button
    /// register. The v0.1 bitstreams have no such register: they wire those
    /// buttons to the TCA9535 I/O expander instead, so they are read from there
    /// using the same bit assignments upstream's v0.1 firmware used. The system
    /// buttons are direct GPIOs on every revision and never come from the FPGA.
    pub fn get_input_state(&mut self) -> Result<InputState, ()> {
        let mut state = InputState::default();

        match self.fpga.generation() {
            fpga::Generation::Framework => {
                let buttons = self
                    .fpga
                    .read_u32(fpga::REG_STATUS_BUTTON)
                    .map_err(|_| ())?;
                state.btn_a = (buttons & (1 << 11)) != 0;
                state.btn_b = (buttons & (1 << 10)) != 0;
                state.btn_x = (buttons & (1 << 9)) != 0;
                state.btn_y = (buttons & (1 << 8)) != 0;
                state.btn_up = (buttons & (1 << 7)) != 0;
                state.btn_down = (buttons & (1 << 6)) != 0;
                state.btn_left = (buttons & (1 << 5)) != 0;
                state.btn_right = (buttons & (1 << 4)) != 0;
                state.btn_start = (buttons & (1 << 1)) != 0;
                state.btn_select = (buttons & (1 << 0)) != 0;
                state.btn_l1 = (buttons & (1 << 3)) != 0;
                state.btn_r1 = (buttons & (1 << 2)) != 0;
            }
            fpga::Generation::V01 => {
                #[cfg(feature = "has_io_expander")]
                {
                    let pins = self.io_expander.get_pins().map_err(|_| ())?;
                    // v0.1 bit assignments; every input is active low.
                    state.btn_r1 = !pins[0];
                    state.btn_x = !pins[1];
                    state.btn_y = !pins[2];
                    state.btn_a = !pins[3];
                    state.btn_b = !pins[4];
                    state.btn_l1 = !pins[9];
                    state.btn_up = !pins[10];
                    state.btn_right = !pins[11];
                    state.btn_left = !pins[12];
                    state.btn_down = !pins[13];
                    state.btn_select = !pins[14];
                    state.btn_start = !pins[15];
                }
                #[cfg(not(feature = "has_io_expander"))]
                {
                    // This revision has no expander, and v0.1 bitstreams predate
                    // the revisions that report buttons through the FPGA.
                    return Err(());
                }
            }
        }

        state.btn_system = self.button_home.is_low();
        state.btn_vol_up = self.button_vol_up.is_low();
        state.btn_vol_down = self.button_vol_down.is_low();
        state.btn_power = self.button_power.is_low();
        Ok(state)
    }
}
