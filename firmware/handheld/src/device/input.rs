use super::Device;
use crate::{device::drivers::fpga, input::InputState};

impl Device<'_> {
    /// Get the current state of the internal buttons from the FPGA.
    pub fn get_input_state(&mut self) -> Result<InputState, ()> {
        let fpga_buttons = self
            .fpga
            .read_u32(fpga::REG_STATUS_BUTTON)
            .map_err(|_| ())?;
        let mut state = InputState::default();
        state.btn_a = (fpga_buttons & (1 << 11)) != 0;
        state.btn_b = (fpga_buttons & (1 << 10)) != 0;
        state.btn_x = (fpga_buttons & (1 << 9)) != 0;
        state.btn_y = (fpga_buttons & (1 << 8)) != 0;
        state.btn_up = (fpga_buttons & (1 << 7)) != 0;
        state.btn_down = (fpga_buttons & (1 << 6)) != 0;
        state.btn_left = (fpga_buttons & (1 << 5)) != 0;
        state.btn_right = (fpga_buttons & (1 << 4)) != 0;
        state.btn_start = (fpga_buttons & (1 << 1)) != 0;
        state.btn_select = (fpga_buttons & (1 << 0)) != 0;
        state.btn_l1 = (fpga_buttons & (1 << 3)) != 0;
        state.btn_r1 = (fpga_buttons & (1 << 2)) != 0;
        state.btn_system = self.button_home.is_low();
        state.btn_vol_up = self.button_vol_up.is_low();
        state.btn_vol_down = self.button_vol_down.is_low();
        state.btn_power = self.button_power.is_low();
        Ok(state)
    }
}
