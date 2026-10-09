#![allow(dead_code)]

use std::{
    io::Read,
    time::{Duration, Instant},
};

use embedded_hal::{
    digital::{InputPin, OutputPin},
    spi::SpiDevice,
};
use esp_idf_svc::hal::{
    spi::{config::LineWidth, Operation, SpiDriver, SpiSharedDeviceDriver, SpiSoftCsDeviceDriver},
    units::Hertz,
};
use thiserror::Error;

use crate::device::DisplayMode;

pub const REG_INFO_FRAMEWORK_VER: u32 = 0xF100_0000;
pub const REG_INFO_SYSCLK_HZ: u32 = 0xF100_0004;
pub const REG_INFO_VIDEO_DIM: u32 = 0xF100_0100;
pub const REG_INFO_VIDEO_DEPTH: u32 = 0xF100_0104;

pub const REG_CTRL_IRQ_ENABLE: u32 = 0xF100_1000;
pub const REG_CTRL_IRQ_PENDING: u32 = 0xF100_1004;
pub const REG_CTRL_BUTTON_FORCE: u32 = 0xF100_1008;
pub const REG_CTRL_DOCK: u32 = 0xF100_100C;
pub const REG_CTRL_FOCUS: u32 = 0xF100_1010;
pub const REG_CTRL_VIBRATE: u32 = 0xF100_1014;

pub const REG_CTRL_CMD_HOST: u32 = 0xF100_1100;
pub const REG_CTRL_CMD_CORE: u32 = 0xF100_1104;

pub const REG_STATUS_BUTTON: u32 = 0xF100_2000;
pub const REG_STATUS_CART_SWITCH: u32 = 0xF100_2004;

pub const REG_CMD_HOST_BASE: u32 = 0xF000_0000;
pub const REG_CMD_CORE_BASE: u32 = 0xF000_1000;

// ── Legacy register map: upstream v0.1 bitstreams (Feb 2025) ──────────────
//
// The bitstreams bundled in `system-src/` predate the framework register map
// this firmware was written against. They speak the same SPI protocol, use the
// same command encoding and the same overlay pixel format, but a different
// address layout: a 16-bit register map at 0x0000_xxxx and the overlay at
// 0x3800_0000. `Generation` plus `translate` below bridge the two.
pub const REG_V01_CONTROL: u32 = 0x0000_0000;
pub const REG_V01_BUTTON: u32 = 0x0000_0004;
pub const REG_V01_DISPLAY: u32 = 0x0000_0008;
pub const REG_V01_IRQ_ENABLE: u32 = 0x0000_000C;
pub const REG_V01_IRQ_STATUS: u32 = 0x0000_0010;
pub const REG_V01_STATUS: u32 = 0x0000_0014;
pub const REG_V01_OVERLAY_XCTRL: u32 = 0x0000_0100;
pub const REG_V01_OVERLAY_YCTRL: u32 = 0x0000_0104;
/// Framebuffer dimensions, read only: `width << 16 | height`.
pub const REG_V01_FB_DIM: u32 = 0x0000_0200;
pub const REG_V01_OVERLAY_BASE: u32 = 0x3800_0000;

/// Which generation of bitstream is running, which decides how register
/// addresses are interpreted.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Generation {
    /// Current bitstreams: framework registers at `0xF1_xxxx`, overlay at
    /// `0xF2_xxxxxx`.
    Framework,
    /// Upstream v0.1 bitstreams, i.e. everything currently shipped in
    /// `system-src/`.
    V01,
}

/// The FPGA (due to the spi implementation) can read at a speed that's some
/// fraction of the SPI domain clock speed. At 200 MHz SPI receiver clock,
/// 16 MHz is a safe speed.
pub const MAX_SPI_READ_CLOCK: Hertz = Hertz(16_000_000);

pub type SpiDataDriver<'a> =
    SpiSoftCsDeviceDriver<'a, SpiSharedDeviceDriver<'a, &'a SpiDriver<'a>>, &'a SpiDriver<'a>>;

mod xilinx;

#[derive(Debug, Error)]
pub enum Error {
    #[error("gpio error")]
    PinError,
    #[error("error programming fpga")]
    ProgramError,
    #[error("error reading bitstream")]
    BitstreamError,
    #[error("incompatible bitstream")]
    IncompatibleBitstream,
    #[error("spi error")]
    SpiError,
}

#[derive(Copy, Clone)]
#[repr(u32)]
pub enum Irq {
    ModuleVblank = 0,
    Button = 1,
    SpiRequestOverflow = 2,
    SpiResponseUnderflow = 3,
}

impl Irq {
    pub const fn as_flag(self) -> u32 {
        1 << (self as u32)
    }
}

pub struct Fpga<
    'a,
    PinDone: InputPin,
    PinProgramB: OutputPin,
    PinInitB: InputPin,
    ProgramSpi: SpiDevice,
> {
    pin_done: PinDone,
    pub pin_program_b: PinProgramB,
    pin_init_b: PinInitB,
    /// List of SPI drivers and their clock speed, from largest to smallest.
    data_spi: Vec<(SpiDataDriver<'a>, Hertz)>,
    program_spi: ProgramSpi,

    /// Top-level "system" clock speed, which determines how fast reads
    /// and writes can occur.
    system_clock: Hertz,

    /// Bitfield of enabled interrupts
    interrupts: u32,

    /// Which bitstream generation is running. Detected once after programming;
    /// until then we assume the current framework layout.
    generation: Generation,
}

impl<'a, PinDone, PinProgramB, PinInitB, ProgramSpi>
    Fpga<'a, PinDone, PinProgramB, PinInitB, ProgramSpi>
where
    PinDone: InputPin,
    PinProgramB: OutputPin,
    PinInitB: InputPin,
    ProgramSpi: SpiDevice,
{
    pub fn new(
        pin_done: PinDone,
        pin_program_b: PinProgramB,
        pin_init_b: PinInitB,
        data_spi: Vec<(SpiDataDriver<'a>, Hertz)>,
        program_spi: ProgramSpi,
    ) -> Self {
        Fpga {
            pin_done,
            pin_program_b,
            pin_init_b,
            data_spi,
            program_spi,
            system_clock: Hertz(8 * 1024 * 1024),
            interrupts: 0,
            generation: Generation::Framework,
        }
    }

    /// Program the FPGA with a new bitstream.
    pub fn program(
        &mut self,
        bitstream: &mut dyn Read,
        scratch_buf: &mut [u8],
    ) -> Result<(), Error> {
        let header =
            xilinx::parse_bitstream_header(bitstream).map_err(|_| Error::BitstreamError)?;

        // Check that the bitstream was built for this hardware.
        let hardware_version = crate::hwinfo::get_hardware_version();
        let expected_id = 0xB010_0000 | (hardware_version.major as u32);
        if header.user_id.is_none() || header.user_id == Some(0xFFFF_FFFF) {
            log::info!("Bitstream has no UserID, assuming it is compatible");
        } else if hardware_version.major == 0 || hardware_version.major == 255 {
            log::info!("Hardware version major=0, skipping bitstream compatibility");
        } else if header.user_id != Some(expected_id) {
            log::error!("Incompatible bitstream, ID={:08X}", header.user_id.unwrap());
            return Err(Error::IncompatibleBitstream);
        }

        // After power-on-reset, INIT_B will be low for 10ms to 35ms (T_POR),
        // configuration can only start after this.
        // Poll INIT_B until it goes high.
        let start_time = Instant::now();
        while self.pin_init_b.is_low().map_err(|_| Error::PinError)? {
            if start_time.elapsed() > Duration::from_millis(35) {
                return Err(Error::ProgramError);
            }
            std::thread::sleep(Duration::from_millis(5));
        }

        // Pull PROGRAM_B low, hold it for at least 250ns.
        self.pin_program_b.set_low().map_err(|_| Error::PinError)?;
        std::thread::sleep(Duration::from_millis(1));
        if self.pin_init_b.is_high().map_err(|_| Error::PinError)? {
            return Err(Error::ProgramError);
        }
        self.pin_program_b.set_high().map_err(|_| Error::PinError)?;

        // INIT_B will go high at most 5ms after PROGRAM_B release.
        std::thread::sleep(Duration::from_millis(5));
        if self.pin_init_b.is_low().map_err(|_| Error::PinError)? {
            return Err(Error::ProgramError);
        }

        log::info!("FPGA is in program mode");
        let start_time = Instant::now();

        let mut num_read = 0;
        while num_read < header.length {
            let amount = (header.length - num_read).min(scratch_buf.len());
            let buf = &mut scratch_buf[0..amount];
            bitstream
                .read_exact(buf)
                .map_err(|_| Error::BitstreamError)?;
            num_read += amount;

            self.program_spi
                .write(buf)
                .map_err(|_| Error::ProgramError)?;
        }

        log::info!(
            "Programmed FPGA, done={}, time={}",
            self.pin_done.is_high().map_err(|_| Error::PinError)?,
            start_time.elapsed().as_millis() as u32,
        );

        Ok(())
    }

    /// Read the FPGA INIT_B configuration pin (low while clearing configuration).
    pub fn get_init_b(&mut self) -> Result<bool, Error> {
        self.pin_init_b.is_high().map_err(|_| Error::PinError)
    }

    /// Read the FPGA DONE configuration pin (high once configuration succeeded).
    pub fn get_done(&mut self) -> Result<bool, Error> {
        self.pin_done.is_high().map_err(|_| Error::PinError)
    }

    /// Drive the PROGRAM_B pin. Pass true to assert it (holds configuration reset).
    pub fn set_program_b(&mut self, low: bool) -> Result<(), Error> {
        let result = if low {
            self.pin_program_b.set_low()
        } else {
            self.pin_program_b.set_high()
        };
        result.map_err(|_| Error::PinError)
    }

    pub fn set_system_clock_rate(&mut self, rate: Hertz) {
        self.system_clock = rate;
    }

    pub fn enable_interrupt(&mut self, irq: Irq) -> Result<(), Error> {
        self.interrupts |= irq.as_flag();
        self.write_u32(REG_CTRL_IRQ_ENABLE, self.interrupts)
    }

    pub fn disable_interrupt(&mut self, irq: Irq) -> Result<(), Error> {
        self.interrupts &= !irq.as_flag();
        self.write_u32(REG_CTRL_IRQ_ENABLE, self.interrupts)
    }

    /// Finds a SPI data driver with the maximum clock speed.
    fn spi_transaction(
        &mut self,
        max_clock: Option<Hertz>,
        operations: &mut [Operation],
    ) -> Result<(), Error> {
        let driver = &mut self.data_spi.iter_mut().find(|(_, clock)| match max_clock {
            Some(max_clock) => *clock <= max_clock,
            None => true,
        });
        let driver = match driver {
            Some(driver) => driver,
            None => panic!("No suitable spi for max clock {:?}", max_clock),
        };
        driver
            .0
            .transaction(operations)
            .map_err(|_| Error::SpiError)
    }

    const fn spi_command(
        read: bool,
        word_size: FpgaSpiWordSize,
        byte_swap: bool,
        auto_increment: bool,
    ) -> u8 {
        (read as u8)
            | ((word_size as u8) << 1)
            | ((byte_swap as u8) << 3)
            | ((auto_increment as u8) << 4)
    }

    /// Generic SPI write function.
    pub fn spi_write(
        &mut self,
        max_clock: Option<Hertz>,
        command: SpiCommand,
        address: u32,
        data: &[u8],
    ) -> Result<(), Error> {
        let width = LineWidth::Quad;
        let mut command = command.as_write_command();
        command |= (width as u8) << 5;
        let address = address.to_be_bytes();
        self.spi_transaction(
            max_clock,
            &mut [
                Operation::Write(&[command]),
                Operation::WriteWithWidth(&address, width),
                Operation::WriteWithWidth(&data, width),
            ],
        )
    }

    /// Generic SPI read function.
    pub fn spi_read(
        &mut self,
        max_clock: Option<Hertz>,
        command: SpiCommand,
        address: u32,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        let width = LineWidth::Quad;
        let mut command = command.as_read_command();
        command |= (width as u8) << 5;
        let address = address.to_be_bytes();
        const DUMMY_BYTES: usize = 8;
        let mut dummy = [0u8; DUMMY_BYTES];
        self.spi_transaction(
            max_clock,
            &mut [
                Operation::Write(&[command]),
                Operation::WriteWithWidth(&address, width),
                Operation::ReadWithWidth(&mut dummy, width),
                Operation::ReadWithWidth(buffer, width),
            ],
        )
    }

    /// Map a framework register address onto the running bitstream's map.
    ///
    /// Returns `None` for registers the running bitstream does not have, so
    /// callers turn that into a no-op instead of writing somewhere unrelated -
    /// in the v0.1 map `0x0000_0004` is the *button* register, so a stray write
    /// to what the framework calls the boot logo's Y position would corrupt
    /// button forcing.
    fn translate(&self, address: u32) -> Option<u32> {
        if self.generation == Generation::Framework {
            return Some(address);
        }
        if (0xF200_0000..0xF300_0000).contains(&address) {
            return Some(REG_V01_OVERLAY_BASE | (address & 0x00FF_FFFF));
        }
        // The core command windows, including the per-word offsets callers derive
        // from their base address, have no v0.1 equivalent at all.
        if (0xF000_0000..0xF100_0000).contains(&address) {
            return None;
        }
        Some(match address {
            // v0.1 has no version register; its framebuffer dimensions serve as
            // both a liveness probe and the geometry source.
            REG_INFO_FRAMEWORK_VER => REG_V01_FB_DIM,
            // IRQ flags keep the same bit order as the framework map.
            REG_CTRL_IRQ_ENABLE => REG_V01_IRQ_ENABLE,
            REG_CTRL_IRQ_PENDING => REG_V01_IRQ_STATUS,
            REG_CTRL_BUTTON_FORCE => REG_V01_BUTTON,
            REG_CTRL_DOCK => REG_V01_DISPLAY,
            REG_STATUS_CART_SWITCH => REG_V01_STATUS,
            // The boot logo registers live in the low core space, which in the
            // v0.1 map is the register map itself. There is no logo to position
            // in that bitstream, so drop these.
            0x0000_0000..=0x0000_0007 => return None,
            // Present in the framework map but with no v0.1 equivalent: drop
            // rather than alias onto an unrelated register. Buttons in particular
            // come from the I/O expander there, not from a register.
            REG_INFO_SYSCLK_HZ
            | REG_INFO_VIDEO_DIM
            | REG_INFO_VIDEO_DEPTH
            | REG_CTRL_FOCUS
            | REG_CTRL_VIBRATE
            | REG_CTRL_CMD_HOST
            | REG_CTRL_CMD_CORE
            | REG_STATUS_BUTTON => return None,
            // Everything else is core memory, which the v0.1 map relocates to its
            // own interface. Not remapped yet - that is what running a cartridge
            // needs.
            other => other,
        })
    }

    pub fn write_u32(&mut self, address: u32, data: u32) -> Result<(), Error> {
        match self.translate(address) {
            Some(address) => self.write_u32_raw(address, data),
            None => Ok(()),
        }
    }

    pub fn read_u32(&mut self, address: u32) -> Result<u32, Error> {
        match self.translate(address) {
            Some(address) => self.read_u32_raw(address),
            // A register the running bitstream does not have reads as zero, so
            // callers see "nothing there" rather than bus noise.
            None => Ok(0),
        }
    }

    fn write_u32_raw(&mut self, address: u32, data: u32) -> Result<(), Error> {
        let command = SpiCommand::new(FpgaSpiWordSize::Bits32);
        let data = data.to_le_bytes();
        self.spi_write(None, command, address, &data)
    }

    fn read_u32_raw(&mut self, address: u32) -> Result<u32, Error> {
        let mut data = [0u8; 4];
        let command = SpiCommand::new(FpgaSpiWordSize::Bits32);
        self.spi_read(Some(MAX_SPI_READ_CLOCK), command, address, &mut data)?;
        Ok(u32::from_le_bytes(data))
    }

    /// Probe the running bitstream's generation without changing any state.
    ///
    /// The framework register exposes a known version; v0.1 has no such
    /// register, so it is recognised by its framebuffer-dimension register
    /// holding a plausible `width << 16 | height`.
    pub fn detect_generation(&mut self) -> Option<Generation> {
        if self.read_u32_raw(REG_INFO_FRAMEWORK_VER).unwrap_or(0) == 0xB000_0001 {
            return Some(Generation::Framework);
        }
        let dim = self.read_u32_raw(REG_V01_FB_DIM).unwrap_or(0);
        let (width, height) = (dim >> 16, dim & 0xFFFF);
        if (16..=4096).contains(&width) && (16..=4096).contains(&height) {
            return Some(Generation::V01);
        }
        None
    }

    pub fn set_generation(&mut self, generation: Generation) {
        self.generation = generation;
    }

    pub fn generation(&self) -> Generation {
        self.generation
    }

    /// Framebuffer dimensions reported by a v0.1 bitstream, as `(width, height)`.
    pub fn get_framebuffer_dimensions(&mut self) -> Option<(u16, u16)> {
        let dim = self.read_u32_raw(REG_V01_FB_DIM).ok()?;
        let (width, height) = ((dim >> 16) as u16, (dim & 0xFFFF) as u16);
        if width == 0 || height == 0 {
            return None;
        }
        Some((width, height))
    }

    /// Configure the overlay drawing window.
    ///
    /// Only v0.1 has these registers; current bitstreams keep the overlay
    /// enabled, so this is deliberately a no-op for them.
    pub fn set_overlay_bounds(
        &mut self,
        start_x: u8,
        end_x: u8,
        scroll_x: u8,
        start_y: u8,
        end_y: u8,
        scroll_y: u8,
    ) -> Result<(), Error> {
        if self.generation != Generation::V01 {
            return Ok(());
        }
        let config_x =
            ((start_x as u32) << 16) | ((end_x as u32) << 8) | (scroll_x as u32);
        let config_y =
            ((start_y as u32) << 16) | ((end_y as u32) << 8) | (scroll_y as u32);
        self.write_u32_raw(REG_V01_OVERLAY_XCTRL, config_x)?;
        self.write_u32_raw(REG_V01_OVERLAY_YCTRL, config_y)
    }

    /// Show the overlay over its whole area, using the same window upstream's
    /// v0.1 firmware used.
    pub fn show_overlay(&mut self) -> Result<(), Error> {
        self.set_overlay_bounds(0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00)
    }

    /// Write overlay framebuffer.
    pub fn write_overlay(&mut self, offset: u32, data: &[u8]) -> Result<(), Error> {
        let command = SpiCommand::new(FpgaSpiWordSize::Bits16);
        // 16 bits per transfer, 2 cycles per transfer.
        let max_clock = (self.system_clock.0 * 16) / (4 * 2);
        let base = match self.generation {
            Generation::Framework => 0xF200_0000,
            Generation::V01 => REG_V01_OVERLAY_BASE,
        };
        self.spi_write(Some(Hertz(max_clock)), command, base | offset, data)
    }

    /// Get the state of the cartridge slot button.
    pub fn get_cartridge_slot_button(&mut self) -> Result<bool, Error> {
        Ok((self.read_u32(REG_STATUS_CART_SWITCH)? & 1) != 0)
    }

    pub fn set_display_mode(&mut self, new_mode: DisplayMode) -> Result<(), Error> {
        self.write_u32(REG_CTRL_DOCK, (new_mode == DisplayMode::External) as u32)
    }
}

#[allow(unused)]
#[derive(Copy, Clone)]
pub enum FpgaSpiWordSize {
    Bits8 = 0,
    Bits16 = 1,
    Bits32 = 2,
    Bits64 = 3,
}

#[derive(Copy, Clone)]
pub struct SpiCommand {
    pub word_size: FpgaSpiWordSize,
    pub byte_swap: bool,
    pub increment_address: bool,
}

impl SpiCommand {
    pub fn new(word_size: FpgaSpiWordSize) -> Self {
        SpiCommand {
            word_size,
            byte_swap: true,
            increment_address: true,
        }
    }

    fn as_read_command(self) -> u8 {
        (1u8)
            | ((self.word_size as u8) << 1)
            | ((self.byte_swap as u8) << 3)
            | ((self.increment_address as u8) << 4)
    }

    fn as_write_command(self) -> u8 {
        (0u8)
            | ((self.word_size as u8) << 1)
            | ((self.byte_swap as u8) << 3)
            | ((self.increment_address as u8) << 4)
    }
}
