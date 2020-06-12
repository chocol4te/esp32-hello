use core::ops::Deref;

use crate::target::{self, i2c, I2C0, I2C1};

#[derive(Clone, Copy, Debug)]
pub struct PinConfig {
    pub pin_num: u32,
    pub pullup: bool,
}

unsafe fn gpio_matrix_out(gpio: u32, signal_idx: u32, out_inverted: bool, oen_inverted: bool) {
    let base_address = 0x3FF44530; // GPIO_FUNC0_OUT_SEL_CFG_REG
    let store_address = (base_address + 4 * gpio) as *mut u32;

    let mut value = signal_idx;

    if out_inverted {
        value |= 1 << 9;
    }

    if oen_inverted {
        value |= 1 << 11;
    }

    core::ptr::write_volatile(store_address, value);
}

unsafe fn gpio_matrix_in(gpio: u32, signal_idx: u32, inverted: bool) {
    let base_address = 0x3FF44130; // GPIO_FUNC0_IN_SEL_CFG_REG
    let store_address = (base_address + 4 * signal_idx) as *mut u32;

    let mut value = gpio;

    if inverted {
        value |= 1 << 6;
    }

    if gpio != 52 {
        value |= 1 << 7;
    }

    core::ptr::write_volatile(store_address, value);
}

pub struct I2C<T>(T);

impl<T> I2C<T>
where
    T: Instance,
{
    pub fn new(
        i2c: T,
        sda: PinConfig,
        scl: PinConfig,
        clk_speed: u32,
        dport: &mut target::DPORT,
    ) -> Self {
        let mut i2c = Self(i2c);

        // i2c_config_t documentation says that clock speed must be no higher than 1 MHz
        assert!(clk_speed <= 1_000_000);

        // sda
        unsafe { gpio_matrix_out(sda.pin_num, 30, false, false) };
        unsafe { gpio_matrix_in(sda.pin_num, 30, false) };

        // scl
        unsafe { gpio_matrix_out(scl.pin_num, 29, false, false) };
        unsafe { gpio_matrix_in(scl.pin_num, 29, false) };

        // i2c_hw_enable
        i2c.reset(dport);
        i2c.enable(dport);

        // i2c_hal_disable_intr_mask
        i2c.0.int_ena.modify(|_, w| unsafe { w.bits(0) });
        // i2c_hal_clr_intsts_mask
        i2c.0.int_clr.modify(|_, w| unsafe { w.bits(0x3FFF) });

        i2c.0.ctr.modify(|_, w| unsafe {
            w.bits(0)
                .ms_mode()
                .set_bit()
                .sda_force_out()
                .set_bit()
                .scl_force_out()
                .set_bit()
                .tx_lsb_first()
                .clear_bit()
                .rx_lsb_first()
                .clear_bit()
        });

        i2c.reset_fifo();

        i2c.set_filter(Some(7), Some(7));

        i2c.set_frequency(clk_speed);

        i2c.0.ctr.modify(|_, w| w.clk_en().set_bit());

        i2c
    }

    /// Resets the interface
    fn reset(&mut self, dport: &mut target::DPORT) {
        dport.perip_rst_en.modify(|_, w| w.i2c0().set_bit());
        dport.perip_rst_en.modify(|_, w| w.i2c0().clear_bit());
    }

    /// Enables the interface
    fn enable(&mut self, dport: &mut target::DPORT) {
        dport.perip_clk_en.modify(|_, w| w.i2c0().set_bit());
        dport.perip_rst_en.modify(|_, w| w.i2c0().clear_bit());
    }

    /// Resets the transmit and receive FIFO buffers
    fn reset_fifo(&mut self) {
        //i2c_ll_txfifo_rst(hal->dev);
        self.0.fifo_conf.modify(|_, w| w.tx_fifo_rst().set_bit());
        self.0.fifo_conf.modify(|_, w| w.tx_fifo_rst().clear_bit());
        //i2c_ll_rxfifo_rst(hal->dev);
        self.0.fifo_conf.modify(|_, w| w.rx_fifo_rst().set_bit());
        self.0.fifo_conf.modify(|_, w| w.rx_fifo_rst().clear_bit());
    }

    /// Sets the filter with a supplied threshold in clock cycles for which a pulse must be present to pass the filter
    fn set_filter(&mut self, sda_threshold: Option<u8>, scl_threshold: Option<u8>) {
        match sda_threshold {
            Some(threshold) => {
                self.0
                    .sda_filter_cfg
                    .modify(|_, w| unsafe { w.sda_filter_thres().bits(threshold) });
                self.0
                    .sda_filter_cfg
                    .modify(|_, w| w.sda_filter_en().set_bit());
            }
            None => self
                .0
                .sda_filter_cfg
                .modify(|_, w| w.sda_filter_en().clear_bit()),
        }

        match scl_threshold {
            Some(threshold) => {
                self.0
                    .scl_filter_cfg
                    .modify(|_, w| unsafe { w.scl_filter_thres().bits(threshold) });
                self.0
                    .scl_filter_cfg
                    .modify(|_, w| w.scl_filter_en().set_bit());
            }
            None => self
                .0
                .scl_filter_cfg
                .modify(|_, w| w.scl_filter_en().clear_bit()),
        }
    }

    /// Sets the freqency of the I2C interface by calculating and applying the associated timings
    fn set_frequency(&mut self, freq: u32) {
        // i2c_hal_set_bus_timing(&(i2c_context[i2c_num].hal), freq, 1);
        // i2c_ll_cal_bus_clk(80000000, freq, 0);
        let half_cycle = ((80_000_000 / freq) / 2) as u16;
        let scl_low = half_cycle;
        let scl_high = half_cycle;
        let sda_hold = half_cycle / 2;
        let sda_sample = scl_high / 2;
        let setup = half_cycle;
        let hold = half_cycle;
        let tout = half_cycle * 20; // By default we set the timeout value to 10 bus cycles.

        // i2c_ll_set_bus_timing(hal->dev, 0);
        unsafe {
            // scl period
            self.0.scl_low_period.write(|w| w.period().bits(scl_low));
            self.0.scl_high_period.write(|w| w.period().bits(scl_high));

            // sda sample
            self.0.sda_hold.write(|w| w.time().bits(sda_hold));
            self.0.sda_sample.write(|w| w.time().bits(sda_sample));

            // setup
            self.0.scl_rstart_setup.write(|w| w.time().bits(setup));
            self.0.scl_stop_setup.write(|w| w.time().bits(setup));

            // hold
            self.0.scl_start_hold.write(|w| w.time().bits(hold));
            self.0.scl_stop_hold.write(|w| w.time().bits(hold));

            // timeout
            self.0.to.write(|w| w.time_out_reg().bits(tout.into()));
        }
    }

    /// Write `bytes` to an I2C device at address `addr`
    pub fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        //TODO: Confirm this hardware limit
        if bytes.len() > 254 {
            return Err(Error::Transmit);
        }

        // Address for I2C0 (obviously this shouldn't make it into the HAL)
        let fifo_addr = 0x6001301c as *mut u8;

        // Reset FIFO
        self.reset_fifo();

        // RSTART command
        self.0.comd0.write(|w| unsafe { w.command0().bits(0) });

        // Load data into hardware FIFO buffer
        unsafe {
            // Address byte
            *fifo_addr = addr << 1 | 0;

            // Data bytes
            for byte in bytes {
                *fifo_addr = *byte;
            }
        }

        // WRITE command
        let length = (1 + bytes.len() as u8) as u16;
        self.0
            .comd1
            .write(|w| unsafe { w.command1().bits(0b00_1100_0000_0000 | length) });

        // STOP command
        self.0
            .comd2
            .write(|w| unsafe { w.command2().bits(0b01_1000_0000_0000) });

        // Start transmission
        self.0.ctr.modify(|_, w| w.trans_start().set_bit());

        // Wait for commands to complete
        while self.0.comd0.read().command0_done().bit() != true {}
        while self.0.comd1.read().command1_done().bit() != true {}
        while self.0.comd2.read().command2_done().bit() != true {}

        Ok(())
    }

    /// Read into `buffer` from an I2C device at address `addr`
    pub fn read(&mut self, _addr: u8, _buffer: &mut [u8]) -> Result<(), Error> {
        unimplemented!()
    }

    /// Write `bytes` to an I2C device at address `addr` whilst reading received data into `buffer` without triggering a STOP condition
    pub fn write_then_read(
        &mut self,
        _addr: u8,
        _bytes: &[u8],
        _buffer: &mut [u8],
    ) -> Result<(), Error> {
        unimplemented!()
    }

    /// Return the raw interface to the underlying I2C peripheral
    pub fn free(self) -> T {
        self.0
    }
}

/// Implementation of embedded_hal::blocking::i2c Traits

impl<T> embedded_hal::blocking::i2c::Write for I2C<T>
where
    T: Instance,
{
    type Error = Error;

    fn write<'w>(&mut self, addr: u8, bytes: &'w [u8]) -> Result<(), Error> {
        self.write(addr, bytes)
    }
}

impl<T> embedded_hal::blocking::i2c::Read for I2C<T>
where
    T: Instance,
{
    type Error = Error;

    fn read<'w>(&mut self, addr: u8, bytes: &'w mut [u8]) -> Result<(), Error> {
        self.read(addr, bytes)
    }
}

impl<T> embedded_hal::blocking::i2c::WriteRead for I2C<T>
where
    T: Instance,
{
    type Error = Error;

    fn write_read<'w>(
        &mut self,
        addr: u8,
        bytes: &'w [u8],
        buffer: &'w mut [u8],
    ) -> Result<(), Error> {
        self.write_then_read(addr, bytes, buffer)
    }
}

#[derive(Debug)]
pub enum Error {
    Transmit,
    Receive,
}

pub trait Instance: Deref<Target = i2c::RegisterBlock> {}

impl Instance for I2C0 {}

impl Instance for I2C1 {}
