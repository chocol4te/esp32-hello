use core::{convert::TryInto, ptr::null_mut};
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use esp_idf_bindgen::{
    esp_err_t, gpio_pullup_t, i2c_ack_type_t, i2c_cmd_handle_t, i2c_cmd_link_create,
    i2c_cmd_link_delete, i2c_config_t, i2c_config_t__bindgen_ty_1,
    i2c_config_t__bindgen_ty_1__bindgen_ty_1, i2c_config_t__bindgen_ty_1__bindgen_ty_2,
    i2c_driver_delete, i2c_driver_install, i2c_master_cmd_begin, i2c_master_read, i2c_master_start,
    i2c_master_stop, i2c_master_write, i2c_master_write_byte, i2c_mode_t, i2c_param_config,
    i2c_port_t, i2c_rw_t, i2c_slave_read_buffer, i2c_slave_write_buffer, TickType_t,
    ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_CRC, ESP_ERR_INVALID_MAC, ESP_ERR_INVALID_RESPONSE,
    ESP_ERR_INVALID_SIZE, ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_VERSION, ESP_ERR_NOT_FOUND,
    ESP_ERR_NOT_SUPPORTED, ESP_ERR_NO_MEM, ESP_ERR_TIMEOUT, ESP_OK,
};

pub const portMAX_DELAY: TickType_t = TickType_t::max_value();

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    Port0,
    Port1,
}

impl From<Port> for i2c_port_t {
    fn from(value: Port) -> Self {
        match value {
            Port::Port0 => 0,
            Port::Port1 => 1,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PinConfig {
    pub pin_num: i32,
    pub pullup: bool,
}

impl PinConfig {
    fn pullup_enable(&self) -> gpio_pullup_t {
        match self.pullup {
            true => gpio_pullup_t::GPIO_PULLUP_ENABLE,
            false => gpio_pullup_t::GPIO_PULLUP_DISABLE,
        }
    }
}

struct CmdHandle(i2c_cmd_handle_t);

impl CmdHandle {
    fn new() -> Result<Self> {
        let cmd = unsafe { i2c_cmd_link_create() };

        if cmd == null_mut() {
            return Err(Error::NoMem);
        }

        Ok(CmdHandle(cmd))
    }
}

impl Drop for CmdHandle {
    fn drop(&mut self) {
        unsafe {
            i2c_cmd_link_delete(self.0);
        }
    }
}

struct MasterCmd(CmdHandle);

impl MasterCmd {
    fn new() -> Result<Self> {
        let cmd = CmdHandle::new()?;

        unsafe {
            EspError(i2c_master_start(cmd.0)).into_result()?;
        }

        Ok(MasterCmd(cmd))
    }

    fn write_byte(&mut self, b: u8, ack_check: bool) -> Result<()> {
        unsafe { EspError(i2c_master_write_byte((self.0).0, b, ack_check)).into_result() }
    }

    fn write(&mut self, buf: &[u8], ack_check: bool) -> Result<()> {
        unsafe {
            EspError(i2c_master_write(
                (self.0).0,
                buf.as_ptr() as *const u8 as *mut u8,
                buf.len() as u32,
                ack_check,
            ))
            .into_result()
        }
    }

    fn read(&mut self, buf: &mut [u8], ack: i2c_ack_type_t) -> Result<()> {
        unsafe {
            EspError(i2c_master_read(
                (self.0).0,
                buf.as_mut_ptr(),
                buf.len() as u32,
                ack,
            ))
            .into_result()
        }
    }

    fn stop(&mut self) -> Result<()> {
        unsafe { EspError(i2c_master_stop((self.0).0)).into_result() }
    }
}

pub struct Master {
    port: Port,
}

impl Master {
    pub unsafe fn new(port: Port, sda: PinConfig, scl: PinConfig, clk_speed: u32) -> Result<Self> {
        // i2c_config_t documentation says that clock speed must be no higher than 1 MHz
        if clk_speed > 1_000_000 {
            return Err(Error::InvalidArg);
        }

        let sys_config = i2c_config_t {
            mode: i2c_mode_t::I2C_MODE_MASTER,
            sda_io_num: sda.pin_num,
            sda_pullup_en: true,
            scl_io_num: scl.pin_num,
            scl_pullup_en: true,
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                master: i2c_config_t__bindgen_ty_1__bindgen_ty_1 { clk_speed },
            },
        };

        EspError(i2c_param_config(port.into(), &sys_config)).into_result()?;

        // TODO: set flags
        let intr_alloc_flags = 0;
        EspError(i2c_driver_install(
            port.into(),
            i2c_mode_t::I2C_MODE_MASTER,
            0, // rx_buf_len,
            0, // tx_buf_len,
            intr_alloc_flags,
        ))
        .into_result()?;

        Ok(Self { port })
    }

    fn begin_cmd(&mut self, cmd: MasterCmd, ticks_to_wait: TickType_t) -> Result<()> {
        unsafe {
            EspError(i2c_master_cmd_begin(
                self.port.into(),
                (cmd.0).0,
                ticks_to_wait,
            ))
            .into_result()
        }
    }
}

impl Drop for Master {
    fn drop(&mut self) {
        unsafe {
            EspError(i2c_driver_delete(self.port.into()))
                .into_result()
                .unwrap();
        }
    }
}

impl Write for Master {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<()> {
        let mut cmd = MasterCmd::new()?;
        cmd.write_byte((addr << 1) | (i2c_rw_t::I2C_MASTER_WRITE as u8), true)?;
        cmd.write(bytes, true)?;
        cmd.stop()?;

        self.begin_cmd(cmd, portMAX_DELAY)?;

        Ok(())
    }
}

impl Read for Master {
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<()> {
        let mut cmd = MasterCmd::new()?;
        cmd.write_byte((address << 1) | (i2c_rw_t::I2C_MASTER_READ as u8), true)?;
        cmd.read(buffer, i2c_ack_type_t::I2C_MASTER_LAST_NACK)?;
        cmd.stop()?;

        self.begin_cmd(cmd, portMAX_DELAY)?;

        Ok(())
    }
}

impl WriteRead for Master {
    type Error = Error;

    fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<()> {
        let mut cmd = MasterCmd::new()?;
        cmd.write_byte((address << 1) | (i2c_rw_t::I2C_MASTER_WRITE as u8), true)?;
        cmd.write(bytes, true)?;
        cmd.read(buffer, i2c_ack_type_t::I2C_MASTER_LAST_NACK)?;
        cmd.stop()?;

        self.begin_cmd(cmd, portMAX_DELAY)?;

        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AddrMode {
    Addr7Bit,
    Addr10Bit,
}
pub struct EspError(pub esp_err_t);

impl EspError {
    pub fn into_result(self) -> Result<()> {
        Result::from(self)
    }
}

// TODO: implement Error
#[derive(Clone, Copy, Debug)]
pub enum Error {
    /// Out of memory
    NoMem,
    /// Invalid argument
    InvalidArg,
    /// Invalid state
    InvalidState,
    /// Invalid size
    InvalidSize,
    /// Requested resource not found
    NotFound,
    /// Operation or feature not supported
    NotSupported,
    /// Operation timed out
    Timeout,
    /// Received response was invalid
    InvalidResponse,
    /// CRC or checksum was invalid
    InvalidCrc,
    /// Version was invalid
    InvalidVersion,
    /// MAC address was invalid
    InvalidMac,

    Other(esp_err_t),
}

pub type Result<T, E = Error> = core::result::Result<T, E>;

impl From<EspError> for Result<()> {
    fn from(value: EspError) -> Self {
        use Error::*;

        Err(match value.0 as u32 {
            ESP_OK => return Ok(()),
            ESP_ERR_NO_MEM => NoMem,
            ESP_ERR_INVALID_ARG => InvalidArg,
            ESP_ERR_INVALID_STATE => InvalidState,
            ESP_ERR_INVALID_SIZE => InvalidSize,
            ESP_ERR_NOT_FOUND => NotFound,
            ESP_ERR_NOT_SUPPORTED => NotSupported,
            ESP_ERR_TIMEOUT => Timeout,
            ESP_ERR_INVALID_RESPONSE => InvalidResponse,
            ESP_ERR_INVALID_CRC => InvalidCrc,
            ESP_ERR_INVALID_VERSION => InvalidVersion,
            ESP_ERR_INVALID_MAC => InvalidMac,
            _ => Other(value.0),
        })
    }
}
