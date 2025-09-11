#![no_std]

use embedded_hal_async::{delay::DelayNs, digital::Wait, i2c};

/// Direct register access
pub mod low_level {
    device_driver::create_device!(
        manifest: "device.kdl"
    );
}

use low_level::*;

/// Abstraction for the i2c bus.
pub struct Interface<BUS> {
    bus: BUS,
    address: u8,
}

impl<BUS> Interface<BUS> {
    fn new(bus: BUS, sdo_state: AddressSdo) -> Self {
        let address = if sdo_state == AddressSdo::Low {
            0x46
        } else {
            0x47
        };
        Self { bus, address }
    }
}

/// Main sensor struct.
pub struct Bmp58x<BUS, WAIT, DELAY> {
    pub ll: LowLevel<Interface<BUS>>,
    int: Option<WAIT>,
    delay: DELAY,
    mode: Mode,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
/// Main user-facing error
pub enum Error<E> {
    /// I2c error, see internal error for details.
    Bus(E),
    /// Interrupt error.
    Wait,
    /// Data was not ready and the driver doesn't have an interrupt pin to wait on.
    DataNotReady,
    /// Output data rate * oversample rate is too high for the sensor to achieve.
    OdrOverSampleOutOfRange,
    /// Tried to read data in standby mode.
    MeasureInStandby,
    /// Tried to send a force measurement command when driver is not in Force mode.
    ForceInNonForceMode,
    /// Tried to read non-volitile memory address outside valid range.
    NvmOutOfRange,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq)]
/// The BMP58x i2c address is set by the level of the SDO pin at reset
pub enum AddressSdo {
    High,
    Low,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
/// Sensor power/data rate mode
pub enum Mode {
    /// Reads at a set data rate, then sleep until it's time for another measurement.
    Normal(Odr),
    /// Measurements are manually triggered by the host.
    Forced,
    /// Reads as fast as possible. Data rate is set by the oversample rate.
    Continous,
    /// Low power, non-measuring mode
    Standby,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
/// Basic sensor configuration for sensor init.
///
/// The default impl represents an even balence of precision and power use and outputs at 1Hz.
pub struct Config {
    /// Sensor power/data rate mode
    pub mode: Mode,
    /// Number of samples to be averaged per ouput pressure measurement.
    pub pressure_oversample: OversamplingRate,
    /// Number of samples to be averaged per ouput temperature measurement.
    pub temperature_oversample: OversamplingRate,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: Mode::Normal(Odr::MilliHz1000),
            temperature_oversample: OversamplingRate::N1,
            pressure_oversample: OversamplingRate::N4,
        }
    }
}

impl<BUS, WAIT: Wait, DELAY: DelayNs> Bmp58x<BUS, WAIT, DELAY> {
    pub fn new(bus: BUS, int: Option<WAIT>, delay: DELAY, sdo_state: AddressSdo) -> Self {
        let ll = LowLevel::new(Interface::new(bus, sdo_state));

        Self {
            ll,
            int,
            delay,
            mode: Mode::Standby,
        }
    }
}

impl<BUS: embedded_hal_async::i2c::I2c, WAIT: Wait, DELAY: DelayNs> Bmp58x<BUS, WAIT, DELAY> {
    /// Configure the driver and sensor the typical usecase.
    pub async fn init(&mut self, config: &Config) -> Result<(), Error<BUS::Error>> {
        self.ll.command().write_async(|r| r.set_cmd(0xB6)).await?;

        self.delay.delay_ms(100).await;

        while !self.ll.interrupt_status().read_async().await?.por() {
            self.delay.delay_ms(10).await;
        }

        self.set_mode(config.mode).await?;

        self.ll
            .over_sample_config()
            .write_async(|r| {
                r.set_press_en(true);
                r.set_osr_p(config.pressure_oversample);
                r.set_osr_t(config.temperature_oversample);
            })
            .await?;

        let effective = self.ll.effective_over_sampling_rate().read_async().await?;
        if !effective.odr_is_valid()
            || effective.osr_p_eff() != config.pressure_oversample
            || effective.osr_t_eff() != config.temperature_oversample
        {
            return Err(Error::OdrOverSampleOutOfRange);
        }

        self.ll
            .interrupt_config()
            .write_async(|r| r.set_int_en(true))
            .await?;

        self.ll
            .int_source()
            .write_async(|r| r.set_drdy_data_reg_en(true))
            .await?;

        Ok(())
    }

    /// Configure the driver and device for the given `Mode`
    pub async fn set_mode(&mut self, mode: Mode) -> Result<(), Error<BUS::Error>> {
        self.ll
            .odr_config()
            .write_async(|r| {
                r.set_pwr_mode(PwrMode::Standby);
            })
            .await?;

        while self.ll.odr_config().read_async().await?.pwr_mode() != PwrMode::Standby {}

        match mode {
            Mode::Normal(odr) => {
                self.ll
                    .odr_config()
                    .write_async(|r| {
                        r.set_pwr_mode(PwrMode::Normal);
                        r.set_odr(odr);
                    })
                    .await?;

                while self.ll.odr_config().read_async().await?.pwr_mode() != PwrMode::Normal {}
            }
            Mode::Continous => {
                self.ll
                    .odr_config()
                    .write_async(|r| {
                        r.set_pwr_mode(PwrMode::Continous);
                    })
                    .await?;

                while self.ll.odr_config().read_async().await?.pwr_mode() != PwrMode::Continous {}
            }
            // We'll set the mode to forced later when we actually take a measurement
            Mode::Forced => (),
            // Already in standby
            Mode::Standby => (),
        }
        self.mode = mode;

        Ok(())
    }

    /// Read the 48 bit UID from the sensors NVM.
    pub async fn get_uid(&mut self) -> Result<u64, Error<BUS::Error>> {
        let top = self.read_nvm(0x26).await? as u64 & 0xFF;
        let mid = self.read_nvm(0x25).await? as u64;
        let low = self.read_nvm(0x24).await? as u64;
        let bot = self.read_nvm(0x23).await? as u64 & 0x00FF;

        let out = top << 40 | mid << 24 | low << 8 | bot >> 8;

        Ok(out)
    }

    /// Read a temperature and pressure sample from the sensor.
    ///
    /// If an interrupt pin is present, that will be used to wait for the data ready interrupt.
    /// If there is not, and we are in force mode, the force command is sent, then data ready
    /// is polled until the data is read. If there is not interrupt pin and we are not in
    /// force mode, and the data is not ready, return `Error::DataNotReady`.
    ///
    /// Return units are (Celsius, Pascals)
    pub async fn get_sample(&mut self) -> Result<(f32, f32), Error<BUS::Error>> {
        self.pre_measure().await?;

        let sample = self.ll.temperature_pressure().read_async().await?;
        let temperature = sample.temperature() as f32 / 65536.0;
        let pressure = sample.pressure() as f32 / 64.0;

        Ok((temperature, pressure))
    }

    /// Read a temperature sample from the sensor.
    ///
    /// If an interrupt pin is present, that will be used to wait for the data ready interrupt.
    /// If there is not, and we are in force mode, the force command is sent, then data ready
    /// is polled until the data is read. If there is not interrupt pin and we are not in
    /// force mode, and the data is not ready, return `Error::DataNotReady`.
    ///
    /// Return units are Celsius
    pub async fn get_temperature(&mut self) -> Result<f32, Error<BUS::Error>> {
        self.pre_measure().await?;

        Ok(self.ll.temperature().read_async().await?.temperature() as f32 / 65536.0)
    }

    /// Read a pressure sample from the sensor.
    ///
    /// If an interrupt pin is present, that will be used to wait for the data ready interrupt.
    /// If there is not, and we are in force mode, the force command is sent, then data ready
    /// is polled until the data is read. If there is not interrupt pin and we are not in
    /// force mode, and the data is not ready, return `Error::DataNotReady`.
    ///
    /// Return units are Pascals
    pub async fn get_pressure(&mut self) -> Result<f32, Error<BUS::Error>> {
        self.pre_measure().await?;

        Ok(self.ll.pressure().read_async().await?.pressure() as f32 / 64.0)
    }

    /// Force a single measurement.
    #[inline(always)]
    pub async fn force_sample(&mut self) -> Result<(), Error<BUS::Error>> {
        if self.mode != Mode::Forced {
            return Err(Error::ForceInNonForceMode);
        }

        self.ll
            .odr_config()
            .write_async(|r| {
                r.set_pwr_mode(PwrMode::Forced);
            })
            .await
    }

    #[inline(always)]
    async fn pre_measure(&mut self) -> Result<(), Error<BUS::Error>> {
        match self.mode {
            Mode::Normal(_) | Mode::Continous => {
                if self.int.is_some() {
                    self.wait_drdy().await?;
                } else {
                    self.check_drdy().await?
                }

                Ok(())
            }
            Mode::Forced => {
                self.force_sample().await?;

                self.wait_drdy().await?;

                Ok(())
            }
            Mode::Standby => Err(Error::MeasureInStandby),
        }
    }

    #[inline(always)]
    async fn check_drdy(&mut self) -> Result<(), Error<BUS::Error>> {
        if self
            .ll
            .interrupt_status()
            .read_async()
            .await?
            .drdy_data_reg()
        {
            Ok(())
        } else {
            Err(Error::DataNotReady)
        }
    }

    #[inline(always)]
    async fn wait_drdy(&mut self) -> Result<(), Error<BUS::Error>> {
        loop {
            if self.int.is_some() {
                self.int
                    .as_mut()
                    .unwrap()
                    .wait_for_low()
                    .await
                    .map_err(|_| Error::Wait)?;
            }

            if self.check_drdy().await.is_ok() {
                break Ok(());
            }
        }
    }

    async fn read_nvm(&mut self, address: u8) -> Result<u16, Error<BUS::Error>> {
        self.ll
            .odr_config()
            .modify_async(|r| r.set_pwr_mode(PwrMode::Standby))
            .await?;

        while !self.ll.status().read_async().await?.nvm_rdy() {
            self.delay.delay_ms(10).await;
        }

        self.ll
            .nvm_access()
            .write_async(|r| {
                r.set_nvm_prog_en(false);
                r.set_nvm_row_address(address);
            })
            .await?;

        self.ll.command().write_async(|r| r.set_cmd(0x5D)).await?;
        self.ll.command().write_async(|r| r.set_cmd(0xA5)).await?;

        while !self.ll.status().read_async().await?.nvm_rdy() {
            self.delay.delay_ms(10).await;
        }

        let nvm = self.ll.nvm_data().read_async().await?;

        Ok(nvm.nvm_data())
    }
}

impl<BUS: i2c::I2c> device_driver::AsyncRegisterInterface for Interface<BUS> {
    type Error = Error<BUS::Error>;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let addr_buf = [address];
        let mut ops = [
            i2c::Operation::Write(&addr_buf),
            i2c::Operation::Write(data),
        ];
        self.bus
            .transaction(self.address, &mut ops)
            .await
            .map_err(Error::Bus)?;

        Ok(())
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let addr_buf = [address];
        let mut ops = [i2c::Operation::Write(&addr_buf), i2c::Operation::Read(data)];
        self.bus
            .transaction(self.address, &mut ops)
            .await
            .map_err(Error::Bus)?;

        Ok(())
    }
}
