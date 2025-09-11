#![no_std]

use embedded_hal_async::{delay::DelayNs, digital::Wait, i2c};

device_driver::create_device!(
    manifest: "device.kdl"
);

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

pub struct Bmp58x<BUS, WAIT, DELAY> {
    pub ll: LowLevel<Interface<BUS>>,
    int: Option<WAIT>,
    delay: DELAY,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    Wait,
    DataNotReady,
    OdrOverSampleOutOfRange,
    NvmOutOfRange,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq)]
pub enum AddressSdo {
    High,
    Low,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Config {
    pub odr: Odr,
    pub pressure_oversample: OversamplingRate,
    pub temperature_oversample: OversamplingRate,
}

impl Default for Config {
    fn default() -> Self {
        Config::standard(Odr::MilliHz1000)
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum ConfigError {
    OdrOutOfRange,
}

impl Config {
    pub fn lowest_power(odr: Odr) -> Self {
        Self {
            odr,
            temperature_oversample: OversamplingRate::N1,
            pressure_oversample: OversamplingRate::N1,
        }
    }

    pub fn standard(odr: Odr) -> Self {
        Self {
            odr,
            temperature_oversample: OversamplingRate::N1,
            pressure_oversample: OversamplingRate::N4,
        }
    }

    pub fn highest_resolution(odr: Odr) -> Result<Self, ConfigError> {
        if [
            Odr::MilliHz10000,
            Odr::MilliHz5000,
            Odr::MilliHz4000,
            Odr::MilliHz3000,
            Odr::MilliHz2000,
            Odr::MilliHz1000,
            Odr::MilliHz0500,
            Odr::MilliHz0250,
            Odr::MilliHz0125,
        ]
        .contains(&odr)
        {
            Ok(Self {
                odr,
                temperature_oversample: OversamplingRate::N8,
                pressure_oversample: OversamplingRate::N128,
            })
        } else {
            Err(ConfigError::OdrOutOfRange)
        }
    }
}

impl<BUS, WAIT: Wait, DELAY: DelayNs> Bmp58x<BUS, WAIT, DELAY> {
    pub fn new(bus: BUS, int: Option<WAIT>, delay: DELAY, sdo_state: AddressSdo) -> Self {
        let ll = LowLevel::new(Interface::new(bus, sdo_state));

        Self { ll, int, delay }
    }
}

impl<BUS: embedded_hal_async::i2c::I2c, WAIT: Wait, DELAY: DelayNs> Bmp58x<BUS, WAIT, DELAY> {
    pub async fn init(&mut self, config: &Config) -> Result<(), Error<BUS::Error>> {
        self.ll.command().write_async(|r| r.set_cmd(0xB6)).await?;

        self.delay.delay_ms(100).await;

        while !self.ll.interrupt_status().read_async().await?.por() {
            self.delay.delay_ms(10).await;
        }

        self.ll
            .odr_config()
            .write_async(|r| {
                r.set_pwr_mode(PwrMode::Normal);
                r.set_odr(config.odr);
            })
            .await?;

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

        if self.int.is_some() {
            self.ll
                .int_source()
                .write_async(|r| r.set_drdy_data_reg_en(true))
                .await?;

            self.ll
                .interrupt_config()
                .write_async(|r| r.set_int_en(true))
                .await?;
        }

        Ok(())
    }

    /// Try to get a sample from the sensor, if interrupt_status.drdy is not set, return
    /// Error::DataNotReady
    ///
    /// Return units are (Celsius, Pascals)
    pub async fn try_sample(&mut self) -> Result<(f32, f32), Error<BUS::Error>> {
        self.check_drdy().await?;

        self.get_sample().await
    }

    /// Wait for a sample to be ready. If we have an interrupt pin, wait on that,
    /// otherwise loop reading the interrupt status dataready bit.
    ///
    /// Return units are (Celsius, Pascals)
    pub async fn wait_sample(&mut self) -> Result<(f32, f32), Error<BUS::Error>> {
        self.wait_drdy().await?;

        self.get_sample().await
    }

    /// Try to get a sample from the sensor, if interrupt_status.drdy is not set, return
    /// Error::DataNotReady
    ///
    /// Return units are Celsius
    pub async fn try_temperature(&mut self) -> Result<f32, Error<BUS::Error>> {
        self.check_drdy().await?;

        self.get_temperature().await
    }

    /// Wait for a sample to be ready. If we have an interrupt pin, wait on that,
    /// otherwise loop reading the interrupt status dataready bit.
    ///
    /// Return units are Celsius
    pub async fn wait_temperature(&mut self) -> Result<f32, Error<BUS::Error>> {
        self.wait_drdy().await?;

        self.get_temperature().await
    }

    /// Try to get a sample from the sensor, if interrupt_status.drdy is not set, return
    /// Error::DataNotReady
    ///
    /// Return units are Pascals
    pub async fn try_pressure(&mut self) -> Result<f32, Error<BUS::Error>> {
        self.check_drdy().await?;

        self.get_pressure().await
    }

    /// Wait for a sample to be ready. If we have an interrupt pin, wait on that,
    /// otherwise loop reading the interrupt status dataready bit.
    ///
    /// Return units are Pascals
    pub async fn wait_pressure(&mut self) -> Result<f32, Error<BUS::Error>> {
        self.wait_drdy().await?;

        self.get_pressure().await
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
            if self.check_drdy().await.is_ok() {
                break Ok(());
            }

            if self.int.is_some() {
                self.int
                    .as_mut()
                    .unwrap()
                    .wait_for_low()
                    .await
                    .map_err(|_| Error::Wait)?;
            }
        }
    }

    #[inline(always)]
    async fn get_temperature(&mut self) -> Result<f32, Error<BUS::Error>> {
        let temperature = self.ll.temperature().read_async().await?.temperature() as f32 / 65536.0;

        Ok(temperature)
    }

    #[inline(always)]
    async fn get_pressure(&mut self) -> Result<f32, Error<BUS::Error>> {
        let temperature = self.ll.pressure().read_async().await?.pressure() as f32 / 64.0;

        Ok(temperature)
    }

    #[inline(always)]
    async fn get_sample(&mut self) -> Result<(f32, f32), Error<BUS::Error>> {
        let sample = self.ll.temperature_pressure().read_async().await?;
        let temperature = sample.temperature() as f32 / 65536.0;
        let pressure = sample.pressure() as f32 / 64.0;

        Ok((temperature, pressure))
    }

    pub async fn get_uid(&mut self) -> Result<u64, Error<BUS::Error>> {
        let top = self.read_nvm(0x26).await? as u64 & 0xFF;
        let mid = self.read_nvm(0x25).await? as u64;
        let low = self.read_nvm(0x24).await? as u64;
        let bot = self.read_nvm(0x23).await? as u64 & 0x00FF;

        let out = top << 40 | mid << 24 | low << 8 | bot >> 8;

        Ok(out)
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
