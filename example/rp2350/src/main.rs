#![no_std]
#![no_main]

use bmp58x_rs::{AddressSdo, Config, Odr, OversamplingRate};
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, gpio, i2c, peripherals};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer, block_for};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<peripherals::I2C1>;
});

// Hardware

pub type SensorI2c = i2c::I2c<'static, peripherals::I2C1, i2c::Async>;
pub type Bmp = bmp58x_rs::Bmp58x<
    I2cDevice<'static, CriticalSectionRawMutex, SensorI2c>,
    gpio::Input<'static>,
    Delay,
>;

static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, SensorI2c>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Startup");

    let p = embassy_rp::init(Default::default());

    let i2c = i2c::I2c::new_async(p.I2C1, p.PIN_3, p.PIN_2, Irqs, Default::default());
    let i2c = I2C_BUS.init(Mutex::<CriticalSectionRawMutex, _>::new(i2c));

    let mut bmp = Bmp::new(
        I2cDevice::new(i2c),
        Some(gpio::Input::new(p.PIN_12, gpio::Pull::Up)),
        //None,
        Delay,
        AddressSdo::High,
    );

    info!("{:X}", bmp.get_uid().await);

    bmp.init(&Config::standard(Odr::MilliHz1000)).await.unwrap();

    let mut led = gpio::Output::new(p.PIN_25, gpio::Level::Low);

    loop {
        led.toggle();
        info!("{}", bmp.wait_sample().await);
    }
}
