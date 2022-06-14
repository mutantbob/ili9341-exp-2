#![no_std]
#![no_main]

use arduino_hal::{default_serial, delay_ms, Adc};
use arduino_spi::{DataOrder, SerialClockRate, Settings};
use avr_hal_generic::hal::blocking::delay::DelayMs;
use avr_hal_generic::void::Void;
use display_interface::WriteOnlyDataCommand;
use display_interface_spi::SPIInterface;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use panic_halt as _;
use rand::rngs::SmallRng;
use rand::{RngCore, SeedableRng};
use ufmt::{uWrite, uwriteln};

mod dragon;

//

struct DelayAvr {}

impl DelayMs<u16> for DelayAvr {
    fn delay_ms(&mut self, ms: u16) {
        delay_ms(ms)
    }
}

//
//

#[arduino_hal::entry]
fn main() -> ! {
    rust_arduino_runtime::arduino_main_init();

    //

    let dp = arduino_hal::Peripherals::take().unwrap();

    if false {
        // neopixel needs the clock divisor set to something other than no_clock().  Normally the arduino init() method does that (and a lot of other stuff)
        let tc0 = dp.TC0;
        tc0.tccr0b.write(|w| w.cs0().prescale_64());

        rust_arduino_runtime::micros();
        rust_arduino_runtime::digital_pin_to_bit_mask_PGM(0);
    }

    // let mut adc = Adc::new(dp.ADC, Default::default());

    let pins = arduino_hal::pins!(dp);

    let mut serial = default_serial!(dp, pins, 115200);

    let _ = uwriteln!(&mut serial, "Hello, world!");

    let mut adc = Adc::new(dp.ADC, Default::default());

    let spi = arduino_spi::default_spi!(
        dp,
        pins,
        Settings {
            data_order: DataOrder::MostSignificantFirst,
            clock: SerialClockRate::OscfOver4,
            mode: embedded_hal::spi::Mode {
                polarity: embedded_hal::spi::Polarity::IdleLow,
                phase: embedded_hal::spi::Phase::CaptureOnFirstTransition,
            },
        }
    );

    //

    let analog_channels = [
        pins.a0.into_analog_input(&mut adc).into_channel(),
        pins.a1.into_analog_input(&mut adc).into_channel(),
        pins.a2.into_analog_input(&mut adc).into_channel(),
        pins.a3.into_analog_input(&mut adc).into_channel(),
        pins.a4.into_analog_input(&mut adc).into_channel(),
        pins.a5.into_analog_input(&mut adc).into_channel(),
    ];

    let mut prng = seed_prng(&mut adc, &analog_channels);

    //

    let dc = pins.d8.into_output_high();
    let cs = pins.d10.into_output_high();
    let iface = SPIInterface::new(spi, dc, cs);

    let mut delay = DelayAvr {};

    let mut monitor = Ili9341::new(
        iface,
        pins.d9.into_output(),
        &mut delay,
        // Orientation::Portrait
        Orientation::Landscape,
        DisplaySize240x320 {},
    )
    .unwrap();

    //    monitor.command()

    //monitor.

    loop {
        match 4 {
            1 => test1(&mut prng, &mut monitor, &mut serial),
            2 => test2(&mut monitor, &mut serial),
            3 => test3(&mut monitor, &mut serial),
            _ => test4(&mut monitor, &mut serial),
        }
    }
}

fn test1<IFACE: WriteOnlyDataCommand, RST>(
    prng: &mut SmallRng,
    monitor: &mut Ili9341<IFACE, RST>,
    serial: &mut dyn uWrite<Error = Void>,
) {
    use dragon::*;
    let dragon = &DRAGON_PIXMAP;

    let x = rand_a_b(
        prng,
        0, //1 - DRAGON_WIDTH as i16
        (monitor.width() - DRAGON_WIDTH as usize) as i16,
    ) as u16;
    let y = rand_a_b(
        prng,
        0, //1 - DRAGON_HEIGHT as i16
        (monitor.height() - DRAGON_HEIGHT as usize) as i16,
    ) as u16;

    let _ = uwriteln!(serial, "at {},{}", x, y);

    let iter = dragon;

    let _ = monitor.draw_raw_iter(x, y, x + DRAGON_WIDTH - 1, y + DRAGON_HEIGHT - 1, iter);

    delay_ms(500);
}

fn test2<IFACE: WriteOnlyDataCommand, RST>(
    monitor: &mut Ili9341<IFACE, RST>,
    serial: &mut dyn uWrite<Error = Void>,
) {
    const RECT_WIDTH: u16 = 50;
    const RECT_HEIGHT: u16 = 50;

    for rotation in [
        Orientation::Landscape,
        Orientation::LandscapeFlipped,
        Orientation::Portrait,
        Orientation::PortraitFlipped,
    ] {
        let _ = monitor.set_orientation(rotation);
        for row in 0..4 {
            for col in 0..4 {
                let idx = row * 4 + col;
                let color = !(1u16 << idx);

                //let x = rand_a_b(prng, 0, max_x) as u16;
                //let y = rand_a_b(prng, 0, max_y) as u16;
                let x = col * RECT_WIDTH;
                let y = row * RECT_HEIGHT;

                let _ = uwriteln!(serial, "at {},{}", x, y);

                let iter = { (0..(RECT_WIDTH * RECT_HEIGHT)).map(|_| color) };

                let _ = monitor.draw_raw_iter(x, y, x + RECT_WIDTH - 1, y + RECT_HEIGHT - 1, iter);
            }
        }
        delay_ms(1000);
    }
}

fn test3<IFACE: WriteOnlyDataCommand, RST>(
    monitor: &mut Ili9341<IFACE, RST>,
    serial: &mut dyn uWrite<Error = Void>,
) {
    const RECT_WIDTH: u16 = 50;
    const RECT_HEIGHT: u16 = 50;

    for rotation in [
        Orientation::Landscape,
        Orientation::LandscapeFlipped,
        Orientation::Portrait,
        Orientation::PortraitFlipped,
    ] {
        let _ = monitor.set_orientation(rotation);
        let mut idx = 0;
        for red in [0xe0, 0] {
            for green in [0x7, 0] {
                for blue in [0x18, 0] {
                    let col = idx % 4;
                    let row = idx / 4;
                    let color = !(red | green | blue);

                    let x = col * RECT_WIDTH;
                    let y = row * RECT_HEIGHT;

                    let _ = uwriteln!(serial, "at {},{}", x, y);

                    let iter = { (0..(RECT_WIDTH * RECT_HEIGHT)).map(|_| color) };

                    let _ =
                        monitor.draw_raw_iter(x, y, x + RECT_WIDTH - 1, y + RECT_HEIGHT - 1, iter);
                    idx += 1;
                }
            }
        }
        delay_ms(1000);
    }
}

fn test4<IFACE: WriteOnlyDataCommand, RST>(
    monitor: &mut Ili9341<IFACE, RST>,
    serial: &mut dyn uWrite<Error = Void>,
) {
    let rect_width: u16 = monitor.width() as u16 / 16;
    let rect_height: u16 = monitor.height() as u16 / 16 - 2;

    for rotation in [
        Orientation::Landscape,
        // Orientation::LandscapeFlipped,
        // Orientation::Portrait,
        // Orientation::PortraitFlipped,
    ] {
        let _ = monitor.set_orientation(rotation);
        for idx in 0..256 {
            let col = idx % 16;
            let row = idx / 16;
            let color = idx;

            let x = col * rect_width;
            let y = row * rect_height;

            let _ = uwriteln!(serial, "at {},{}", x, y);

            let iter = { (0..(rect_width * rect_height)).map(|_| color) };

            let _ = monitor.draw_raw_iter(x, y, x + rect_width - 1, y + rect_height - 1, iter);
        }
        delay_ms(1000);
    }
}

fn seed_prng<H, ADCOPS: avr_hal_generic::adc::AdcOps<H>, CLOCK: avr_hal_generic::clock::Clock>(
    adc: &mut avr_hal_generic::adc::Adc<H, ADCOPS, CLOCK>,
    analog_channels: &[avr_hal_generic::adc::Channel<H, ADCOPS>],
) -> SmallRng {
    let seed = {
        // grab some (hopefully) random values from the analog pins and mash them together into a
        // seed for the random number generator

        analog_channels
            .iter()
            .map(|ch| adc.read_blocking(ch))
            .fold(0, |acc, item| item ^ (acc * 13))
    };
    //let _ = uwriteln!(&mut serial, "PRNG seed = {}", seed);

    let mut array = [0; 16];
    array[0] = seed as u8;
    array[1] = (seed >> 8) as u8;
    SmallRng::from_seed(array)
}

pub fn rand_a_b(prng: &mut dyn RngCore, low: i16, after_high: i16) -> i16 {
    let span = (after_high - low) as u32;
    low + (prng.next_u32() % span) as i16
}

/*
cargo build --release --features atmega328p --target ../avr-specs/avr-atmega328p.json &&
elf=$(echo target/avr-atmega328p/release/*.elf) &&
avrdude -C /etc/avrdude.conf -v -p atmega328p -c arduino -P /dev/ttyACM0  -D -Uflash:w:$elf:e
 */

*/
