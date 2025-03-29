//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use utils::u8_to_decimal_str;
use waveshare_rp2040_zero as bsp;

use bsp::entry;
use bsp::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{FunctionI2C, Pin},
        pac,
        pio::PIOExt,
        timer::Timer,
        watchdog::Watchdog,
        Sio,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};
use core::iter::once;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use panic_probe as _;
use smart_leds::{brightness, SmartLedsWrite};
use ws2812_pio::Ws2812;

use bsp::hal::fugit::RateExtU32;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text, // Text is here now
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use rotary_encoder_hal::{Direction, Rotary};

mod neopixel;
mod utils;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gp0.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gp1.reconfigure();

    let rot_a_pin = pins.gp2.into_pull_up_input();
    let rot_b_pin = pins.gp3.into_pull_up_input();

    let mut enc = Rotary::new(rot_a_pin, rot_b_pin);

    // Create the I²C drive, using the two pre-configured pins.
    let i2c = bsp::hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate180)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #16 on the Waveshare RP2040-Zero.
        pins.neopixel.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    // Build a text style
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // Infinite colour wheel loop
    let mut n: u8 = 128;
    let mut timer = timer; // rebind to force a copy of the timer

    loop {
        if let Ok(direction) = enc.update() {
            if direction != Direction::None {
                if direction == Direction::Clockwise {
                    n = n.wrapping_add(1);
                } else {
                    n = n.wrapping_sub(1);
                }

                // Avoid microsteps
                if n % 4 == 0 {
                    // Write valuet to the neopixel
                    ws.write(brightness(once(neopixel::wheel(n)), 122)).unwrap();

                    // Show number in the screen
                    let mut buffer = [0u8; 3];
                    let n_str = u8_to_decimal_str(n, &mut buffer);
                    display.clear(BinaryColor::Off).unwrap();
                    Text::new(n_str, Point::new(32, 32), style)
                        .draw(&mut display)
                        .unwrap();
                    display.flush().unwrap();
                }
            }
        }
        timer.delay_ms(1);
    }
}
