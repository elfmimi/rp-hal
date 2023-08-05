//! This example toggles the GPIO25 pin, using a PIO program.
//!
//! If a LED is connected to that pin, like on a Pico board, the LED should blink.
#![no_std]
#![no_main]

// use cortex_m_rt::entry;
use rp2040_hal_macros::entry;
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::pio::PIOExt;
use hal::Sio;
use hal::Clock;
use panic_halt as _;
use rp2040_hal as hal;
use embedded_time::fixed_point::FixedPoint;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[link_section = ".ram_func"]
#[inline(never)]
fn forever() -> ! {
    forever2()
}

#[link_section = ".ram_func"]
#[inline(never)]
fn forever2() -> ! {
    #[allow(clippy::empty_loop)]
    loop {
    }
}

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // configure LED pin for Pio0.
    let _led: Pin<_, FunctionPio0> = pins.gpio0.into_mode();
    // let _led: Pin<_, FunctionPio0> = pins.gpio25.into_mode();
    // PIN id for use inside of PIO
    let led_pin_id = 0;
    // let led_pin_id = 25;

    // Define some simple PIO program.
    // const MAX_DELAY: u8 = 31;
    let mut a = pio::Assembler::<{pio::RP2040_MAX_PROGRAM_SIZE}>::new();
    let mut jump_target = a.label();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    // Set pin as Out
    a.set(pio::SetDestination::PINDIRS, 1);
    // Define begin of program loop
    a.bind(&mut wrap_target);
    a.pull(false, true);
    a.mov(pio::MovDestination::X, pio::MovOperation::None, pio::MovSource::OSR);
    // a.set(pio::SetDestination::X, 1);
    a.bind(&mut jump_target);
    // Set pin low
    // a.set_with_delay(pio::SetDestination::PINS, 0, MAX_DELAY);
    // a.set(pio::SetDestination::PINS, 0);
    a.set_with_delay(pio::SetDestination::PINS, 0, 1);
    // Set pin high
    // a.set_with_delay(pio::SetDestination::PINS, 1, MAX_DELAY);
    a.set(pio::SetDestination::PINS, 1);
    a.jmp(pio::JmpCondition::XDecNonZero, &mut jump_target);
    a.set(pio::SetDestination::PINS, 0);
    // Define end of program loop
    a.bind(&mut wrap_source);
    // The labels wrap_target and wrap_source, as set above,
    // define a loop which is executed repeatedly by the PIO
    // state machine.
    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    // let div = 0f32; // as slow as possible (0 is interpreted as 65536)
    let div = 125f32; // as slow as possible (0 is interpreted as 65536)
    // let div = 10000f32; // as slow as possible (0 is interpreted as 65536)
    let (sm, _, mut tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .set_pins(led_pin_id, 1)
        .clock_divisor(div)
        .build(sm0);
    let _running = sm.start();
    delay.delay_ms(500);
    tx.write(5-1);
    delay.delay_us(20);
    tx.write(2-1);

    // PIO runs in background, independently from CPU
    // #[allow(clippy::empty_loop)]
    // loop {}
    forever();
}
