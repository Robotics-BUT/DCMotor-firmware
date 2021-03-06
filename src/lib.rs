#![no_std]

pub mod adc;
pub mod api;
pub mod board;
pub mod bridge;
pub mod can;
pub mod canopen;
mod consts;
pub mod controller;
pub mod encoder;

use core::sync::atomic::{AtomicUsize, Ordering};

use defmt_rtt as _; // global logger
use panic_probe as _;
// TODO(5) adjust HAL import
// use some_hal as _; // memory layout
use stm32f0xx_hal as _;

#[defmt::timestamp]
fn timestamp() -> u64 {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n as u64
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
