#![no_std]
#![no_main]

use dcdriver as _; // global logger + panicking-behavior + memory layout

use core::borrow::Borrow;
use stm32f0xx_hal::delay::Delay;
use stm32f0xx_hal::gpio::gpioc::*;
use stm32f0xx_hal::gpio::{Output, PushPull};
use stm32f0xx_hal::pac::CAN;
use stm32f0xx_hal::{prelude::*, stm32};

type LED = PC13<Output<PushPull>>;

#[rtic::app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        led: LED,
        delay: Delay,
        can: CAN,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        defmt::info!("Hello, world!");

        let core: cortex_m::Peripherals = cx.core;
        let mut device: stm32f0xx_hal::stm32::Peripherals = cx.device;

        let raw_rcc = device.RCC;
        raw_rcc.apb1enr.write(|w| w.canen().set_bit());
        raw_rcc.apb1rstr.write(|w| w.canrst().set_bit());
        raw_rcc.apb1rstr.write(|w| w.canrst().clear_bit());
        // todo perform manipulation of RCC register values for custom peripherals
        let mut rcc = raw_rcc
            .configure()
            .hsi48()
            .sysclk(48.mhz())
            .pclk(48.mhz())
            .hclk(48.mhz())
            .freeze(&mut device.FLASH);

        let gpioc = device.GPIOC.split(&mut rcc);
        let gpioa = device.GPIOA.split(&mut rcc);
        let mut led = cortex_m::interrupt::free(|cs| gpioc.pc13.into_push_pull_output(cs));
        led.set_high();
        let delay = Delay::new(core.SYST, &rcc);

        // CAN bus
        let (can_rx, can_tx) = cortex_m::interrupt::free(|cs| {
            (
                gpioa.pa11.into_alternate_af4(cs),
                gpioa.pa12.into_alternate_af4(cs),
            )
        });
        let can = device.CAN;
        defmt::debug!(
            "bus is in sleep state, we need to request transition to the initialization state"
        );
        can.mcr.write(|w| w.sleep().clear_bit());
        can.mcr.modify(|r, w| w.inrq().set_bit());
        defmt::debug!("next we wait for the bus to move to the initialization state");
        while !can.msr.read().inak().bit() {}
        defmt::debug!("bus is now in initialization state");
        defmt::debug!("initializing bus parameters");
        can.mcr.modify(|r, w| {
            w.ttcm()
                .clear_bit() // no time triggered communication
                .abom()
                .set_bit() // bus automatically recovers itself after error state
                .awum()
                .set_bit() // bus is automatically waken up on message RX
                .nart()
                .clear_bit() // automatic message retransmission enabled
                .rflm()
                .clear_bit() // new RX message overwrite unread older ones
                .txfp()
                .clear_bit() // TX message priority driven by the message identifier
                .sleep()
                .clear_bit() // do not sleep
        });
        defmt::debug!("bus is now initializing timing");
        // calculated using http://www.bittiming.can-wiki.info/ for STMicroelectronics bxCAN 48 MHz clock, 87.6% sample point, SJW = 1, bitrate 250 kHz
        const TIME_SEGMENT1: u8 = 13;
        const TIME_SEGMENT2: u8 = 2;
        const RESYNC_WIDTH: u8 = 1;
        const PRESCALER: u16 = 12;
        can.btr.modify(|r, w| unsafe {
            w.silm()
                .clear_bit() // disable silent mode
                .lbkm()
                .clear_bit() // disable loopback mode
                .sjw()
                .bits(RESYNC_WIDTH - 1)
                .ts2()
                .bits(TIME_SEGMENT2 - 1)
                .ts1()
                .bits(TIME_SEGMENT1 - 1)
                .brp()
                .bits(PRESCALER - 1)
        });
        defmt::debug!("bus is now initializing interrupts");

        defmt::debug!("bus is now leaving initialization");
        can.mcr.modify(|r, w| w.inrq().clear_bit());
        while !can.msr.read().inak().bit() {}

        // initialize message pending interrupts
        can.ier
            .modify(|r, w| w.fmpie0().set_bit().fmpie1().set_bit());

        defmt::debug!("bus is now initializing filters");
        can.fmr.modify(|r, w| w.finit().set_bit()); // filter init enabled
        can.fa1r.write(|w| w.fact0().clear_bit()); // filter is inactive

        can.fm1r.write(|w| w.fbm0().clear_bit()); // identifier mask mode for fbm0
        can.fs1r.write(|w| w.fsc0().set_bit()); // 32 bit scale configuration

        const FILTER0_ID: u16 = 0x0;
        const FILTER0_MASK: u16 = 0x00;
        const FILTER1_ID: u16 = 0x00;
        const FILTER1_MASK: u16 = 0x00;
        can.fb[0].fr1.write(|w| unsafe { w.bits(0) });
        can.fb[0].fr2.write(|w| unsafe { w.bits(0) });

        can.fa1r.write(|w| w.fact0().set_bit()); // filter is active
        can.fmr.modify(|r, w| w.finit().clear_bit()); // filter init disabled
        defmt::debug!("filters initialized");

        let bits = can.mcr.read().bits();
        defmt::debug!("fmp0: {0:0..31}, MAP: {0:8..9}", bits);

        defmt::info!("Init complete, moving to idle.");
        init::LateResources { led, delay, can }
    }

    #[idle(resources = [led, delay])]
    fn main(cx: main::Context) -> ! {
        loop {
            cx.resources.led.set_high();
            cx.resources.delay.delay_ms(500u16);
            cx.resources.led.set_low();
            cx.resources.delay.delay_ms(500u16);
        }
    }

    #[task(binds = CEC_CAN, resources = [can])]
    fn process_can_message(cx: process_can_message::Context) {
        let can: &mut CAN = cx.resources.can;
        let bits = can.rfr[0].read().bits();
        defmt::debug!("fmp0: {0:0..1}, MAP: {0:8..9}", bits);
    }
};
