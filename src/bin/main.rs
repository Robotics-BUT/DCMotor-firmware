#![no_std]
#![no_main]

use dcdriver as _; // global logger + panicking-behavior + memory layout

use core::borrow::Borrow;
use core::num::Wrapping;
use stm32f0xx_hal::adc::Adc;
use stm32f0xx_hal::delay::Delay;
use stm32f0xx_hal::gpio::gpioa::*;
use stm32f0xx_hal::gpio::gpiob::*;
use stm32f0xx_hal::gpio::gpioc::*;
use stm32f0xx_hal::gpio::{Alternate, Output, PushPull, AF1, AF2};
use stm32f0xx_hal::pac::CAN;
use stm32f0xx_hal::stm32f0::stm32f0x1::adc::cfgr1::{ALIGN_A, RES_A};
use stm32f0xx_hal::stm32f0::stm32f0x1::adc::smpr::SMP_A;
use stm32f0xx_hal::timers::{Event, Timer};
use stm32f0xx_hal::{prelude::*, stm32};

type PWM1P = PA8<Alternate<AF2>>;
type PWM1N = PB13<Alternate<AF2>>;
type PWM2P = PA9<Alternate<AF2>>;
type PWM2N = PB14<Alternate<AF2>>;
type ENC1 = PB4<Alternate<AF1>>;
type ENC2 = PB5<Alternate<AF1>>;
type ENCI = PB0<Alternate<AF1>>; // is PB3 AF2 on REVA boards
type LED = PB2<Output<PushPull>>;
type ControlTimer = Timer<stm32::TIM2>;

const CONTROL_LOOP_PERIOD_HZ: u32 = 10;

#[rtic::app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        led: LED,
        delay: Delay,
        can: CAN,
        control_timer: ControlTimer,
        tim3: stm32::TIM3,
        #[init(0)]
        last_position: u16,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        defmt::info!("Hello, world!");

        let core: cortex_m::Peripherals = cx.core;
        let mut device: stm32f0xx_hal::stm32::Peripherals = cx.device;

        let raw_rcc = device.RCC;
        raw_rcc.apb1enr.modify(|r, w| w.canen().set_bit());
        raw_rcc.apb1rstr.modify(|r, w| w.canrst().set_bit());
        raw_rcc.apb1rstr.modify(|r, w| w.canrst().clear_bit());

        // TIMER 3 - used for encoder interface
        raw_rcc.apb1enr.modify(|r, w| w.tim3en().set_bit());
        raw_rcc.apb1rstr.modify(|r, w| w.tim3rst().set_bit());
        raw_rcc.apb1rstr.modify(|r, w| w.tim3rst().clear_bit());
        // adc

        // raw_rcc.apb2enr.modify(|_, w| w.adcen().enabled());
        // raw_rcc.cr2.modify(|_, w| w.hsi14on().on());
        // while raw_rcc.cr2.read().hsi14rdy().is_not_ready() {}

        // todo perform manipulation of RCC register values for custom peripherals
        let mut rcc = raw_rcc
            .configure()
            .hsi48()
            .sysclk(48.mhz())
            .pclk(48.mhz())
            .hclk(48.mhz())
            .freeze(&mut device.FLASH);

        let gpioa = device.GPIOA.split(&mut rcc);
        let gpiob = device.GPIOB.split(&mut rcc);
        let gpioc = device.GPIOC.split(&mut rcc);

        let (can_rx, can_tx, mut pwm1p, mut pwm2p, mut input_voltage_pin, mut current_pin) =
            cortex_m::interrupt::free(|cs| {
                (
                    gpioa.pa11.into_alternate_af4(cs),
                    gpioa.pa12.into_alternate_af4(cs),
                    gpioa.pa8.into_push_pull_output(cs),
                    gpioa.pa9.into_push_pull_output(cs),
                    gpioa.pa5.into_analog(cs),
                    gpioa.pa6.into_analog(cs),
                )
            });

        let (enc_a, enc_b, enc_i, mut led, mut pwm1n, mut pwm2n) =
            cortex_m::interrupt::free(|cs| {
                (
                    gpiob.pb4.into_alternate_af1(cs),
                    gpiob.pb5.into_alternate_af1(cs),
                    gpiob.pb0.into_alternate_af1(cs),
                    gpiob.pb2.into_push_pull_output(cs),
                    gpiob.pb13.into_push_pull_output(cs),
                    gpiob.pb14.into_push_pull_output(cs),
                )
            });
        led.set_low();
        let mut delay = Delay::new(core.SYST, &rcc);

        // CAN bus
        let can = device.CAN;
        initialize_can(&can);

        let tim3 = device.TIM3;
        // enable counting on both edges of encoder pulses
        tim3.smcr.modify(|r, w| w.sms().encoder_mode_3());
        //
        tim3.ccmr2_input().modify(|r, w| w.cc3s().ti3());
        // enable capture on channel 3, set inverted polarity (CC3NP = 0, CC3P = 1)
        tim3.ccer.modify(|r, w| w.cc3e().set_bit().cc3p().set_bit());
        tim3.psc.write(|w| unsafe { w.bits(0) }); // no prescaler
        tim3.arr.write(|w| unsafe { w.bits(0xffff) }); // reload value
        tim3.cnt.write(|w| unsafe { w.bits(0) }); // reset counter value
        tim3.ccr3.write(|w| unsafe { w.bits(0) }); // reset compare value
        tim3.cr1.modify(|r, w| w.cen().set_bit());

        // set-up control loop sampling timer
        let mut timer = Timer::tim2(device.TIM2, CONTROL_LOOP_PERIOD_HZ.hz(), &mut rcc);
        timer.listen(Event::TimeOut);

        // pwm1p.set_high();
        // pwm1n.set_low();
        // pwm2p.set_low();
        // pwm2n.set_high();
        //
        pwm1p.set_low();
        pwm1n.set_high();
        pwm2p.set_high();
        pwm2n.set_low();

        // set up adc
        // let adc = device.ADC;
        // // calibrate
        // if adc.cr.read().aden().is_enabled() {
        //     adc.cr.modify(|_, w| w.addis().disable());
        // }
        // while adc.cr.read().aden().bit_is_set() {}
        // adc.cfgr1.modify(|_, w| w.dmaen().disabled());
        // adc.cr.modify(|_, w| w.adcal().start_calibration());
        //
        // defmt::debug!("Waiting for ADC calibration.");
        // while adc.cr.read().adcal().is_calibrating() {}
        // defmt::debug!("ADC calibration done.");
        //
        // if adc.isr.read().adrdy().is_ready() {
        //     adc.isr.modify(|_, w| w.adrdy().clear());
        // }
        // adc.cr.modify(|_, w| w.aden().enabled());
        // while adc.isr.read().adrdy().is_not_ready() {}

        let mut adc = Adc::new(device.ADC, &mut rcc);

        loop {
            // defmt::debug!(
            //     "V: {:f32}",
            //     (adc.read_abs_mv(&mut input_voltage_pin) * 5) as f32 / 100.0f32
            // );
            defmt::debug!(
                "I: {:f32}",
                ((adc.read_abs_mv(&mut current_pin) as i16) - 635) as f32 / 50.0f32
            );
            delay.delay_ms(200u32);
        }

        defmt::info!("Init complete, moving to idle.");
        init::LateResources {
            led,
            delay,
            can,
            control_timer: timer,
            tim3,
        }
    }

    #[idle(resources = [led, delay])]
    fn main(cx: main::Context) -> ! {
        loop {
            cx.resources.led.set_high();
            cx.resources.delay.delay_ms(100u32);
            cx.resources.led.set_low();
            cx.resources.delay.delay_ms(100u32);
        }
    }

    #[task(binds = CEC_CAN, resources = [can])]
    fn process_can_message(cx: process_can_message::Context) {
        let can: &mut CAN = cx.resources.can;
        let bits = can.rfr[0].read().bits();
        defmt::debug!("fmp0: {0:0..1}, MAP: {0:8..9}", bits);
    }

    #[task(binds = TIM2, resources = [control_timer, tim3, last_position])]
    fn control_loop_tick(cx: control_loop_tick::Context) {
        let control_timer: &mut ControlTimer = cx.resources.control_timer;
        let tim3: &mut stm32::TIM3 = cx.resources.tim3;
        control_timer.wait(); // FIXME add method to timer to clear interrupt, without the need to call wait.

        let cnt = tim3.cnt.read().bits() as u16;
        let diff = cnt - *cx.resources.last_position;
        *cx.resources.last_position = cnt;

        let speed = ((diff as i16) as f32) / 65535.0f32 * 3.14f32 * (CONTROL_LOOP_PERIOD_HZ as f32);
        defmt::debug!("speed: {:f32}", speed);
        // read_adc(cx.resources.adc, 4);
    }
};

fn read_adc(adc: &mut stm32::ADC, channel: u8) -> f32 {
    adc.chselr.write(|w| unsafe { w.chsel13().selected() });

    adc.smpr.write(|w| w.smp().variant(SMP_A::CYCLES41_5));

    adc.cfgr1.modify(|_, w| {
        w.res()
            .variant(RES_A::TWELVEBIT)
            .align()
            .variant(ALIGN_A::RIGHT)
    });

    adc.cr.modify(|_, w| w.adstart().start_conversion());
    while adc.isr.read().eoc().is_not_complete() {}

    let res = adc.dr.read().bits() as u32;

    adc.cr.modify(|_, w| w.adstp().stop_conversion());
    while adc.cr.read().adstp().is_stopping() {}
    defmt::debug!("adc: {:u32}", res);
    0f32
}

fn initialize_can(can: &CAN) {
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
}
