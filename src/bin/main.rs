#![no_std]
#![no_main]

use dcdriver as _; // global logger + panicking-behavior + memory layout

use stm32f0xx_hal::delay::Delay;
use stm32f0xx_hal::timers::{Event, Timer};
use stm32f0xx_hal::{prelude::*, stm32};

use dcdriver::adc::ADC;
use dcdriver::board::*;
use dcdriver::can::CANBus;
use dcdriver::encoder::Encoder;
use stm32f0xx_hal::pac::Interrupt::ADC_COMP;

const CONTROL_LOOP_PERIOD_HZ: u32 = 10;

#[rtic::app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        led: LED,
        delay: Delay,
        can: CANBus,
        encoder: Encoder,
        adc: ADC,
        control_timer: Timer<ControlTimer>,
        #[init(0)]
        last_position: u16,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        defmt::info!("Hello, world!");

        let core: cortex_m::Peripherals = cx.core;
        let mut device: stm32f0xx_hal::stm32::Peripherals = cx.device;

        let raw_rcc = device.RCC;
        raw_rcc.apb2enr.modify(|_, w| w.tim1en().enabled());
        raw_rcc.apb2rstr.modify(|_, w| w.tim1rst().reset());
        raw_rcc.apb2rstr.modify(|_, w| w.tim1rst().clear_bit());

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

        let (can_rx, can_tx, mut pwm1p, mut pwm2p, mut input_voltage_pin, mut current_pin) =
            cortex_m::interrupt::free(|cs| {
                (
                    gpioa.pa11.into_alternate_af4(cs),
                    gpioa.pa12.into_alternate_af4(cs),
                    gpioa.pa8.into_alternate_af2(cs),
                    gpioa.pa9.into_alternate_af2(cs),
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
                    gpiob.pb13.into_alternate_af2(cs),
                    gpiob.pb14.into_alternate_af2(cs),
                )
            });

        let can = CANBus::new(device.CAN, can_rx, can_tx);
        let encoder = Encoder::new(
            device.TIM3,
            enc_a,
            enc_b,
            enc_i,
            CONTROL_LOOP_PERIOD_HZ as f32,
        );

        let delay = Delay::new(core.SYST, &rcc);

        // set-up control loop sampling timer
        let mut timer = Timer::tim2(device.TIM2, CONTROL_LOOP_PERIOD_HZ.hz(), &mut rcc);
        timer.listen(Event::TimeOut);

        // set up motor control pwm
        let tim1 = device.TIM1;
        tim1.cr1.modify(|_, w| {
            w.ckd()
                .div1() // clock deadtime the same as timer
                .arpe()
                .enabled() // buffer ARR
                .cms()
                .center_aligned2() // center aligned mode 2 - interrupt flags set only when counting up
                .urs()
                .counter_only() // only counter events overflow, underflow generate interrupt
        });

        tim1.psc.write(|w| w.psc().bits(0));
        tim1.rcr.write

        let adc = ADC::new(device.ADC);
        adc.start_periodic_reading();

        defmt::info!("Init complete, moving to idle.");
        init::LateResources {
            led,
            delay,
            can,
            encoder,
            adc,
            control_timer: timer,
        }
    }

    #[idle(resources = [led, delay, adc])]
    fn main(mut cx: main::Context) -> ! {
        loop {
            cx.resources
                .led
                .set_high()
                .expect("Failed to set LED high.");
            cx.resources.delay.delay_ms(100u32);
            cx.resources.led.set_low().expect("Failed to set LED low.");
            cx.resources.delay.delay_ms(100u32);
            cx.resources
                .adc
                .lock(|adc| defmt::debug!("V {:f32}", adc.get_system_voltage()));
            cx.resources
                .adc
                .lock(|adc| defmt::debug!("I {:f32}", adc.get_motor_current()));
        }
    }

    #[task(binds = CEC_CAN, resources = [can])]
    fn process_can_message(cx: process_can_message::Context) {
        cx.resources.can.interrupt();
    }

    #[task(binds = TIM2, resources = [control_timer, encoder, last_position])]
    fn control_loop_tick(cx: control_loop_tick::Context) {
        let control_timer: &mut Timer<ControlTimer> = cx.resources.control_timer;
        if control_timer.wait().is_err() {
            defmt::error!("Timer wait errored unexpectedly.");
        }; // FIXME add method to timer to clear interrupt, without the need to call wait.

        // defmt::debug!("speed: {:f32}", cx.resources.encoder.get_speed());
        // read_adc(cx.resources.adc, 4);
    }

    #[task(binds = ADC_COMP, resources = [adc])]
    fn adc_conversion_complete(cx: adc_conversion_complete::Context) {
        cx.resources.adc.interrupt();
    }
};
