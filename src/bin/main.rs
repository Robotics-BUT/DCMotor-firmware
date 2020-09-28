#![no_std]
#![no_main]

use dcdriver as _; // global logger + panicking-behavior + memory layout

use stm32f0xx_hal::delay::Delay;
use stm32f0xx_hal::timers::{Event, Timer};
use stm32f0xx_hal::{prelude::*, stm32};

use dcdriver::board::*;
use dcdriver::can::CANBus;
use dcdriver::encoder::Encoder;

const CONTROL_LOOP_PERIOD_HZ: u32 = 10;

#[rtic::app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        led: LED,
        delay: Delay,
        can: CANBus,
        encoder: Encoder,
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

        // pwm1p.set_high();
        // pwm1n.set_low();
        // pwm2p.set_low();
        // pwm2n.set_high();
        //
        pwm1p.set_low();
        pwm1n.set_high();
        pwm2p.set_high();
        pwm2n.set_low();

        // let mut adc = Adc::new(device.ADC, &mut rcc);
        //
        // loop {
        //     // defmt::debug!(
        //     //     "V: {:f32}",
        //     //     (adc.read_abs_mv(&mut input_voltage_pin) * 5) as f32 / 100.0f32
        //     // );
        //     defmt::debug!(
        //         "I: {:f32}",
        //         ((adc.read_abs_mv(&mut current_pin) as i16) - 635) as f32 / 50.0f32
        //     );
        //     delay.delay_ms(200u32);
        // }
        //
        // defmt::info!("Init complete, moving to idle.");
        init::LateResources {
            led,
            delay,
            can,
            encoder,
            control_timer: timer,
        }
    }

    #[idle(resources = [led, delay])]
    fn main(cx: main::Context) -> ! {
        loop {
            cx.resources
                .led
                .set_high()
                .expect("Failed to set LED high.");
            cx.resources.delay.delay_ms(100u32);
            cx.resources.led.set_low().expect("Failed to set LED low.");
            cx.resources.delay.delay_ms(100u32);
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

        defmt::debug!("speed: {:f32}", cx.resources.encoder.get_speed());
        // read_adc(cx.resources.adc, 4);
    }
};
