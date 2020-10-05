#![no_std]
#![no_main]

use dcdriver as _; // global logger + panicking-behavior + memory layout

use stm32f0xx_hal::delay::Delay;
use stm32f0xx_hal::timers::{Event, Timer};
use stm32f0xx_hal::{prelude::*, stm32};

use dcdriver::adc::ADC;
use dcdriver::board::*;
use dcdriver::bridge::Bridge;
use dcdriver::can;
use dcdriver::can::{CANBus, CANError, CANFrame};
use dcdriver::controller::Controller;
use dcdriver::encoder::Encoder;
use nb::Error;
use stm32f0xx_hal::pac::Interrupt::{ADC_COMP, TIM1_BRK_UP_TRG_COM};
use stm32f0xx_hal::time::Hertz;

const CONTROL_LOOP_FREQUENCY_HZ: u32 = 1000;

#[rtic::app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        led: LED,
        led2: LED2,
        delay: Delay,
        can: CANBus,
        encoder: Encoder,
        adc: ADC,
        control_timer: Timer<ControlTimer>,
        bridge: Bridge,
        controller: Controller,
        #[init(0)]
        last_position: u16,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
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
                    gpioa.pa8.into_alternate_af2(cs),
                    gpioa.pa9.into_alternate_af2(cs),
                    gpioa.pa5.into_analog(cs),
                    gpioa.pa6.into_analog(cs),
                )
            });

        let (enc_a, enc_b, enc_i, mut led, mut led2, mut pwm1n, mut pwm2n) =
            cortex_m::interrupt::free(|cs| {
                (
                    gpiob.pb4.into_alternate_af1(cs),
                    gpiob.pb5.into_alternate_af1(cs),
                    gpiob.pb0.into_alternate_af1(cs),
                    gpiob.pb2.into_push_pull_output(cs),
                    gpiob.pb10.into_push_pull_output(cs),
                    gpiob.pb13.into_alternate_af2(cs),
                    gpiob.pb14.into_alternate_af2(cs),
                )
            });

        let can = CANBus::new(device.CAN, can_rx, can_tx);
        can.listen(can::Event::RxMessagePending);

        let encoder = Encoder::new(
            device.TIM3,
            enc_a,
            enc_b,
            enc_i,
            CONTROL_LOOP_FREQUENCY_HZ as f32,
        );

        let delay = Delay::new(core.SYST, &rcc);

        // set-up control loop sampling timer
        let mut timer = Timer::tim2(device.TIM2, CONTROL_LOOP_FREQUENCY_HZ.hz(), &mut rcc);
        timer.listen(Event::TimeOut);

        // set up motor control pwm
        let tim1 = device.TIM1;
        let bridge = Bridge::new(tim1, pwm1p, pwm1n, pwm2p, pwm2n);

        let dt = 1.0f32 / (CONTROL_LOOP_FREQUENCY_HZ as f32);
        let controller = Controller::new(0.2, 0.4, dt);

        let adc = ADC::new(device.ADC);
        adc.start_periodic_reading();

        init::LateResources {
            led,
            led2,
            delay,
            can,
            encoder,
            adc,
            control_timer: timer,
            bridge,
            controller,
        }
    }

    #[task(binds = TIM1_BRK_UP_TRG_COM, resources = [bridge])]
    fn tim1_irs(cx: tim1_irs::Context) {
        // let tim1: &mut stm32::TIM1 = cx.resources.tim1;
        // tim1.sr.modify(|_, w| w.uif().clear());
        // defmt::debug!("picifuk")
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
            // cx.resources
            //     .adc
            //     .lock(|adc| defmt::debug!("V {:f32}", adc.get_system_voltage()));
            // cx.resources
            //     .adc
            //     .lock(|adc| defmt::debug!("I {:f32}", adc.get_motor_current()));
        }
    }

    #[task(binds = CEC_CAN, resources = [can])]
    fn process_can_message(cx: process_can_message::Context) {
        let can: &CANBus = cx.resources.can;
        match can.read() {
            Ok(frame) => match frame.id & 0xff80 {
                0x80 => {
                    defmt::debug!("SYNC");
                    can.write(&CANFrame {
                        id: 0x500,
                        rtr: false,
                        dlc: 3,
                        data: [0u8; 8],
                    });
                }
                _ => {
                    defmt::debug!("unsupported id: {:u16}", frame.id & 0xff80);
                }
            },
            Err(_) => {
                defmt::debug!("Failed to read.");
            }
        }
        // match can.read() {
        //     Ok(frame) => {
        //         defmt::debug!("id: {:u16}", frame.id);
        //         match can.write(&frame) {
        //             Ok(_) => {
        //                 defmt::debug!("sent");
        //             }
        //             Err(_) => {
        //                 defmt::debug!("wb");
        //             }
        //         }
        //     }
        //     Err(_) => {
        //         defmt::debug!("would block");
        //     }
        // }
    }

    #[task(binds = TIM2, resources = [control_timer, encoder, last_position, bridge, adc, led2, controller])]
    fn control_loop_tick(cx: control_loop_tick::Context) {
        let control_timer: &mut Timer<ControlTimer> = cx.resources.control_timer;
        if control_timer.wait().is_err() {
            defmt::error!("Timer wait errored unexpectedly.");
        };

        let led2: &mut LED2 = cx.resources.led2;
        led2.set_high();

        let controller: &mut Controller = cx.resources.controller;
        let bridge: &mut Bridge = cx.resources.bridge;

        let target_speed = 0.0f32;
        let current_speed: f32 = cx.resources.encoder.get_speed();

        bridge.set_duty(controller.calculate_action(target_speed, current_speed));
        led2.set_low();
    }

    #[task(binds = ADC_COMP, resources = [adc])]
    fn adc_conversion_complete(cx: adc_conversion_complete::Context) {
        cx.resources.adc.interrupt();
    }
};
