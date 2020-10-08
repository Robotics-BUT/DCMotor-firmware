#![no_std]
#![no_main]

use dcdriver as _; // global logger + panicking-behavior + memory layout

use stm32f0xx_hal::delay::Delay;
use stm32f0xx_hal::prelude::*;
use stm32f0xx_hal::timers::{Event, Timer};

use core::convert::TryFrom;
use core::ptr;
use dcdriver::adc::ADC;
use dcdriver::board::*;
use dcdriver::bridge::Bridge;
use dcdriver::can::CANBus;
use dcdriver::canopen::*;
use dcdriver::controller::Controller;
use dcdriver::encoder::Encoder;
use dcdriver::{api, can, canopen};
use stm32f0xx_hal::stm32;

const CONTROL_LOOP_FREQUENCY_HZ: u32 = 1000;
const ID: u8 = 9;
const FAILSAFE_FULL: u8 = 3;

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
        nmt_timer: Timer<NMTTimer>,
        #[init(0)]
        last_position: u16,
        #[init(canopen::NMTState::BootUp)]
        nmt_state: canopen::NMTState,
        #[init(0)]
        failsafe_counter: u8, // when counter reaches 0, motors must stop
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

        let (can_rx, can_tx, pwm1p, pwm2p, _input_voltage_pin, _current_pin) =
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

        let (enc_a, enc_b, enc_i, led, led2, pwm1n, pwm2n) = cortex_m::interrupt::free(|cs| {
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
        let mut control_timer = Timer::tim2(device.TIM2, CONTROL_LOOP_FREQUENCY_HZ.hz(), &mut rcc);
        control_timer.listen(Event::TimeOut);

        // set-up nmt timer loop
        let mut nmt_timer = Timer::tim6(device.TIM6, 1.hz(), &mut rcc);
        nmt_timer.listen(Event::TimeOut);

        // set up motor control pwm
        let tim1 = device.TIM1;
        let bridge = Bridge::new(tim1, pwm1p, pwm1n, pwm2p, pwm2n);

        let dt = 1.0f32 / (CONTROL_LOOP_FREQUENCY_HZ as f32);
        let mut controller = Controller::new(0.2, 0.4, dt);

        let adc = device.ADC;
        let adc = ADC::new(adc);

        controller.set_target(0.0);

        init::LateResources {
            led,
            led2,
            delay,
            can,
            encoder,
            adc,
            control_timer,
            nmt_timer,
            bridge,
            controller,
        }
    }

    #[task(binds = TIM6_DAC, resources = [nmt_timer, can, nmt_state, failsafe_counter, controller])]
    fn nmt(cx: nmt::Context) {
        let timer: &mut Timer<NMTTimer> = cx.resources.nmt_timer;
        if timer.wait().is_err() {
            defmt::error!("Timer wait errored unexpectedly.");
        };

        let can: &mut CANBus = cx.resources.can;
        let nmt: &NMTState = cx.resources.nmt_state;
        can.write(&canopen::message_to_frame(
            ID,
            TxCANMessage::NMTHeartbeat(*nmt),
        ));
        defmt::trace!("Sending NMT heartbeat.");

        // let failsafe_counter: &mut u8 = cx.resources.failsafe_counter;
        // if *failsafe_counter == 0 {
        //     let controller: &mut Controller = cx.resources.controller;
        //     controller.set_target(0f32);
        // } else {
        //     *failsafe_counter -= 1;
        // }
    }

    #[idle(resources = [led, delay, adc])]
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

    #[task(binds = CEC_CAN, resources = [can, nmt_state, failsafe_counter, adc, controller])]
    fn process_can_message(cx: process_can_message::Context) {
        let can: &CANBus = cx.resources.can;
        let nmt_state: &mut NMTState = cx.resources.nmt_state;
        let failsafe_counter: &mut u8 = cx.resources.failsafe_counter;
        let adc: &mut ADC = cx.resources.adc;
        match can.read() {
            Ok(frame) => match canopen::message_from_frame(ID, &frame) {
                Ok(message) => match message {
                    RxCANMessage::Sync(_, _) => {
                        defmt::debug!("SYNC");
                        let tx_pdo1 = api::TxPDO1 {
                            motor_current: 0.0,
                            system_voltage: 0.0,
                        };
                        let tx_pdo2 = api::TxPDO2 {
                            die_temperature: 0.0,
                        };
                        can.write(&canopen::message_to_frame(ID, tx_pdo1.as_message()));
                        can.write(&canopen::message_to_frame(ID, tx_pdo2.as_message()));
                    }
                    RxCANMessage::PDO(pdo, data, dlc) => match pdo {
                        PDO::PDO1 => match api::RxPDO1::try_from(&data[..dlc]) {
                            Ok(pdo) => {
                                *failsafe_counter = FAILSAFE_FULL;
                                cx.resources.controller.set_target(pdo.target_speed);
                            }
                            Err(_) => {}
                        },
                        PDO::PDO2 => {}
                        PDO::PDO3 => {}
                        PDO::PDO4 => {}
                    },
                    RxCANMessage::NMT(command) => match command {
                        NMTRequestedState::Operational => {
                            *nmt_state = NMTState::Operational;
                        }
                        NMTRequestedState::Stopped => {
                            *nmt_state = NMTState::Stopped;
                        }
                        NMTRequestedState::PreOperational => {
                            *nmt_state = NMTState::PreOperational;
                        }
                        NMTRequestedState::ResetNode => {
                            *nmt_state = NMTState::BootUp;
                        }
                        NMTRequestedState::ResetCommunication => {
                            *nmt_state = NMTState::BootUp;
                        }
                    },
                    RxCANMessage::SDO => {}
                },
                Err(_) => defmt::debug!("Failed to parse frame."),
            },
            Err(_) => {
                defmt::debug!("Failed to read.");
            }
        }
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
        let current: i16 = cx.resources.adc.get_averaged_current();

        if current.abs() > 200 {
            controller.set_target(0.0);
            defmt::debug!("current: {:i16}", current);
        }
        let current_speed: f32 = cx.resources.encoder.get_speed();

        bridge.set_duty(controller.calculate_action(current_speed));
        led2.set_low();
    }

    #[task(binds = ADC_COMP, resources = [adc, led2])]
    fn adc_conversion_complete(cx: adc_conversion_complete::Context) {
        let adc: &mut ADC = cx.resources.adc;
        adc.interrupt();
    }
};
