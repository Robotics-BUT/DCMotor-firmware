#![no_std]
#![no_main]

use dcdriver as _; // global logger + panicking-behavior + memory layout

use stm32f0xx_hal::delay::Delay;
use stm32f0xx_hal::prelude::*;
use stm32f0xx_hal::timers::{Event, Timer};

use core::convert::TryFrom;
use core::ops::DerefMut;
use core::ptr;
use dcdriver::adc::{ADC, BUFFER, CHANNEL_COUNT};
// use dcdriver::adc::BUFFER;
use dcdriver::board::*;
use dcdriver::bridge::Bridge;
use dcdriver::can::CANBus;
use dcdriver::canopen::*;
use dcdriver::controller::Controller;
use dcdriver::encoder::Encoder;
use dcdriver::{api, can, canopen};
use stm32f0xx_hal::pac::DMA1;
use stm32f0xx_hal::stm32;

const CONTROL_LOOP_FREQUENCY_HZ: u32 = 1000;
const ID: u8 = 9;
const FAILSAFE_FULL: u8 = 2;
const MAX_ALLOWED_ABS_CURRENT: i16 = 1000;
const SPEED_CONTROLLER_P: f32 = 0.3;
const SPEED_CONTROLLER_S: f32 = 0.8;
const OVERCURRENT_TIMER_THRESHOLD: u16 = 50;


fn truncate_action(requested_action: f32, last_action: f32, current: i16) -> f32 {
    if current.abs() > MAX_ALLOWED_ABS_CURRENT {
        let overcurrent_ratio = (current.abs() as f32) / (MAX_ALLOWED_ABS_CURRENT as f32);
        let truncated_action = last_action / overcurrent_ratio;
        defmt::debug!("{:?} {:?} {:?} {:?} {:?}", current.abs(), MAX_ALLOWED_ABS_CURRENT, overcurrent_ratio, truncated_action, requested_action);
        truncated_action
    } else {
        let p: f32 = 0.01;
        last_action * (1.0-p) + requested_action * p
    }
}

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
        speed_controller: Controller,
        nmt_timer: Timer<NMTTimer>,
        #[init(0)]
        last_position: u16,
        #[init(canopen::NMTState::BootUp)]
        nmt_state: canopen::NMTState,
        #[init(0)]
        failsafe_counter: u8, // when counter reaches 0, motors must stop,
        dma: DMA1,
        #[init(0)]
        overcurrent_duration_counter: u16,
        speed_target: f32,
        action: f32,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let core: cortex_m::Peripherals = cx.core;
        let mut device: stm32f0xx_hal::stm32::Peripherals = cx.device;

        let raw_rcc = device.RCC;
        raw_rcc.ahbenr.modify(|_, w| w.dma1en().enabled());
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

        let (pwm1p, pwm2p, _input_voltage_pin, _current_pin) = cortex_m::interrupt::free(|cs| {
            (
                gpioa.pa8.into_alternate_af2(cs),
                gpioa.pa9.into_alternate_af2(cs),
                gpioa.pa5.into_analog(cs),
                gpioa.pa6.into_analog(cs),
            )
        });

        let (can_rx, can_tx, enc_a, enc_b, enc_i, led, led2, pwm1n, pwm2n) =
            cortex_m::interrupt::free(|cs| {
                (
                    gpiob.pb8.into_alternate_af4(cs),
                    gpiob.pb9.into_alternate_af4(cs),
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
        let mut nmt_timer = Timer::tim6(device.TIM6, 2.hz(), &mut rcc);
        nmt_timer.listen(Event::TimeOut);

        // set up motor control pwm
        let tim1 = device.TIM1;
        let bridge = Bridge::new(tim1, pwm1p, pwm1n, pwm2p, pwm2n);

        let dt = 1.0f32 / (CONTROL_LOOP_FREQUENCY_HZ as f32);
        let mut speed_controller = Controller::new(SPEED_CONTROLLER_P, SPEED_CONTROLLER_S, dt);

        let adc = device.ADC;
        let adc = ADC::new(adc);

        let dma = device.DMA1;
        // channel 1, 2 can be used for ADC
        let channel = &dma.ch1;
        channel.cr.reset();
        channel.cr.write(|w| {
            w.mem2mem()
                .disabled()
                .pl()
                .very_high()
                .msize()
                .bits16()
                .psize()
                .bits16()
                .minc()
                .enabled()
                .pinc()
                .disabled()
                .dir()
                .from_peripheral()
                .tcie()
                .enabled()
                .circ()
                .enabled()
        });

        // number of data - current and voltage channels
        channel.ndtr.write(|w| w.ndt().bits(CHANNEL_COUNT as u16));

        let dr_addr = unsafe { &(*stm32::ADC::ptr()).dr as *const _ as u32 };
        channel.par.write(|w| w.pa().bits(dr_addr));

        let buffer_address = unsafe { BUFFER.as_ptr() as u32 };
        channel.mar.write(|w| w.ma().bits(buffer_address));

        // enable channel
        channel.cr.modify(|_, w| w.en().enabled());

        let speed_target = 2.0;
        let action = 0.0;
        speed_controller.set_target(speed_target);

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
            speed_controller,
            dma,
            speed_target,
            action,
        }
    }

    #[task(binds = DMA1_CH1, resources = [dma, adc])]
    fn dma_handler(cx: dma_handler::Context) {
        let dma: &mut DMA1 = cx.resources.dma;
        let adc: &mut ADC = cx.resources.adc;
        if dma.isr.read().tcif1().bit_is_set() {
            dma.ifcr.write(|w| w.ctcif1().set_bit());
            adc.dma_interrupt();
        }
    }

    #[task(binds = TIM6_DAC, resources = [nmt_timer, can, nmt_state, failsafe_counter, speed_controller])]
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

        let failsafe_counter: &mut u8 = cx.resources.failsafe_counter;
        if *failsafe_counter == 0 {
            let speed_controller: &mut Controller = cx.resources.speed_controller;
            speed_controller.set_target(0.0);
        } else {
            *failsafe_counter -= 1;
        }
    }

    #[idle(resources = [led, delay, adc, speed_controller])]
    fn main(cx: main::Context) -> ! {
        loop {
            cx.resources
                .led
                .set_high()
                .expect("Failed to set LED high.");
            cx.resources.delay.delay_ms(500u32);
            cx.resources.led.set_low().expect("Failed to set LED low.");
            cx.resources.delay.delay_ms(500u32);
        }
    }

    #[task(binds = CEC_CAN, resources = [can, nmt_state, failsafe_counter, adc, speed_controller, encoder, speed_target])]
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
                            current_speed: cx.resources.encoder.get_speed(),
                            motor_current: adc.get_averaged_current() as f32,
                        };
                        let tx_pdo2 = api::TxPDO2 {
                            system_voltage: adc.get_system_voltage() as f32 / 1000.0f32,
                            die_temperature: adc.get_die_temperature() as f32 / 100.0f32,
                        };
                        can.write(&canopen::message_to_frame(ID, tx_pdo1.as_message()));
                        can.write(&canopen::message_to_frame(ID, tx_pdo2.as_message()));
                    }
                    RxCANMessage::PDO(pdo, data, dlc) => match pdo {
                        PDO::PDO1 => match api::RxPDO1::try_from(&data[..dlc]) {
                            Ok(pdo) => {
                                *failsafe_counter = FAILSAFE_FULL;
                                cx.resources.speed_controller.set_target(pdo.target_speed);
                                *cx.resources.speed_target = pdo.target_speed;
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



    #[task(binds = TIM2, resources = [control_timer, encoder, last_position, bridge, adc, led2, speed_controller, overcurrent_duration_counter, speed_target, action])]
    fn control_loop_tick(cx: control_loop_tick::Context) {
        let control_timer: &mut Timer<ControlTimer> = cx.resources.control_timer;
        if control_timer.wait().is_err() {
            defmt::error!("Timer wait errored unexpectedly.");
        };
        let led2: &mut LED2 = cx.resources.led2;
        led2.set_high();

        let speed_controller: &mut Controller = cx.resources.speed_controller;
        let bridge: &mut Bridge = cx.resources.bridge;
        let current: i16 = cx.resources.adc.get_averaged_current();
        let voltage: u16 = cx.resources.adc.get_system_voltage();
        let temp: i16 = cx.resources.adc.get_die_temperature();
        let target_speed: f32 = *cx.resources.speed_target;


        speed_controller.set_target(target_speed);
        let current_speed: f32 = cx.resources.encoder.calculate_speed();
        let action = speed_controller.calculate_action(current_speed);

        let truncated_action = truncate_action(action, *cx.resources.action, current);
        bridge.set_duty(truncated_action);
        *cx.resources.action = truncated_action;

        led2.set_low();
    }

    #[task(binds = ADC_COMP, resources = [adc, led2])]
    fn adc_conversion_complete(cx: adc_conversion_complete::Context) {
        let adc: &mut ADC = cx.resources.adc;
        defmt::debug!("adc: irq");
        // adc.interrupt();
    }
};
