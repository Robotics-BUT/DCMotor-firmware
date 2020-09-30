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
use stm32f0xx_hal::pac::Interrupt::{ADC_COMP, TIM1_BRK_UP_TRG_COM};
use stm32f0xx_hal::time::Hertz;

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
        tim1: stm32::TIM1,
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

        tim1.cr1.write(|w| {
            w.ckd()
                .div1()
                .arpe()
                .enabled()
                .cms()
                .center_aligned3()
                .opm()
                .disabled()
                .urs()
                .counter_only()
                .cen()
                .disabled()
        });

        tim1.cr2.write(|w| {
            w.ccpc()
                .set_bit() // enable compare regs preloading
                .ccus()
                .clear_bit() // update compare regs by COM event only
                .ois1()
                .clear_bit()
                .ois1n()
                .set_bit()
        });

        tim1.ccmr1_output().modify(|_, w| {
            w.cc1s()
                .output()
                .oc1fe()
                .set_bit()
                .oc1pe()
                .set_bit()
                .oc1m()
                .pwm_mode1()
        });
        tim1.ccer.write(|w| {
            w.cc1p()
                .clear_bit()
                .cc1ne()
                .set_bit()
                .cc1np()
                .clear_bit()
                .cc1e()
                .set_bit()
        });

        let repetitions = 2;
        let target_frequency: Hertz = 20.khz().into();
        let cpu_frequency: Hertz = 48.mhz().into();
        let arr = cpu_frequency.0 / (repetitions * target_frequency.0);
        tim1.arr.write(|w| w.arr().bits(arr as u16));
        tim1.rcr
            .write(|w| unsafe { w.rep().bits((repetitions - 1) as u8) });

        let deadtime = 61u16;
        let raw_deadtime = if deadtime < 128 {
            deadtime
        } else if deadtime < 256 {
            ((deadtime - 128) / 2) | 0x80
        } else if deadtime < 512 {
            ((deadtime - 256) / 8) | 0xc0
        } else if deadtime < 1024 {
            ((deadtime - 512) / 16) | 0xe0
        } else {
            0xff
        };

        tim1.cr1.modify(|_, w| w.urs().set_bit());
        tim1.egr.write(|w| w.ug().set_bit());
        tim1.cr1.modify(|_, w| w.urs().clear_bit());

        tim1.bdtr.write(|w| unsafe {
            w.moe()
                .enabled()
                .ossr()
                .set_bit()
                .ossi()
                .set_bit()
                .dtg()
                .bits(raw_deadtime as u8)
        });
        tim1.cr1.modify(|_, w| w.cen().enabled());

        tim1.ccr1.write(|w| w.ccr().bits(500));

        tim1.cr1.modify(|_, w| w.urs().set_bit());
        tim1.egr.write(|w| w.comg().set_bit()); // invoke change to compare register values
        tim1.cr1.modify(|_, w| w.urs().clear_bit());

        // tim1.cr1.modify(|_, w| {
        //     w.ckd()
        //         .div1() // clock deadtime the same as timer
        //         .arpe()
        //         .enabled() // buffer ARR
        //         .cms()
        //         .center_aligned2() // center aligned mode 2 - interrupt flags set only when counting up
        //         .urs()
        //         .counter_only() // only counter events overflow, underflow generate interrupt
        // });
        //
        // let repetitions = 2;
        // let target_frequency: Hertz = 20.khz().into();
        // let cpu_frequency: Hertz = 48.mhz().into();
        // let arr = cpu_frequency.0 / (repetitions * target_frequency.0);
        //
        // let deadtime = 31u16;
        // let raw_deadtime = if deadtime < 128 {
        //     deadtime
        // } else if deadtime < 256 {
        //     ((deadtime - 128) / 2) | 0x80
        // } else if deadtime < 512 {
        //     ((deadtime - 256) / 8) | 0xc0
        // } else if deadtime < 1024 {
        //     ((deadtime - 512) / 16) | 0xe0
        // } else {
        //     0xff
        // };
        //
        // tim1.psc.write(|w| w.psc().bits(0));
        // tim1.rcr
        //     .write(|w| unsafe { w.rep().bits((repetitions - 1) as u8) });
        // tim1.arr.write(|w| w.arr().bits(arr as u16));
        //
        // // break and deadtime
        // tim1.bdtr.modify(|_, w| unsafe {
        //     w.moe()
        //         .disabled_idle() // outputs are disabled or forced to idle state
        //         .aoe()
        //         .clear_bit() // outputs (MOE) can be enabled only in the software
        //         .bkp()
        //         .set_bit() // brake input is active high
        //         .bke()
        //         .clear_bit() // break is disabled
        //         .ossr()
        //         .idle_level() // when inactive go to idle level of outputs in run mode
        //         .ossi()
        //         .idle_level() // when inactive go to idle level of outputs in idle mode
        //         .lock()
        //         .bits(0) // disable brake write protection
        //         .dtg()
        //         .bits(raw_deadtime as u8)
        // });
        //
        // // disable capture/compare outputs
        // tim1.ccer.modify(|_, w| {
        //     w.cc1e()
        //         .clear_bit()
        //         .cc1ne()
        //         .clear_bit()
        //         .cc2e()
        //         .clear_bit()
        //         .cc2ne()
        //         .clear_bit()
        // });
        //
        // // enable preload, set output compare modes to pwm1
        // tim1.ccmr1_output().modify(|_, w| {
        //     w.oc2pe()
        //         .set_bit()
        //         .oc1pe()
        //         .set_bit()
        //         .oc1m()
        //         .pwm_mode1()
        //         .oc2m()
        //         .pwm_mode1()
        // });
        //
        // // set driver polarity
        // tim1.ccer
        //     .modify(|_, w| w.cc1p().clear_bit().cc2p().clear_bit());
        //
        // // set driver idle - when idle the motor is shorted using the low side.
        // tim1.cr2.modify(|_, w| w.ois1().set_bit().ois2().set_bit());
        //
        // // set trigger for ADC in master mode
        // tim1.cr2.modify(|_, w| w.mms().compare_oc4());
        // tim1.ccmr2_output().modify(|_, w| w.oc4m().pwm_mode1());
        // tim1.ccer.modify(|_, w| w.cc4p().clear_bit());
        // tim1.cr2.modify(|_, w| w.ois4().set_bit());
        //
        // // set up values in compare registers
        // tim1.cr1.modify(|_, w| w.udis().disabled());
        // tim1.ccr1.write(|w| unsafe { w.ccr().bits(0) });
        // tim1.ccr2.write(|w| unsafe { w.ccr().bits(0) });
        // tim1.ccr3.write(|w| unsafe { w.ccr().bits(0) });
        // tim1.ccr4.write(|w| unsafe { w.ccr().bits(211) }); // T.F. is this number?
        // tim1.cr1.modify(|_, w| w.udis().enabled());
        //
        // // preload register values for complementary channels
        // tim1.cr2.modify(|_, w| w.ccpc().set_bit());
        // // force update
        // tim1.egr.write(|w| w.ug().update());
        // // enable brake main output
        // tim1.bdtr.modify(|_, w| w.moe().enabled());
        // // enable timer
        // tim1.cr1.modify(|_, w| w.cen().enabled());
        //
        // // enable update interrupt
        // // tim1.dier.modify(|_, w| w.uie().enabled());
        //
        // // force commute
        // tim1.egr.write(|w| w.comg().set_bit());
        //
        // let speed: u16 = 2000;
        // tim1.cr1.modify(|_, w| w.udis().disabled());
        // tim1.ccr1.write(|w| unsafe { w.ccr().bits(speed) });
        // tim1.ccr2.write(|w| unsafe { w.ccr().bits(speed) });
        // tim1.ccr4.write(|w| unsafe { w.ccr().bits(211) }); // T.F. is this number?
        // tim1.cr1.modify(|_, w| w.udis().enabled());
        // tim1.egr.write(|w| w.comg().set_bit());

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
            tim1,
        }
    }

    #[task(binds = TIM1_BRK_UP_TRG_COM, resources = [tim1])]
    fn tim1_irs(cx: tim1_irs::Context) {
        let tim1: &mut stm32::TIM1 = cx.resources.tim1;
        tim1.sr.modify(|_, w| w.uif().clear());
        defmt::debug!("picifuk")
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
