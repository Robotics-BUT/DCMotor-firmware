use crate::board;
use stm32f0xx_hal::pac::TIM1;
use stm32f0xx_hal::time::Hertz;
use stm32f0xx_hal::{prelude::*, stm32};

const MAX_DUTY_COEF: f32 = 0.95;

pub struct Bridge {
    timer: TIM1,
    _p1p: board::PWM1P,
    _p1n: board::PWM1N,
    _p2p: board::PWM2P,
    _p2n: board::PWM2N,
}

impl Bridge {
    pub fn new(
        timer: TIM1,
        p1p: board::PWM1P,
        p1n: board::PWM1N,
        p2p: board::PWM2P,
        p2n: board::PWM2N,
    ) -> Self {
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb2enr.modify(|_, w| w.tim1en().enabled());
            rcc.apb2rstr.modify(|_, w| w.tim1rst().reset());
            rcc.apb2rstr.modify(|_, w| w.tim1rst().clear_bit());
        }

        let bridge = Self {
            timer,
            _p1p: p1p,
            _p1n: p1n,
            _p2p: p2p,
            _p2n: p2n,
        };

        bridge.timer.cr1.write(|w| {
            w.ckd()
                .div1() // no prescaler
                .arpe()
                .enabled() // enable preloading of automatic reload value
                .cms()
                .center_aligned3() // interrupt flags of channels are set when both counting up and down
                .opm()
                .disabled() // one pulse mode disabled
                .urs()
                .counter_only() // only overflow/underflow generates interrupt request
                .cen()
                .disabled() // counter is disabled while initializing
        });

        // by configuring ois** bits like this, then when idling, the motor will be shorted to the ground via the low side of the drivers
        bridge.timer.cr2.write(|w| {
            w.ccpc()
                .set_bit() // enable compare registers preloading
                .ccus()
                .clear_bit() // update compare registers by COM event only
                .ois1()
                .clear_bit() // output 1 P idle state is LOW
                .ois1n()
                .set_bit() // output 1 N idle state is HIGH
                .ois2()
                .clear_bit() // output 2 P idle state is LOW
                .ois2n()
                .set_bit() // output 2 N idle state is HIGH
                .mms()
                .compare_pulse() // enable TRGO event generation - triggers ADC
        });

        bridge.timer.ccmr1_output().modify(|_, w| {
            w.cc1s()
                .output() // configure channel 1 as output
                .oc1fe()
                .set_bit() // enable fast output
                .oc1pe()
                .set_bit() // enable preloading
                .oc1m()
                .pwm_mode1() // refer to the DS
                .cc2s() // channel 2 as output
                .output()
                .oc2fe() // enable fast output
                .set_bit()
                .oc2pe() // enable preloading
                .set_bit()
                .oc2m()
                .pwm_mode1() // refer to the DS
        });
        bridge.timer.ccmr2_output().modify(
            |_, w| {
                w.cc4s() // channel 2 as output
                    .output()
                    .oc4fe() // enable fast output
                    .set_bit()
                    .oc4pe() // enable preloading
                    .set_bit()
                    .oc4m()
                    .toggle()
            }, // refer to the DS
        );

        bridge.timer.ccer.write(|w| {
            w.cc1e()
                .set_bit() // channel 1 is enabled
                .cc1p()
                .clear_bit() // channel 1 polarity is active HIGH
                .cc1ne()
                .set_bit() // enable complementary output for channel 1
                .cc1np()
                .clear_bit() // channel 1 complementary polarity is active HIGH
                .cc2e()
                .set_bit() // channel 2 is enabled
                .cc2p()
                .clear_bit() // channel 2 polarity is active HIGH
                .cc2ne()
                .set_bit() // enable complementary output for channel 2
                .cc2np()
                .clear_bit() // channel 2 complementary polarity is active HIGH
                .cc4e()
                .set_bit()
        });

        let arr = Self::get_arr();
        bridge.timer.arr.write(|w| w.arr().bits(arr));
        bridge.timer.rcr.write(|w| unsafe { w.rep().bits(2u8) });
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

        bridge.generate_update();

        bridge.timer.bdtr.write(|w| unsafe {
            w.moe()
                .enabled() // main output enable - this is a master switch
                .ossr()
                .set_bit() // when MOE = 1, use inactive levels on outputs when inactive
                .ossi()
                .set_bit() // when MOE = 0, force outputs with idle level
                .dtg()
                .bits(raw_deadtime as u8) // set deadtime
        });

        // enable timer
        bridge.timer.cr1.modify(|_, w| w.cen().enabled());

        // set compare register values
        bridge.timer.ccr1.write(|w| w.ccr().bits(0));
        bridge.timer.ccr2.write(|w| w.ccr().bits(0));
        bridge.timer.ccr4.write(|w| w.ccr().bits(0));

        // write compare registers
        bridge.generate_com_event();

        bridge
    }

    fn get_arr() -> u16 {
        let repetitions = 2;
        let target_frequency: Hertz = 20.khz().into();
        let cpu_frequency: Hertz = 48.mhz().into();
        (cpu_frequency.0 / (repetitions * target_frequency.0)) as u16
    }

    /// Sets duty cycle of the bridge and direction of movement
    /// # Arguments
    ///
    /// * `duty` a number within the range <-1;1> representing duty cycle of the bridge, its sign controls direction
    pub fn set_duty(&self, duty: f32) {
        let inverse = duty < 0f32;
        let duty = fmaxf(-MAX_DUTY_COEF, fminf(duty, MAX_DUTY_COEF));
        // abs is not implemented for no_std
        let duty = if duty < 0f32 { -duty } else { duty };
        // defmt::debug!("{:f32}", duty);
        let duty = (duty * ((Self::get_arr() - 1) as f32)) as u16;
        if inverse {
            self.timer.ccer.modify(|_, w| {
                w.cc1e()
                    .clear_bit()
                    .cc1ne()
                    .set_bit()
                    .cc2e()
                    .set_bit()
                    .cc2ne()
                    .clear_bit()
            });
        } else {
            self.timer.ccer.modify(|_, w| {
                w.cc1e()
                    .set_bit()
                    .cc1ne()
                    .clear_bit()
                    .cc2e()
                    .clear_bit()
                    .cc2ne()
                    .set_bit()
            });
        }
        self.timer.ccr1.write(|w| w.ccr().bits(duty));
        self.timer.ccr2.write(|w| w.ccr().bits(duty));
        self.timer.ccr4.write(|w| w.ccr().bits(duty));
        self.generate_com_event();
    }

    fn generate_update(&self) {
        self.timer.cr1.modify(|_, w| w.urs().set_bit());
        self.timer.egr.write(|w| w.ug().set_bit());
        self.timer.cr1.modify(|_, w| w.urs().clear_bit());
    }

    /// Updates channels and their complementary outputs
    fn generate_com_event(&self) {
        self.timer.cr1.modify(|_, w| w.urs().set_bit());
        self.timer.egr.write(|w| w.comg().set_bit()); // invoke change to compare register values
        self.timer.cr1.modify(|_, w| w.urs().clear_bit());
    }
}
#[no_mangle]
pub fn fminf(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

#[no_mangle]
pub fn fmaxf(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}
