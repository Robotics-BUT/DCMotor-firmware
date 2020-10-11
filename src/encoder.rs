use crate::board;
use stm32f0xx_hal::stm32;

pub struct Encoder {
    timer: stm32::TIM3,
    _a: board::ENC1,
    _b: board::ENC2,
    _i: board::ENCI,
    last_position: u16,
    control_loop_frequency: f32,
    last_calculated_speed: f32,
}

impl Encoder {
    pub fn new(
        timer: stm32::TIM3,
        a: board::ENC1,
        b: board::ENC2,
        i: board::ENCI,
        control_loop_frequency: f32,
    ) -> Self {
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb1enr.modify(|_, w| w.tim3en().enabled());
            rcc.apb1rstr.modify(|_, w| w.tim3rst().reset());
            rcc.apb1rstr.modify(|_, w| w.tim3rst().clear_bit());
        }

        // enable counting on both edges of encoder pulses
        timer.smcr.modify(|_, w| w.sms().encoder_mode_3());
        //
        timer.ccmr2_input().modify(|_, w| w.cc3s().ti3());
        // enable capture on channel 3, set inverted polarity (CC3NP = 0, CC3P = 1)
        timer
            .ccer
            .modify(|_, w| w.cc3e().set_bit().cc3p().set_bit());
        timer.psc.write(|w| unsafe { w.bits(0) }); // no prescaler
        timer.arr.write(|w| unsafe { w.bits(0xffff) }); // reload value
        timer.cnt.write(|w| unsafe { w.bits(0) }); // reset counter value
        timer.ccr3.write(|w| unsafe { w.bits(0) }); // reset compare value
        timer.cr1.modify(|_, w| w.cen().enabled());

        Self {
            timer,
            _a: a,
            _b: b,
            last_position: 0,
            control_loop_frequency,
            _i: i,
            last_calculated_speed: 0.0,
        }
    }

    pub fn get_position(&self) -> u16 {
        self.timer.cnt.read().cnt().bits()
    }

    pub fn calculate_speed(&mut self) -> f32 {
        let position = self.get_position();

        let diff = position - self.last_position;
        self.last_position = position;

        self.last_calculated_speed = ((diff as i16) as f32) / 65535.0f32
            * core::f32::consts::PI
            * (self.control_loop_frequency as f32);

        self.last_calculated_speed
    }

    pub fn get_speed(&self) -> f32 {
        self.last_calculated_speed
    }
}
