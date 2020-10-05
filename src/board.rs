use stm32f0xx_hal::gpio::gpioa::*;
use stm32f0xx_hal::gpio::gpiob::*;
use stm32f0xx_hal::gpio::{Alternate, Output, PushPull, AF1, AF2, AF4};
use stm32f0xx_hal::stm32;

pub type PWM1P = PA8<Alternate<AF2>>;
pub type PWM1N = PB13<Alternate<AF2>>;
pub type PWM2P = PA9<Alternate<AF2>>;
pub type PWM2N = PB14<Alternate<AF2>>;
pub type ENC1 = PB4<Alternate<AF1>>;
pub type ENC2 = PB5<Alternate<AF1>>;
pub type ENCI = PB0<Alternate<AF1>>; // is PB3 AF2 on REVA boards
pub type LED = PB2<Output<PushPull>>;
pub type LED2 = PB10<Output<PushPull>>;

pub type CanRx = PA11<Alternate<AF4>>;
pub type CanTx = PA12<Alternate<AF4>>;

pub type ControlTimer = stm32::TIM2;

const ADC_CURRENT_CHANNEL: u8 = 5;
const ADC_VOLTAGE_CHANNEL: u8 = 6;
