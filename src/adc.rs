use stm32f0xx_hal::pac::adc::cfgr1::{ALIGN_A, RES_A};
use stm32f0xx_hal::pac::adc::smpr::SMP_A;
use stm32f0xx_hal::{prelude::*, stm32};

const VREFCAL: *const u16 = 0x1FFF_F7BA as *const u16;
const VTEMPCAL30: *const u16 = 0x1FFF_F7B8 as *const u16;
const VTEMPCAL110: *const u16 = 0x1FFF_F7C2 as *const u16;
const VDD_CALIB: u16 = 3300;

// TODO maybe enable triggering from TIM1
pub struct ADC {
    adc: stm32::ADC,
    channel_index: u8,
    voltage_value: u16,
    current_value: u16,
}

impl ADC {
    pub fn new(adc: stm32::ADC) -> Self {
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb2enr.modify(|_, w| w.adcen().enabled());
            rcc.cr2.modify(|_, w| w.hsi14on().on());
            while rcc.cr2.read().hsi14rdy().is_not_ready() {}
        }

        if adc.cr.read().aden().is_enabled() {
            adc.cr.modify(|_, w| w.addis().disable());
        }
        while adc.cr.read().aden().bit_is_set() {}
        adc.cfgr2.write(|w| w.ckmode().pclk_div4());

        ADC::calibrate(&adc);

        if adc.isr.read().adrdy().is_ready() {
            adc.isr.modify(|_, w| w.adrdy().clear());
        }
        adc.cr.modify(|_, w| w.aden().enabled());
        while adc.isr.read().adrdy().is_not_ready() {}

        Self {
            adc,
            channel_index: 0,
            voltage_value: 0,
            current_value: 0,
        }
    }

    fn calibrate(adc: &stm32::ADC) {
        // calibrate
        if adc.cr.read().aden().is_enabled() {
            adc.cr.modify(|_, w| w.addis().disable());
        }
        while adc.cr.read().aden().bit_is_set() {}
        adc.cfgr1.modify(|_, w| w.dmaen().disabled());
        adc.cr.modify(|_, w| w.adcal().start_calibration());

        defmt::trace!("Waiting for ADC calibration.");
        while adc.cr.read().adcal().is_calibrating() {}
        defmt::trace!("ADC calibration done.");
    }

    pub fn start_periodic_reading(&self) {
        let adc = &self.adc;
        adc.chselr.write(|w| {
            w.chsel5()
                .selected()
                .chsel6()
                .selected()
                .chsel16()
                .selected()
                .chsel17()
                .selected()
        });

        adc.ier
            .modify(|_, w| w.eocie().enabled().eoseqie().enabled());

        // enable temperature sensor
        adc.ccr.modify(|_, w| w.tsen().enabled().vrefen().enabled());

        adc.smpr.write(|w| w.smp().variant(SMP_A::CYCLES239_5));

        adc.cfgr1.modify(|_, w| {
            w.res()
                .variant(RES_A::TWELVEBIT)
                .align()
                .variant(ALIGN_A::RIGHT)
                .cont()
                .continuous()
                .ovrmod()
                .overwritten()
        });

        adc.cr.modify(|_, w| w.adstart().start_conversion());
    }

    // pub fn read(&self, channel: u8) -> f32 {
    //     let adc = &self.adc;
    //     // FIXME select channel
    //     adc.chselr.write(|w| unsafe { w.bits(1u32 << channel) });
    //
    //     adc.smpr.write(|w| w.smp().variant(SMP_A::CYCLES41_5));
    //
    //     adc.cfgr1.modify(|_, w| {
    //         w.res()
    //             .variant(RES_A::TWELVEBIT)
    //             .align()
    //             .variant(ALIGN_A::RIGHT)
    //     });
    //
    //     adc.cr.modify(|_, w| w.adstart().start_conversion());
    //     while adc.isr.read().eoc().is_not_complete() {}
    //
    //     let res = adc.dr.read().bits() as u32;
    //
    //     adc.cr.modify(|_, w| w.adstp().stop_conversion());
    //     while adc.cr.read().adstp().is_stopping() {}
    //     defmt::trace!("adc: {:u32}", res);
    //     // FIXME return real value
    //     0f32
    // }

    pub fn interrupt(&mut self) {
        let adc = &self.adc;
        if adc.isr.read().eoc().bit_is_set() {
            let result = adc.dr.read().data().bits();
            // defmt::debug!("adc: {:u8} {:u16}", self.channel_index, result);
            match self.channel_index {
                0 => self.voltage_value = result,
                1 => self.current_value = result,
                _ => {}
            }
            self.channel_index += 1;
        }
        if adc.isr.read().eoseq().bit_is_set() {
            self.channel_index = 0;
            adc.isr.modify(|_, w| w.eoseq().clear());
        }
        adc.isr.modify(|_, w| w.eoc().clear());
    }

    // FIXME extract constants
    pub fn get_system_voltage(&self) -> f32 {
        (self.voltage_value * 5) as f32 / 100.0
    }

    pub fn get_motor_current(&self) -> f32 {
        ((self.current_value as i16) - 635) as f32 / (185.0 / (2.0 / 3.0))
    }

    pub fn get_die_temperature(&self) -> f32 {
        defmt::error!("Getting temperature is not implemented.");
        0f32
    }

    // TODO figure out howto read VDDA from VREF
    // pub fn get_temperature(&self) -> i16 {
    //     let vtemp = 0u16; // FIXME implement
    //     let vtemp30_cal = i32::from(unsafe { core::ptr::read(VTEMPCAL30) }) * 100;
    //     let vtemp110_cal = i32::from(unsafe { core::ptr::read(VTEMPCAL110) }) * 100;
    //
    //     let mut temperature = i32::from(vtemp) * 100;
    //     temperature = (temperature * (i32::from(vdda) / i32::from(VDD_CALIB))) - vtemp30_cal;
    //     temperature *= (110 - 30) * 100;
    //     temperature /= vtemp110_cal - vtemp30_cal;
    //     temperature += 3000;
    //     temperature as i16
    // }
}
