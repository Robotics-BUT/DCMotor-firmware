use stm32f0xx_hal::pac::adc::cfgr1::{ALIGN_A, RES_A};
use stm32f0xx_hal::pac::adc::smpr::SMP_A;
use stm32f0xx_hal::{prelude::*, stm32};

pub struct ADC {
    adc: stm32::ADC,
}

impl ADC {
    pub fn new(adc: stm32::ADC) -> Self {
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb2enr.modify(|_, w| w.adcen().enabled());
            rcc.cr2.modify(|_, w| w.hsi14on().on());
            while rcc.cr2.read().hsi14rdy().is_not_ready() {}
        }
        ADC::calibrate(&adc);

        if adc.isr.read().adrdy().is_ready() {
            adc.isr.modify(|_, w| w.adrdy().clear());
        }
        adc.cr.modify(|_, w| w.aden().enabled());
        while adc.isr.read().adrdy().is_not_ready() {}

        Self { adc }
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

    pub fn read(&self, channel: u8) -> f32 {
        let adc = &self.adc;
        // FIXME select channel
        adc.chselr.write(|w| unsafe { w.chsel13().selected() });

        adc.smpr.write(|w| w.smp().variant(SMP_A::CYCLES41_5));

        adc.cfgr1.modify(|_, w| {
            w.res()
                .variant(RES_A::TWELVEBIT)
                .align()
                .variant(ALIGN_A::RIGHT)
        });

        adc.cr.modify(|_, w| w.adstart().start_conversion());
        while adc.isr.read().eoc().is_not_complete() {}

        let res = adc.dr.read().bits() as u32;

        adc.cr.modify(|_, w| w.adstp().stop_conversion());
        while adc.cr.read().adstp().is_stopping() {}
        defmt::trace!("adc: {:u32}", res);
        // FIXME return real value
        0f32
    }
}
