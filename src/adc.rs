use stm32f0xx_hal::pac::adc::cfgr1::{ALIGN_A, RES_A};
use stm32f0xx_hal::pac::adc::smpr::SMP_A;
use stm32f0xx_hal::{prelude::*, stm32};

use core::ptr;
use heapless::consts::*;
use heapless::spsc::Queue;

const VREFCAL: *const u16 = 0x1FFF_F7BA as *const u16;
const VTEMPCAL30: *const u16 = 0x1FFF_F7B8 as *const u16;
const VTEMPCAL110: *const u16 = 0x1FFF_F7C2 as *const u16;
const VDD_CALIB: u32 = 3300;

const FILTER_LEN: usize = 20;

pub struct ADC {
    adc: stm32::ADC,
    vdda: u16,
    channel_index: u8,
    voltage_value: u16,
    current_value: u16,
    current_queue: [u32; FILTER_LEN],
    filter_index: usize,
}

impl ADC {
    pub fn new(adc: stm32::ADC) -> Self {
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb2enr.modify(|_, w| w.adcen().enabled());
            rcc.cr2.modify(|_, w| w.hsi14on().on());
            while rcc.cr2.read().hsi14rdy().is_not_ready() {}
        }

        Self::disable(&adc);
        Self::calibrate(&adc);
        Self::enable(&adc);
        let vdda = Self::read_vdda(&adc);
        Self::start_periodic_reading(&adc);
        defmt::debug!("VDDA: {:u16}", vdda);

        //
        // if adc.cr.read().aden().is_enabled() {
        //     adc.cr.modify(|_, w| w.addis().disable());
        // }
        // while adc.cr.read().aden().bit_is_set() {}
        // // adc.cfgr2.write(|w| w.ckmode().pclk_div4());
        //
        // if adc.isr.read().adrdy().is_ready() {
        //     adc.isr.modify(|_, w| w.adrdy().clear());
        // }
        // adc.cr.modify(|_, w| w.aden().enabled());
        // while adc.isr.read().adrdy().is_not_ready() {}

        Self {
            adc,
            vdda,
            channel_index: 0,
            voltage_value: 0,
            current_value: 0,
            current_queue: [0; 20],
            filter_index: 0,
        }
    }

    /// Disables the peripheral so that it can be configured
    ///
    /// Should be called first when the peripheral is enabled using RCC
    /// # Arguments
    /// * `adc` the peripheral
    fn disable(adc: &stm32::ADC) {
        defmt::trace!("Checking for enabled ADC.");
        if adc.cr.read().aden().is_enabled() {
            defmt::trace!("Disabling ADC.");
            adc.cr.modify(|_, w| w.addis().disable());
        }
        defmt::trace!("Waiting for the ADC to disable.");
        while adc.cr.read().aden().is_enabled() {}
    }

    /// Begins the integrated calibration of the ADC
    ///
    /// This function calls [disable](self::disable) before calibration.
    /// # Arguments
    /// * `adc` the peripheral
    fn calibrate(adc: &stm32::ADC) {
        Self::disable(adc);
        defmt::trace!("Disabling ADC DMA.");
        adc.cfgr1.modify(|_, w| w.dmaen().disabled());
        defmt::trace!("Starting ADC calibration.");
        adc.cr.modify(|_, w| w.adcal().start_calibration());
        defmt::trace!("Waiting for ADC calibration.");
        while adc.cr.read().adcal().is_calibrating() {}
        defmt::trace!("ADC calibration done.");
    }

    /// Enable the ADC peripheral.
    ///
    /// # Arguments
    /// * `adc` the peripheral
    fn enable(adc: &stm32::ADC) {
        defmt::trace!("Enabling the ADC.");
        adc.cr.modify(|_, w| w.aden().enabled());
        defmt::trace!("Waiting for ADC to become ready.");
        while adc.isr.read().adrdy().is_not_ready() {}
    }

    /// Reads VDDA using the internal calibrated reference.
    ///
    /// This value is then used to calibrate all read values.
    /// The peripheral needs to be enabled first by calling [enable](self::enable)
    /// # Arguments
    /// * `adc` the peripheral
    fn read_vdda(adc: &stm32::ADC) -> u16 {
        defmt::trace!("Enabling internal voltage reference.");
        adc.ccr.modify(|_, w| w.vrefen().set_bit());
        defmt::trace!("Enabling the channel for the internal voltage reference.");
        adc.chselr.write(|w| unsafe { w.chsel17().selected() });
        defmt::trace!("Setting up the sample time to the longest possible.");
        adc.smpr.write(|w| w.smp().cycles239_5());
        defmt::trace!(
            "Configuring the peripheral to take a single right-aligned 12-bit measurement."
        );
        adc.cfgr1
            .modify(|_, w| w.res().twelve_bit().align().right().cont().single());

        defmt::trace!("Starting the internal reference conversion.");
        adc.cr.modify(|_, w| w.adstart().start_conversion());
        defmt::trace!("Waiting for the internal reference conversion to finish.");
        while adc.isr.read().eoc().is_not_complete() {}

        let vrefint_read = u32::from(adc.dr.read().bits());
        let vrefint_cal = u32::from(unsafe { ptr::read(VREFCAL) });
        let res = (VDD_CALIB * vrefint_cal / vrefint_read) as u16;

        defmt::trace!("Disabling internal voltage reference.");
        adc.ccr.modify(|_, w| w.vrefen().disabled());
        defmt::trace!("Disabling the channel for the internal voltage reference.");
        adc.chselr.reset();
        res
    }

    /// Enables periodic reading of the motor current and system voltage
    ///
    /// The reading is synchronized with the H-bridge by the TIM1 acting as a master.
    /// The peripheral needs to be enabled first by calling [enable](self::enable)
    /// # Arguments
    /// * `adc` the peripheral
    fn start_periodic_reading(adc: &stm32::ADC) {
        adc.chselr
            .write(|w| w.chsel5().selected().chsel6().selected());
        adc.smpr.write(|w| w.smp().cycles13_5());
        adc.cfgr1
            .modify(|_, w| w.res().twelve_bit().align().right().cont().single());
        adc.cfgr1
            .modify(|_, w| w.exten().falling_edge().extsel().tim1_cc4());
        // adc.cfgr2.write(|w| w.ckmode().pclk_div4());
        adc.ier.write(|w| w.eoseqie().enabled().eocie().enabled());
        adc.cr.modify(|_, w| w.adstart().start_conversion());
    }

    pub fn interrupt(&mut self) {
        let adc = &self.adc;
        if adc.isr.read().eoc().bit_is_set() {
            let result = adc.dr.read().data().bits();
            match self.channel_index {
                0 => {
                    self.channel_index = 1;
                }
                _ => {
                    self.current_value = result;
                    self.current_queue[self.filter_index] = result as u32;
                    self.filter_index += 1;

                    if self.filter_index == FILTER_LEN {
                        self.filter_index = 0;
                    }
                    self.channel_index = 0;
                }
            }

            adc.isr.modify(|_, w| w.eoc().clear());
        }
        if adc.isr.read().eoseq().bit_is_set() {
            self.channel_index = 0;
            adc.isr.modify(|_, w| w.eoseq().clear());
        }
    }

    /// Returns the system voltage in millivolts.
    pub fn get_system_voltage(&self) -> u16 {
        (u32::from(self.vdda) * (self.voltage_value as u32) * 48 / (2u32).pow(12)) as u16
    }
    //
    pub fn get_motor_current(&self) -> i16 {
        self.raw_to_current(self.current_value)
        // ((u32::from(self.vdda) * (self.current_value as u32) * 4 / (2u32).pow(12)) as i16)
    }
    //
    fn raw_to_current(&self, raw: u16) -> i16 {
        let volt = 2500i16 - ((u32::from(self.vdda) * (raw as u32) * 4 / (2u32).pow(12)) as i16);
        ((volt as i32) * 1000 / 180) as i16
    }

    pub fn get_averaged_current(&self) -> i16 {
        let number_of_samples = self.current_queue.len();
        let sum = self.current_queue.iter().fold(0u32, |acc, x| acc + *x);

        let sum = (sum / number_of_samples as u32);
        self.raw_to_current(sum as u16)
    }
}
