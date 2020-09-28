use crate::board;
use stm32f0xx_hal::stm32;

pub struct CANBus {
    can: stm32::CAN,
    _rx: board::CanRx,
    _tx: board::CanTx,
}

impl CANBus {
    pub fn new(can: stm32::CAN, rx: board::CanRx, tx: board::CanTx) -> Self {
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb1enr.modify(|_, w| w.canen().enabled());
            rcc.apb1rstr.modify(|_, w| w.canrst().reset());
            rcc.apb1rstr.modify(|_, w| w.canrst().clear_bit());
        }

        defmt::trace!(
            "bus is in sleep state, we need to request transition to the initialization state"
        );
        can.mcr.write(|w| w.sleep().clear_bit());
        can.mcr.modify(|_, w| w.inrq().set_bit());
        defmt::trace!("next we wait for the bus to move to the initialization state");
        while !can.msr.read().inak().bit() {}
        defmt::trace!("bus is now in initialization state");
        defmt::trace!("initializing bus parameters");
        can.mcr.modify(|_, w| {
            w.ttcm()
                .clear_bit() // no time triggered communication
                .abom()
                .set_bit() // bus automatically recovers itself after error state
                .awum()
                .set_bit() // bus is automatically waken up on message RX
                .nart()
                .clear_bit() // automatic message retransmission enabled
                .rflm()
                .clear_bit() // new RX message overwrite unread older ones
                .txfp()
                .clear_bit() // TX message priority driven by the message identifier
                .sleep()
                .clear_bit() // do not sleep
        });
        defmt::trace!("bus is now initializing timing");
        // calculated using http://www.bittiming.can-wiki.info/ for STMicroelectronics bxCAN 48 MHz clock, 87.6% sample point, SJW = 1, bitrate 250 kHz
        const TIME_SEGMENT1: u8 = 13;
        const TIME_SEGMENT2: u8 = 2;
        const RESYNC_WIDTH: u8 = 1;
        const PRESCALER: u16 = 12;
        can.btr.modify(|_, w| unsafe {
            w.silm()
                .clear_bit() // disable silent mode
                .lbkm()
                .clear_bit() // disable loopback mode
                .sjw()
                .bits(RESYNC_WIDTH - 1)
                .ts2()
                .bits(TIME_SEGMENT2 - 1)
                .ts1()
                .bits(TIME_SEGMENT1 - 1)
                .brp()
                .bits(PRESCALER - 1)
        });
        defmt::trace!("bus is now initializing interrupts");

        defmt::trace!("bus is now leaving initialization");
        can.mcr.modify(|_, w| w.inrq().clear_bit());
        while !can.msr.read().inak().bit() {}

        // initialize message pending interrupts
        can.ier
            .modify(|_, w| w.fmpie0().set_bit().fmpie1().set_bit());

        defmt::trace!("bus is now initializing filters");
        can.fmr.modify(|_, w| w.finit().set_bit()); // filter init enabled
        can.fa1r.write(|w| w.fact0().clear_bit()); // filter is inactive

        can.fm1r.write(|w| w.fbm0().clear_bit()); // identifier mask mode for fbm0
        can.fs1r.write(|w| w.fsc0().set_bit()); // 32 bit scale configuration

        const FILTER0_ID: u16 = 0x0;
        const FILTER0_MASK: u16 = 0x00;
        const FILTER1_ID: u16 = 0x00;
        const FILTER1_MASK: u16 = 0x00;
        can.fb[0].fr1.write(|w| unsafe { w.bits(0) });
        can.fb[0].fr2.write(|w| unsafe { w.bits(0) });

        can.fa1r.write(|w| w.fact0().set_bit()); // filter is active
        can.fmr.modify(|_, w| w.finit().clear_bit()); // filter init disabled
        defmt::trace!("filters initialized");

        let bits = can.mcr.read().bits();
        defmt::trace!("fmp0: {0:0..31}, MAP: {0:8..9}", bits);

        Self {
            can,
            _rx: rx,
            _tx: tx,
        }
    }

    pub fn interrupt(&mut self) {
        // let bits = can.rfr[0].read().bits();
        // defmt::debug!("fmp0: {0:0..1}, MAP: {0:8..9}", bits);
    }
}
