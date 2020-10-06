use crate::canopen::{TxCANMessage, PDO};
use byteorder::{ByteOrder, LittleEndian};
use core::convert::TryFrom;

pub struct RxPDO1 {
    pub target_speed: f32, // in rads/s
}

impl TryFrom<&[u8]> for RxPDO1 {
    type Error = ();

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() != 4 {
            return Err(());
        }
        Ok(RxPDO1 {
            target_speed: LittleEndian::read_f32(&value),
        })
    }
}

// add another PDO with temperature
pub struct TxPDO1 {
    pub motor_current: f32,  // A
    pub system_voltage: f32, // V
}

impl TxPDO1 {
    pub fn as_message(&self) -> TxCANMessage {
        let mut buffer = [0u8; 8];
        LittleEndian::write_f32(&mut buffer[0..4], self.motor_current);
        LittleEndian::write_f32(&mut buffer[4..8], self.system_voltage);

        TxCANMessage::PDO(PDO::PDO1, buffer, 8)
    }
}

pub struct TxPDO2 {
    pub die_temperature: f32, // deg C
}

impl TxPDO2 {
    pub fn as_message(&self) -> TxCANMessage {
        let mut buffer = [0u8; 8];
        LittleEndian::write_f32(&mut buffer[0..4], self.die_temperature);

        TxCANMessage::PDO(PDO::PDO2, buffer, 8)
    }
}
