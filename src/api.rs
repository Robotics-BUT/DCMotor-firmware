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

pub struct TxPDO1 {
    pub current_speed: f32, // rad/s
    pub motor_current: f32, // A
}

impl TxPDO1 {
    pub fn as_message(&self) -> TxCANMessage {
        let mut buffer = [0u8; 8];
        LittleEndian::write_f32(&mut buffer[0..4], self.current_speed);
        LittleEndian::write_f32(&mut buffer[4..8], self.motor_current);

        // defmt::debug!(
        //     "{:u8} {:u8} {:u8} {:u8} {:u8} {:u8} {:u8} {:u8}",
        //     buffer[0],
        //     buffer[1],
        //     buffer[2],
        //     buffer[3],
        //     buffer[4],
        //     buffer[5],
        //     buffer[6],
        //     buffer[7]
        // );
        TxCANMessage::PDO(PDO::PDO1, buffer, 8)
    }
}

pub struct TxPDO2 {
    pub system_voltage: f32,  // V
    pub die_temperature: f32, // deg C
}

impl TxPDO2 {
    pub fn as_message(&self) -> TxCANMessage {
        let mut buffer = [0u8; 8];
        LittleEndian::write_f32(&mut buffer[0..4], self.system_voltage);
        LittleEndian::write_f32(&mut buffer[4..8], self.die_temperature);

        TxCANMessage::PDO(PDO::PDO2, buffer, 8)
    }
}
