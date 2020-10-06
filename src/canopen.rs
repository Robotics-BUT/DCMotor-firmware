use crate::can::CANFrame;
use core::convert::TryFrom;

pub enum NMTRequestedState {
    Operational,
    Stopped,
    PreOperational,
    ResetNode,
    ResetCommunication,
}

#[derive(Copy, Clone)]
pub enum NMTState {
    BootUp,
    Stopped,
    Operational,
    PreOperational,
}

impl From<NMTState> for u8 {
    fn from(raw: NMTState) -> Self {
        match raw {
            NMTState::BootUp => 0x00,
            NMTState::Stopped => 0x04,
            NMTState::Operational => 0x05,
            NMTState::PreOperational => 0x7f,
        }
    }
}

impl TryFrom<u8> for NMTRequestedState {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(Self::Operational),
            0x02 => Ok(Self::Stopped),
            0x80 => Ok(Self::PreOperational),
            0x81 => Ok(Self::ResetNode),
            0x82 => Ok(Self::ResetCommunication),
            _ => Err(()),
        }
    }
}

pub enum PDO {
    PDO1,
    PDO2,
    PDO3,
    PDO4,
}

impl PDO {
    pub fn tx_id(&self) -> u16 {
        match self {
            PDO::PDO1 => 0x180,
            PDO::PDO2 => 0x280,
            PDO::PDO3 => 0x380,
            PDO::PDO4 => 0x480,
        }
    }

    pub fn rx_id(&self) -> u16 {
        match self {
            PDO::PDO1 => 0x200,
            PDO::PDO2 => 0x300,
            PDO::PDO3 => 0x400,
            PDO::PDO4 => 0x500,
        }
    }

    pub fn from_rx(id: u16) -> Result<Self, ()> {
        match id {
            0x200 => Ok(PDO::PDO1),
            0x300 => Ok(PDO::PDO2),
            0x400 => Ok(PDO::PDO3),
            0x500 => Ok(PDO::PDO4),
            _ => Err(()),
        }
    }
}

pub enum RxCANMessage {
    Sync([u8; 8], usize),
    PDO(PDO, [u8; 8], usize),
    NMT(NMTRequestedState),
    SDO,
}

pub fn message_from_frame(id: u8, frame: &CANFrame) -> Result<RxCANMessage, ()> {
    let target_device = (frame.id & 0x7f) as u8;
    let message_id = frame.id & 0xff80;
    match message_id {
        0x00 => {
            if frame.dlc < 2 {
                return Err(());
            }
            if frame.data[1] != id {
                return Err(());
            }
            match NMTRequestedState::try_from(frame.data[0]) {
                Ok(state) => Ok(RxCANMessage::NMT(state)),
                Err(_) => Err(()),
            }
        }
        0x80 => Ok(RxCANMessage::Sync(frame.data, frame.dlc as usize)),
        0x200 | 0x300 | 0x400 | 0x500 => {
            if target_device != id {
                return Err(());
            }
            // unwrapping is here because if there is an Err, it is a programming error and should fail fast.
            let pdo = PDO::from_rx(message_id).unwrap();
            Ok(RxCANMessage::PDO(pdo, frame.data, frame.dlc as usize))
        }
        _ => Err(()),
    }
}

pub enum TxCANMessage {
    NMTHeartbeat(NMTState),
    PDO(PDO, [u8; 8], usize),
}

pub fn message_to_frame(id: u8, message: TxCANMessage) -> CANFrame {
    match message {
        TxCANMessage::NMTHeartbeat(state) => CANFrame {
            id: 0x700 | (id as u16),
            rtr: false,
            dlc: 1,
            data: [state.into(), 0, 0, 0, 0, 0, 0, 0],
        },
        TxCANMessage::PDO(pdo, data, size) => CANFrame {
            id: pdo.tx_id() | (id as u16),
            rtr: false,
            dlc: size as u8,
            data,
        },
    }
}
