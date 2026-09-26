#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Color10 {
    Blue,
    Orange,
    Green,
    Red,
    Purple,
    Brown,
    Pink,
    Gray,
    Olive,
    Cyan,
    DarkGray,
}

impl Color10 {
    pub fn to_hex(self) -> u32 {
        match self {
            Color10::Blue => 0x4472C4,
            Color10::Orange => 0xED7D31,
            Color10::Green => 0x70AD47,
            Color10::Red => 0xFF0000,
            Color10::Purple => 0x7030A0,
            Color10::Brown => 0x833C00,
            Color10::Pink => 0xFF99CC,
            Color10::Gray => 0x808080,
            Color10::Olive => 0x808000,
            Color10::Cyan => 0x00B0F0,
            Color10::DarkGray => 0x3C3C3C,
        }
    }
}
