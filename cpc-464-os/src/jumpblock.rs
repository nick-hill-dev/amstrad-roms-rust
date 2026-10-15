//! The jumpblock (ROM `0888` builder, `08AC`-`0A89` table).
//!
//! Applications reach the firmware through a block of three-byte entries in RAM, starting
//! at `BB00`.  Each entry is a `RST` instruction followed by a 16-bit address:
//!
//! * the first 191 entries (`BB00`-`BD39`) are `RST 1` (*LOW JUMP*): run the routine with
//!   the lower ROM enabled and the upper ROM disabled; bit 15 of the address is set so the
//!   kernel knows to switch the upper ROM off;
//! * the remaining 48 (`BD3A`-`BDCA`: the line editor and the arithmetic package) are `RST 5` (*FIRM JUMP*):
//!   lower ROM on, the upper ROM left as it is.
//!
//! The RAM copy exists so a program can patch an entry to redirect a firmware call.  The
//! ROM keeps the pristine addresses in the table below; [`Cpc::jump_restore`] (`JUMP
//! RESTORE`, `BD37`) rebuilds the RAM copy from it.

use crate::{Cpc, Hardware};

/// One jumpblock entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JumpEntry {
    /// Firmware name (`KM INITIALISE`, ...).
    pub name: &'static str,
    /// Address of the routine in the lower ROM.
    pub target: u16,
}

/// Number of entries that use `RST 1`; the rest use `RST 5`.
pub const LOW_JUMP_ENTRIES: usize = 191;
/// Total number of entries.
pub const ENTRIES: usize = 239;
/// Address of the first entry in RAM.
pub const JUMPBLOCK_BASE: u16 = 0xBB00;
/// Opcode of `RST 1` (`LOW JUMP`).
pub const RST_LOW_JUMP: u8 = 0xCF;
/// Opcode of `RST 5` (`FIRM JUMP`).
pub const RST_FIRM_JUMP: u8 = 0xEF;

/// The table, in jumpblock order.
pub const JUMPBLOCK: [JumpEntry; ENTRIES] = [
    JumpEntry { name: "KM INITIALISE", target: 0x19E0 }, // BB00
    JumpEntry { name: "KM RESET", target: 0x1A1E }, // BB03
    JumpEntry { name: "KM WAIT CHAR", target: 0x1A3C }, // BB06
    JumpEntry { name: "KM READ CHAR", target: 0x1A42 }, // BB09
    JumpEntry { name: "KM CHAR RETURN", target: 0x1A77 }, // BB0C
    JumpEntry { name: "KM SET EXPAND", target: 0x1ABD }, // BB0F
    JumpEntry { name: "KM GET EXPAND", target: 0x1B2E }, // BB12
    JumpEntry { name: "KM EXP BUFFER", target: 0x1A7B }, // BB15
    JumpEntry { name: "KM WAIT KEY", target: 0x1B56 }, // BB18
    JumpEntry { name: "KM READ KEY", target: 0x1B5C }, // BB1B
    JumpEntry { name: "KM TEST KEY", target: 0x1CBD }, // BB1E
    JumpEntry { name: "KM GET STATE", target: 0x1BB3 }, // BB21
    JumpEntry { name: "KM GET JOYSTICK", target: 0x1C5C }, // BB24
    JumpEntry { name: "KM SET TRANSLATE", target: 0x1D52 }, // BB27
    JumpEntry { name: "KM GET TRANSLATE", target: 0x1D3E }, // BB2A
    JumpEntry { name: "KM SET SHIFT", target: 0x1D57 }, // BB2D
    JumpEntry { name: "KM GET SHIFT", target: 0x1D43 }, // BB30
    JumpEntry { name: "KM SET CONTROL", target: 0x1D5C }, // BB33
    JumpEntry { name: "KM GET CONTROL", target: 0x1D48 }, // BB36
    JumpEntry { name: "KM SET REPEAT", target: 0x1CAB }, // BB39
    JumpEntry { name: "KM GET REPEAT", target: 0x1CA6 }, // BB3C
    JumpEntry { name: "KM SET DELAY", target: 0x1C6D }, // BB3F
    JumpEntry { name: "KM GET DELAY", target: 0x1C69 }, // BB42
    JumpEntry { name: "KM ARM BREAK", target: 0x1C71 }, // BB45
    JumpEntry { name: "KM DISARM BREAK", target: 0x1C82 }, // BB48
    JumpEntry { name: "KM BREAK EVENT", target: 0x1C90 }, // BB4B
    JumpEntry { name: "TXT INITIALISE", target: 0x1078 }, // BB4E
    JumpEntry { name: "TXT RESET", target: 0x1088 }, // BB51
    JumpEntry { name: "TXT VDU ENABLE", target: 0x1451 }, // BB54
    JumpEntry { name: "TXT VDU DISABLE", target: 0x144B }, // BB57
    JumpEntry { name: "TXT OUTPUT", target: 0x1400 }, // BB5A
    JumpEntry { name: "TXT WR CHAR", target: 0x1334 }, // BB5D
    JumpEntry { name: "TXT RD CHAR", target: 0x13AB }, // BB60
    JumpEntry { name: "TXT SET GRAPHIC", target: 0x13A7 }, // BB63
    JumpEntry { name: "TXT WIN ENABLE", target: 0x120C }, // BB66
    JumpEntry { name: "TXT GET WINDOW", target: 0x1256 }, // BB69
    JumpEntry { name: "TXT CLEAR WINDOW", target: 0x1540 }, // BB6C
    JumpEntry { name: "TXT SET COLUMN", target: 0x115E }, // BB6F
    JumpEntry { name: "TXT SET ROW", target: 0x1169 }, // BB72
    JumpEntry { name: "TXT SET CURSOR", target: 0x1174 }, // BB75
    JumpEntry { name: "TXT GET CURSOR", target: 0x1180 }, // BB78
    JumpEntry { name: "TXT CUR ENABLE", target: 0x1289 }, // BB7B
    JumpEntry { name: "TXT CUR DISABLE", target: 0x129A }, // BB7E
    JumpEntry { name: "TXT CUR ON", target: 0x1279 }, // BB81
    JumpEntry { name: "TXT CUR OFF", target: 0x1281 }, // BB84
    JumpEntry { name: "TXT VALIDATE", target: 0x11CE }, // BB87
    JumpEntry { name: "TXT PLACE CURSOR", target: 0x1268 }, // BB8A
    JumpEntry { name: "TXT REMOVE CURSOR", target: 0x1268 }, // BB8D
    JumpEntry { name: "TXT SET PEN", target: 0x12A9 }, // BB90
    JumpEntry { name: "TXT GET PEN", target: 0x12BD }, // BB93
    JumpEntry { name: "TXT SET PAPER", target: 0x12AE }, // BB96
    JumpEntry { name: "TXT GET PAPER", target: 0x12C3 }, // BB99
    JumpEntry { name: "TXT INVERSE", target: 0x12C9 }, // BB9C
    JumpEntry { name: "TXT SET BACK", target: 0x137A }, // BB9F
    JumpEntry { name: "TXT GET BACK", target: 0x1387 }, // BBA2
    JumpEntry { name: "TXT GET MATRIX", target: 0x12D3 }, // BBA5
    JumpEntry { name: "TXT SET MATRIX", target: 0x12F1 }, // BBA8
    JumpEntry { name: "TXT SET M TABLE", target: 0x12FD }, // BBAB
    JumpEntry { name: "TXT GET M TABLE", target: 0x132A }, // BBAE
    JumpEntry { name: "TXT GET CONTROLS", target: 0x14CB }, // BBB1
    JumpEntry { name: "TXT STR SELECT", target: 0x10E8 }, // BBB4
    JumpEntry { name: "TXT SWAP STREAMS", target: 0x1107 }, // BBB7
    JumpEntry { name: "GRA INITIALISE", target: 0x15B0 }, // BBBA
    JumpEntry { name: "GRA RESET", target: 0x15DF }, // BBBD
    JumpEntry { name: "GRA MOVE ABSOLUTE", target: 0x15F4 }, // BBC0
    JumpEntry { name: "GRA MOVE RELATIVE", target: 0x15F1 }, // BBC3
    JumpEntry { name: "GRA ASK CURSOR", target: 0x15FC }, // BBC6
    JumpEntry { name: "GRA SET ORIGIN", target: 0x1604 }, // BBC9
    JumpEntry { name: "GRA GET ORIGIN", target: 0x1612 }, // BBCC
    JumpEntry { name: "GRA WIN WIDTH", target: 0x1734 }, // BBCF
    JumpEntry { name: "GRA WIN HEIGHT", target: 0x1779 }, // BBD2
    JumpEntry { name: "GRA GET W WIDTH", target: 0x17A6 }, // BBD5
    JumpEntry { name: "GRA GET W HEIGHT", target: 0x17BC }, // BBD8
    JumpEntry { name: "GRA CLEAR WINDOW", target: 0x17C5 }, // BBDB
    JumpEntry { name: "GRA SET PEN", target: 0x17F6 }, // BBDE
    JumpEntry { name: "GRA GET PEN", target: 0x1804 }, // BBE1
    JumpEntry { name: "GRA SET PAPER", target: 0x17FD }, // BBE4
    JumpEntry { name: "GRA GET PAPER", target: 0x180A }, // BBE7
    JumpEntry { name: "GRA PLOT ABSOLUTE", target: 0x1813 }, // BBEA
    JumpEntry { name: "GRA PLOT RELATIVE", target: 0x1810 }, // BBED
    JumpEntry { name: "GRA TEST ABSOLUTE", target: 0x1827 }, // BBF0
    JumpEntry { name: "GRA TEST RELATIVE", target: 0x1824 }, // BBF3
    JumpEntry { name: "GRA LINE ABSOLUTE", target: 0x1839 }, // BBF6
    JumpEntry { name: "GRA LINE RELATIVE", target: 0x1836 }, // BBF9
    JumpEntry { name: "GRA WR CHAR", target: 0x1945 }, // BBFC
    JumpEntry { name: "SCR INITIALISE", target: 0x0AA0 }, // BBFF
    JumpEntry { name: "SCR RESET", target: 0x0AB1 }, // BC02
    JumpEntry { name: "SCR SET OFFSET", target: 0x0B3C }, // BC05
    JumpEntry { name: "SCR SET BASE", target: 0x0B45 }, // BC08
    JumpEntry { name: "SCR GET LOCATION", target: 0x0B50 }, // BC0B
    JumpEntry { name: "SCR SET MODE", target: 0x0ACA }, // BC0E
    JumpEntry { name: "SCR GET MODE", target: 0x0AEC }, // BC11
    JumpEntry { name: "SCR CLEAR", target: 0x0AF7 }, // BC14
    JumpEntry { name: "SCR CHAR LIMITS", target: 0x0B57 }, // BC17
    JumpEntry { name: "SCR CHAR POSITION", target: 0x0B64 }, // BC1A
    JumpEntry { name: "SCR DOT POSITION", target: 0x0BA9 }, // BC1D
    JumpEntry { name: "SCR NEXT BYTE", target: 0x0BF9 }, // BC20
    JumpEntry { name: "SCR PREV BYTE", target: 0x0C05 }, // BC23
    JumpEntry { name: "SCR NEXT LINE", target: 0x0C13 }, // BC26
    JumpEntry { name: "SCR PREV LINE", target: 0x0C2D }, // BC29
    JumpEntry { name: "SCR INK ENCODE", target: 0x0C86 }, // BC2C
    JumpEntry { name: "SCR INK DECODE", target: 0x0CA0 }, // BC2F
    JumpEntry { name: "SCR SET INK", target: 0x0CEC }, // BC32
    JumpEntry { name: "SCR GET INK", target: 0x0D14 }, // BC35
    JumpEntry { name: "SCR SET BORDER", target: 0x0CF1 }, // BC38
    JumpEntry { name: "SCR GET BORDER", target: 0x0D19 }, // BC3B
    JumpEntry { name: "SCR SET FLASHING", target: 0x0CE4 }, // BC3E
    JumpEntry { name: "SCR GET FLASHING", target: 0x0CE8 }, // BC41
    JumpEntry { name: "SCR FILL BOX", target: 0x0DB3 }, // BC44
    JumpEntry { name: "SCR FLOOD BOX", target: 0x0DB7 }, // BC47
    JumpEntry { name: "SCR CHAR INVERT", target: 0x0DDF }, // BC4A
    JumpEntry { name: "SCR HW ROLL", target: 0x0DFA }, // BC4D
    JumpEntry { name: "SCR SW ROLL", target: 0x0E3E }, // BC50
    JumpEntry { name: "SCR UNPACK", target: 0x0EF3 }, // BC53
    JumpEntry { name: "SCR REPACK", target: 0x0F49 }, // BC56
    JumpEntry { name: "SCR ACCESS", target: 0x0C49 }, // BC59
    JumpEntry { name: "SCR PIXELS", target: 0x0C6B }, // BC5C
    JumpEntry { name: "SCR HORIZONTAL", target: 0x0FC4 }, // BC5F
    JumpEntry { name: "SCR VERTICAL", target: 0x102F }, // BC62
    JumpEntry { name: "CAS INITIALISE", target: 0x2370 }, // BC65
    JumpEntry { name: "CAS SET SPEED", target: 0x237F }, // BC68
    JumpEntry { name: "CAS NOISY", target: 0x238E }, // BC6B
    JumpEntry { name: "CAS START MOTOR", target: 0x2A4B }, // BC6E
    JumpEntry { name: "CAS STOP MOTOR", target: 0x2A4F }, // BC71
    JumpEntry { name: "CAS RESTORE MOTOR", target: 0x2A51 }, // BC74
    JumpEntry { name: "CAS IN OPEN", target: 0x2392 }, // BC77
    JumpEntry { name: "CAS IN CLOSE", target: 0x23FC }, // BC7A
    JumpEntry { name: "CAS IN ABANDON", target: 0x2401 }, // BC7D
    JumpEntry { name: "CAS IN CHAR", target: 0x2435 }, // BC80
    JumpEntry { name: "CAS IN DIRECT", target: 0x24AB }, // BC83
    JumpEntry { name: "CAS RETURN", target: 0x249A }, // BC86
    JumpEntry { name: "CAS TEST EOF", target: 0x2496 }, // BC89
    JumpEntry { name: "CAS OUT OPEN", target: 0x23AB }, // BC8C
    JumpEntry { name: "CAS OUT CLOSE", target: 0x2415 }, // BC8F
    JumpEntry { name: "CAS OUT ABANDON", target: 0x242E }, // BC92
    JumpEntry { name: "CAS OUT CHAR", target: 0x245B }, // BC95
    JumpEntry { name: "CAS OUT DIRECT", target: 0x24EA }, // BC98
    JumpEntry { name: "CAS CATALOG", target: 0x2528 }, // BC9B
    JumpEntry { name: "CAS WRITE", target: 0x283F }, // BC9E
    JumpEntry { name: "CAS READ", target: 0x2836 }, // BCA1
    JumpEntry { name: "CAS CHECK", target: 0x2851 }, // BCA4
    JumpEntry { name: "SOUND RESET", target: 0x1E68 }, // BCA7
    JumpEntry { name: "SOUND QUEUE", target: 0x1F9F }, // BCAA
    JumpEntry { name: "SOUND CHECK", target: 0x206C }, // BCAD
    JumpEntry { name: "SOUND ARM EVENT", target: 0x2089 }, // BCB0
    JumpEntry { name: "SOUND RELEASE", target: 0x204A }, // BCB3
    JumpEntry { name: "SOUND HOLD", target: 0x1ECB }, // BCB6
    JumpEntry { name: "SOUND CONTINUE", target: 0x1EE6 }, // BCB9
    JumpEntry { name: "SOUND AMPL ENVELOPE", target: 0x2338 }, // BCBC
    JumpEntry { name: "SOUND TONE ENVELOPE", target: 0x233D }, // BCBF
    JumpEntry { name: "SOUND A ADDRESS", target: 0x2349 }, // BCC2
    JumpEntry { name: "SOUND T ADDRESS", target: 0x234E }, // BCC5
    JumpEntry { name: "KL CHOKE OFF", target: 0x005C }, // BCC8
    JumpEntry { name: "KL ROM WALK", target: 0x0329 }, // BCCB
    JumpEntry { name: "KL INIT BACK", target: 0x0332 }, // BCCE
    JumpEntry { name: "KL LOG EXT", target: 0x02A1 }, // BCD1
    JumpEntry { name: "KL FIND COMMAND", target: 0x02B2 }, // BCD4
    JumpEntry { name: "KL NEW FRAME FLY", target: 0x0163 }, // BCD7
    JumpEntry { name: "KL ADD FRAME FLY", target: 0x016A }, // BCDA
    JumpEntry { name: "KL DEL FRAME FLY", target: 0x0170 }, // BCDD
    JumpEntry { name: "KL NEW FAST TICKER", target: 0x0176 }, // BCE0
    JumpEntry { name: "KL ADD FAST TICKER", target: 0x017D }, // BCE3
    JumpEntry { name: "KL DEL FAST TICKER", target: 0x0183 }, // BCE6
    JumpEntry { name: "KL ADD TICKER", target: 0x01B3 }, // BCE9
    JumpEntry { name: "KL DEL TICKER", target: 0x01C5 }, // BCEC
    JumpEntry { name: "KL INIT EVENT", target: 0x01D2 }, // BCEF
    JumpEntry { name: "KL EVENT", target: 0x01E2 }, // BCF2
    JumpEntry { name: "KL SYNC RESET", target: 0x0228 }, // BCF5
    JumpEntry { name: "KL DEL SYNCHRONOUS", target: 0x0285 }, // BCF8
    JumpEntry { name: "KL NEXT SYNC", target: 0x0256 }, // BCFB
    JumpEntry { name: "KL DO SYNC", target: 0x021A }, // BCFE
    JumpEntry { name: "KL DONE SYNC", target: 0x0277 }, // BD01
    JumpEntry { name: "KL EVENT DISABLE", target: 0x0295 }, // BD04
    JumpEntry { name: "KL EVENT ENABLE", target: 0x029B }, // BD07
    JumpEntry { name: "KL DISARM EVENT", target: 0x028E }, // BD0A
    JumpEntry { name: "KL TIME PLEASE", target: 0x0099 }, // BD0D
    JumpEntry { name: "KL TIME SET", target: 0x00A3 }, // BD10
    JumpEntry { name: "MC BOOT PROGRAM", target: 0x05DC }, // BD13
    JumpEntry { name: "MC START PROGRAM", target: 0x060B }, // BD16
    JumpEntry { name: "MC WAIT FLYBACK", target: 0x07BA }, // BD19
    JumpEntry { name: "MC SET MODE", target: 0x0776 }, // BD1C
    JumpEntry { name: "MC SCREEN OFFSET", target: 0x07C6 }, // BD1F
    JumpEntry { name: "MC CLEAR INKS", target: 0x0786 }, // BD22
    JumpEntry { name: "MC SET INKS", target: 0x0799 }, // BD25
    JumpEntry { name: "MC RESET PRINTER", target: 0x07E6 }, // BD28
    JumpEntry { name: "MC PRINT CHAR", target: 0x07F2 }, // BD2B
    JumpEntry { name: "MC BUSY PRINTER", target: 0x081B }, // BD2E
    JumpEntry { name: "MC SEND PRINTER", target: 0x0807 }, // BD31
    JumpEntry { name: "MC SOUND REGISTER", target: 0x0826 }, // BD34
    JumpEntry { name: "JUMP RESTORE", target: 0x0888 }, // BD37
    JumpEntry { name: "EDIT", target: 0x2A98 }, // BD3A
    JumpEntry { name: "REAL COPY", target: 0x2E18 }, // BD3D
    JumpEntry { name: "INT TO REAL", target: 0x2E29 }, // BD40
    JumpEntry { name: "BIN TO REAL", target: 0x2E55 }, // BD43
    JumpEntry { name: "REAL TO INT", target: 0x2E66 }, // BD46
    JumpEntry { name: "REAL TO BIN", target: 0x2E8E }, // BD49
    JumpEntry { name: "REAL FIX", target: 0x2EA1 }, // BD4C
    JumpEntry { name: "REAL INT", target: 0x2EAC }, // BD4F
    JumpEntry { name: "REAL DECIMAL PREPARE", target: 0x2EB6 }, // BD52
    JumpEntry { name: "REAL SCALE POW10", target: 0x2F1D }, // BD55
    JumpEntry { name: "REAL ADD", target: 0x333F }, // BD58
    JumpEntry { name: "REAL SUB", target: 0x3337 }, // BD5B
    JumpEntry { name: "REAL REVERSE SUB", target: 0x333B }, // BD5E
    JumpEntry { name: "REAL MULTIPLY", target: 0x3415 }, // BD61
    JumpEntry { name: "REAL DIVIDE", target: 0x349E }, // BD64
    JumpEntry { name: "REAL MUL POW2", target: 0x3578 }, // BD67
    JumpEntry { name: "REAL COMPARE", target: 0x359A }, // BD6A
    JumpEntry { name: "REAL NEGATE", target: 0x35F8 }, // BD6D
    JumpEntry { name: "REAL SIGN", target: 0x35E8 }, // BD70
    JumpEntry { name: "SET ANGLE MODE", target: 0x31AE }, // BD73
    JumpEntry { name: "REAL PI", target: 0x31A3 }, // BD76
    JumpEntry { name: "REAL SQRT", target: 0x310A }, // BD79
    JumpEntry { name: "REAL POWER", target: 0x310D }, // BD7C
    JumpEntry { name: "REAL LN", target: 0x3014 }, // BD7F
    JumpEntry { name: "REAL LOG10", target: 0x300F }, // BD82
    JumpEntry { name: "REAL EXP", target: 0x3090 }, // BD85
    JumpEntry { name: "REAL SIN", target: 0x31BC }, // BD88
    JumpEntry { name: "REAL COS", target: 0x31B2 }, // BD8B
    JumpEntry { name: "REAL TAN", target: 0x3231 }, // BD8E
    JumpEntry { name: "REAL ATN", target: 0x3241 }, // BD91
    JumpEntry { name: "BIN TO REAL SCALED", target: 0x2E5E }, // BD94
    JumpEntry { name: "RANDOMIZE RESET", target: 0x2F94 }, // BD97
    JumpEntry { name: "RANDOMIZE SEED", target: 0x2FA1 }, // BD9A
    JumpEntry { name: "RND", target: 0x2FB7 }, // BD9D
    JumpEntry { name: "RND LAST", target: 0x2FE6 }, // BDA0
    JumpEntry { name: "INT ABS SPLIT", target: 0x3708 }, // BDA3
    JumpEntry { name: "INT UNSIGNED SPLIT", target: 0x370E }, // BDA6
    JumpEntry { name: "INT APPLY SIGN", target: 0x3715 }, // BDA9
    JumpEntry { name: "INT ADD", target: 0x3728 }, // BDAC
    JumpEntry { name: "INT SUB", target: 0x3731 }, // BDAF
    JumpEntry { name: "INT REVERSE SUB", target: 0x3730 }, // BDB2
    JumpEntry { name: "INT MULTIPLY", target: 0x3739 }, // BDB5
    JumpEntry { name: "INT DIVIDE", target: 0x377A }, // BDB8
    JumpEntry { name: "INT MODULUS", target: 0x3781 }, // BDBB
    JumpEntry { name: "INT MULTIPLY UNSIGNED", target: 0x3750 }, // BDBE
    JumpEntry { name: "INT DIVIDE UNSIGNED", target: 0x378C }, // BDC1
    JumpEntry { name: "INT COMPARE", target: 0x37E9 }, // BDC4
    JumpEntry { name: "INT NEGATE", target: 0x37D4 }, // BDC7
    JumpEntry { name: "INT SIGN", target: 0x37E0 }, // BDCA
];

impl JumpEntry {
    /// RAM address of this entry's three bytes.
    pub fn ram_address(index: usize) -> u16 { JUMPBLOCK_BASE + 3 * index as u16 }
}

/// Find an entry by its firmware name (spaces, as in the manuals: `"TXT OUTPUT"`).
pub fn find(name: &str) -> Option<(usize, JumpEntry)> {
    JUMPBLOCK.iter().copied().enumerate().find(|(_, e)| e.name == name)
}

impl<H: Hardware> Cpc<H> {
    /// `JUMP RESTORE` (`BD37`, ROM `0888`): rebuild the whole RAM jumpblock from the ROM table.
    ///
    /// Entry `n` becomes `RST 1` / `RST 5`, the low byte of the target, and the high byte of
    /// the target with bit 7 set for `RST 1` entries (`CPL; RLCA; RLCA; AND 80` of the opcode
    /// yields `80` for `CF` and `00` for `EF`).
    pub fn jump_restore(&mut self) {
        for (i, entry) in JUMPBLOCK.iter().enumerate() {
            let at = JumpEntry::ram_address(i);
            let opcode = if i < LOW_JUMP_ENTRIES { RST_LOW_JUMP } else { RST_FIRM_JUMP };
            let flag: u8 = if opcode == RST_LOW_JUMP { 0x80 } else { 0x00 };
            self.wr8(at, opcode);
            self.wr8(at + 1, entry.target as u8);
            self.wr8(at + 2, (entry.target >> 8) as u8 | flag);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_has_the_documented_shape() {
        assert_eq!(JUMPBLOCK.len(), 239);
        assert_eq!(JumpEntry::ram_address(LOW_JUMP_ENTRIES), 0xBD3D);
        assert_eq!(find("KM INITIALISE").map(|(i, e)| (i, e.target)), Some((0, 0x19E0)));
        assert_eq!(find("JUMP RESTORE").map(|(i, e)| (i, e.target)), Some((189, 0x0888)));
    }
}
