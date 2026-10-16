//! # `SOUND`, `RELEASE`, `SQ`, `ENV`, `ENT`  (ROM `D2C0 .. D408`)
//!
//! The statements fill parameter blocks in BASIC's workspace and hand them to the
//! firmware's sound manager:
//!
//! ```text
//!   SOUND_PARAMS (ADB2):    channels, envelope, tone envelope, period(2), noise,
//!                           volume, duration(2)                    -> SOUND QUEUE
//!   ENVELOPE_PARAMS (ADBB): count, then up to 5 sections of 3 bytes -> SOUND AMPL/TONE ENVELOPE
//! ```
//!
//! When the channel's queue is full, `SOUND` does not wait inside the statement: it
//! abandons it and restarts it from its beginning (`JP DD71`), so events and ESC are
//! still serviced while a program waits for the sound queue.

use crate::fw::*;
use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

impl<H: Hardware> Basic<H> {
    /// `SOUND channels,period[,duration[,volume[,envelope[,ent[,noise]]]]]` (ROM `D2C0`).
    /// Defaults: duration 20, volume 12, no envelopes, no noise.
    pub fn stmt_sound(&mut self) -> Res {
        let channels = self.eval_byte()?;
        self.wr(SOUND_PARAMS, channels);
        self.expect(b',')?;
        let period = self.eval_tone_period()?;            // D3FF
        self.wr16(SOUND_PARAMS + 3, period);
        let duration = if self.comma() { self.eval_int()? } else { 20 };
        self.wr16(SOUND_PARAMS + 7, duration);
        let volume = self.optional_byte(16, 12)?;         // D30D
        self.wr(SOUND_PARAMS + 6, volume);
        let envelope = self.optional_byte(16, 0)?;
        self.wr(SOUND_PARAMS + 1, envelope);
        let ent = self.optional_byte(16, 0)?;
        self.wr(SOUND_PARAMS + 2, ent);
        let noise = self.optional_byte(32, 0)?;
        self.wr(SOUND_PARAMS + 5, noise);
        self.check_end()?;
        if self.sound_queue(SOUND_PARAMS) { return Ok(()); }
        // The queue is full: start the statement again (DD71: from STATEMENT_PTR).
        Err(Exit::Continue(self.rd16(STATEMENT_PTR)))
    }

    /// ROM `D30D`: `[,value]` below `limit`; an omitted value (no comma, or `,,`) is
    /// `default`.
    fn optional_byte(&mut self, limit: u8, default: u8) -> Res<u8> {
        if !self.comma() { return Ok(default); }
        if self.cur() == b',' { return Ok(default); }
        self.eval_byte_below(limit)                       // D317
    }

    /// `RELEASE channels` (ROM `D31E`).
    pub fn stmt_release(&mut self) -> Res {
        let channels = self.eval_byte_below(8)?;
        self.sound_release(channels);
        Ok(())
    }

    /// `SQ(channel)` (ROM `D329`): the queue status of channel 1, 2 or 4.
    pub fn fn_sq(&mut self) -> Res {
        let n = self.acc_to_int()?;
        if !matches!(n, 1 | 2 | 4) { return Err(Exit::Error(5)); }
        let s = self.sound_check(n as u8);
        self.set_acc_int(s as u16);
        Ok(())
    }

    /// ROM `D341`: an integer -128..127 (*Improper argument* otherwise).
    fn eval_signed_byte(&mut self) -> Res<u16> {
        let v = self.eval_int()?;
        let e = v as u8;
        let ext = if e & 0x80 != 0 { 0xFF } else { 0 };
        if ext != (v >> 8) as u8 { return Err(Exit::Error(5)); }
        Ok(v)
    }

    /// `ENV n,sections...` (ROM `D34E`): amplitude envelope 1..15.  A section is
    /// `count,step,pause` or `=shape,period` (a hardware envelope).
    pub fn stmt_env(&mut self) -> Res {
        let n = self.eval_nonzero_byte()?;
        if n >= 16 { return Err(Exit::Error(5)); }
        let count = self.envelope_sections(false)?;       // D3D8 with D367
        self.wr(ENVELOPE_PARAMS, count);
        self.sound_envelope(SOUND_AMPL_ENVELOPE, n, ENVELOPE_PARAMS);
        Ok(())
    }

    /// `ENT n,sections...` (ROM `D385`): tone envelope 1..15 (negative: repeating).  A
    /// section is `count,step,pause` or `=period,pause` (an absolute period).
    pub fn stmt_ent(&mut self) -> Res {
        let v = self.eval_signed_byte()?;
        let negative = v & 0x8000 != 0;
        let n = if negative { (v as u8).wrapping_neg() } else { v as u8 };
        if n == 0 || n >= 16 { return Err(Exit::Error(5)); }
        let count = self.envelope_sections(true)?;        // D3D8 with D3AE
        self.wr(ENVELOPE_PARAMS, if negative { 0x80 } else { 0 } | count);
        self.sound_envelope(SOUND_TONE_ENVELOPE, n, ENVELOPE_PARAMS);
        Ok(())
    }

    /// ROM `D3D8`: up to five `,section`s into [`ENVELOPE_PARAMS`] + 1; the statement must
    /// end after them.  Returns how many there were.
    fn envelope_sections(&mut self, tone: bool) -> Res<u8> {
        let mut count = 0u8;
        for _ in 0..5 {
            if !self.comma() { break; }
            let (c, e, d) = if tone { self.tone_section()? } else { self.amplitude_section()? };
            let at = ENVELOPE_PARAMS + 1 + 3 * count as u16;
            self.wr(at, c);
            self.wr(at + 1, e);
            self.wr(at + 2, d);
            count += 1;
        }
        self.check_end()?;
        Ok(count)
    }

    /// ROM `D367`: an amplitude section: `(C, E, D)` = `(count, step, pause)` or
    /// `(0x80 | shape, period low, period high)`.
    fn amplitude_section(&mut self) -> Res<(u8, u8, u8)> {
        if self.cur() == 0xEF {
            self.next();
            let shape = self.eval_byte_below(16)? | 0x80;
            self.expect(b',')?;
            let period = self.eval_address()?;            // CE91
            return Ok((shape, period as u8, (period >> 8) as u8));
        }
        let count = self.eval_byte_below(0x80)?;
        self.step_and_pause(count)                        // D3C5
    }

    /// ROM `D3AE`: a tone section: `(count, step, pause)` or `(0xF0 + period high,
    /// period low, pause)`.
    fn tone_section(&mut self) -> Res<(u8, u8, u8)> {
        if self.cur() == 0xEF {
            self.next();
            let period = self.eval_tone_period()?;
            let c = ((period >> 8) as u8).wrapping_add(0xF0);
            let low = period as u8;
            return self.pause_after(c, low);              // D3CE
        }
        let count = self.eval_byte_below(0xF0)?;
        self.step_and_pause(count)
    }

    /// ROM `D3C5`: `,step,pause` after the count.
    fn step_and_pause(&mut self, count: u8) -> Res<(u8, u8, u8)> {
        self.expect(b',')?;
        let step = self.eval_signed_byte()? as u8;
        self.pause_after(count, step)
    }

    /// ROM `D3CE`: `,pause`.
    fn pause_after(&mut self, c: u8, e: u8) -> Res<(u8, u8, u8)> {
        self.expect(b',')?;
        let pause = self.eval_byte()?;
        Ok((c, e, pause))
    }

    /// ROM `D3FF`: a tone period 0..4095.
    fn eval_tone_period(&mut self) -> Res<u16> {
        let v = self.eval_int()?;
        if v & 0xF000 != 0 { return Err(Exit::Error(5)); }
        Ok(v)
    }
}
