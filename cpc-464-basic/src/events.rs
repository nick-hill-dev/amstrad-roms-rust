//! # Events: `ON BREAK`, `ON SQ`, `AFTER`/`EVERY`, `REMAIN`, `DI`/`EI`  (ROM `C807 .. C9C4`)
//!
//! BASIC's events are firmware synchronous events whose routine is in BASIC's ROM
//! (`C879`; `C45E` for the break key).  The firmware queues them; between statements the
//! interpreter checks the queue ([`Basic::interpret`]) and calls
//! [`Basic::process_events`].
//!
//! Each `ON SQ` and timer event block is followed by five bytes of BASIC's own:
//!
//! ```text
//!   +7  priority to restore (saved when the GOSUB starts)
//!   +8  the interrupted statement (the address DD74 was entered with)
//!   +10 the GOSUB line (address of the byte before the line)
//! ```
//!
//! When the event fires while a program runs, a `GOSUB` frame of kind 1 is pushed whose
//! "return address" is the event block + 7.  Execution continues at the line; `RETURN`
//! ([`Basic::stmt_return`]) sees the kind, calls `KL DONE SYNC` (so the event can fire
//! again - until then events of the same or lower priority wait) and *restarts* the
//! interrupted statement from its beginning.  `ON BREAK GOSUB` does the same with the
//! pseudo block at [`BREAK_GOSUB`] and kind 2.
//!
//! [`EVENT_FLAGS`] collects what the event routines decided while the queue was
//! processed: see its documentation.  In direct mode an event stops the processing
//! without being handled (it stays queued: no `KL DONE SYNC` is skipped, but the event
//! is not kicked again until the next count).

use crate::fw::KL_DO_SYNC;
use crate::streams::{BREAK_ROUTINE, ROM_SELECT_UPPER};
use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

/// BASIC's routine for `ON SQ` and timer events.
pub const EVENT_ROUTINE: u16 = 0xC879;
/// Distance from one ticker block to the next.
const TICKER_SIZE: u16 = 0x12;
/// Event flag bits (see [`EVENT_FLAGS`]).
const EV_BREAK: u8 = 0x01;
const EV_GOSUB: u8 = 0x02;
const EV_REARM: u8 = 0x04;
const EV_STOP: u8 = 0x40;
const EV_KEEP: u8 = 0x80;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Processing the queue   (C807 .. C846)
    // ===============================================================================

    /// ROM `C807`: run the pending synchronous events, then do what they asked for: re-arm
    /// the break key, report *Break*, or start an event's `GOSUB` (by continuing at its
    /// line - the statement loop is abandoned).  Leaves the text pointer at the
    /// statement start ([`STATEMENT_PTR`]).
    pub fn process_events(&mut self) -> Res {
        self.wr(EVENT_FLAGS, 0);
        while let Some((event, priority)) = self.kl_next_sync() {
            let f = self.rd(EVENT_FLAGS) & 0x7F;
            self.wr(EVENT_FLAGS, f);
            self.do_sync(event, priority)?;
            let f = self.rd(EVENT_FLAGS);
            if f & EV_KEEP == 0 { self.kl_done_sync(event, priority); }
            if f & EV_STOP != 0 { break; }
        }
        let f = self.rd(EVENT_FLAGS);
        if f & EV_REARM != 0 { self.arm_break(); }         // C453
        let p = self.rd16(STATEMENT_PTR);
        self.ptr = p;
        if f & (EV_BREAK | EV_GOSUB) == 0 { return Ok(()); }
        if f & EV_BREAK != 0 { return self.do_break().map(|n| match n {}); }   // CB6B
        // C842: an event GOSUB: STATEMENT_PTR now holds its line.
        Err(Exit::ContinueLine(p.wrapping_add(1)))
    }

    /// `KL DO SYNC` (`BCFE`) for the event at `event`: BASIC's own routines run here in
    /// Rust (they are far routines in "the current upper ROM", select byte `FD`); any
    /// other routine goes to the firmware (and from there to [`Hardware::call`]).  The
    /// routine gets `HL` = event + 4 and `B` = the priority from `KL NEXT SYNC`.
    fn do_sync(&mut self, event: u16, priority: u8) -> Res {
        if self.jumpblock_patched(KL_DO_SYNC) {
            let mut r = cpc464_os::Regs { hl: event, bc: (priority as u16) << 8, ..Default::default() };
            r.set_a(priority);
            self.cpc.hw.call(KL_DO_SYNC, &mut r);
            return Ok(());
        }
        let class = self.rd(event + 3);
        let routine = self.rd16(event + 4);
        let rom = self.rd(event + 6);
        if class & 1 == 0 && rom == ROM_SELECT_UPPER {
            match routine {
                BREAK_ROUTINE => return self.break_event_routine(event + 4, priority),
                EVENT_ROUTINE => return self.event_routine(event + 4, priority),
                _ => {}
            }
        }
        self.cpc.kl_do_sync(event);
        Ok(())
    }

    /// ROM `C847`: the second half of the break event routine (`C45E`).  `hl` = the break
    /// event + 4, `broke` = the pause ended with a second ESC.  Without a second ESC the
    /// break key is just re-armed; with one, either *Break* is reported (direct mode or no
    /// `ON BREAK GOSUB`) or the `ON BREAK GOSUB` starts.
    pub fn break_event_decide(&mut self, hl: u16, priority: u8, broke: bool) -> Res {
        self.wr16(BREAK_SAVED_PTR, hl);
        if !broke { self.set_event_flags(EV_REARM); return Ok(()); }
        if self.rd16(ON_BREAK_LINE) == 0 || self.current_line_number().is_none() {
            self.set_event_flags(EV_BREAK | EV_STOP);
            return Ok(());
        }
        self.event_gosub(BREAK_GOSUB, 2, priority)
    }

    /// ROM `C879`: BASIC's `ON SQ` / `AFTER` / `EVERY` event routine.  `hl` = event + 4.
    /// In direct mode the event only stops the processing.
    fn event_routine(&mut self, hl: u16, priority: u8) -> Res {
        let extra = hl.wrapping_add(3);
        if self.current_line_number().is_none() { self.set_event_flags(EV_STOP); return Ok(()); }
        self.event_gosub(extra, 1, priority)
    }

    /// ROM `C886`: push a `GOSUB` frame of `kind` returning to `extra` (an event's extra
    /// bytes), save the priority and the interrupted statement there and make the line the
    /// place to continue.
    fn event_gosub(&mut self, extra: u16, kind: u8, priority: u8) -> Res {
        self.push_gosub_frame(kind, extra)?;              // C6F6
        let stmt = self.rd16(STATEMENT_PTR);
        self.wr(extra, priority);
        self.wr16(extra + 1, stmt);
        let line = self.rd16(extra + 3);
        self.wr16(STATEMENT_PTR, line);
        self.set_event_flags(EV_KEEP | EV_STOP | EV_GOSUB);
        Ok(())
    }

    /// ROM `C89E`.
    fn set_event_flags(&mut self, bits: u8) {
        let f = self.rd(EVENT_FLAGS) | bits;
        self.wr(EVENT_FLAGS, f);
    }

    /// ROM `C8A4`: `RETURN` from an `ON SQ` / timer `GOSUB` (`extra` = event + 7):
    /// `KL DONE SYNC`, then restart the interrupted statement.
    pub fn return_from_event(&mut self, extra: u16) -> Res {
        let priority = self.rd(extra);
        let stmt = self.rd16(extra + 1);
        self.kl_done_sync(extra.wrapping_sub(7), priority);
        Err(Exit::Continue(stmt))
    }

    /// ROM `C8B6`: `RETURN` from an `ON BREAK GOSUB`.
    pub fn return_from_break(&mut self, extra: u16) -> Res {
        let priority = self.rd(extra);
        let event = self.rd16(BREAK_SAVED_PTR).wrapping_sub(4);
        self.kl_done_sync(event, priority);
        self.arm_break();
        Err(Exit::Continue(self.rd16(EVENT_SAVED_PTR)))
    }

    // ===============================================================================
    // ON BREAK, DI, EI, reset   (C8CB .. C93F)
    // ===============================================================================

    /// `ON BREAK GOSUB line` / `ON BREAK STOP` (ROM `C8CB`).
    pub fn stmt_on_break(&mut self) -> Res {
        let mut line = 0;
        if self.cur() != 0xCE {
            self.expect(0x9F)?;                           // GOSUB
            line = self.line_operand()?;                  // E767
            self.ptr = self.ptr.wrapping_sub(1);
        }
        self.wr16(ON_BREAK_LINE, line);
        self.next();
        Ok(())
    }

    /// `DI` (ROM `C8E1`).
    pub fn stmt_di(&mut self) -> Res { self.kl_event_disable(); Ok(()) }

    /// `EI` (ROM `C8E7`).
    pub fn stmt_ei(&mut self) -> Res { self.kl_event_enable(); Ok(()) }

    /// ROM `C8ED`: reset sound and all events; re-initialise the event blocks.  `ON SQ`
    /// events get class 8 (synchronous, far, priority 4); timers 0..3 classes 2, 4, 8, 16
    /// (timer 3 has the highest priority).
    pub fn reset_events(&mut self) {
        self.sound_reset();
        for i in 0..4 { self.kl_del_ticker(TICKERS + i * TICKER_SIZE); }
        self.km_disarm_break();
        self.kl_sync_reset();
        self.wr16(ON_BREAK_LINE, 0);
        self.arm_break();
        self.init_event_blocks(SQ_EVENTS, 3, 5, 0x08, false);
        self.init_event_blocks(TICKERS + 6, 4, 0x0B, 0x02, true);
    }

    /// ROM `C924`: `count` event blocks from `block`, each followed by `gap` bytes, class
    /// `class` (doubled for each next block if `double`).
    fn init_event_blocks(&mut self, mut block: u16, count: u8, gap: u16, mut class: u8, double: bool) {
        for _ in 0..count {
            self.kl_init_event(block, class, ROM_SELECT_UPPER, EVENT_ROUTINE);
            block = block.wrapping_add(7).wrapping_add(gap);   // KL INIT EVENT returns HL + 7
            if double { class = class.wrapping_add(class); }
        }
    }

    // ===============================================================================
    // ON SQ, AFTER, EVERY, REMAIN   (C940 .. C9C4)
    // ===============================================================================

    /// `ON SQ(channel) GOSUB line` (ROM `C940`): channel 1, 2 or 4.
    pub fn stmt_on_sq(&mut self) -> Res {
        self.expect(b'(')?;
        let channel = self.eval_byte()?;
        let block = match channel {
            1 => SQ_EVENTS,
            2 => SQ_EVENTS + 0x0C,
            4 => SQ_EVENTS + 0x18,
            _ => return Err(Exit::Error(5)),
        };
        self.expect(b')')?;
        self.event_line(block)?;                          // C861
        self.sound_arm_event(channel, block);
        Ok(())
    }

    /// ROM `C861`: `GOSUB line` of an event statement: store the line in the event's
    /// extra bytes (event + 10).
    fn event_line(&mut self, block: u16) -> Res {
        self.expect(0x9F)?;
        let line = self.line_operand()?;                  // E767
        self.skip_white();                                // DD61
        self.wr16(block + 0x0A, line);
        Ok(())
    }

    /// `AFTER time[,timer] GOSUB line` (ROM `C971`): once.
    pub fn stmt_after(&mut self) -> Res {
        let count = self.eval_positive_int()?;
        self.ticker_statement(count, 0)
    }

    /// `EVERY time[,timer] GOSUB line` (ROM `C979`): repeatedly.
    pub fn stmt_every(&mut self) -> Res {
        let count = self.eval_positive_int()?;
        self.ticker_statement(count, count)
    }

    /// ROM `C97E`.
    fn ticker_statement(&mut self, count: u16, recharge: u16) -> Res {
        let timer = if self.comma() { self.eval_int()? } else { 0 };
        let block = ticker_block(timer)?;                 // C9B1
        self.event_line(block + 6)?;
        self.kl_add_ticker(block, count, recharge);
        Ok(())
    }

    /// `REMAIN(timer)` (ROM `C99F`): stop the timer; its remaining count (0 if it was not
    /// running).
    pub fn fn_remain(&mut self) -> Res {
        let timer = self.acc_to_int()?;                   // FE8D
        let block = ticker_block(timer)?;
        let left = self.kl_del_ticker(block).unwrap_or(0);
        self.set_acc_int(left);
        Ok(())
    }
}

/// ROM `C9B1`: the ticker block of timer 0..3, else *Improper argument*.
fn ticker_block(timer: u16) -> Res<u16> {
    if timer >= 4 { return Err(Exit::Error(5)); }
    Ok(TICKERS + timer * TICKER_SIZE)
}
