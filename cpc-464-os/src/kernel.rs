//! The kernel (`KL`): time, interrupt tick, event queues, tickers, ROM management and
//! the RAM-resident "banking" helpers.
//!
//! ROM ranges: `0000-0390` (ROM-resident part) and `0391-057F` (code that the reset
//! routine copies to RAM at `B900`, because it must keep running while the ROMs are
//! switched out).
//!
//! ## Event blocks
//!
//! Every event is described by a 7-byte block that lives in the *caller's* RAM:
//!
//! | offset | meaning                                                                   |
//! |--------|---------------------------------------------------------------------------|
//! | `+0,1` | chain pointer (queue link, high byte 0 = end)                              |
//! | `+2`   | pending count (`0` idle, `>0` that many occurrences waiting, `>=0x80` disarmed) |
//! | `+3`   | class: bit7 = asynchronous, bit6 = express, bit0 = *near* routine address; for synchronous events the whole byte is the priority |
//! | `+4,5` | address of the event routine                                               |
//! | `+6`   | ROM select byte (for far routines, class bit0 = 0)                         |
//!
//! Frame-flyback tickers start with a 2-byte chain and then hold an event block
//! (`+2 ..= +8`); ordinary tickers have chain, 16-bit count, 16-bit reload, then the
//! event block at `+6`.

use crate::hw::{Hardware, Regs};
use crate::sysvars::*;
use crate::Cpc;

/// Offsets inside an event block.
pub mod ev {
    pub const CHAIN: u16 = 0;
    pub const COUNT: u16 = 2;
    pub const CLASS: u16 = 3;
    pub const ROUTINE: u16 = 4;
    pub const ROM: u16 = 6;
    /// Class bit flags.
    pub const ASYNC: u8 = 0x80;
    pub const EXPRESS: u8 = 0x40;
    pub const NEAR: u8 = 0x01;
}

/// Bits of [`KL_FLAGS`].
mod fl {
    pub const TICKS_DUE: u8 = 0x01;
    pub const ASYNC_QUEUED: u8 = 0x40;
    pub const BUSY: u8 = 0x80;
}

impl<H: Hardware> Cpc<H> {
    // ===============================================================================
    // Time                                                           (0099 / 00A3)
    // ===============================================================================

    /// `KL TIME PLEASE` (`BD0D`, ROM `0099`).
    /// Exit: the 32-bit count of 1/300 s interrupts as `(DE:HL)` = `(high, low)`.
    pub fn kl_time_please(&self) -> (u16, u16) {
        (self.rd16(KL_TIME + 2), self.rd16(KL_TIME))
    }

    /// `KL TIME SET` (`BD10`, ROM `00A3`).
    /// Entry: new time `(DE:HL)`; also clears the overflow byte above the counter.
    pub fn kl_time_set(&mut self, high: u16, low: u16) {
        self.wr8(KL_TIME_OVERFLOW, 0);
        self.wr16(KL_TIME + 2, high);
        self.wr16(KL_TIME, low);
    }

    // ===============================================================================
    // Interrupt service                                      (00B1, 010A, B939)
    // ===============================================================================

    /// The maskable-interrupt handler (`RST 7` -> `B939` -> this).
    ///
    /// Roughly the logic of `B939`, `00B1` and `010A` without the register/ROM juggling:
    /// 1. [`Self::interrupt_tick`] does the time-critical work (counter, flyback lists,
    ///    sound, keyboard);
    /// 2. if that left work in [`KL_FLAGS`] and event processing is not already
    ///    running (bit 7), [`Self::interrupt_deferred`] runs queued events and tickers.
    pub fn interrupt(&mut self) {
        let was = self.in_interrupt;
        self.in_interrupt = true;
        self.interrupt_tick();
        let f = self.rd8(KL_FLAGS);
        if f != 0 && f & fl::BUSY == 0 {
            self.interrupt_deferred();
        }
        self.in_interrupt = was;
    }

    /// ROM `00B1`: the part of the interrupt that runs with interrupts disabled.
    pub fn interrupt_tick(&mut self) {
        // 32 (+8) bit tick counter: ripple the carry upwards.
        let mut a = KL_TIME;
        loop {
            let v = self.rd8(a).wrapping_add(1);
            self.wr8(a, v);
            a = a.wrapping_add(1);
            if v != 0 { break; }
        }

        // CRTC vsync is bit 0 of PPI port B.  Only on a flyback interrupt do the frame
        // and fast-ticker lists run.
        let vsync = self.hw.inp(crate::hw::PPI_B) & 1 != 0;
        if vsync {
            let head = self.rd16(KL_FRAME_FLY_LIST);
            if head >> 8 != 0 { self.run_event_list(head); }
        }
        let head = self.rd16(KL_FAST_TICKER_LIST);
        if head >> 8 != 0 { self.run_event_list(head); }

        // Sound manager gets a look in on *every* interrupt.
        self.sound_interrupt();

        // Every sixth interrupt (50 Hz) scan the keyboard and arm the ticker list.
        let d = self.rd8(KL_TICK_DIVIDER).wrapping_sub(1);
        self.wr8(KL_TICK_DIVIDER, d);
        if d != 0 { return; }
        self.wr8(KL_TICK_DIVIDER, 6);
        self.keyboard_scan();
        if self.rd16(KL_TICKER_LIST) >> 8 != 0 {
            let f = self.rd8(KL_FLAGS) | fl::TICKS_DUE;
            self.wr8(KL_FLAGS, f);
        }
    }

    /// ROM `010A`: processing that runs with interrupts re-enabled, on the private
    /// interrupt stack (`SP = B187`).
    ///
    /// Alternates between draining the asynchronous event queue and, when the tick bit is
    /// set, running the ticker list (which may queue more events).
    pub fn interrupt_deferred(&mut self) {
        let mut flags = self.rd8(KL_FLAGS);
        // bit 6 clear -> straight to the tickers (L_0139), otherwise start with the queue.
        let mut run_tickers = flags & fl::ASYNC_QUEUED == 0;
        loop {
            if run_tickers {
                // L_0139
                self.wr8(KL_FLAGS, 0);
                let saved = self.in_interrupt;
                self.in_interrupt = true; // SCF / EX AF,AF'
                self.run_tickers();
                self.in_interrupt = saved;
                flags = self.rd8(KL_FLAGS);
                if flags == 0 { break; }
            }
            // L_011B: mark busy, drain the queue
            flags = self.rd8(KL_FLAGS) | fl::BUSY;
            self.wr8(KL_FLAGS, flags);
            loop {
                let head = self.rd16(KL_ASYNC_HEAD);
                if head >> 8 == 0 { break; }
                let next = self.rd16(head); // pop
                self.wr16(KL_ASYNC_HEAD, next);
                self.run_event_now(head + ev::COUNT);
            }
            // L_0132: more tickers due?
            if self.rd8(KL_FLAGS) & fl::TICKS_DUE == 0 { break; }
            run_tickers = true;
        }
        self.wr8(KL_FLAGS, 0);
    }

    // ===============================================================================
    // Frame-fly / fast-ticker / ticker lists                  (0153 .. 01E1)
    // ===============================================================================

    /// ROM `0153`: walk a chain of frame-fly / fast-ticker blocks and `KL EVENT` each one.
    fn run_event_list(&mut self, mut block: u16) {
        loop {
            let next = self.rd16(block);
            // the event block starts two bytes into the list entry
            self.kl_event(block + 2);
            if next >> 8 == 0 { return; }
            block = next;
        }
    }

    /// `KL NEW FRAME FLY` (`BCD7`, ROM `0163`): initialise the embedded event block, then
    /// add it. Entry: `block`, event routine `routine`, ROM select `rom`, class `class`.
    pub fn kl_new_frame_fly(&mut self, block: u16, routine: u16, rom: u8, class: u8) {
        self.kl_init_event(block + 2, routine, rom, class);
        self.kl_add_frame_fly(block);
    }
    /// `KL ADD FRAME FLY` (`BCDA`, ROM `016A`).
    pub fn kl_add_frame_fly(&mut self, block: u16) { self.list_add(KL_FRAME_FLY_LIST, block); }
    /// `KL DEL FRAME FLY` (`BCDD`, ROM `0170`). Returns whether it was on the list.
    pub fn kl_del_frame_fly(&mut self, block: u16) -> bool { self.list_del(KL_FRAME_FLY_LIST, block) }

    /// `KL NEW FAST TICKER` (`BCE0`, ROM `0176`).
    pub fn kl_new_fast_ticker(&mut self, block: u16, routine: u16, rom: u8, class: u8) {
        self.kl_init_event(block + 2, routine, rom, class);
        self.kl_add_fast_ticker(block);
    }
    /// `KL ADD FAST TICKER` (`BCE3`, ROM `017D`).
    pub fn kl_add_fast_ticker(&mut self, block: u16) { self.list_add(KL_FAST_TICKER_LIST, block); }
    /// `KL DEL FAST TICKER` (`BCE6`, ROM `0183`).
    pub fn kl_del_fast_ticker(&mut self, block: u16) -> bool { self.list_del(KL_FAST_TICKER_LIST, block) }

    /// ROM `0189`: one pass over the ticker list (called every 6th interrupt, via
    /// [`Self::interrupt_deferred`]).  A ticker with count 0 is idle; otherwise the count is
    /// decremented and on reaching 0 the event fires and the count is reloaded from the
    /// recharge value.
    fn run_tickers(&mut self) {
        let mut block = self.rd16(KL_TICKER_LIST);
        while block >> 8 != 0 {
            let next = self.rd16(block);
            let count = self.rd16(block + 2);
            if count != 0 {
                let count = count - 1;
                if count != 0 {
                    self.wr16(block + 2, count);
                } else {
                    self.kl_event(block + 6);
                    let recharge = self.rd16(block + 4);
                    self.wr16(block + 2, recharge);
                }
            }
            block = next;
        }
    }

    /// `KL ADD TICKER` (`BCE9`, ROM `01B3`).
    /// Entry: `block` (11 bytes), initial count `initial`, recharge count `recharge`.
    pub fn kl_add_ticker(&mut self, block: u16, initial: u16, recharge: u16) {
        self.wr16(block + 2, initial);
        self.wr16(block + 4, recharge);
        self.list_add(KL_TICKER_LIST, block);
    }

    /// `KL DEL TICKER` (`BCEC`, ROM `01C5`).
    /// Exit: `Some(remaining_count)` if the ticker was running, `None` if it wasn't on the list.
    pub fn kl_del_ticker(&mut self, block: u16) -> Option<u16> {
        if self.list_del(KL_TICKER_LIST, block) { Some(self.rd16(block + 2)) } else { None }
    }

    // ===============================================================================
    // Events                                                         (01D2 .. 02A0)
    // ===============================================================================

    /// `KL INIT EVENT` (`BCEF`, ROM `01D2`): fill in an event block (count 0).
    pub fn kl_init_event(&mut self, event: u16, routine: u16, rom: u8, class: u8) {
        self.wr8(event + ev::COUNT, 0);
        self.wr8(event + ev::CLASS, class);
        self.wr16(event + ev::ROUTINE, routine);
        self.wr8(event + ev::ROM, rom);
    }

    /// `KL EVENT` (`BCF2`, ROM `01E2`): an event has happened.
    ///
    /// * The pending count is bumped. If it would go negative (event disarmed, or 127
    ///   occurrences already waiting) the call is ignored.
    /// * If the count was already non-zero the event is already queued/running: done.
    /// * Synchronous events (class bit 7 clear) go into the priority-ordered sync queue.
    /// * Asynchronous events outside the interrupt run immediately; inside the interrupt
    ///   they are queued (normal) or called at once (express, class bit 6).
    pub fn kl_event(&mut self, event: u16) {
        let count_addr = event + ev::COUNT;
        let old = self.rd8(count_addr);
        let new = old.wrapping_add(1);
        self.wr8(count_addr, new);
        if new & 0x80 != 0 { self.wr8(count_addr, old); return; } // disarmed / saturated
        if old != 0 { return; }                                    // already pending

        let class = self.rd8(event + ev::CLASS);
        if class & ev::ASYNC == 0 {
            self.sync_insert(count_addr, class);
        } else if !self.in_interrupt {
            self.run_event_now(count_addr);                       // L_0209
        } else if class & ev::EXPRESS == 0 {
            self.async_enqueue(event);                            // L_00E8
        } else {
            // Express event: call the near routine immediately, count untouched.
            let addr = self.rd16(event + ev::ROUTINE);
            let mut r = Regs { hl: addr, de: addr, ..Regs::default() };
            self.call_routine(addr, &mut r);
        }
    }

    /// Call the routine at `addr` the way the Z80 `CALL` would: routines of the lower ROM that
    /// this crate implements (the sound event at `1F03`, the ink-flash event at `0D5B`) run
    /// as Rust; anything else is machine code handed to [`Hardware::call`].
    pub(crate) fn call_routine(&mut self, addr: u16, regs: &mut Regs) {
        match addr {
            0x1F03 if self.banking.lower_rom => self.sound_event(),
            0x0D5B if self.banking.lower_rom => self.scr_flash_tick(),
            _ => self.hw.call(addr, regs),
        }
    }

    /// ROM `020A`: run an event routine once per pending count. `count_addr = event + 2`.
    fn run_event_now(&mut self, count_addr: u16) {
        if self.rd8(count_addr) & 0x80 != 0 { return; }
        loop {
            self.kl_do_sync(count_addr - ev::COUNT);
            let c = self.rd8(count_addr).wrapping_sub(1);
            self.wr8(count_addr, c);
            if c == 0 { return; }
            if c & 0x80 != 0 { self.wr8(count_addr, c.wrapping_add(1)); return; }
        }
    }

    /// ROM `00E8`: append an asynchronous event to the queue the interrupt exit drains.
    fn async_enqueue(&mut self, event: u16) {
        self.wr8(event + 1, 0); // end-of-chain marker
        if self.rd8(KL_ASYNC_HEAD + 1) != 0 {
            let tail = self.rd16(KL_ASYNC_TAIL);
            self.wr16(KL_ASYNC_TAIL, event);
            self.wr16(tail, event);
        } else {
            self.wr16(KL_ASYNC_HEAD, event);
            self.wr16(KL_ASYNC_TAIL, event);
            let f = self.rd8(KL_FLAGS) | fl::ASYNC_QUEUED;
            self.wr8(KL_FLAGS, f);
        }
    }

    /// `KL DO SYNC` (`BCFE`, ROM `021A`): call the routine of the event at `event`.
    /// Class bit 0 set = near routine, clear = far (routine + ROM select byte).
    pub fn kl_do_sync(&mut self, event: u16) {
        let class = self.rd8(event + ev::CLASS);
        let addr = self.rd16(event + ev::ROUTINE);
        let mut r = Regs { hl: addr, de: addr, ..Regs::default() };
        r.set_a(class >> 1);
        if class & ev::NEAR != 0 {
            self.call_routine(addr, &mut r);
        } else {
            let rom = self.rd8(event + ev::ROM);
            self.hw.far_call(addr, rom, &mut r);
        }
    }

    /// `KL DISARM EVENT` (`BD0A`, ROM `028E`): set count to `0xC0` so the event is ignored.
    pub fn kl_disarm_event(&mut self, event: u16) { self.wr8(event + ev::COUNT, 0xC0); }

    // ---- synchronous queue -------------------------------------------------------

    /// `KL SYNC RESET` (`BCF5`, ROM `0228`): empty the sync queue, priority 0.
    pub fn kl_sync_reset(&mut self) {
        self.wr8(KL_SYNC_HEAD + 1, 0);
        self.wr8(KL_SYNC_PRIORITY, 0);
    }

    /// ROM `022F`: insert `event` (given as its count address) in descending class order.
    /// An event is placed *before* the first queued event of strictly lower class.
    /// (`KL DONE SYNC` re-enters here with the *restored priority* as the class, which
    /// effectively appends the event at the tail.)
    fn sync_insert(&mut self, count_addr: u16, class: u8) {
        let event = count_addr - ev::COUNT;
        // `link` is the address of the pointer to patch.
        let mut link = KL_SYNC_HEAD;
        loop {
            let node = self.rd16(link);
            if node >> 8 == 0 { break; }
            if self.rd8(node + ev::CLASS) < class { break; }
            link = node;
        }
        let next = self.rd16(link);
        self.wr16(event, next);
        self.wr16(link, event);
    }

    /// `KL NEXT SYNC` (`BCFB`, ROM `0256`): fetch the next runnable synchronous event.
    /// Exit: `Some((event, previous_priority))` – pass the priority back to
    /// [`Self::kl_done_sync`]. `None` if the queue is empty or the head event is not of
    /// higher priority than the one currently running.
    pub fn kl_next_sync(&mut self) -> Option<(u16, u8)> {
        let head = self.rd16(KL_SYNC_HEAD);
        if head >> 8 == 0 { return None; }
        let cur = self.rd8(KL_SYNC_PRIORITY);
        let class = self.rd8(head + ev::CLASS);
        if cur >= class { return None; }
        let next = self.rd16(head);
        self.wr8(KL_SYNC_PRIORITY, class);
        self.wr16(KL_SYNC_HEAD, next);
        Some((head, cur))
    }

    /// `KL DONE SYNC` (`BD01`, ROM `0277`): finished running an event; restore `priority`
    /// and, if more occurrences are pending, queue it again.
    pub fn kl_done_sync(&mut self, event: u16, priority: u8) {
        self.wr8(KL_SYNC_PRIORITY, priority);
        let count_addr = event + ev::COUNT;
        let c = self.rd8(count_addr).wrapping_sub(1);
        self.wr8(count_addr, c);
        if c == 0 { return; }
        if c & 0x80 == 0 {
            self.sync_insert(count_addr, priority);
        } else {
            self.wr8(count_addr, c.wrapping_add(1));
        }
    }

    /// `KL DEL SYNCHRONOUS` (`BCF8`, ROM `0285`): disarm and remove from the sync queue.
    pub fn kl_del_synchronous(&mut self, event: u16) {
        self.kl_disarm_event(event);
        self.list_del(KL_SYNC_HEAD, event);
    }

    /// `KL EVENT DISABLE` (`BD04`, ROM `0295`): stop synchronous events being dispatched.
    pub fn kl_event_disable(&mut self) {
        let p = self.rd8(KL_SYNC_PRIORITY) | 0x20;
        self.wr8(KL_SYNC_PRIORITY, p);
    }
    /// `KL EVENT ENABLE` (`BD07`, ROM `029B`).
    pub fn kl_event_enable(&mut self) {
        let p = self.rd8(KL_SYNC_PRIORITY) & !0x20;
        self.wr8(KL_SYNC_PRIORITY, p);
    }

    /// `KL POLL SYNCHRONOUS` (`B921`, ROM `03B2`): is a synchronous event of higher
    /// priority than the running one waiting?
    pub fn kl_poll_synchronous(&self) -> bool {
        let head = self.rd16(KL_SYNC_HEAD);
        if head >> 8 == 0 { return false; }
        self.rd8(KL_SYNC_PRIORITY) < self.rd8(head + ev::CLASS)
    }

    // ---- singly linked list helpers ------------------------------------------------

    /// ROM `0363`: search the chain starting at the *pointer cell* `list` for `block`.
    /// Returns the address of the pointer cell that holds `block`.
    fn list_find(&self, list: u16, block: u16) -> Option<u16> {
        let mut cell = list;
        loop {
            let v = self.rd16(cell);
            if v == block { return Some(cell); }
            if v >> 8 == 0 { return None; }
            cell = v;
        }
    }

    /// ROM `0373`: append `block` to the chain unless it is already there.
    fn list_add(&mut self, list: u16, block: u16) {
        if self.list_find(list, block).is_some() { return; }
        // walk to the last cell
        let mut cell = list;
        while self.rd16(cell) >> 8 != 0 { cell = self.rd16(cell); }
        self.wr16(cell, block);
        self.wr8(block + 1, 0);
    }

    /// ROM `0382`: unlink `block`; returns whether it was present.
    fn list_del(&mut self, list: u16, block: u16) -> bool {
        match self.list_find(list, block) {
            Some(cell) => { let n = self.rd16(block); self.wr16(cell, n); true }
            None => false,
        }
    }

    // ===============================================================================
    // Cold-start support                                           (005C, 0044)
    // ===============================================================================

    /// `KL CHOKE OFF` (`BCC8`, ROM `005C`): wipe the event/kernel variables (`B100-B1BF`)
    /// ready for a new foreground program.
    /// Exit: `(de, b, c)` – entry address of the old foreground program in `DE`, its
    /// ROM select in `B`; `C = 0xFF` unless the program was in RAM (`B = 0xFF`) in which
    /// case `DE = C = 0`.
    pub fn kl_choke_off(&mut self) -> (u16, u8, u8) {
        let rom = self.rd8(KL_ENTRY_ROM);
        let entry = self.rd16(KL_ENTRY_ADDR);
        for a in 0..0xC0u16 { self.wr8(0xB100 + a, 0); }
        if rom ^ 0xFF != 0 { (entry, rom, 0xFF) } else { (0, rom, 0) }
    }

    // ===============================================================================
    // ROM extensions                                               (02A1 .. 0360)
    // ===============================================================================

    /// `KL LOG EXT` (`BCD1`, ROM `02A1`): link the 4-byte RAM block `ram_block` into the
    /// list of ROM extensions. `value` (BC) is stored in the block; for ROM sets
    /// it holds the ROM number. Exit: `ram_block` (HL preserved).
    pub fn kl_log_ext(&mut self, ram_block: u16, value: u16) {
        let old = self.rd16(KL_ROM_CHAIN);
        self.wr16(KL_ROM_CHAIN, ram_block);
        self.wr16(ram_block, old);
        self.wr16(ram_block + 2, value);
    }

    /// `KL ROM WALK` (`BCCB`, ROM `0329`): initialise every background ROM (7 down to 1).
    /// Entry/Exit: the `(de, hl)` memory limits are threaded through each ROM's init.
    pub fn kl_rom_walk(&mut self, mut de: u16, mut hl: u16) -> (u16, u16) {
        for c in (1..=7u8).rev() {
            let r = self.kl_init_back(c, de, hl);
            de = r.0; hl = r.1;
        }
        (de, hl)
    }

    /// `KL INIT BACK` (`BCCE`, ROM `0332`): if upper ROM `c` is a *background* ROM
    /// (class byte `& 3 == 1`) call its init routine at `C006` with the free-memory range
    /// `(de = lowest, hl = highest)` and record where it left the top of memory.
    /// Exit: updated `(de, hl)`.
    pub fn kl_init_back(&mut self, c: u8, de: u16, hl: u16) -> (u16, u16) {
        if c >= 8 { return (de, hl); }
        let saved = self.kl_rom_select(c);
        let mut out = (de, hl);
        if self.read(0xC000) & 3 == 1 {
            let mut r = Regs { de, hl, ..Regs::default() };
            self.hw.far_call(0xC006, c, &mut r);
            let (new_de, new_hl) = (r.de, r.hl);
            // remember the ROM's top-of-memory (workspace pointer handed out in IY)
            self.wr16(KL_ROM_TABLE + 2 * c as u16, new_hl.wrapping_add(1));
            // the ROM reserved the four bytes below it for its chain block
            let block = new_hl.wrapping_add(1).wrapping_sub(4);
            self.kl_log_ext(block, c as u16);
            out = (new_de, block.wrapping_sub(1));
        }
        self.kl_rom_deselect(saved);
        out
    }

    /// `KL FIND COMMAND` (`BCD4`, ROM `02B2`): look a command name up in the logged
    /// extension ROMs, then in every foreground ROM.
    ///
    /// `name` is the command with the last character's bit 7 set (as stored in ROM
    /// tables). Exit: `Some((routine_address, rom_select))`.  A hit in a *foreground* ROM
    /// does not return: the firmware jumps straight into `MC START PROGRAM`; the Rust
    /// port reports it as `FoundCommand::Foreground`.
    pub fn kl_find_command(&mut self, name: &[u8]) -> FoundCommand {
        // copy (up to 16 bytes of) the name to the scratch buffer, force a terminator
        for (i, b) in name.iter().take(16).enumerate() { self.wr8(KL_COMMAND_NAME + i as u16, *b); }
        let l = name.len().min(16);
        let last = KL_COMMAND_NAME + l as u16 - 1;
        let v = self.rd8(last) | 0x80;
        self.wr8(last, v);

        // 1. RAM-logged ROMs
        let mut blk = self.rd16(KL_ROM_CHAIN);
        while blk != 0 {
            let bc = self.rd16(blk + 2);
            if let Some(hit) = self.search_command_table(bc) { return FoundCommand::Extension(hit.0, hit.1); }
            blk = self.rd16(blk);
        }
        // 2. foreground ROMs (class byte & 3 == 0); stop at the first absent ROM
        let mut c: u8 = 0xFF;
        loop {
            c = c.wrapping_add(1);
            let saved = self.kl_rom_select(c);
            let class = self.read(0xC000);
            let mut found = None;
            if class & 3 == 0 {
                found = self.search_command_table(c as u16 & 0xFF).map(|h| h);
            }
            self.kl_rom_deselect(saved);
            if let Some((addr, rom)) = found { return FoundCommand::Foreground { entry: addr, rom }; }
            if class & 0x80 != 0 && c != 0 { return FoundCommand::NotFound; }
            if c == 255 { return FoundCommand::NotFound; }
        }
    }

    /// ROM `02F4`: search one ROM's command table. `bc` high byte 0 = ROM number in the low
    /// byte (table pointer at `C004`); otherwise `bc` is the address of the name table.
    fn search_command_table(&mut self, bc: u16) -> Option<(u16, u8)> {
        let (rom_sel, table_ptr_addr) = if bc >> 8 == 0 { (bc as u8, 0xC004) } else { (0xFF, bc) };
        let saved = if rom_sel != 0xFF { Some(self.kl_rom_select(rom_sel)) } else { None };
        // The table starts with a pointer to the names; names and jump entries are parallel.
        let mut names = self.read16(table_ptr_addr);
        let mut entry = names.wrapping_add(0); // `DE` walks the 3-byte jump entries below
        // layout: names (bit7-terminated strings, ending with a 0 byte), then jumps.
        // The ROM keeps (HL = name list, DE = jump table); the jump table follows the pointer.
        let mut jumps = table_ptr_addr.wrapping_add(2);
        let mut result = None;
        let mut p = names;
        'outer: loop {
            if self.read(p) == 0 { break; }
            // compare against the command in the scratch buffer
            let mut q = KL_COMMAND_NAME;
            let mut matched = true;
            loop {
                let a = self.rd8(q);
                let b = self.read(p);
                if a != b { matched = false; break; }
                p = p.wrapping_add(1);
                q = q.wrapping_add(1);
                if a & 0x80 != 0 { break; }
            }
            if matched { result = Some((jumps, rom_sel)); break 'outer; }
            // skip the rest of this name
            loop {
                let ch = self.read(p);
                p = p.wrapping_add(1);
                if ch & 0x80 != 0 { break; }
            }
            jumps = jumps.wrapping_add(3);
        }
        let _ = (&mut names, &mut entry);
        if let Some(s) = saved { self.kl_rom_deselect(s); }
        result
    }
}

/// Result of [`Cpc::kl_find_command`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FoundCommand {
    /// Not a known command.
    NotFound,
    /// RSX in a background / extension ROM: call `routine` in ROM `rom`.
    Extension(u16, u8),
    /// Name of a foreground ROM: the firmware starts it via `MC START PROGRAM`.
    Foreground { entry: u16, rom: u8 },
}

// ===================================================================================
// RAM-resident ROM banking helpers  (ROM 0391-057F copied to B900-BAE8)
// ===================================================================================

/// Snapshot of the ROM-enable bits so they can be restored (`KL ROM RESTORE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RomState(pub u8);

/// Return value of [`Cpc::kl_rom_select`]: the previous upper ROM number and ROM state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SavedRom { pub rom: u8, pub state: RomState }

impl<H: Hardware> Cpc<H> {
    fn write_mode_rom(&mut self) {
        let v = self.banking.mode_rom_register;
        self.hw.out(crate::hw::GATE_ARRAY | 0x7F, v);
        // (the real code does `OUT (C),C` with B = 7F, so the port high byte is 7F)
    }

    fn current_state(&self) -> RomState { RomState(self.banking.mode_rom_register & 0x0C) }

    fn set_rom_bits(&mut self, lower_off: bool, upper_off: bool) {
        let mut v = self.banking.mode_rom_register & !0x0C;
        if lower_off { v |= 0x04; }
        if upper_off { v |= 0x08; }
        self.banking.mode_rom_register = v;
        self.banking.lower_rom = !lower_off;
        self.banking.upper_rom = !upper_off;
        self.write_mode_rom();
    }

    /// `KL U ROM ENABLE` (`B900` -> `BA5E`). Exit: previous state for [`Self::kl_rom_restore`].
    pub fn kl_u_rom_enable(&mut self) -> RomState {
        let s = self.current_state();
        let lower_off = !self.banking.lower_rom;
        self.set_rom_bits(lower_off, false);
        s
    }
    /// `KL U ROM DISABLE` (`B903` -> `BA68`).
    pub fn kl_u_rom_disable(&mut self) -> RomState {
        let s = self.current_state();
        let lower_off = !self.banking.lower_rom;
        self.set_rom_bits(lower_off, true);
        s
    }
    /// `KL L ROM ENABLE` (`B906` -> `BA4A`).
    pub fn kl_l_rom_enable(&mut self) -> RomState {
        let s = self.current_state();
        let upper_off = !self.banking.upper_rom;
        self.set_rom_bits(false, upper_off);
        s
    }
    /// `KL L ROM DISABLE` (`B909` -> `BA54`).
    pub fn kl_l_rom_disable(&mut self) -> RomState {
        let s = self.current_state();
        let upper_off = !self.banking.upper_rom;
        self.set_rom_bits(true, upper_off);
        s
    }
    /// `KL ROM RESTORE` (`B90C` -> `BA72`): put back both ROM-enable bits from `s`.
    pub fn kl_rom_restore(&mut self, s: RomState) {
        self.set_rom_bits(s.0 & 0x04 != 0, s.0 & 0x08 != 0);
    }

    /// `KL ROM SELECT` (`B90F` -> `BA7E`): enable the upper ROM and switch to number `c`.
    /// Exit: what is needed to undo it with [`Self::kl_rom_deselect`].
    pub fn kl_rom_select(&mut self, c: u8) -> SavedRom {
        let state = self.kl_u_rom_enable();
        let prev = self.rd8(KL_CURR_ROM);
        self.banking.upper_rom_number = c;
        self.hw.out(crate::hw::ROM_SELECT, c);
        self.wr8(KL_CURR_ROM, c);
        SavedRom { rom: prev, state }
    }

    /// `KL CURR SELECTION` (`B912` -> `BAA2`).
    pub fn kl_curr_selection(&self) -> u8 { self.rd8(KL_CURR_ROM) }

    /// `KL PROBE ROM` (`B915` -> `BA83`): `(class byte at C000, version word at C001)` of ROM `c`.
    pub fn kl_probe_rom(&mut self, c: u8) -> (u8, u16) {
        let saved = self.kl_rom_select(c);
        let class = self.read(0xC000);
        let ver = self.read16(0xC001);
        self.kl_rom_deselect(saved);
        (class, ver)
    }

    /// `KL ROM DESELECT` (`B918` -> `BA8C`): undo [`Self::kl_rom_select`].
    pub fn kl_rom_deselect(&mut self, s: SavedRom) {
        self.kl_rom_restore(s.state);
        self.banking.upper_rom_number = s.rom;
        self.hw.out(crate::hw::ROM_SELECT, s.rom);
        self.wr8(KL_CURR_ROM, s.rom);
    }

    /// `KL LDIR` (`B91B` -> `BAA6`): block copy with *both* ROMs switched out, so it can
    /// copy RAM that is normally hidden under them (and write to it).
    pub fn kl_ldir(&mut self, mut hl: u16, mut de: u16, mut bc: u16) {
        let s = self.current_state();
        self.set_rom_bits(true, true);
        // (a Z80 LDIR with BC = 0 copies 65536 bytes)
        loop {
            let v = self.rd8(hl);
            self.wr8(de, v);
            hl = hl.wrapping_add(1);
            de = de.wrapping_add(1);
            bc = bc.wrapping_sub(1);
            if bc == 0 { break; }
        }
        self.kl_rom_restore(s);
    }
    /// `KL LDDR` (`B91E` -> `BAAC`): backwards copy, both ROMs out.
    pub fn kl_lddr(&mut self, mut hl: u16, mut de: u16, mut bc: u16) {
        let s = self.current_state();
        self.set_rom_bits(true, true);
        loop {
            let v = self.rd8(hl);
            self.wr8(de, v);
            hl = hl.wrapping_sub(1);
            de = de.wrapping_sub(1);
            bc = bc.wrapping_sub(1);
            if bc == 0 { break; }
        }
        self.kl_rom_restore(s);
    }
    /// `KL RAM LAM` (`RST 4` -> `BACB`): read a byte from RAM at `hl` regardless of ROMs.
    /// Like the real routine it briefly switches both ROMs off at the gate array (interrupts
    /// are disabled meanwhile) and then writes the old mode/ROM byte back.
    pub fn kl_ram_lam(&mut self, hl: u16) -> u8 {
        let v = self.banking.mode_rom_register;
        self.hw.out(crate::hw::GATE_ARRAY | 0x7F, v | 0x0C);
        let byte = self.rd8(hl);
        self.hw.out(crate::hw::GATE_ARRAY | 0x7F, v);
        byte
    }

    // ---- control-transfer primitives (RST 1,2,3,5) -----------------------------------

    /// Decode the ROM-configuration bits of a `LOW JUMP` / `SIDE CALL` address word:
    /// bit 15 = disable upper ROM, bit 14 = disable lower ROM.  Returns `(address, lower_off, upper_off)`.
    fn decode_low_address(w: u16) -> (u16, bool, bool) {
        (w & 0x3FFF, w & 0x4000 != 0, w & 0x8000 != 0)
    }

    /// `RST 1` / `KL LOW JUMP` (`0008` -> `B982`) and `KL LOW PCHL` (`000B` -> `B97C`):
    /// run the routine at `word & 0x3FFF` with the ROM configuration given by bits 14-15
    /// of `word`, then restore the previous ROM bits (the screen-mode bits are *kept* as
    /// the routine left them).
    pub fn rst_low_jump(&mut self, word: u16, regs: &mut Regs) {
        let (addr, lower_off, upper_off) = Self::decode_low_address(word);
        let old = self.current_state();
        self.set_rom_bits(lower_off, upper_off);
        self.hw.call(addr, regs);
        self.kl_rom_restore(old);
    }

    /// `RST 3` / `KL FAR CALL` (`0018` -> `B9BF`): `rom >= 0xFC` encodes a ROM
    /// configuration (`FC` both on, `FD` lower off, `FE` upper off, `FF` both off);
    /// otherwise upper ROM `rom` is selected, the lower ROM is switched *off*, and
    /// IY is loaded with that ROM's workspace pointer from [`KL_ROM_TABLE`] (ROMs 1-7).
    pub fn rst_far_call(&mut self, addr: u16, rom: u8, regs: &mut Regs) {
        let old = self.current_state();
        if rom >= 0xFC {
            self.set_rom_bits(rom & 1 != 0, rom & 2 != 0);
            self.hw.call(addr, regs);
            self.kl_rom_restore(old);
        } else {
            let prev_sel = self.rd8(KL_CURR_ROM);
            self.hw.out(crate::hw::ROM_SELECT, rom);
            self.wr8(KL_CURR_ROM, rom);
            self.banking.upper_rom_number = rom;
            if (1..=7).contains(&rom) {
                regs.iy = self.rd16(KL_ROM_TABLE + 2 * rom as u16);
            }
            self.set_rom_bits(true, false);
            self.hw.call(addr, regs);
            self.kl_rom_restore(old);
            self.hw.out(crate::hw::ROM_SELECT, prev_sel);
            self.wr8(KL_CURR_ROM, prev_sel);
            self.banking.upper_rom_number = prev_sel;
        }
    }

    /// `RST 5` / `KL FIRM JUMP` (`0028` -> `BA2E`): call with the lower ROM enabled, then
    /// disable it again.
    pub fn rst_firm_jump(&mut self, addr: u16, regs: &mut Regs) {
        let upper_off = !self.banking.upper_rom;
        self.set_rom_bits(false, upper_off);
        self.hw.call(addr, regs);
        let upper_off = !self.banking.upper_rom;
        self.set_rom_bits(true, upper_off);
    }
}
