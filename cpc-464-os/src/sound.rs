//! # Sound manager  (ROM `1E68 .. 2363`)
//!
//! Three independent channels (A, B, C = PSG channels 0-2).  Each channel has a 63-byte
//! control block, a queue of four pending notes, an amplitude-envelope state machine and
//! a tone-envelope state machine.  Envelope steps are timed in 1/100 s by
//! [`Cpc::sound_interrupt`] (called from the 300 Hz interrupt), which raises an
//! *asynchronous event* ([`Cpc::sound_event`], ROM `1F03`) whenever a step is due.
//!
//! ## Channel block (63 bytes, `B55C` A / `B59B` B / `B5DA` C)
//!
//! | Off | Meaning |
//! |-----|---------|
//! | 0   | channel number 0-2 |
//! | 1   | channel bit (1, 2, 4) |
//! | 2   | mixer "noise off" bit (8, 16, 32) |
//! | 3   | status: bits 0-2 waiting for rendezvous with those channels, 3 held, 4 playing |
//! | 4,5 | tone-envelope step due flag (bit 7 → bit 0 when due) / countdown |
//! | 6,7 | amplitude-envelope countdown / step due flag |
//! | 8,9 | remaining duration (>0) or repeat count (<0, bit 15 set) |
//! | 10,11 | amplitude envelope start |
//! | 12  | amplitude envelope sections left |
//! | 13,14 | amplitude envelope: pointer to the current step-size byte |
//! | 15  | current volume (bit 7 set until the first step is applied) |
//! | 16  | amplitude steps left in this section (`FF` = finished) |
//! | 17,18 | tone envelope start |
//! | 19  | tone envelope sections left |
//! | 20,21 | tone envelope pointer |
//! | 22,23 | current tone period |
//! | 24  | tone steps left in this section |
//! | 25  | queue head index; 26 = entries queued; 27 = tail index; 28 = free entries |
//! | 29,30 | event block to raise when a queue slot frees (30 = 0: none) |
//! | 31.. | 4 queue entries of 8 bytes |
//!
//! Queue entry: `[0]` rendezvous channels (bits 0-2, own bit removed) + hold (bit 3);
//! `[1]` amplitude envelope << 4 | tone envelope; `[2,3]` tone period; `[4]` noise period;
//! `[5]` initial volume; `[6,7]` duration (0 = forever, negative = repeat count).
//!
//! ## Envelopes (16 bytes each)
//!
//! `B60A + 16*n` amplitude, `B6FA + 16*n` tone (n = 1..15): first byte = number of
//! sections (tone: bit 7 = repeat), then 3-byte sections `(step count, step size, pause)`
//! with pause in 1/100 s.  A step count ≥ 0x80 in an amplitude envelope programs the
//! hardware envelope generator instead (shape = step count, period = size/pause); a tone
//! step count ≥ 0xF0 sets an absolute period.
//!
//! ## Global state
//!
//! `B550` channels whose note finished during event processing; `B551` channels that just
//! started (or were held); `B552` channels active; `B553` 3-interrupt divider (→ 100 Hz);
//! `B554` events pending; `B555` the event block; `B619` shadow of PSG mixer register 7.

use crate::hw::Hardware;
use crate::Cpc;

const SND_PENDING_END: u16 = 0xB550;
const SND_STARTED: u16 = 0xB551;
const SND_ACTIVE: u16 = 0xB552;
const SND_DIVIDER: u16 = 0xB553;
const SND_EVENTS: u16 = 0xB554;
const SND_EVENT_BLOCK: u16 = 0xB555;
const SND_MIXER: u16 = 0xB619;
/// Base such that channel *n* (A=1,B=2,C=3 for the shift loop) is at `+ 0x3F * n`.
const SND_CH_BASE: u16 = 0xB51D;
const SND_AMP_ENVELOPES: u16 = 0xB60A;
const SND_TONE_ENVELOPES: u16 = 0xB6FA;
const CH_SIZE: u16 = 0x3F;

/// ROM `21B2`: the default amplitude envelope `[1 section] [1 step, size 0, pause 200]`.
const DEFAULT_ENVELOPE_ADDR: u16 = 0x21B2;
const DEFAULT_ENVELOPE: [u8; 4] = [0x01, 0x01, 0x00, 0xC8];

// channel block offsets
const NUMBER: u16 = 0;
const MASK: u16 = 1;
const NOISE_BIT: u16 = 2;
const STATUS: u16 = 3;
const TONE_FLAG: u16 = 4;
const TONE_COUNT: u16 = 5;
const AMP_COUNT: u16 = 6;
const AMP_FLAG: u16 = 7;
const DURATION: u16 = 8;
const AMP_START: u16 = 10;
const AMP_SECTIONS: u16 = 12;
const AMP_PTR: u16 = 13;
const VOLUME: u16 = 15;
const AMP_STEPS: u16 = 16;
const TONE_START: u16 = 17;
const TONE_SECTIONS: u16 = 19;
const TONE_PTR: u16 = 20;
const PERIOD: u16 = 22;
const TONE_STEPS: u16 = 24;
const Q_HEAD: u16 = 25;
const Q_COUNT: u16 = 26;
const Q_TAIL: u16 = 27;
const Q_FREE: u16 = 28;
const EVENT: u16 = 29;
const QUEUE: u16 = 31;

/// Address of channel block `n` (0, 1, 2).
fn ch_addr(n: u8) -> u16 { SND_CH_BASE + CH_SIZE * (n as u16 + 1) }

/// Channel blocks selected by a channel bit mask (bit 0 = A ...).
fn channels_in(mask: u8) -> impl Iterator<Item = u16> {
    (0..3u8).filter(move |n| mask & (1 << n) != 0).map(ch_addr)
}

impl<H: Hardware> Cpc<H> {
    #[inline] fn c8(&self, ix: u16, off: u16) -> u8 { self.rd8(ix + off) }
    #[inline] fn set_c8(&mut self, ix: u16, off: u16, v: u8) { self.wr8(ix + off, v); }

    /// Read a byte of an envelope or default-envelope data (the default lives in ROM).
    fn env_byte(&self, addr: u16) -> u8 {
        if (DEFAULT_ENVELOPE_ADDR..DEFAULT_ENVELOPE_ADDR + 4).contains(&addr) {
            DEFAULT_ENVELOPE[(addr - DEFAULT_ENVELOPE_ADDR) as usize]
        } else {
            self.rd8(addr)
        }
    }

    // ===============================================================================
    // Reset                                                            (1E68 .. 1ECA)
    // ===============================================================================

    /// `SOUND RESET` (`BCA7`, ROM `1E68`): silence everything, create the three channel
    /// blocks and register the sound event.
    pub fn sound_reset(&mut self) {
        self.wr8(SND_ACTIVE, 0);
        self.wr8(SND_STARTED, 0);
        self.kl_init_event(SND_EVENT_BLOCK, 0x1F03, 0, 0x81);
        self.wr8(SND_MIXER, 0x3F);                       // all tone and noise off
        let (mut tone_bit, mut noise_bit) = (1u8, 8u8);
        for n in 0..3u8 {
            let ix = ch_addr(n);
            self.set_c8(ix, NUMBER, n);
            self.set_c8(ix, MASK, tone_bit);
            self.set_c8(ix, NOISE_BIT, noise_bit);
            tone_bit <<= 1;
            noise_bit <<= 1;
        }
        self.sound_flush(0x07);
    }

    /// ROM `1E9A`: re-initialise (flush) the channels in `mask`: silence them, remove
    /// them from the active set, clear their queues and state.
    fn sound_flush(&mut self, mask: u8) {
        for ix in channels_in(mask).collect::<Vec<_>>() {
            self.sound_stop_channel(ix);                       // L_227F
            for off in STATUS..=(CH_SIZE - 1) { self.set_c8(ix, off, 0); }
            self.set_c8(ix, Q_FREE, 4);
        }
    }

    // ===============================================================================
    // Hold / continue                                                  (1ECB .. 1F00)
    // ===============================================================================

    /// `SOUND HOLD` (`BCB6`, ROM `1ECB`): stop all sound, remembering which channels were
    /// active.  `true` if anything was active.
    pub fn sound_hold(&mut self) -> bool {
        let active = self.rd8(SND_ACTIVE);
        self.wr8(SND_ACTIVE, 0);
        if active == 0 { return false; }
        self.wr8(SND_STARTED, active);
        for reg in [10u8, 9, 8] { self.mc_sound_register(reg, 0); }
        true
    }

    /// `SOUND CONTINUE` (`BCBC`, ROM `1EE6`): restart channels stopped by
    /// [`Self::sound_hold`] by rewriting their volumes.
    pub fn sound_continue(&mut self) {
        let held = self.rd8(SND_STARTED);
        if held == 0 { return; }
        for ix in channels_in(held).collect::<Vec<_>>() {
            let v = self.c8(ix, VOLUME);
            self.sound_set_volume_reg(ix, v);                  // L_2276
        }
        self.sound_note_started();                              // L_201E
    }

    // ===============================================================================
    // Interrupt and event                                              (1F03 .. 1F9E)
    // ===============================================================================

    /// ROM `1F61`: called from the 300 Hz interrupt.  Every third call (100 Hz) the
    /// envelope countdowns of all active channels are decremented; steps that fall due are
    /// flagged and the sound event is raised once for the batch.
    pub fn sound_interrupt(&mut self) {
        let active = self.rd8(SND_ACTIVE);
        if active == 0 { return; }
        let d = self.rd8(SND_DIVIDER).wrapping_sub(1);
        self.wr8(SND_DIVIDER, d);
        if d != 0 { return; }
        self.wr8(SND_DIVIDER, 1);
        if self.rd8(SND_EVENTS) != 0 { return; }               // previous event not finished
        self.wr8(SND_DIVIDER, 3);
        let mut due = 0u8;
        for ix in channels_in(active).collect::<Vec<_>>() {
            let t = self.c8(ix, TONE_COUNT).wrapping_sub(1);
            self.set_c8(ix, TONE_COUNT, t);
            if t == 0 {
                let f = self.c8(ix, TONE_FLAG);
                self.set_c8(ix, TONE_FLAG, f.rotate_left(1));
                due += f >> 7;
            }
            let a = self.c8(ix, AMP_COUNT).wrapping_sub(1);
            self.set_c8(ix, AMP_COUNT, a);
            if a == 0 {
                let f = self.c8(ix, AMP_FLAG);
                self.set_c8(ix, AMP_FLAG, f.rotate_left(1));
                due += f >> 7;
            }
        }
        if due == 0 { return; }
        self.wr8(SND_EVENTS, due);
        self.kl_event(SND_EVENT_BLOCK);
    }

    /// The sound event routine (ROM `1F03`, near, asynchronous): perform every envelope
    /// step flagged by [`Self::sound_interrupt`], start queued notes on channels whose
    /// note ended, and deactivate channels left with nothing to do.
    pub fn sound_event(&mut self) {
        self.wr8(SND_PENDING_END, 0);
        let b551_at_entry = self.rd8(SND_STARTED);
        let active = self.rd8(SND_ACTIVE);
        for ix in channels_in(active).collect::<Vec<_>>() {
            if self.c8(ix, TONE_FLAG) & 1 != 0 { self.sound_tone_step(ix, true); }
            if self.c8(ix, AMP_FLAG) & 1 != 0 {
                let finished = self.sound_amp_step(ix, true);
                if finished { self.sound_note_ended(ix); }
            }
        }
        let ended = self.rd8(SND_PENDING_END);
        if ended != 0 {
            let started_now = self.rd8(SND_STARTED);
            self.wr8(SND_STARTED, b551_at_entry);
            let new = started_now ^ b551_at_entry;
            let act = self.rd8(SND_ACTIVE) | new;
            self.wr8(SND_ACTIVE, act);
            let idle = !new & ended;                            // ended but did not start again
            for ix in channels_in(idle).collect::<Vec<_>>() { self.sound_stop_channel(ix); }
        }
        self.wr8(SND_EVENTS, 0);
    }

    /// ROM `20A8`: a note ended.  Silence the channel if nothing is queued, otherwise mark
    /// it and start the next entry.
    fn sound_note_ended(&mut self, ix: u16) {
        if self.c8(ix, Q_COUNT) == 0 {
            self.sound_stop_channel(ix);
            return;
        }
        let m = self.c8(ix, MASK);
        let v = self.rd8(SND_PENDING_END) | m;
        self.wr8(SND_PENDING_END, v);
        self.sound_start_head(ix);
    }

    // ===============================================================================
    // Queueing                                                         (1F9F .. 2088)
    // ===============================================================================

    /// `SOUND QUEUE` (`BCAA`, ROM `1F9F`): queue a note described by the 9-byte block
    /// `[channel status, amplitude envelope, tone envelope, tone period (2), noise,
    /// volume, duration (2)]`.  Channel status: bits 0-2 channels, 3-5 rendezvous with
    /// A/B/C, 6 hold, 7 flush.  Returns `false` if a selected queue is full (nothing queued).
    pub fn sound_queue(&mut self, params: &[u8; 9]) -> bool {
        self.sound_continue();
        let status = params[0];
        let channels = status & 0x07;
        if channels == 0 { return true; }
        if status & 0x80 != 0 { self.sound_flush(channels); }
        let targets: Vec<u16> = channels_in(channels).collect();
        let mut full = false;
        for &ix in &targets {
            self.set_c8(ix, EVENT + 1, 0);                      // forget any pending event
            if self.c8(ix, Q_FREE) == 0 { full = true; }
        }
        if full { return false; }

        let flags = ((status >> 3) | channels) & 0x0F;
        for ix in targets {
            let tail = self.c8(ix, Q_TAIL);
            self.set_c8(ix, Q_TAIL, tail.wrapping_add(1));
            let free = self.c8(ix, Q_FREE) - 1;
            self.set_c8(ix, Q_FREE, free);
            let entry = Self::sound_entry(ix, tail);
            let own = self.c8(ix, MASK);
            self.wr8(entry, !own & flags);
            self.wr8(entry + 1, params[1] << 4 | (params[2] & 0x0F));
            for i in 0..6u16 { self.wr8(entry + 2 + i, params[3 + i as usize]); }
            let was_queued = self.c8(ix, Q_COUNT);
            self.set_c8(ix, Q_COUNT, was_queued.wrapping_add(1));
            if was_queued | self.c8(ix, STATUS) == 0 { self.sound_play_entry_or_wait(ix, entry); }
        }
        self.sound_note_started();
        true
    }

    /// ROM `203A`: address of queue entry `index & 3` of the channel at `ix`.
    fn sound_entry(ix: u16, index: u8) -> u16 { ix + QUEUE + 8 * (index & 3) as u16 }

    /// ROM `201E`: move the "started" set into the active set (arming the 100 Hz divider if
    /// nothing was active before).
    fn sound_note_started(&mut self) {
        let started = self.rd8(SND_STARTED);
        if started == 0 { return; }
        self.wr8(SND_STARTED, 0);
        let before = self.rd8(SND_ACTIVE);
        self.wr8(SND_ACTIVE, before | started);
        if before == 0 {
            self.wr8(SND_DIVIDER, 3);
            self.wr8(SND_EVENTS, 0);
        }
    }

    /// `SOUND RELEASE` (`BCAD`, ROM `204A`): release channels held with the hold flag.
    pub fn sound_release(&mut self, channels: u8) {
        self.sound_continue();
        let channels = channels & 0x07;
        if channels == 0 { return; }
        for ix in channels_in(channels).collect::<Vec<_>>() {
            if self.c8(ix, STATUS) & 0x08 != 0 { self.sound_start_head(ix); }
        }
        self.sound_note_started();
    }

    /// `SOUND CHECK` (`BCB0`, ROM `206C`): bits 0-2 free queue entries, 3-5 channels being
    /// rendezvoused with, 6 held, 7 playing.  Also forgets a pending arm-event.
    pub fn sound_check(&mut self, channel: u8) -> u8 {
        let Some(ix) = channels_in(channel & 7).next() else { return 0 };
        let status = self.c8(ix, STATUS);
        let r = (status << 3) | self.c8(ix, Q_FREE);
        self.set_c8(ix, EVENT + 1, 0);
        r
    }

    /// `SOUND ARM EVENT` (`BCB3`, ROM `2089`): raise `event` when the channel has a free
    /// queue slot (immediately if it has one now).
    pub fn sound_arm_event(&mut self, channel: u8, event: u16) {
        let Some(ix) = channels_in(channel & 7).next() else { return };
        if self.c8(ix, Q_FREE) == 0 {
            self.wr16(ix + EVENT, event);                       // low at +29, high at +30
        } else {
            // The low byte is stored either way; a zero high byte means "not armed".
            self.set_c8(ix, EVENT, event as u8);
            self.set_c8(ix, EVENT + 1, 0);
            self.kl_event(event);
        }
    }

    // ===============================================================================
    // Starting notes                                                   (20B7 .. 2174)
    // ===============================================================================

    /// ROM `20B7`: start the note at the head of the queue.
    fn sound_start_head(&mut self, ix: u16) {
        let entry = Self::sound_entry(ix, self.c8(ix, Q_HEAD));
        self.sound_play_entry_or_wait(ix, entry);
    }

    /// ROM `20BD`: honour hold / rendezvous flags, then play.
    fn sound_play_entry_or_wait(&mut self, ix: u16, entry: u16) {
        let flags = self.rd8(entry);
        if flags != 0 {
            if flags & 0x08 != 0 {                              // L_2118: hold
                self.wr8(entry, flags & !0x08);
                self.set_c8(ix, STATUS, 0x08);
                return;
            }
            self.wr8(entry, 0);
            if !self.sound_rendezvous(ix, flags) { return; }
        }
        self.sound_play_entry(ix, entry);                       // L_20CD
    }

    /// ROM `211F`: try to rendezvous channel `ix` with the channels in `wanted`.  Returns
    /// `true` (and starts the partners) if all are waiting for us.
    fn sound_rendezvous(&mut self, ix: u16, wanted: u8) -> bool {
        let me = self.c8(ix, MASK);
        let first = if wanted & 1 != 0 { ch_addr(0) } else if wanted & 2 != 0 { ch_addr(1) } else { ch_addr(2) };
        if self.c8(first, STATUS) & me == 0 {
            self.set_c8(ix, STATUS, wanted);                    // L_216D: wait
            return false;
        }
        if wanted != self.c8(first, MASK) {
            // two partners: the second is channel C if bit 2 set else B
            let second = if wanted & 4 != 0 { ch_addr(2) } else { ch_addr(1) };
            if self.c8(second, STATUS) & me == 0 {
                self.set_c8(ix, STATUS, wanted);
                return false;
            }
            self.sound_start_head(second);
        }
        self.set_c8(first, STATUS, 0);
        self.sound_start_head(first);
        true
    }

    /// ROM `20CD`: begin the note stored at `entry`.
    fn sound_play_entry(&mut self, ix: u16, entry: u16) {
        self.set_c8(ix, STATUS, 0x10);
        let envs = self.rd8(entry + 1);
        let amp_env = envs & 0xF0;
        let tone_env = envs & 0x0F;
        let period = self.rd16(entry + 2);
        let mut tone_on = 0u8;
        if tone_env | (period as u8) | (period >> 8) as u8 != 0 {
            self.sound_tone_setup(ix, period, tone_env);       // L_22AB
            tone_on = self.c8(ix, MASK);
        }
        let noise = self.rd8(entry + 4);
        let volume = self.rd8(entry + 5);
        let duration = self.rd16(entry + 6);
        self.sound_amp_setup(ix, amp_env, noise, volume, duration, tone_on);

        let v = self.rd8(SND_STARTED) | self.c8(ix, MASK);
        self.wr8(SND_STARTED, v);
        let h = self.c8(ix, Q_HEAD).wrapping_add(1);
        self.set_c8(ix, Q_HEAD, h);
        let c = self.c8(ix, Q_COUNT).wrapping_sub(1);
        self.set_c8(ix, Q_COUNT, c);
        let f = self.c8(ix, Q_FREE) + 1;
        self.set_c8(ix, Q_FREE, f);
        let hi = self.c8(ix, EVENT + 1);
        self.set_c8(ix, EVENT + 1, 0);
        if hi != 0 {
            let ev = (hi as u16) << 8 | self.c8(ix, EVENT) as u16;
            self.kl_event(ev);
        }
    }

    /// ROM `2175`: start noise, mixer, volume and the amplitude envelope.
    fn sound_amp_setup(&mut self, ix: u16, amp_env: u8, noise: u8, volume: u8, duration: u16, tone_on: u8) {
        self.set_c8(ix, VOLUME, volume | 0x80);
        let duration = if duration == 0 { 0xFFFF } else { duration };
        self.wr16(ix + DURATION, duration);
        let mut enable = tone_on;
        if noise != 0 {
            self.mc_sound_register(6, noise);
            enable |= self.c8(ix, NOISE_BIT);
        }
        self.sound_mixer(ix, enable);                            // L_228B
        let mut env = DEFAULT_ENVELOPE_ADDR;
        if amp_env != 0 {
            let a = SND_AMP_ENVELOPES + amp_env as u16;          // amp_env is already n*16
            if self.rd8(a) != 0 { env = a; }
        }
        self.wr16(ix + AMP_START, env);
        let (hl, e) = self.sound_env_begin(ix, env);             // L_2265
        self.sound_amp_run(ix, hl, e);                           // JR L_21BF
    }

    // ===============================================================================
    // Amplitude envelope                                               (21B6 .. 2272)
    // ===============================================================================
    //
    // The Z80 code keeps the envelope read pointer in HL and the remaining step count
    // in E; the two are mirrored in the channel block at (IX+13/14) and (IX+16).

    /// ROM `2265`: begin an envelope at `env` (its section-count byte).
    /// Returns `(HL, E)` = pointer to the first section's step size and its step count.
    fn sound_env_begin(&mut self, ix: u16, env: u16) -> (u16, u8) {
        let sections = self.env_byte(env);
        self.set_c8(ix, AMP_SECTIONS, sections);
        self.sound_env_enter_section(ix, env)
    }

    /// ROM `2269`: `hl` = address of the byte *before* a section's step count.
    fn sound_env_enter_section(&mut self, ix: u16, hl: u16) -> (u16, u8) {
        let hl = hl.wrapping_add(1);
        let e = self.env_byte(hl);
        let hl = hl.wrapping_add(1);
        self.wr16(ix + AMP_PTR, hl);
        (hl, e)
    }

    /// ROM `21B6`: step the amplitude envelope from its saved position.  Returns `true`
    /// when the whole note has finished (carry set by `223A`).
    fn sound_amp_step(&mut self, ix: u16, _from_event: bool) -> bool {
        let hl = self.rd16(ix + AMP_PTR);
        let e = self.c8(ix, AMP_STEPS);
        self.sound_amp_run(ix, hl, e)
    }

    /// ROM `21BF`: interpret one envelope step; `hl`/`e` as described above.
    fn sound_amp_run(&mut self, ix: u16, mut hl: u16, mut e: u8) -> bool {
        loop {
            if e == 0xFF { return self.sound_note_finished(ix); }   // L_223A
            let a = self.env_byte(hl);
            hl = hl.wrapping_add(1);
            if e & 0x80 != 0 {
                // L_2213: hardware envelope section: E = shape, A = period lo, (HL) = period hi
                self.mc_sound_register(13, e);
                self.mc_sound_register(11, a);
                let hi = self.env_byte(hl);
                self.mc_sound_register(12, hi);
                self.sound_set_volume(ix, 0x10);
                self.sound_env_advance(ix, &mut hl, &mut e);
                if e == 0xFF {
                    let (h, ee) = self.sound_env_begin(ix, DEFAULT_ENVELOPE_ADDR);
                    hl = h;
                    e = ee;
                }
                continue;
            }
            let vol = self.c8(ix, VOLUME);
            let mut a = a;
            let mut write = true;
            if e != 0 {
                e -= 1;
                if a == 0 {
                    a |= vol;
                    if a & 0x80 == 0 { write = false; }              // JP P,L_21DD
                }
                if write { a = a.wrapping_add(vol); }                // L_21D5
            }
            if write { self.sound_set_volume(ix, a & 0x0F); }        // L_21D8 / L_2273

            // L_21DD: pause time and remaining-duration bookkeeping
            let mut c = self.env_byte(hl);
            let dur_hi = self.c8(ix, DURATION + 1);
            if dur_hi & 0x80 == 0 {
                let dur_lo = self.c8(ix, DURATION);
                let sum = 0u8.wrapping_sub(c) as u16 + dur_lo as u16;
                let mut a2 = sum as u8;
                let mut b = dur_hi;
                if sum <= 0xFF {
                    b = b.wrapping_sub(1);
                    if b & 0x80 != 0 {
                        c = dur_lo;                                  // last, shortened pause
                        a2 = 0;
                        b = 0;
                    }
                    self.set_c8(ix, DURATION + 1, b);
                }
                self.set_c8(ix, DURATION, a2);
                if a2 | b == 0 { e = 0xFF; }
            }
            // L_2200
            if e == 0 { self.sound_env_advance(ix, &mut hl, &mut e); }
            self.set_c8(ix, AMP_STEPS, e);
            self.set_c8(ix, AMP_COUNT, c);
            self.set_c8(ix, AMP_FLAG, 0x80);
            return false;
        }
    }

    /// ROM `2246`: the current section is exhausted: go to the next one, restart the
    /// envelope, or finish (`e = 0xFF`).  `hl` points at the section's last byte.
    fn sound_env_advance(&mut self, ix: u16, hl: &mut u16, e: &mut u8) {
        let left = self.c8(ix, AMP_SECTIONS).wrapping_sub(1);
        self.set_c8(ix, AMP_SECTIONS, left);
        if left != 0 {
            let (h, ee) = self.sound_env_enter_section(ix, *hl);
            *hl = h; *e = ee;
            return;
        }
        let dur_hi = self.c8(ix, DURATION + 1);
        if dur_hi & 0x80 == 0 {
            let (h, ee) = self.sound_env_begin(ix, DEFAULT_ENVELOPE_ADDR);   // hold final volume
            *hl = h; *e = ee;
            return;
        }
        let lo = self.c8(ix, DURATION).wrapping_add(1);
        self.set_c8(ix, DURATION, lo);
        if lo == 0 {
            let hi = dur_hi.wrapping_add(1);
            self.set_c8(ix, DURATION + 1, hi);
            *e = 0xFF;
            if hi == 0 { return; }                                   // repeat count exhausted
        }
        let start = self.rd16(ix + AMP_START);
        let (h, ee) = self.sound_env_begin(ix, start);               // repeat the envelope
        *hl = h; *e = ee;
    }

    /// ROM `223A`: the note is over; the channel is idle.  Returns `true` (carry).
    fn sound_note_finished(&mut self, ix: u16) -> bool {
        self.set_c8(ix, STATUS, 0);
        self.set_c8(ix, AMP_FLAG, 0);
        self.set_c8(ix, TONE_FLAG, 0);
        true
    }

    /// ROM `2273`: remember and write the channel volume.
    fn sound_set_volume(&mut self, ix: u16, v: u8) {
        self.set_c8(ix, VOLUME, v);
        self.sound_set_volume_reg(ix, v);
    }

    /// ROM `2276`: write PSG register `8 + channel`.
    fn sound_set_volume_reg(&mut self, ix: u16, v: u8) {
        let reg = self.c8(ix, NUMBER) + 8;
        self.mc_sound_register(reg, v);
    }

    // ===============================================================================
    // Mixer / silence                                                  (227F .. 22AA)
    // ===============================================================================

    /// ROM `227F`: deactivate a channel and silence it.
    fn sound_stop_channel(&mut self, ix: u16) {
        let m = self.c8(ix, MASK);
        let a = self.rd8(SND_ACTIVE) & !m;
        self.wr8(SND_ACTIVE, a);
        self.sound_mixer(ix, 0);
    }

    /// ROM `228B`: enable the mixer bits in `enable` for this channel (tone bit, noise bit)
    /// and disable the others.  When nothing is enabled, or the mixer changed, the channel
    /// volume is zeroed and register 7 rewritten.
    fn sound_mixer(&mut self, ix: u16, enable: u8) {
        let both = self.c8(ix, MASK) | self.c8(ix, NOISE_BIT);
        let old = self.rd8(SND_MIXER);
        let new = (old | both) ^ enable;
        self.wr8(SND_MIXER, new);
        if new == old && enable != 0 { return; }
        self.sound_set_volume_reg(ix, 0);                        // L_22A0
        self.mc_sound_register(7, new);
    }

    // ===============================================================================
    // Tone envelope                                                    (22AB .. 2337)
    // ===============================================================================

    /// ROM `22AB`: set the tone period and begin tone envelope `env` (if any).
    fn sound_tone_setup(&mut self, ix: u16, period: u16, env: u8) {
        self.sound_set_period(ix, period);
        let Some(addr) = self.sound_t_address(env) else { return };
        let sections = self.rd8(addr.0) & 0x7F;
        if sections == 0 { return; }
        self.wr16(ix + TONE_START, addr.0);
        let e = self.sound_tone_section_init(ix, addr.0, sections);
        self.sound_tone_run(ix, self.rd16(ix + TONE_PTR), e);
    }

    /// ROM `2313`: set the sections-left counter and position on the first/next section.
    /// `hl` = address of the byte before the section's step-count byte.  Returns `E`.
    fn sound_tone_section_init(&mut self, ix: u16, hl: u16, sections: u8) -> u8 {
        self.set_c8(ix, TONE_SECTIONS, sections);
        let at = hl.wrapping_add(1);
        let mut e = self.rd8(at);
        self.wr16(ix + TONE_PTR, at.wrapping_add(1));
        if e == 0 { e = 1; }
        e
    }

    /// ROM `22C2`..`2312`: perform one tone step.
    fn sound_tone_step(&mut self, ix: u16, _event: bool) {
        let hl = self.rd16(ix + TONE_PTR);
        let e = self.c8(ix, TONE_STEPS);
        self.sound_tone_run(ix, hl, e);
    }

    fn sound_tone_run(&mut self, ix: u16, mut hl: u16, mut e: u8) {
        let mut c = self.rd8(hl);
        hl = hl.wrapping_add(1);
        let hi: u8;
        if e >= 0xF0 {                                            // absolute period
            hi = e - 0xF0;
            e = 0;
            self.sound_set_period(ix, (hi as u16) << 8 | c as u16);
        } else {
            e -= 1;
            let delta = c as i8 as i16;                            // signed step
            let p = (self.rd16(ix + PERIOD) as i16).wrapping_add(delta) as u16;
            self.sound_set_period(ix, p);
        }
        c = self.rd8(hl);                                         // pause time
        if e == 0 {
            let left = self.c8(ix, TONE_SECTIONS).wrapping_sub(1);
            let mut next = left != 0;
            let mut sections = left;
            if !next {
                let start = self.rd16(ix + TONE_START);
                let first = self.rd8(start);
                if first & 0x80 != 0 { next = true; sections = first & 0x7F; }
                else { self.set_c8(ix, TONE_FLAG, 0); return; }   // envelope finished
            }
            if next {
                let base = if left != 0 { hl } else { self.rd16(ix + TONE_START) };
                e = self.sound_tone_section_init(ix, base, sections);
            }
        }
        self.set_c8(ix, TONE_STEPS, e);
        self.set_c8(ix, TONE_COUNT, c);
        self.set_c8(ix, TONE_FLAG, 0x80);
    }

    /// ROM `2324`: store and write the tone period (PSG registers `2n`, `2n+1`).
    fn sound_set_period(&mut self, ix: u16, period: u16) {
        self.wr16(ix + PERIOD, period);
        let reg = self.c8(ix, NUMBER) * 2;
        self.mc_sound_register(reg, period as u8);
        self.mc_sound_register(reg + 1, (period >> 8) as u8);
    }

    // ===============================================================================
    // Envelope tables                                                  (2338 .. 2363)
    // ===============================================================================

    /// `SOUND AMPL ENVELOPE` (`BCBF`, ROM `2338`): store the 16 bytes `data` as amplitude
    /// envelope `n` (1..15).
    pub fn sound_ampl_envelope(&mut self, n: u8, data: &[u8; 16]) -> bool {
        self.sound_store_envelope(self.sound_a_address(n), data)
    }

    /// `SOUND TONE ENVELOPE` (`BCC2`, ROM `233D`).
    pub fn sound_tone_envelope(&mut self, n: u8, data: &[u8; 16]) -> bool {
        self.sound_store_envelope(self.sound_t_address(n), data)
    }

    fn sound_store_envelope(&mut self, at: Option<(u16, u16)>, data: &[u8; 16]) -> bool {
        let Some((addr, _)) = at else { return false };
        for (i, &b) in data.iter().enumerate() { self.wr8(addr + i as u16, b); }
        true
    }

    /// `SOUND A ADDRESS` (`BCC5`, ROM `2349`): `(address, length 16)` of amplitude envelope `n`.
    pub fn sound_a_address(&self, n: u8) -> Option<(u16, u16)> { Self::sound_env_address(SND_AMP_ENVELOPES, n) }

    /// `SOUND T ADDRESS` (`BCC8`, ROM `234E`).
    pub fn sound_t_address(&self, n: u8) -> Option<(u16, u16)> { Self::sound_env_address(SND_TONE_ENVELOPES, n) }

    /// ROM `2351`.
    fn sound_env_address(base: u16, n: u8) -> Option<(u16, u16)> {
        if n == 0 || n >= 0x10 { return None; }
        Some((base + (n as u16) * 16, 16))
    }
}
