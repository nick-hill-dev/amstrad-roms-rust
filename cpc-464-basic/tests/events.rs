//! Differential tests of BASIC's events, with interrupts delivered while BASIC waits for
//! a line (`#TICKS n`).  Needs `OS464_ROM` and `BASIC_ROM`.
//!
//! (The firmware's 1/50 s divider starts at 0 after power on, so the first 255 interrupts
//! do not run the tickers: each test lets 300 happen.)
mod support;
use support::session::check;

#[test] fn ticks_in_direct_mode() { check(&["#TICKS 10", "PRINT TIME"]); }
#[test] fn after_fires() { check(&["10 AFTER 1 GOSUB 100:INPUT a:PRINT \"after input\":END", "100 PRINT \"event\":RETURN", "RUN", "#TICKS 300", "5"]); }
#[test] fn every_fires_and_remain() { check(&["10 EVERY 1,2 GOSUB 100:INPUT a:INPUT b:PRINT n;REMAIN(2):END", "100 n=n+1:RETURN", "RUN", "#TICKS 300", "1", "#TICKS 300", "2"]); }
#[test] fn di_holds_events() { check(&["10 AFTER 1 GOSUB 100:DI:INPUT a:PRINT \"di\":EI:PRINT \"ei\":END", "100 PRINT \"event\":RETURN", "RUN", "#TICKS 300", "5"]); }
#[test] fn event_in_direct_mode_waits() { check(&["10 AFTER 1 GOSUB 100:END", "100 PRINT \"event\":RETURN", "RUN", "#TICKS 300", "PRINT 1", "GOTO 100"]); }
#[test] fn timers_priority() { check(&["10 AFTER 1,0 GOSUB 100:AFTER 1,3 GOSUB 200:INPUT a:END", "100 PRINT \"t0\":RETURN", "200 PRINT \"t3\":RETURN", "RUN", "#TICKS 300", "1"]); }
#[test] fn every_counts() { check(&["10 EVERY 1,2 GOSUB 100:INPUT a:INPUT b:PRINT n:END", "100 n=n+1:RETURN", "RUN", "#TICKS 300", "1", "#TICKS 300", "2"]); }
#[test] fn event_during_loop_restarts_statement() { check(&["10 AFTER 1 GOSUB 100:INPUT a:FOR i=1 TO 3:PRINT i:NEXT:END", "100 PRINT \"in event\":RETURN", "RUN", "#TICKS 300", "7"]); }
#[test] fn remain_stops_timer() { check(&["10 EVERY 5,1 GOSUB 100:INPUT a:r=REMAIN(1):INPUT b:PRINT n;r:END", "100 n=n+1:RETURN", "RUN", "#TICKS 300", "1", "#TICKS 300", "2"]); }
#[test] fn sound_and_time() { check(&["SOUND 1,100,5", "#TICKS 300", "PRINT SQ(1);TIME"]); }
