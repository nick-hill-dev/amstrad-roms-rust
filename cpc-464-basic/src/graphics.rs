//! # Graphics commands  (ROM `C48C .. C528`)
//!
//! Thin wrappers around the graphics VDU.  Coordinates are evaluated as 16-bit integers
//! (`CE86`, any value that fits in an integer) and handed to the firmware unchanged.
//! The plotting commands pick their firmware entry from a register (`LD BC,entry`) and
//! share one body, as here.

use crate::fw::*;
use crate::{Basic, Res};
use cpc464_os::Hardware;

impl<H: Hardware> Basic<H> {
    /// `ORIGIN x,y[,left,right,top,bottom]` (ROM `C48C`): the window (height first, then
    /// width) and then the origin.
    pub fn stmt_origin(&mut self) -> Res {
        let (x, y) = self.int_pair()?;
        if self.comma() {
            let (left, right) = self.int_pair()?;
            self.expect(b',')?;
            let (top, bottom) = self.int_pair()?;
            self.gra_win_height(top, bottom);
            self.gra_win_width(left, right);
        }
        self.gra_xy(GRA_SET_ORIGIN, x, y);
        Ok(())
    }

    /// `CLG [ink]` (ROM `C4B5`).
    pub fn stmt_clg(&mut self) -> Res {
        if !crate::is_end(self.cur()) {
            let ink = self.eval_below(16)?;
            self.gra_set_paper(ink);
        }
        self.gra_clear_window();
        Ok(())
    }

    /// `DRAW x,y[,ink]` (ROM `C4C6`).
    pub fn stmt_draw(&mut self) -> Res { self.plot_like(GRA_LINE_ABSOLUTE) }
    /// `DRAWR x,y[,ink]` (ROM `C4CB`).
    pub fn stmt_drawr(&mut self) -> Res { self.plot_like(GRA_LINE_RELATIVE) }
    /// `PLOT x,y[,ink]` (ROM `C4D0`).
    pub fn stmt_plot(&mut self) -> Res { self.plot_like(GRA_PLOT_ABSOLUTE) }
    /// `PLOTR x,y[,ink]` (ROM `C4D5`).
    pub fn stmt_plotr(&mut self) -> Res { self.plot_like(GRA_PLOT_RELATIVE) }

    /// ROM `C4D8`: `x,y[,ink]`, set the graphics pen if an ink was given, then the entry.
    fn plot_like(&mut self, entry: u16) -> Res {
        let (x, y) = self.int_pair()?;
        if self.comma() {
            let ink = self.eval_below(16)?;
            self.gra_set_pen(ink);
        }
        self.gra_xy(entry, x, y);
        Ok(())
    }

    /// `TEST(x,y)` (ROM `C4E9`): the ink at a point.
    pub fn fn_test(&mut self) -> Res { self.test_like(GRA_TEST_ABSOLUTE) }
    /// `TESTR(x,y)` (ROM `C4EE`).
    pub fn fn_testr(&mut self) -> Res { self.test_like(GRA_TEST_RELATIVE) }

    /// ROM `C4F1`: `x,y)` and the integer result of the test entry.
    fn test_like(&mut self, entry: u16) -> Res {
        let (x, y) = self.int_pair()?;
        self.expect(b')')?;
        let ink = self.gra_test(entry, x, y);
        self.set_acc_int(ink as u16);                     // FF0A
        Ok(())
    }

    /// `MOVE x,y` (ROM `C505`).
    pub fn stmt_move(&mut self) -> Res {
        let (x, y) = self.int_pair()?;
        self.gra_xy(GRA_MOVE_ABSOLUTE, x, y);
        Ok(())
    }
    /// `MOVER x,y` (ROM `C50A`).
    pub fn stmt_mover(&mut self) -> Res {
        let (x, y) = self.int_pair()?;
        self.gra_xy(GRA_MOVE_RELATIVE, x, y);
        Ok(())
    }

    /// ROM `C51A`: two integer expressions separated by a comma (`DE`, `BC`).
    pub fn int_pair(&mut self) -> Res<(u16, u16)> {
        let a = self.eval_int()?;                         // CE86
        self.expect(b',')?;
        let b = self.eval_int()?;
        Ok((a, b))
    }
}
