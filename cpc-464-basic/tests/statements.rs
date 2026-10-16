//! Differential tests: typed direct commands and small programs.
//! Needs `OS464_ROM` and `BASIC_ROM`.
mod support;
use support::session::check;

#[test] fn print_number() { check(&["PRINT 1"]); }
#[test] fn print_string() { check(&["PRINT \"HELLO\""]); }
#[test] fn print_expression() { check(&["PRINT 1+2*3-4/5"]); }
#[test] fn print_separators() { check(&["PRINT 1;2,3;\"A\";", "PRINT"]); }
#[test] fn assign_and_print() { check(&["a=5", "b%=7", "c$=\"xy\"", "PRINT a,b%,c$"]); }
#[test] fn syntax_error() { check(&["PRINT +"]); }
#[test] fn store_line() { check(&["10 PRINT \"HI\""]); }
#[test] fn list_program() { check(&["10 PRINT \"HI\"", "20 GOTO 10", "LIST"]); }
#[test] fn run_program() { check(&["10 PRINT \"HI\"", "20 a=a+1:IF a<3 THEN 10", "RUN"]); }
#[test] fn for_loop() { check(&["10 FOR i=1 TO 5:PRINT i;:NEXT", "RUN"]); }
#[test] fn for_loop_real() { check(&["10 FOR i=0 TO 1 STEP 0.25:PRINT i;:NEXT i", "RUN"]); }
#[test] fn while_loop() { check(&["10 WHILE i<5:i=i+1:WEND:PRINT i", "RUN"]); }
#[test] fn gosub() { check(&["10 GOSUB 100:PRINT \"back\":END", "100 PRINT \"sub\":RETURN", "RUN"]); }
#[test] fn strings() { check(&["a$=\"hello\"+\" \"+\"world\"", "PRINT a$;LEN(a$);LEFT$(a$,3);RIGHT$(a$,2);MID$(a$,2,3)"]); }
#[test] fn string_functions() { check(&["PRINT CHR$(65);ASC(\"B\");STR$(12);VAL(\"3.5\");UPPER$(\"ab\");LOWER$(\"CD\");INSTR(\"abcabc\",\"ca\")"]); }
#[test] fn arrays() { check(&["DIM a(5),b$(3,2)", "a(3)=7:b$(1,2)=\"x\"", "PRINT a(3);b$(1,2)"]); }
#[test] fn auto_dim() { check(&["x(4)=2:PRINT x(4)+x(10)"]); }
#[test] fn data_read() { check(&["10 DATA 1,2,\"three\",4.5", "20 READ a,b,c$,d:PRINT a;b;c$;d", "RUN"]); }
#[test] fn maths() { check(&["PRINT SIN(1);COS(1);SQR(2);EXP(1);LOG(10);ATN(1);2^10;INT(-3.5);FIX(-3.5);ABS(-2)"]); }
#[test] fn integer_ops() { check(&["PRINT 7\\2;7 MOD 3;5 AND 3;5 OR 3;5 XOR 3;NOT 0"]); }
#[test] fn comparisons() { check(&["PRINT 1<2;2<1;1=1;\"a\"<\"b\";\"b\"<=\"a\";3<>3"]); }
#[test] fn def_fn() { check(&["10 DEF FNf(x)=x*x+1", "20 PRINT FNf(3)", "RUN"]); }
#[test] fn on_goto() { check(&["10 FOR i=1 TO 3:ON i GOSUB 100,200,300:NEXT:END", "100 PRINT \"a\":RETURN", "200 PRINT \"b\":RETURN", "300 PRINT \"c\":RETURN", "RUN"]); }
#[test] fn errors_reported() { check(&["10 PRINT 1/0", "20 x=1E39*1E39", "30 GOTO 999", "RUN"]); }
#[test] fn on_error() { check(&["10 ON ERROR GOTO 100", "20 ERROR 5", "30 PRINT \"after\":END", "100 PRINT ERR;ERL:RESUME NEXT", "RUN"]); }
#[test] fn delete_and_renum() { check(&["10 PRINT 1", "20 GOTO 10", "30 GOSUB 20", "RENUM 100,,5", "LIST", "DELETE 105", "LIST"]); }
#[test] fn edit_line_replace() { check(&["10 PRINT 1", "10 PRINT 2", "5 REM x", "10", "LIST"]); }
#[test] fn print_using() { check(&["PRINT USING \"##.##\";3.14159", "PRINT USING \"+###,###.##\";1234.5;-5", "PRINT USING \"!&\\  \\\";\"abc\";\"de\";\"fghij\""]); }
#[test] fn hex_bin() { check(&["PRINT HEX$(255);BIN$(5,8);&FF;&X101"]); }
#[test] fn round_max_min() { check(&["PRINT ROUND(3.14159,2);MAX(1,5,3);MIN(4,2)"]); }
#[test] fn write_stmt() { check(&["WRITE 1,\"a\",2.5"]); }
#[test] fn cls_mode_pen() { check(&["MODE 1:PEN 2:PAPER 3:CLS:PRINT \"X\":BORDER 5:INK 1,24"]); }
#[test] fn locate_window() { check(&["LOCATE 10,5:PRINT \"Z\":WINDOW 2,20,2,10:PRINT POS(#0);VPOS(#0)"]); }
#[test] fn graphics() { check(&["PLOT 100,100:DRAW 200,150:MOVE 10,10:DRAWR 20,20:PRINT XPOS;YPOS;TEST(200,150)"]); }
#[test] fn peek_poke() { check(&["POKE &8000,123:PRINT PEEK(&8000)"]); }
#[test] fn fre_himem() { check(&["PRINT FRE(0);HIMEM", "MEMORY &8000", "PRINT HIMEM;FRE(\"\")"]); }
#[test] fn defint() { check(&["DEFINT a-c", "a=3.7:PRINT a", "DEFSTR s", "s=\"q\":PRINT s"]); }
#[test] fn erase() { check(&["DIM q(3):q(1)=4:ERASE q:PRINT q(1)"]); }
#[test] fn symbol() { check(&["SYMBOL 240,1,2,3,4,5,6,7,8", "SYMBOL AFTER 200", "PRINT HIMEM"]); }
#[test] fn clear_new() { check(&["a=1", "CLEAR", "PRINT a", "10 PRINT", "NEW", "LIST"]); }
#[test] fn string_gc() { check(&["10 FOR i=1 TO 200:a$=STRING$(100,\"x\")+STR$(i):NEXT:PRINT LEN(a$);FRE(\"\")", "RUN"]); }
#[test] fn tron() { check(&["10 PRINT 1:PRINT 2", "TRON", "RUN", "TROFF"]); }
#[test] fn stop_cont() { check(&["10 PRINT 1:STOP:PRINT 2", "RUN", "CONT"]); }
#[test] fn input_from_keyboard() { check(&["10 INPUT \"n\";a,b$:PRINT a*2;b$", "RUN", "21,hello"]); }
#[test] fn input_redo() { check(&["10 INPUT a:PRINT a", "RUN", "abc", "5"]); }
#[test] fn line_input() { check(&["10 LINE INPUT \"? \",a$:PRINT a$", "RUN", "a, b, c"]); }
#[test] fn auto_numbering() { check(&["AUTO 100,5", "PRINT 1", "PRINT 2"]); }
#[test] fn edit_cmd() { check(&["10 PRINT 1", "EDIT 10", "10 PRINT 3", "LIST"]); }
#[test] fn mid_assign() { check(&["a$=\"abcdef\":MID$(a$,2,3)=\"XYZW\":PRINT a$"]); }
#[test] fn rnd() { check(&["RANDOMIZE 5:PRINT RND;RND(0);RND(-1)"]); }
#[test] fn deg_rad() { check(&["DEG:PRINT SIN(90):RAD:PRINT SIN(PI/2)"]); }
#[test] fn zone_spc_tab() { check(&["ZONE 5:PRINT 1,2;SPC(3);3;TAB(20);4"]); }
#[test] fn dec_str() { check(&["PRINT DEC$((3.5,\"##.#\")", "a$=DEC$((-12.345,\"+###.##\")+\"!\":PRINT a$"]); }
#[test] fn sound_env() { check(&["ENV 1,5,2,3:ENT 1,10,-1,2:SOUND 1,200,50,10,1,1"]); }
#[test] fn key_def() { check(&["KEY 128,\"run\"+CHR$(13):KEY DEF 10,1,65,66,67:SPEED KEY 20,3:SPEED INK 5,5"]); }
#[test] fn if_else() { check(&["10 a=1:IF a=1 THEN PRINT \"y\" ELSE PRINT \"n\"", "20 IF a=2 THEN PRINT \"y\" ELSE PRINT \"n\"", "RUN"]); }
#[test] fn nested_for_next() { check(&["10 FOR i=1 TO 2:FOR j=1 TO 2:PRINT i*j;:NEXT j,i", "RUN"]); }
#[test] fn type_mismatch() { check(&["a$=1"]); }
#[test] fn long_program() {
    check(&["10 REM test", "20 DIM s(10)", "30 FOR i=1 TO 10:s(i)=i*i:NEXT", "40 t=0:FOR i=1 TO 10:t=t+s(i):NEXT", "50 PRINT \"total\";t", "60 a$=\"\":FOR i=1 TO 10:a$=a$+CHR$(64+i):NEXT:PRINT a$", "RUN", "LIST 20-40"]);
}
