//! Differential tests: errors, quirks and edge cases.  Needs `OS464_ROM` and `BASIC_ROM`.
mod support;
use support::session::check;

#[test] fn log_zero() { check(&["PRINT LOG(0)"]); }
#[test] fn log_negative() { check(&["PRINT LOG(-1)"]); }
#[test] fn sqr_negative() { check(&["PRINT SQR(-4)"]); }
#[test] fn exp_overflow() { check(&["PRINT EXP(1000)"]); }
#[test] fn power_edge() { check(&["PRINT (-8)^(1/3);0^0;2^-2;(-2)^3;10^40"]); }
#[test] fn division_by_zero_soft() { check(&["PRINT 1/0;5\\0;5 MOD 0"]); }
#[test] fn overflow_int() { check(&["a%=32767:a%=a%+1"]); }
#[test] fn int_promotes() { check(&["PRINT 32767+1;-32768-1;200*200;-(-32768)"]); }
#[test] fn tan_atn() { check(&["PRINT TAN(1.5707963);ATN(1E30);SIN(1E10)"]); }
#[test] fn number_formats() { check(&["PRINT 1E10;1E-10;123456789;1234567890;0.0001;.5;-0;1/3;2/3;1E38;3.4E38"]); }
#[test] fn number_parse() { check(&["PRINT 1.5E+3;1.5e-3;&H7FFF;&HFFFF;&X1111;&777;1.23456789012345;000012"]); }
#[test] fn val_edge() { check(&["PRINT VAL(\"\");VAL(\"12abc\");VAL(\"  -3.5e2\");VAL(\"&FF\")", "PRINT VAL(\"1e\")"]); }
#[test] fn str_edge() { check(&["PRINT STR$(-1.5);STR$(0);LEN(STR$(1))"]); }
#[test] fn next_without_for() { check(&["NEXT"]); }
#[test] fn return_without_gosub() { check(&["RETURN"]); }
#[test] fn wend_without_while() { check(&["WEND"]); }
#[test] fn for_without_next() { check(&["10 FOR i=1 TO 2", "RUN"]); }
#[test] fn while_without_wend() { check(&["10 WHILE 1", "RUN"]); }
#[test] fn undefined_line() { check(&["GOTO 100"]); }
#[test] fn subscript_range() { check(&["DIM a(3):a(4)=1"]); }
#[test] fn redim() { check(&["DIM a(3):DIM a(4)"]); }
#[test] fn erase_missing() { check(&["ERASE q"]); }
#[test] fn type_mismatch_ops() { check(&["PRINT \"a\"+1"]); }
#[test] fn string_too_long() { check(&["a$=STRING$(200,\"x\"):b$=a$+a$"]); }
#[test] fn complex_strings() { check(&["PRINT \"a\"+(\"b\"+(\"c\"+(\"d\"+(\"e\"+(\"f\"+(\"g\"+(\"h\"+(\"i\"+(\"j\"+(\"k\"+\"l\"))))))))))"]); }
#[test] fn improper_args() { check(&["PRINT CHR$(256)"]); }
#[test] fn mid_errors() { check(&["PRINT MID$(\"abc\",0)", "PRINT MID$(\"abc\",5);LEFT$(\"abc\",10);RIGHT$(\"abc\",0)"]); }
#[test] fn instr_edge() { check(&["PRINT INSTR(\"abc\",\"\");INSTR(4,\"abc\",\"\");INSTR(2,\"abcabc\",\"bc\");INSTR(\"abc\",\"x\")"]); }
#[test] fn cont_after_error() { check(&["10 PRINT 1", "20 ERROR 30", "30 PRINT 3", "RUN", "CONT"]); }
#[test] fn resume_variants() {
    check(&["10 ON ERROR GOTO 100", "20 a=1/0:PRINT \"x\"", "30 b=LOG(-1)", "40 PRINT \"end\":END",
            "100 PRINT \"err\";ERR;ERL", "110 IF ERL=30 THEN RESUME 40 ELSE RESUME NEXT", "RUN"]);
}
#[test] fn error_in_handler() { check(&["10 ON ERROR GOTO 100", "20 ERROR 7", "100 ERROR 9", "RUN"]); }
#[test] fn resume_missing() { check(&["10 ON ERROR GOTO 100", "20 ERROR 7", "100 PRINT 1", "RUN"]); }
#[test] fn on_error_goto_0() { check(&["10 ON ERROR GOTO 100", "20 ERROR 7", "100 ON ERROR GOTO 0", "RUN"]); }
#[test] fn unknown_error() { check(&["ERROR 200", "ERROR 0"]); }
#[test] fn direct_def_fn() { check(&["DEF FNa(x)=x"]); }
#[test] fn fn_strings() { check(&["10 DEF FNj$(a$,n)=LEFT$(a$,n)+\"!\"", "20 DEF FNk=7", "30 PRINT FNj$(\"hello\",2);FNk", "RUN"]); }
#[test] fn fn_unknown() { check(&["PRINT FNz(1)"]); }
#[test] fn many_variables() { check(&["10 FOR i=1 TO 30:a(i)=i:NEXT", "20 aa=1:ab=2:ba=3:zz9=4:long.name.var=5", "30 PRINT aa;ab;ba;zz9;long.name.var;a(30)", "RUN", "LIST"]); }
#[test] fn multidim() { check(&["DIM m(2,3,4):m(1,2,3)=5:m(2,3,4)=6:PRINT m(1,2,3)+m(2,3,4);m(0,0,0)"]); }
#[test] fn string_arrays_gc() { check(&["10 DIM s$(20):FOR i=0 TO 20:s$(i)=STRING$(i,65+i):NEXT", "20 FOR j=1 TO 5:FOR i=0 TO 20:s$(i)=s$(i)+\"x\":NEXT:NEXT:PRINT FRE(\"\");s$(20)", "RUN"]); }
#[test] fn memory_errors() { check(&["MEMORY &2000", "MEMORY &A000", "SYMBOL AFTER 0", "PRINT HIMEM"]); }
#[test] fn memory_full_array() { check(&["DIM a(10000)", "DIM b(9000)"]); }
#[test] fn renum_errors() { check(&["10 GOTO 20", "20 GOTO 30", "30 GOTO 99", "RENUM", "LIST", "RENUM 5,20"]); }
#[test] fn delete_errors() { check(&["10 PRINT", "DELETE 20-30", "DELETE 10", "LIST"]); }
#[test] fn list_ranges() { check(&["10 PRINT 1", "20 PRINT 2", "30 PRINT 3", "LIST -20", "LIST 20-", "LIST 15", "LIST 20"]); }
#[test] fn listing_tokens() { check(&["10 IF a THEN b=1 ELSE c=2:'comment", "20 REM rem text", "30 PRINT USING \"##\";a%:a$=b$+c$:d!=1.5", "40 x=&HFF+&X10+1E5+.5:GOSUB 10:ON x GOTO 10,20", "LIST"]); }
#[test] fn listing_keywords() { check(&["10 FOR i=1 TO 10 STEP 2:NEXT:WHILE 0:WEND:SOUND 1,2:ENV 1,1,1,1:MODE 1:BORDER 1", "20 a=SIN(1)+INT(2)+INKEY(3)+MAX(1,2)+ROUND(1)+INSTR(\"a\",\"b\")", "30 b$=HEX$(1)+BIN$(2)+CHR$(3)+STRING$(2,\"a\")+SPACE$(2)+INKEY$", "LIST"]); }
#[test] fn auto_existing() { check(&["20 PRINT", "AUTO 10,10", "PRINT 1", "", "PRINT 3"]); }
#[test] fn print_spc_tab_edge() { check(&["PRINT SPC(-1);\"a\";TAB(-5);\"b\";TAB(100);\"c\";SPC(50);\"d\""]); }
#[test] fn print_comma_wrap() { check(&["PRINT \"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\",\"b\",\"c\",\"d\""]); }
#[test] fn print_long_line() { check(&["PRINT STRING$(39,\"a\");\"bb\";STRING$(80,\"c\")"]); }
#[test] fn width_printer() { check(&["WIDTH 20"]); }
#[test] fn using_more() {
    check(&["PRINT USING \"$$###.##\";12.5;-3", "PRINT USING \"**###.#\";5", "PRINT USING \"**$##.##\";7.25",
            "PRINT USING \"#.##^^^^\";12345", "PRINT USING \"###-\";-5;5", "PRINT USING \"x_#y###z\";5;6;7", "PRINT USING \"##\";123",
            "PRINT USING \"&\";\"\""]);
}
#[test] fn using_errors() { check(&["PRINT USING \"abc\";1", "PRINT USING \"\";1", "PRINT USING \"#\";\"s\""]); }
#[test] fn write_mixed() { check(&["WRITE", "WRITE -1.5;\"x y\";1E20"]); }
#[test] fn keywords_as_names() { check(&["total=1:PRINT total", "fn1=2:PRINT fn1"]); }
#[test] fn ink_errors() { check(&["INK 16,1", "MODE 3", "PEN 16", "BORDER 32"]); }
#[test] fn stream_errors() { check(&["PRINT #10,1", "CLS #8", "PRINT #1,\"w1\":LOCATE #1,5,5:PRINT #1,\"x\""]); }
#[test] fn origin_clg() { check(&["ORIGIN 100,100,0,300,0,200:CLG 2:PLOT 0,0,3:DRAWR 50,50:PRINT TESTR(0,0)"]); }
#[test] fn tag() { check(&["TAG:MOVE 100,100:PRINT \"tagged\";:TAGOFF"]); }
#[test] fn key_errors() { check(&["KEY DEF 80,1", "KEY 0,\"a\"", "SPEED KEY 0,1", "SPEED WRITE 1"]); }
#[test] fn sound_queue_and_sq() { check(&["SOUND 1,100,10:SOUND 2,200,10:PRINT SQ(1);SQ(2);SQ(4)", "PRINT SQ(3)"]); }
#[test] fn release_env_errors() { check(&["RELEASE 8", "ENV 0", "ENT 16", "ENV 1,=8,1000", "ENT -2,=100,10,5,1,1"]); }
#[test] fn out_inp() { check(&["OUT &F4FF,&C0:PRINT INP(&F6FF)"]); }
#[test] fn call_routine() { check(&["POKE &9000,&C9:CALL &9000,1,2,3:CALL &9000"]); }
#[test] fn rsx_unknown() { check(&["|FOO,1"]); }
#[test] fn after_every_remain() { check(&["10 AFTER 100,1 GOSUB 100:EVERY 50 GOSUB 100:PRINT REMAIN(1);REMAIN(0);REMAIN(2)", "20 END", "100 RETURN", "RUN"]); }
#[test] fn on_break() { check(&["10 ON BREAK GOSUB 100:ON BREAK STOP:DI:EI:END", "100 RETURN", "RUN"]); }
#[test] fn on_sq() { check(&["ON SQ(1) GOSUB 10", "ON SQ(3) GOSUB 10"]); }
#[test] fn data_errors() { check(&["10 DATA 1,abc,\"q\"", "20 READ a,b", "RUN"]); }
#[test] fn data_exhausted() { check(&["10 DATA 1", "20 READ a,b", "RUN"]); }
#[test] fn data_strings_and_restore() { check(&["10 DATA hello world , \"x,y\",, 3", "20 READ a$,b$,c$,d", "30 RESTORE 10:READ e$:PRINT a$;\"|\";b$;\"|\";c$;\"|\";d;e$", "RUN"]); }
#[test] fn restore_missing_line() { check(&["RESTORE 50"]); }
#[test] fn input_multiple() { check(&["10 INPUT \"vals\",a,b,c$:PRINT a+b;c$", "RUN", "1,2", "1,2,3,4", "1, 2 ,  \"q,r\""]); }
#[test] fn input_semicolon() { check(&["10 INPUT ;a:PRINT \"#\";a", "RUN", "7"]); }
#[test] fn line_input_quoted() { check(&["10 LINE INPUT ;\"p\";a$:PRINT a$;LEN(a$)", "RUN", "  \"quoted\", stuff  "]); }
#[test] fn input_ints() { check(&["10 INPUT a%:PRINT a%", "RUN", "40000", "123.7"]); }
#[test] fn tron_lines() { check(&["10 FOR i=1 TO 2:GOSUB 20:NEXT:END", "20 RETURN", "TRON", "RUN"]); }
#[test] fn goto_cached() { check(&["10 i=i+1:IF i<5 THEN GOTO 10", "20 PRINT i", "RUN", "LIST", "RUN"]); }
#[test] fn edit_after_syntax_error() { check(&["10 PRINT +", "RUN", "10 PRINT 5", "RUN"]); }
#[test] fn stop_in_loop_cont() { check(&["10 FOR i=1 TO 3:PRINT i:STOP:NEXT", "RUN", "CONT", "CONT", "CONT", "CONT"]); }
#[test] fn end_cont() { check(&["10 PRINT 1:END:PRINT 2", "RUN", "CONT"]); }
#[test] fn clear_in_program() { check(&["10 a=1:DIM b(5):CLEAR:PRINT a", "RUN"]); }
#[test] fn chain_needs_tape_skipped() { check(&["NEW", "10 REM", "RUN 10"]); }
#[test] fn run_line() { check(&["10 PRINT 1", "20 PRINT 2", "RUN 20", "RUN 30"]); }
#[test] fn new_inside_program() { check(&["10 PRINT 1:NEW", "RUN", "LIST"]); }
#[test] fn erl_err_direct() { check(&["ERROR 5", "PRINT ERR;ERL"]); }
#[test] fn time_xpos() { check(&["PRINT TIME;XPOS;YPOS;HIMEM;PI"]); }
#[test] fn rnd_sequence() { check(&["RANDOMIZE 1", "PRINT RND;RND;RND(0);RND(1)"]); }
#[test] fn randomize_prompt() { check(&["RANDOMIZE", "abc", "42"]); }
#[test] fn peek_rom() { check(&["PRINT PEEK(0);PEEK(&C000);PEEK(-1)"]); }
#[test] fn varptr() { check(&["a=1:PRINT @a;@b", "b$=\"x\":PRINT @b$"]); }
#[test] fn defreal_defstr() { check(&["DEFSTR a-z", "x=\"s\":PRINT x", "DEFREAL a-z", "x=1:PRINT x", "DEFINT z-a"]); }
#[test] fn if_nested() { check(&["10 FOR i=1 TO 4:IF i>1 THEN IF i>2 THEN PRINT \"c\" ELSE PRINT \"b\" ELSE PRINT \"a\"", "20 NEXT", "RUN"]); }
#[test] fn while_nested() { check(&["10 i=0:WHILE i<3:j=0:WHILE j<2:j=j+1:WEND:i=i+1:WEND:PRINT i;j", "RUN"]); }
#[test] fn for_step_zero_neg() { check(&["10 FOR i=5 TO 1 STEP -2:PRINT i;:NEXT:FOR j=1 TO 0:PRINT \"no\":NEXT:PRINT \"done\"", "RUN"]); }
#[test] fn for_next_quirk() { check(&["10 FOR i=1 TO 3:k=k+1:IF k>10 THEN 30", "15 FOR j=1 TO 0", "20 NEXT j,i", "30 PRINT i;k", "RUN"]); }
#[test] fn gosub_nesting() { check(&["10 GOSUB 20:PRINT \"end\":END", "20 GOSUB 30:RETURN", "30 FOR i=1 TO 2:RETURN", "RUN"]); }
#[test] fn mid_stmt_edge() { check(&["a$=\"abc\":MID$(a$,3)=\"xyz\":PRINT a$", "MID$(a$,5)=\"q\"", "b$=\"lit\":MID$(b$,1,1)=\"L\":PRINT b$"]); }
#[test] fn line_too_long() { check(&[&"PRINT \"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\""]); }
#[test] fn many_tokens_line() { check(&["10 a=1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1", "LIST", "RUN", "PRINT a"]); }
#[test] fn expression_complex() { check(&["PRINT (((((((((((((((((((((1)))))))))))))))))))))", "PRINT -2^2;NOT -1;NOT 1.5;1<2<3;\"a\"=\"a\" AND 1"]); }
#[test] fn exponent_tokens() { check(&["PRINT 1E;1E+;2E-;1.E2;.E2"]); }

// ---- added after measuring which parts of the ROM the tests did not reach ----
#[test] fn integer_for() { check(&["10 FOR i%=1 TO 5 STEP 2:PRINT i%;:NEXT:FOR j%=32760 TO 32767 STEP 5:NEXT", "RUN"]); }
#[test] fn for_string_var() { check(&["FOR a$=1 TO 2"]); }
#[test] fn window_swap() { check(&["WINDOW #1,1,10,1,5:WINDOW SWAP 0,1:PRINT \"x\":WINDOW SWAP 1"]); }
#[test] fn erase_many() { check(&["DIM a(2),b(3),c$(4),d%(5):b(1)=7:c$(2)=\"q\":d%(3)=9:ERASE b,a:PRINT c$(2);d%(3)", "ERASE c$,d%"]); }
#[test] fn input_arrays() { check(&["10 DIM a(3):INPUT a(1),b$(2):PRINT a(1);b$(2)", "RUN", "5,six"]); }
#[test] fn memory_moves_strings() { check(&["a$=STRING$(10,\"x\")+\"y\":b$=a$+\"z\"", "MEMORY HIMEM-100", "PRINT a$;b$;HIMEM", "MEMORY HIMEM+50", "PRINT b$"]); }
#[test] fn gc_with_temporaries() { check(&["10 DIM a$(30):FOR i=1 TO 30:a$(i)=STRING$(250,\"a\"):NEXT", "20 PRINT FRE(0)", "30 FOR i=1 TO 30 STEP 2:a$(i)=\"\":NEXT:b$=STRING$(250,\"b\")+\"\":PRINT FRE(\"\");LEN(b$)", "RUN"]); }
#[test] fn string_space_full() { check(&["10 DIM a$(200):FOR i=1 TO 200:a$(i)=STRING$(255,\"a\"):NEXT", "RUN", "PRINT i"]); }
#[test] fn inkey_joy_unt() { check(&["PRINT INKEY(1);INKEY$;JOY(0);JOY(1);UNT(65535);UNT(-1);INKEY(80)"]); }
#[test] fn round_variants() { check(&["PRINT ROUND(1234.567,-2);ROUND(-2.5);ROUND(2.5);ROUND(1.005,2);ROUND(7,40)", "PRINT ROUND(1,39);ROUND(1E30,-30);ROUND(123%,1)"]); }
#[test] fn using_point_fields() { check(&["PRINT USING \".##\";0.5;.123", "PRINT USING \"###.\";5", "PRINT USING \"+.##^^^^\";-0.000123", "PRINT USING \"##,###\";1234567"]); }
#[test] fn resume_line() { check(&["10 ON ERROR GOTO 100", "20 ERROR 5", "30 PRINT \"no\"", "40 PRINT \"yes\":END", "100 RESUME 40", "RUN"]); }
#[test] fn fn_string_params_gc() { check(&["10 DEF FNr$(a$,b$)=b$+a$+STRING$(200,\"z\")", "20 FOR i=1 TO 40:x$=FNr$(\"aa\",STR$(i)):NEXT:PRINT LEN(x$);FRE(\"\")", "RUN"]); }
#[test] fn number_formatting_more() { check(&["PRINT 1E-5;123456.7;1234567;12345678;0.1;0.01;0.001;1E9;999999999;-1E-39;1E-38", "PRINT 2^31;-2^31;2^-30;1/7;100/7;1E5/7"]); }
#[test] fn int_overflow_subtract() { check(&["a%=-32768:b%=1:PRINT a%-b%;-a%;a%*b%;a%\\-1"]); }
#[test] fn string_literal_unterminated() { check(&["PRINT \"abc   ", "a$=\"x  ", "PRINT LEN(a$)"]); }
#[test] fn mid_on_literal() { check(&["10 a$=\"literal\":MID$(a$,2)=\"XY\":PRINT a$", "RUN", "LIST"]); }
#[test] fn renum_with_refs() { check(&["10 GOSUB 30:GOTO 20", "20 RESTORE 30:IF 1 THEN 10 ELSE 30", "30 RETURN", "RENUM 100,10,10", "LIST", "RENUM", "LIST"]); }
#[test] fn ink_flash_speed() { check(&["INK 1,2,20:SPEED INK 10,10", "#TICKS 400", "PRINT 1"]); }
#[test] fn on_sq_channels() { check(&["10 ON SQ(2) GOSUB 100:ON SQ(4) GOSUB 100:SOUND 2,100,5:END", "100 RETURN", "RUN"]); }
#[test] fn space_function() { check(&["PRINT \"[\";SPACE$(5);\"]\";LEN(SPACE$(0))", "PRINT SPACE$(256)"]); }
#[test] fn gc_inside_expression() { check(&["10 DIM a$(130):FOR i=1 TO 130:a$(i)=STRING$(255,\"a\"):NEXT", "20 FOR i=1 TO 130 STEP 3:a$(i)=\"\":NEXT", "30 b$=LEFT$(STRING$(255,\"x\")+STRING$(10,\"y\"),5)+MID$(STRING$(200,\"z\"),3,2):PRINT b$", "RUN"]); }
#[test] fn gc_inside_fn() { check(&["10 DEF FNg$(p$)=p$+STRING$(250,\"q\")+\"\"", "20 DIM a$(130):FOR i=1 TO 130:a$(i)=STRING$(255,\"a\"):NEXT:FOR i=1 TO 130 STEP 2:a$(i)=\"\":NEXT", "30 FOR i=1 TO 20:x$=FNg$(STR$(i)):NEXT:PRINT LEN(x$)", "RUN"]); }
#[test] fn list_control_characters() { check(&["10 PRINT \"a\x07b\x01c\"", "LIST"]); }
#[test] fn using_exponents() { check(&["PRINT USING \"##.##^^^^\";0;1;-1E-20;9.999E37", "PRINT USING \"^^^^\";5", "PRINT USING \".^^^^\";5", "PRINT USING \"#^^^^\";123;0.5"]); }
#[test] fn data_followed_by_statement() { check(&["10 DATA 1,2:PRINT \"after data\"", "20 READ a,b:PRINT a;b", "RUN"]); }
#[test] fn dec_brackets() { check(&["PRINT DEC$((3.14159,\"##.##\"))", "PRINT DEC$(3,\"#\")", "PRINT (DEC$((7,\"###\")))", "PRINT DEC$((7,\"###\");DEC$((1E3,\"#,###\")"]); }
#[test] fn print_speed_separator() { check(&["PRINT \"AB\" SPEED 5 \"CD\"", "PRINT SPEED", "PRINT 1;SPEED;2", "10 ON ERROR GOTO 30:PRINT 1 SPEED 2:END", "30 PRINT ERR,ERL:END", "RUN"]); }
