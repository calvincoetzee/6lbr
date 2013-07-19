
/******************************************************************************
 *
 * enet_io.ld - Linker configuration file for enet_io.
 *
 * Copyright (c) 2008-2012 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 * 
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's microcontroller products. The software is owned by
 * TI and/or its suppliers, and is protected under applicable copyright
 * laws. You may not combine this software with "viral" open-source
 * software in order to form a larger program.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 * 
 * This is part of revision 8555 of the EK-LM3S6965 Firmware Package.
 *
 *****************************************************************************/
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
ENTRY(ResetISR)
__Stack_Size = 0x1000 ;

PROVIDE ( _Stack_Size = __Stack_Size ) ;

__Stack_Init = 0x20000000;

_estack = __Stack_Init + __Stack_Size;


/*"PROVIDE" allows to easily override these values from an object file or the commmand line.*/
PROVIDE ( _Stack_Init = __Stack_Init ) ;

MEMORY
{
    CODE (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
    DATA (rwx) : ORIGIN = 0x20000500, LENGTH = 0x00010000
}
/* Section Definitions */

SECTIONS
{

/* Make sure the vector table is at address 0 */

.vectrom :
{
	KEEP(*(.isr_vector))
} >CODE =0

.text :
{
	KEEP(*(.init))
	*(.text .text.*)
	KEEP(*(.fini))
} >CODE =0

. = ALIGN(4);

.rodata :
  {
   	*(.rodata .rodata.*)
	*(.gnu.linkonce.r.*)
  } >CODE

  _etext = . ;
  PROVIDE (etext = .);

  .data :
  {
    _data = . ;
    *(.data)
    _edata = . ;
    PROVIDE (edata = .);
  } >DATA AT >CODE
  . = ALIGN(4);

/* .bss section which is used for uninitialized data */

  .bss :
  {
    __bss_start = . ;
    __bss_start__ = . ;
    *(.bss)
    *(COMMON)
    __bss_end = . ;
    __bss_end__ = . ;
    *(.noinit)
  } >DATA
  . = ALIGN(4);

   _end = .;
  PROVIDE (end = .);
  PROVIDE (_bss = __bss_start);
  PROVIDE (_ebss =__bss_end);
Main_Stack_Size = 0x00002000;
Process_Stack_Size = 0x00000000;

Stack_Size = Main_Stack_Size + Process_Stack_Size;
  .stack ORIGIN(DATA) + LENGTH(DATA) - Stack_Size :
  {
    __stack_start__ = . ;
    Main_Stack_Start = . ;
    . += Main_Stack_Size;
    Main_Stack_End = . ;
    Process_Stack_Start = . ;
    . += Process_Stack_Size;
    Process_Stack_End = . ;
    . = ALIGN(4);
    __stack_end__ = . ;
    Top_Stack = .;
  } >DATA
    /* Section start can be overridden via command line. */
    .coffee :
    {
    	_coffee_start = ABSOLUTE(.);
    	. = ALIGN(1024);
    	*(.coffeefiles)
    	. = ORIGIN(CODE);
    } > CODE
  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
}
