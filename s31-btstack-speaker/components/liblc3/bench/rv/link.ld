/* Flat memory map for rv32sim (bench only, never in firmware) */
ENTRY(_start)
MEMORY { RAM (rwx) : ORIGIN = 0x00100000, LENGTH = 15M }
SECTIONS
{
    .text : { *(.text.start) *(.text .text.*) } > RAM
    .rodata : { *(.rodata .rodata.* .srodata .srodata.*) } > RAM
    .data : { *(.data .data.*) } > RAM
    .sdata : { __global_pointer$ = . + 0x800; *(.sdata .sdata.*) } > RAM
    .bss : { *(.sbss .sbss.* .bss .bss.* COMMON) } > RAM
    __stack_top = ORIGIN(RAM) + LENGTH(RAM) + 1M - 16;
}
