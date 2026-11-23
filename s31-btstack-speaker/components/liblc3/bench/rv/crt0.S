/* Bare-metal startup for rv32sim (bench only, never in firmware) */
    .section .text.start, "ax"
    .globl _start
_start:
    .option push
    .option norelax
    la   gp, __global_pointer$
    .option pop
    la   sp, __stack_top
    call main
    li   a7, 93
    ecall
1:  j 1b
