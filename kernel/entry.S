        # qemu -kernel loads the kernel at 0x80000000
        # and causes each hart (i.e. CPU) to jump there.
        # kernel.ld causes the following code to
        # be placed at 0x80000000.
.section .text
.global _entry
.extern __bss_start
.extern __bss_end
_entry:

	/* Clear bss section */
	la t0, __bss_start
	la t1, __bss_end
	bgeu t0, t1, 2f
1:
	sw zero, (t0)
	addi t0, t0, 4
	bltu t0, t1, 1b
2:
        # set up a stack for C.
        # stack0 is declared in start.c,
        # with a 4096-byte stack per CPU.
        # sp = stack0 + (hartid * 4096)
        la sp, stack0
        li a0, 1024*4
        addi a1, zero, 1
        mul a0, a0, a1
        add sp, sp, a0
        # jump to start() in start.c
        call start
spin:
        j spin
