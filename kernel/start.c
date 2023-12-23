#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "defs.h"
#include "sbi.h"

void main();
// entry.S needs one stack per CPU.
__attribute__ ((aligned (16))) char stack0[4096 * NCPU];

// a scratch area per CPU for machine-mode timer interrupts.
//uint64 timer_scratch[NCPU][5];

// assembly code in kernelvec.S for machine-mode timer interrupt.
extern void timervec();

void
sbi_init(void)
{
}

static void exception_puts(const char *str) {
  int i;
  for (i = 0; str[i] != 0; i++) {
    if(str[i] == '\n') {
      sbi_console_putchar('\r');
    }
    sbi_console_putchar(str[i]);
  }
}

void __attribute__ ((aligned(4)))
trap_pre_sched()
{
#if 0
  uint64 cause, tval, epc;

  cause = r_scause();
  tval = r_stval();
  epc = r_sepc();
#endif

  exception_puts("exception!!!\n");
  while(1);
}

// entry.S jumps here in machine mode on stack0.
void
start()
{
#if 0
  // set M Previous Privilege mode to Supervisor, for mret.
  unsigned long x = r_sstatus();
  x |= SSTATUS_SPP;
  w_sstatus(x);

  // set M Exception Program Counter to main, for mret.
  // requires gcc -mcmodel=medany
  w_sepc((uint64)main);

  // disable paging for now.
  w_satp(0);

  w_sie(r_sie() | SIE_SEIE | SIE_STIE | SIE_SSIE);

  // ask for clock interrupts.
  //timerinit();
  // call opensbi

  // keep each CPU's hartid in its tp register, for cpuid().
  //int id = r_mhartid();
  int id = 0;
  w_tp(id);

  // switch to supervisor mode and jump to main().
  asm volatile("sret");
#endif
  w_stvec((uint64)trap_pre_sched);
  w_satp(0);
  w_tp(0);
  main();
  while(1);
}
