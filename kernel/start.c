#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "defs.h"
#include "sbi.h"

void main();
void timerinit();

// entry.S needs one stack per CPU.
__attribute__ ((aligned (16))) char stack0[4096 * NCPU];

// a scratch area per CPU for machine-mode timer interrupts.
//uint64 timer_scratch[NCPU][5];

// assembly code in kernelvec.S for machine-mode timer interrupt.
extern void timervec();

// entry.S jumps here in machine mode on stack0.
void
start()
{
  // set M Previous Privilege mode to Supervisor, for mret.
  unsigned long x = r_sstatus();
  x |= SSTATUS_SPP;
  w_sstatus(x);

  // set M Exception Program Counter to main, for mret.
  // requires gcc -mcmodel=medany
  w_sepc((uint64)main);

  // ask for clock interrupts.
  timerinit();

  // disable paging for now.
  w_satp(0);

  w_sie(r_sie() | SIE_SEIE | SIE_STIE | SIE_SSIE);

  // keep each CPU's hartid in its tp register, for cpuid().
  //int id = r_mhartid();
  int id = 0;
  w_tp(id);

  // switch to supervisor mode and jump to main().
  asm volatile("sret");
}


// arrange to receive timer interrupts.
// they will arrive in machine mode at
// at timervec in kernelvec.S,
// which turns them into software interrupts for
// devintr() in trap.c.
void
timerinit()
{
  struct sbi_ret ret;

  ret = SBI_CALL1(SBI_EXT_ID_TIME, SBI_TIME_SET_TIMER, r_time() + (MTIMER_FREQ / TICKRATE));
  if (ret.error != SBI_SUCCESS)
    panic("ticks");
}
