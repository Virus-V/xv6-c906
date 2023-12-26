#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"

#include "fs.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "proc.h"
#include "defs.h"
#include "file.h"

struct spinlock devlock;

static int
gpioread(int user_dst, uint64 dst, int n)
{
  uint32 value;

  acquire(&devlock);

  /* TODO: read balabala.. */
  value = 0xdeadbeef;

  if(either_copyout(user_dst, dst, &value, 4) == -1) {
    release(&devlock);
    return -1;
  }

  release(&devlock);
  return 4;
}

static int
gpiowrite(int user_src, uint64 src, int n)
{
  uint32 value;
  acquire(&devlock);
  if(either_copyin(&value, user_src, src, 4) == -1) {
    release(&devlock);
    return -1;
  }

  /* TODO: write balabala... */
  printf("gpio write: %p\n", value);

  release(&devlock);
  return 4;
}

void
clockintr()
{
  acquire(&tickslock);
  ticks++;
  wakeup(&ticks);
  release(&tickslock);
}

void
devinit()
{
  initlock(&devlock, "dev");

  /* gpio init */
  devsw[GPIO].read = gpioread;
  devsw[GPIO].write = gpiowrite;
}

// check if it's an external interrupt or software interrupt,
// and handle it.
// returns 2 if timer interrupt,
// 1 if other device,
// 0 if not recognized.
int
devintr()
{
  uint64 scause = r_scause();

  if((scause & 0x8000000000000000L) &&
     (scause & 0xff) == 9){
    // this is a supervisor external interrupt, via PLIC.

    // irq indicates which device interrupted.
    int irq = plic_claim();

    if(irq == UART0_IRQ){
      uartintr();
    } else if(irq == VIRTIO0_IRQ){
      virtio_disk_intr();
    } else if(irq){
      printf("unexpected interrupt irq=%d\n", irq);
    }

    // the PLIC allows each device to raise at most one
    // interrupt at a time; tell the PLIC the device is
    // now allowed to interrupt again.
    if(irq)
      plic_complete(irq);

    return 1;
  } else if(scause == 0x8000000000000005L){
    if(cpuid() == 0){
      struct sbi_ret ret;

      clockintr();

      ret = SBI_CALL1(SBI_EXT_ID_TIME, SBI_TIME_SET_TIMER, r_time() + (MTIMER_FREQ / TICKRATE));
      if (ret.error != SBI_SUCCESS)
        panic("ticks");
    }

    return 2;
  } else {
    return 0;
  }
}
