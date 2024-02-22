#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "intr_conf.h"

#include "fs.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "proc.h"
#include "defs.h"
#include "file.h"

extern struct spinlock devlock;
extern pagetable_t kernel_pagetable;

#define CVI_GPIOA_BASE		0x03020000
#define CVI_GPIOB_BASE		0x03021000
#define CVI_GPIOC_BASE		0x03022000
#define CVI_GPIOD_BASE		0x03023000

#define GPIO_PIN 24

int
gpioread(int user_dst, uint64 dst, int n)
{
  uint32 value;

  acquire(&devlock);

  /* read gpio reg */
  value = mmio_read_32(CVI_GPIOC_BASE + 0x050) >> GPIO_PIN;
  value &= 0x1;

  if(either_copyout(user_dst, dst, &value, 1) == -1) {
    release(&devlock);
    return -1;
  }

  release(&devlock);
  return 1;
}

int
gpiowrite(int user_src, uint64 src, int n)
{
  uint32 value;
  acquire(&devlock);
  if(either_copyin(&value, user_src, src, 1) == -1) {
    release(&devlock);
    return -1;
  }

  value &= 0x1;
  value <<= GPIO_PIN;

  mmio_write_32(CVI_GPIOC_BASE, value);

  release(&devlock);
  return 1;
}

int
gpioinit(void)
{
  /* map gpio */
  kvmmap(kernel_pagetable, CVI_GPIOC_BASE, CVI_GPIOC_BASE, PGSIZE, PTE_R | PTE_W | PMA_DEVICE);
  /* set pinmux */
  PINMUX_CONFIG(PAD_AUD_AOUTR, XGPIOC_24);
  /* enable output */
	mmio_write_32(CVI_GPIOC_BASE + 4, 1 << GPIO_PIN);

  return 0;
}
