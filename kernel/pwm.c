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

#define CVI_PWM0_BASE 0x03060000
#define CVI_PWM1_BASE 0x03061000
#define CVI_PWM2_BASE 0x03062000
#define CVI_PWM3_BASE 0x03063000

struct cvitek_pwm_regs {
	volatile uint32_t hlperiod0;/* 0x00 */
	volatile uint32_t period0;/* 0x04 */
	volatile uint32_t hlperiod1;/* 0x08 */
	volatile uint32_t period1;/* 0x0c */
	volatile uint32_t hlperiod2;/* 0x10 */
	volatile uint32_t period2;/* 0x14 */
	volatile uint32_t hlperiod3;/* 0x18 */
	volatile uint32_t period3;/* 0x1c */
	volatile uint32_t reserved_1[8];
	volatile uint32_t polarity;/* 0x40 */
	volatile uint32_t pwmstart;/* 0x44 */
	volatile uint32_t pwmdone;/* 0x48 */
	volatile uint32_t pwmupdate;/* 0x4c */
	volatile uint32_t pcount0;/* 0x50 */
	volatile uint32_t pcount1;/* 0x54 */
	volatile uint32_t pcount2;/* 0x58 */
	volatile uint32_t pcount3;/* 0x5c */
	volatile uint32_t pulsecount0;/* 0x60 */
	volatile uint32_t pulsecount1;/* 0x64 */
	volatile uint32_t pulsecount2;/* 0x68 */
	volatile uint32_t pulsecount3;/* 0x6c */
	volatile uint32_t reserved_2[4];
	volatile uint32_t shiftcount0;/* 0x80 */
	volatile uint32_t shiftcount1;/* 0x84 */
	volatile uint32_t shiftcount2;/* 0x88 */
	volatile uint32_t shiftcount3;/* 0x8c */
	volatile uint32_t shiftstart;/* 0x90 */
	volatile uint32_t reserved_3[15];
	volatile uint32_t pwm_oe;/* 0xd0 */
};

static struct cvitek_pwm_regs *pwm1 = (struct cvitek_pwm_regs *)CVI_PWM1_BASE;

struct pwm_ctrl {
  uint32_t period;
  uint32_t low_period;
  uint8_t enable;
};

int
pwmread(int user_dst, uint64 dst, int n)
{
  struct pwm_ctrl value;

  acquire(&devlock);

  /* read reg */
  value.enable = pwm1->pwm_oe & (0x1 << 1);
  value.period = pwm1->period1;
  value.low_period = pwm1->hlperiod1;

  if(either_copyout(user_dst, dst, &value, sizeof(value)) == -1) {
    release(&devlock);
    return -1;
  }

  release(&devlock);

  return sizeof(value);
}

int
pwmwrite(int user_src, uint64 src, int n)
{
  struct pwm_ctrl value;

  acquire(&devlock);
  if(either_copyin(&value, user_src, src, sizeof(value)) == -1) {
    release(&devlock);
    return -1;
  }

  pwm1->period1 = value.period;
  pwm1->hlperiod1 = value.low_period;

  if (value.enable) {
    /* start pwm*/
    pwm1->pwm_oe |= 0x1 << 1;
    pwm1->pwmstart |= 0x1 << 1;
  } else {
    pwm1->pwm_oe &= ~(0x1 << 1);
    pwm1->pwmstart &= ~(0x1 << 1);
  }

  release(&devlock);
  return sizeof(value);
}

int
pwminit(void)
{
  /* map device */
  kvmmap(kernel_pagetable, CVI_PWM1_BASE, CVI_PWM1_BASE, PGSIZE, PTE_R | PTE_W | PMA_DEVICE);
  /* set pinmux */
  PINMUX_CONFIG(SD1_D2, PWM_5);

  return 0;
}
