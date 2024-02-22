#ifndef __ASSEMBLER__

#include "stdint.h"
#include "sbi.h"

// which hart (core) is this?
static inline uint64_t
r_mhartid()
{
  uint64_t x;
  asm volatile("csrr %0, mhartid" : "=r" (x) );
  return x;
}

// Machine Status Register, mstatus

#define MSTATUS_MPP_MASK (3L << 11) // previous mode.
#define MSTATUS_MPP_M (3L << 11)
#define MSTATUS_MPP_S (1L << 11)
#define MSTATUS_MPP_U (0L << 11)
#define MSTATUS_MIE (1L << 3)    // machine-mode interrupt enable.

static inline uint64_t
r_mstatus()
{
  uint64_t x;
  asm volatile("csrr %0, mstatus" : "=r" (x) );
  return x;
}

static inline void
w_mstatus(uint64_t x)
{
  asm volatile("csrw mstatus, %0" : : "r" (x));
}

// machine exception program counter, holds the
// instruction address to which a return from
// exception will go.
static inline void
w_mepc(uint64_t x)
{
  asm volatile("csrw mepc, %0" : : "r" (x));
}

// Supervisor Status Register, sstatus

#define SSTATUS_SPP (1L << 8)  // Previous mode, 1=Supervisor, 0=User
#define SSTATUS_SPIE (1L << 5) // Supervisor Previous Interrupt Enable
#define SSTATUS_UPIE (1L << 4) // User Previous Interrupt Enable
#define SSTATUS_SIE (1L << 1)  // Supervisor Interrupt Enable
#define SSTATUS_UIE (1L << 0)  // User Interrupt Enable

static inline uint64_t
r_sstatus()
{
  uint64_t x;
  asm volatile("csrr %0, sstatus" : "=r" (x) );
  return x;
}

static inline void
w_sstatus(uint64_t x)
{
  asm volatile("csrw sstatus, %0" : : "r" (x));
}

// Supervisor Interrupt Pending
static inline uint64_t
r_sip()
{
  uint64_t x;
  asm volatile("csrr %0, sip" : "=r" (x) );
  return x;
}

static inline void
w_sip(uint64_t x)
{
  asm volatile("csrw sip, %0" : : "r" (x));
}

// Supervisor Interrupt Enable
#define SIE_SEIE (1L << 9) // external
#define SIE_STIE (1L << 5) // timer
#define SIE_SSIE (1L << 1) // software
static inline uint64_t
r_sie()
{
  uint64_t x;
  asm volatile("csrr %0, sie" : "=r" (x) );
  return x;
}

static inline void
w_sie(uint64_t x)
{
  asm volatile("csrw sie, %0" : : "r" (x));
}

// Machine-mode Interrupt Enable
#define MIE_MEIE (1L << 11) // external
#define MIE_MTIE (1L << 7)  // timer
#define MIE_MSIE (1L << 3)  // software
static inline uint64_t
r_mie()
{
  uint64_t x;
  asm volatile("csrr %0, mie" : "=r" (x) );
  return x;
}

static inline void
w_mie(uint64_t x)
{
  asm volatile("csrw mie, %0" : : "r" (x));
}

// supervisor exception program counter, holds the
// instruction address to which a return from
// exception will go.
static inline void
w_sepc(uint64_t x)
{
  asm volatile("csrw sepc, %0" : : "r" (x));
}

static inline uint64_t
r_sepc()
{
  uint64_t x;
  asm volatile("csrr %0, sepc" : "=r" (x) );
  return x;
}

// Machine Exception Delegation
static inline uint64_t
r_medeleg()
{
  uint64_t x;
  asm volatile("csrr %0, medeleg" : "=r" (x) );
  return x;
}

static inline void
w_medeleg(uint64_t x)
{
  asm volatile("csrw medeleg, %0" : : "r" (x));
}

// Machine Interrupt Delegation
static inline uint64_t
r_mideleg()
{
  uint64_t x;
  asm volatile("csrr %0, mideleg" : "=r" (x) );
  return x;
}

static inline void
w_mideleg(uint64_t x)
{
  asm volatile("csrw mideleg, %0" : : "r" (x));
}

// Supervisor Trap-Vector Base Address
// low two bits are mode.
static inline void
w_stvec(uint64_t x)
{
  asm volatile("csrw stvec, %0" : : "r" (x));
}

static inline uint64_t
r_stvec()
{
  uint64_t x;
  asm volatile("csrr %0, stvec" : "=r" (x) );
  return x;
}

// Machine-mode interrupt vector
static inline void
w_mtvec(uint64_t x)
{
  asm volatile("csrw mtvec, %0" : : "r" (x));
}

// Physical Memory Protection
static inline void
w_pmpcfg0(uint64_t x)
{
  asm volatile("csrw pmpcfg0, %0" : : "r" (x));
}

static inline void
w_pmpaddr0(uint64_t x)
{
  asm volatile("csrw pmpaddr0, %0" : : "r" (x));
}

// supervisor address translation and protection;
// holds the address of the page table.
static inline void
w_satp(uint64_t x)
{
  asm volatile("csrw satp, %0" : : "r" (x));
}

static inline uint64_t
r_satp()
{
  uint64_t x;
  asm volatile("csrr %0, satp" : "=r" (x) );
  return x;
}

static inline void
w_mscratch(uint64_t x)
{
  asm volatile("csrw mscratch, %0" : : "r" (x));
}

static inline void
w_sscratch(uint64_t x)
{
  asm volatile("csrw sscratch, %0" : : "r" (x));
}

// Supervisor Trap Cause
static inline uint64_t
r_scause()
{
  uint64_t x;
  asm volatile("csrr %0, scause" : "=r" (x) );
  return x;
}

// Supervisor Trap Value
static inline uint64_t
r_stval()
{
  uint64_t x;
  asm volatile("csrr %0, stval" : "=r" (x) );
  return x;
}

// Machine-mode Counter-Enable
static inline void
w_mcounteren(uint64_t x)
{
  asm volatile("csrw mcounteren, %0" : : "r" (x));
}

static inline uint64_t
r_mcounteren()
{
  uint64_t x;
  asm volatile("csrr %0, mcounteren" : "=r" (x) );
  return x;
}

// machine-mode cycle counter
static inline uint64_t
r_time()
{
  uint64_t x;
  asm volatile("csrr %0, time" : "=r" (x) );
  return x;
}

// enable device interrupts
static inline void
intr_on()
{
  w_sstatus(r_sstatus() | SSTATUS_SIE);
}

// disable device interrupts
static inline void
intr_off()
{
  w_sstatus(r_sstatus() & ~SSTATUS_SIE);
}

// are device interrupts enabled?
static inline int
intr_get()
{
  uint64_t x = r_sstatus();
  return (x & SSTATUS_SIE) != 0;
}

static inline uint64_t
r_sp()
{
  uint64_t x;
  asm volatile("mv %0, sp" : "=r" (x) );
  return x;
}

// read and write tp, the thread pointer, which xv6 uses to hold
// this core's hartid (core number), the index into cpus[].
static inline uint64_t
r_tp()
{
  uint64_t x;
  asm volatile("mv %0, tp" : "=r" (x) );
  return x;
}

static inline void
w_tp(uint64_t x)
{
  asm volatile("mv tp, %0" : : "r" (x));
}

static inline uint64_t
r_ra()
{
  uint64_t x;
  asm volatile("mv %0, ra" : "=r" (x) );
  return x;
}

// flush the TLB.
static inline void
sfence_vma()
{
  // the zero, zero means flush all TLB entries.
  asm volatile("sfence.vma zero, zero");
}

typedef uint64_t pte_t;
typedef uint64_t *pagetable_t; // 512 PTEs

#endif // __ASSEMBLER__

#define PGSIZE 4096 // bytes per page
#define PGSHIFT 12  // bits of offset within a page

// use riscv's sv39 page table scheme.
#define SATP_SV39 (8L << 60)

#define MAKE_SATP(pagetable) (SATP_SV39 | (((uint64_t)pagetable) >> PGSHIFT))

#define PGROUNDUP(sz)  (((sz)+PGSIZE-1) & ~(PGSIZE-1))
#define PGROUNDDOWN(a) (((a)) & ~(PGSIZE-1))

#define PTE_V (1L << 0) // valid
#define PTE_R (1L << 1)
#define PTE_W (1L << 2)
#define PTE_X (1L << 3)
#define PTE_U (1L << 4) // user can access
#define PTE_A (1L << 6)
#define PTE_D (1L << 7)

/* T-Head extended attribute */
#define PTE_SEC (0x1L << 59)
#define PTE_B   (0x1L << 61)
#define PTE_C   (0x1L << 62)
#define PTE_SO  (0x1L << 63)

/* Physical Memory Attribute */
#if 1
#define PMA_DEVICE  (PTE_SO)
#define PMA_MEMORY  (PTE_B | PTE_C)
#else
#define PMA_DEVICE  (0)
#define PMA_MEMORY  (0)
#endif

// shift a physical address to the right place for a PTE.
#define PA2PTE(pa) ((((uint64_t)pa) >> PGSHIFT) << 10)

#define PTE2PA(pte) ((((pte) >> 10) & ((0x1L << 28) - 1)) << PGSHIFT)

#define PTE_FLAGS(pte) ((pte) & (0x3FF | (0x1DUL << 58)))

// extract the three 9-bit page table indices from a virtual address.
#define PXMASK          0x1FF // 9 bits
#define PXSHIFT(level)  (PGSHIFT+(9*(level)))
#define PX(level, va) ((((uint64_t) (va)) >> PXSHIFT(level)) & PXMASK)

// one beyond the highest possible virtual address.
// MAXVA is actually one bit less than the max allowed by
// Sv39, to avoid having to sign-extend virtual addresses
// that have the high bit set.
#define MAXVA (1L << (9 + 9 + 9 + 12 - 1))

#ifndef __ASSEMBLER__
/* milkv-duo */
static inline void mmio_write_32(uintptr_t addr, uint32_t  value)
{
	*(volatile uint32_t *)addr = value;
}

static inline uint32_t  mmio_read_32(uintptr_t addr)
{
	return *(volatile uint32_t *)addr;
}

static inline void mmio_clrsetbits_32(uintptr_t addr,
				uint32_t  clear,
				uint32_t  set)
{
	mmio_write_32(addr, (mmio_read_32(addr) & ~clear) | set);
}

/* clang-format off */

static inline void __raw_writeb(uint8_t val, volatile void *addr)
{
  asm volatile("sb %0, 0(%1)" : : "r"(val), "r"(addr));
}

static inline void __raw_writew(uint16_t val, volatile void *addr)
{
  asm volatile("sh %0, 0(%1)" : : "r"(val), "r"(addr));
}

static inline void __raw_writel(uint32_t val, volatile void *addr)
{
  asm volatile("sw %0, 0(%1)" : : "r"(val), "r"(addr));
}

#if __riscv_xlen != 32
static inline void __raw_writeq(uint64_t val, volatile void *addr)
{
  asm volatile("sd %0, 0(%1)" : : "r"(val), "r"(addr));
}
#endif

static inline uint8_t __raw_readb(const volatile void *addr)
{
  uint8_t val;

  asm volatile("lb %0, 0(%1)" : "=r"(val) : "r"(addr));
  return val;
}

static inline uint16_t __raw_readw(const volatile void *addr)
{
  uint16_t val;

  asm volatile("lh %0, 0(%1)" : "=r"(val) : "r"(addr));
  return val;
}

static inline uint32_t __raw_readl(const volatile void *addr)
{
  uint32_t val;

  asm volatile("lw %0, 0(%1)" : "=r"(val) : "r"(addr));
  return val;
}

#if __riscv_xlen != 32
static inline uint64_t __raw_readq(const volatile void *addr)
{
  uint64_t val;

  asm volatile("ld %0, 0(%1)" : "=r"(val) : "r"(addr));
  return val;
}
#endif

#define __io_br() do {} while (0)
#define __io_ar() __asm__ __volatile__ ("fence i,r" : : : "memory");
#define __io_bw() __asm__ __volatile__ ("fence w,o" : : : "memory");
#define __io_aw() do {} while (0)

#define readb(c)  ({ uint8_t  __v; __io_br(); __v = __raw_readb(c); __io_ar(); __v; })
#define readw(c)  ({ uint16_t __v; __io_br(); __v = __raw_readw(c); __io_ar(); __v; })
#define readl(c)  ({ uint32_t __v; __io_br(); __v = __raw_readl(c); __io_ar(); __v; })

#define writeb(v,c) ({ __io_bw(); __raw_writeb((v),(c)); __io_aw(); })
#define writew(v,c) ({ __io_bw(); __raw_writew((v),(c)); __io_aw(); })
#define writel(v,c) ({ __io_bw(); __raw_writel((v),(c)); __io_aw(); })
/* clang-format on */

#define  FMUX_GPIO_REG_IOCTRL_SD0_CLK  0x0
#define  FMUX_GPIO_REG_IOCTRL_SD0_CMD  0x4
#define  FMUX_GPIO_REG_IOCTRL_SD0_D0  0x8
#define  FMUX_GPIO_REG_IOCTRL_SD0_D1  0xc
#define  FMUX_GPIO_REG_IOCTRL_SD0_D2  0x10
#define  FMUX_GPIO_REG_IOCTRL_SD0_D3  0x14
#define  FMUX_GPIO_REG_IOCTRL_SD0_CD  0x18
#define  FMUX_GPIO_REG_IOCTRL_SD0_PWR_EN  0x1c
#define  FMUX_GPIO_REG_IOCTRL_SPK_EN  0x20
#define  FMUX_GPIO_REG_IOCTRL_UART0_TX  0x24
#define  FMUX_GPIO_REG_IOCTRL_UART0_RX  0x28
#define  FMUX_GPIO_REG_IOCTRL_SPINOR_HOLD_X  0x2c
#define  FMUX_GPIO_REG_IOCTRL_SPINOR_SCK  0x30
#define  FMUX_GPIO_REG_IOCTRL_SPINOR_MOSI  0x34
#define  FMUX_GPIO_REG_IOCTRL_SPINOR_WP_X  0x38
#define  FMUX_GPIO_REG_IOCTRL_SPINOR_MISO  0x3c
#define  FMUX_GPIO_REG_IOCTRL_SPINOR_CS_X  0x40
#define  FMUX_GPIO_REG_IOCTRL_JTAG_CPU_TMS  0x44
#define  FMUX_GPIO_REG_IOCTRL_JTAG_CPU_TCK  0x48
#define  FMUX_GPIO_REG_IOCTRL_IIC0_SCL  0x4c
#define  FMUX_GPIO_REG_IOCTRL_IIC0_SDA  0x50
#define  FMUX_GPIO_REG_IOCTRL_AUX0  0x54
#define  FMUX_GPIO_REG_IOCTRL_GPIO_ZQ  0x58
#define  FMUX_GPIO_REG_IOCTRL_PWR_VBAT_DET  0x5c
#define  FMUX_GPIO_REG_IOCTRL_PWR_RSTN  0x60
#define  FMUX_GPIO_REG_IOCTRL_PWR_SEQ1  0x64
#define  FMUX_GPIO_REG_IOCTRL_PWR_SEQ2  0x68
#define  FMUX_GPIO_REG_IOCTRL_PWR_WAKEUP0  0x6c
#define  FMUX_GPIO_REG_IOCTRL_PWR_BUTTON1  0x70
#define  FMUX_GPIO_REG_IOCTRL_XTAL_XIN  0x74
#define  FMUX_GPIO_REG_IOCTRL_PWR_GPIO0  0x78
#define  FMUX_GPIO_REG_IOCTRL_PWR_GPIO1  0x7c
#define  FMUX_GPIO_REG_IOCTRL_PWR_GPIO2  0x80
#define  FMUX_GPIO_REG_IOCTRL_SD1_GPIO1  0x84
#define  FMUX_GPIO_REG_IOCTRL_SD1_GPIO0  0x88
#define  FMUX_GPIO_REG_IOCTRL_SD1_D3  0x8c
#define  FMUX_GPIO_REG_IOCTRL_SD1_D2  0x90
#define  FMUX_GPIO_REG_IOCTRL_SD1_D1  0x94
#define  FMUX_GPIO_REG_IOCTRL_SD1_D0  0x98
#define  FMUX_GPIO_REG_IOCTRL_SD1_CMD  0x9c
#define  FMUX_GPIO_REG_IOCTRL_SD1_CLK  0xa0
#define  FMUX_GPIO_REG_IOCTRL_PWM0_BUCK  0xa4
#define  FMUX_GPIO_REG_IOCTRL_ADC1  0xa8
#define  FMUX_GPIO_REG_IOCTRL_USB_VBUS_DET  0xac
#define  FMUX_GPIO_REG_IOCTRL_MUX_SPI1_MISO  0xb0
#define  FMUX_GPIO_REG_IOCTRL_MUX_SPI1_MOSI  0xb4
#define  FMUX_GPIO_REG_IOCTRL_MUX_SPI1_CS  0xb8
#define  FMUX_GPIO_REG_IOCTRL_MUX_SPI1_SCK  0xbc
#define  FMUX_GPIO_REG_IOCTRL_PAD_ETH_TXP  0xc0
#define  FMUX_GPIO_REG_IOCTRL_PAD_ETH_TXM  0xc4
#define  FMUX_GPIO_REG_IOCTRL_PAD_ETH_RXP  0xc8
#define  FMUX_GPIO_REG_IOCTRL_PAD_ETH_RXM  0xcc
#define  FMUX_GPIO_REG_IOCTRL_GPIO_RTX  0xd0
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPIRX4N  0xd4
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPIRX4P  0xd8
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPIRX3N  0xdc
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPIRX3P  0xe0
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPIRX2N  0xe4
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPIRX2P  0xe8
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPIRX1N  0xec
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPIRX1P  0xf0
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPIRX0N  0xf4
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPIRX0P  0xf8
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPI_TXM2  0xfc
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPI_TXP2  0x100
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPI_TXM1  0x104
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPI_TXP1  0x108
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPI_TXM0  0x10c
#define  FMUX_GPIO_REG_IOCTRL_PAD_MIPI_TXP0  0x110
#define  FMUX_GPIO_REG_IOCTRL_PKG_TYPE0  0x114
#define  FMUX_GPIO_REG_IOCTRL_PKG_TYPE1  0x118
#define  FMUX_GPIO_REG_IOCTRL_PKG_TYPE2  0x11c
#define  FMUX_GPIO_REG_IOCTRL_PAD_AUD_AINL_MIC  0x120
#define  FMUX_GPIO_REG_IOCTRL_PAD_AUD_AINR_MIC  0x124
#define  FMUX_GPIO_REG_IOCTRL_PAD_AUD_AOUTL  0x128
#define  FMUX_GPIO_REG_IOCTRL_PAD_AUD_AOUTR  0x12c
#define  FMUX_GPIO_REG_DEVMATRIX_UART0_IP_SEL  0x1d4
#define  FMUX_GPIO_REG_DEVMATRIX_UART1_IP_SEL  0x1d8
#define  FMUX_GPIO_REG_DEVMATRIX_UART2_IP_SEL  0x1dc
#define  FMUX_GPIO_REG_DEVMATRIX_UART3_IP_SEL  0x1e0
#define  FMUX_GPIO_REG_DEVMATRIX_UART4_IP_SEL  0x1e4
#define  FMUX_GPIO_FUNCSEL_SD0_CLK   0x0
#define  FMUX_GPIO_FUNCSEL_SD0_CLK_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD0_CLK_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD0_CMD   0x4
#define  FMUX_GPIO_FUNCSEL_SD0_CMD_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD0_CMD_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD0_D0   0x8
#define  FMUX_GPIO_FUNCSEL_SD0_D0_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD0_D0_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD0_D1   0xc
#define  FMUX_GPIO_FUNCSEL_SD0_D1_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD0_D1_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD0_D2   0x10
#define  FMUX_GPIO_FUNCSEL_SD0_D2_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD0_D2_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD0_D3   0x14
#define  FMUX_GPIO_FUNCSEL_SD0_D3_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD0_D3_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD0_CD   0x18
#define  FMUX_GPIO_FUNCSEL_SD0_CD_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD0_CD_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD0_PWR_EN   0x1c
#define  FMUX_GPIO_FUNCSEL_SD0_PWR_EN_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD0_PWR_EN_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SPK_EN   0x20
#define  FMUX_GPIO_FUNCSEL_SPK_EN_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SPK_EN_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_UART0_TX   0x24
#define  FMUX_GPIO_FUNCSEL_UART0_TX_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_UART0_TX_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_UART0_RX   0x28
#define  FMUX_GPIO_FUNCSEL_UART0_RX_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_UART0_RX_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SPINOR_HOLD_X   0x2c
#define  FMUX_GPIO_FUNCSEL_SPINOR_HOLD_X_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SPINOR_HOLD_X_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SPINOR_SCK   0x30
#define  FMUX_GPIO_FUNCSEL_SPINOR_SCK_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SPINOR_SCK_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SPINOR_MOSI   0x34
#define  FMUX_GPIO_FUNCSEL_SPINOR_MOSI_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SPINOR_MOSI_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SPINOR_WP_X   0x38
#define  FMUX_GPIO_FUNCSEL_SPINOR_WP_X_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SPINOR_WP_X_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SPINOR_MISO   0x3c
#define  FMUX_GPIO_FUNCSEL_SPINOR_MISO_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SPINOR_MISO_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SPINOR_CS_X   0x40
#define  FMUX_GPIO_FUNCSEL_SPINOR_CS_X_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SPINOR_CS_X_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_JTAG_CPU_TMS   0x44
#define  FMUX_GPIO_FUNCSEL_JTAG_CPU_TMS_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_JTAG_CPU_TMS_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_JTAG_CPU_TCK   0x48
#define  FMUX_GPIO_FUNCSEL_JTAG_CPU_TCK_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_JTAG_CPU_TCK_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_IIC0_SCL   0x4c
#define  FMUX_GPIO_FUNCSEL_IIC0_SCL_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_IIC0_SCL_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_IIC0_SDA   0x50
#define  FMUX_GPIO_FUNCSEL_IIC0_SDA_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_IIC0_SDA_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_AUX0   0x54
#define  FMUX_GPIO_FUNCSEL_AUX0_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_AUX0_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_GPIO_ZQ   0x58
#define  FMUX_GPIO_FUNCSEL_GPIO_ZQ_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_GPIO_ZQ_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PWR_VBAT_DET   0x5c
#define  FMUX_GPIO_FUNCSEL_PWR_VBAT_DET_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PWR_VBAT_DET_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PWR_RSTN   0x60
#define  FMUX_GPIO_FUNCSEL_PWR_RSTN_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PWR_RSTN_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PWR_SEQ1   0x64
#define  FMUX_GPIO_FUNCSEL_PWR_SEQ1_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PWR_SEQ1_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PWR_SEQ2   0x68
#define  FMUX_GPIO_FUNCSEL_PWR_SEQ2_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PWR_SEQ2_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PWR_WAKEUP0   0x6c
#define  FMUX_GPIO_FUNCSEL_PWR_WAKEUP0_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PWR_WAKEUP0_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PWR_BUTTON1   0x70
#define  FMUX_GPIO_FUNCSEL_PWR_BUTTON1_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PWR_BUTTON1_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_XTAL_XIN   0x74
#define  FMUX_GPIO_FUNCSEL_XTAL_XIN_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_XTAL_XIN_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PWR_GPIO0   0x78
#define  FMUX_GPIO_FUNCSEL_PWR_GPIO0_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PWR_GPIO0_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PWR_GPIO1   0x7c
#define  FMUX_GPIO_FUNCSEL_PWR_GPIO1_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PWR_GPIO1_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PWR_GPIO2   0x80
#define  FMUX_GPIO_FUNCSEL_PWR_GPIO2_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PWR_GPIO2_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD1_GPIO1   0x84
#define  FMUX_GPIO_FUNCSEL_SD1_GPIO1_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD1_GPIO1_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD1_GPIO0   0x88
#define  FMUX_GPIO_FUNCSEL_SD1_GPIO0_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD1_GPIO0_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD1_D3   0x8c
#define  FMUX_GPIO_FUNCSEL_SD1_D3_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD1_D3_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD1_D2   0x90
#define  FMUX_GPIO_FUNCSEL_SD1_D2_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD1_D2_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD1_D1   0x94
#define  FMUX_GPIO_FUNCSEL_SD1_D1_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD1_D1_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD1_D0   0x98
#define  FMUX_GPIO_FUNCSEL_SD1_D0_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD1_D0_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD1_CMD   0x9c
#define  FMUX_GPIO_FUNCSEL_SD1_CMD_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD1_CMD_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_SD1_CLK   0xa0
#define  FMUX_GPIO_FUNCSEL_SD1_CLK_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_SD1_CLK_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PWM0_BUCK   0xa4
#define  FMUX_GPIO_FUNCSEL_PWM0_BUCK_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PWM0_BUCK_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_ADC1   0xa8
#define  FMUX_GPIO_FUNCSEL_ADC1_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_ADC1_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_USB_VBUS_DET   0xac
#define  FMUX_GPIO_FUNCSEL_USB_VBUS_DET_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_USB_VBUS_DET_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_MISO   0xb0
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_MISO_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_MISO_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_MOSI   0xb4
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_MOSI_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_MOSI_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_CS   0xb8
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_CS_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_CS_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_SCK   0xbc
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_SCK_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_MUX_SPI1_SCK_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_TXP   0xc0
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_TXP_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_TXP_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_TXM   0xc4
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_TXM_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_TXM_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_RXP   0xc8
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_RXP_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_RXP_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_RXM   0xcc
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_RXM_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_ETH_RXM_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_GPIO_RTX   0xd0
#define  FMUX_GPIO_FUNCSEL_GPIO_RTX_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_GPIO_RTX_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX4N   0xd4
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX4N_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX4N_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX4P   0xd8
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX4P_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX4P_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX3N   0xdc
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX3N_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX3N_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX3P   0xe0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX3P_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX3P_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX2N   0xe4
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX2N_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX2N_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX2P   0xe8
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX2P_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX2P_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX1N   0xec
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX1N_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX1N_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX1P   0xf0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX1P_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX1P_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX0N   0xf4
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX0N_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX0N_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX0P   0xf8
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX0P_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPIRX0P_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXM2   0xfc
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXM2_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXM2_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXP2   0x100
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXP2_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXP2_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXM1   0x104
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXM1_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXM1_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXP1   0x108
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXP1_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXP1_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXM0   0x10c
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXM0_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXM0_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXP0   0x110
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXP0_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_MIPI_TXP0_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PKG_TYPE0   0x114
#define  FMUX_GPIO_FUNCSEL_PKG_TYPE0_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PKG_TYPE0_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PKG_TYPE1   0x118
#define  FMUX_GPIO_FUNCSEL_PKG_TYPE1_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PKG_TYPE1_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PKG_TYPE2   0x11c
#define  FMUX_GPIO_FUNCSEL_PKG_TYPE2_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PKG_TYPE2_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AINL_MIC   0x120
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AINL_MIC_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AINL_MIC_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AINR_MIC   0x124
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AINR_MIC_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AINR_MIC_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AOUTL   0x128
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AOUTL_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AOUTL_MASK   0x7
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AOUTR   0x12c
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AOUTR_OFFSET 0
#define  FMUX_GPIO_FUNCSEL_PAD_AUD_AOUTR_MASK   0x7
#define  FMUX_GPIO_MUX_UART0_IP_SEL   0x1d4
#define  FMUX_GPIO_MUX_UART0_IP_SEL_OFFSET 0
#define  FMUX_GPIO_MUX_UART0_IP_SEL_MASK   0x7
#define  FMUX_GPIO_MUX_UART1_IP_SEL   0x1d8
#define  FMUX_GPIO_MUX_UART1_IP_SEL_OFFSET 0
#define  FMUX_GPIO_MUX_UART1_IP_SEL_MASK   0x7
#define  FMUX_GPIO_MUX_UART2_IP_SEL   0x1dc
#define  FMUX_GPIO_MUX_UART2_IP_SEL_OFFSET 0
#define  FMUX_GPIO_MUX_UART2_IP_SEL_MASK   0x7
#define  FMUX_GPIO_MUX_UART3_IP_SEL   0x1e0
#define  FMUX_GPIO_MUX_UART3_IP_SEL_OFFSET 0
#define  FMUX_GPIO_MUX_UART3_IP_SEL_MASK   0x7
#define  FMUX_GPIO_MUX_UART4_IP_SEL   0x1e4
#define  FMUX_GPIO_MUX_UART4_IP_SEL_OFFSET 0
#define  FMUX_GPIO_MUX_UART4_IP_SEL_MASK   0x7

#define SD0_CLK__SDIO0_CLK 0
#define SD0_CLK__IIC1_SDA 1
#define SD0_CLK__SPI0_SCK 2
#define SD0_CLK__XGPIOA_7 3
#define SD0_CLK__PWM_15 5
#define SD0_CLK__EPHY_LNK_LED 6
#define SD0_CLK__DBG_0 7
#define SD0_CMD__SDIO0_CMD 0
#define SD0_CMD__IIC1_SCL 1
#define SD0_CMD__SPI0_SDO 2
#define SD0_CMD__XGPIOA_8 3
#define SD0_CMD__PWM_14 5
#define SD0_CMD__EPHY_SPD_LED 6
#define SD0_CMD__DBG_1 7
#define SD0_D0__SDIO0_D_0 0
#define SD0_D0__CAM_MCLK1 1
#define SD0_D0__SPI0_SDI 2
#define SD0_D0__XGPIOA_9 3
#define SD0_D0__UART3_TX 4
#define SD0_D0__PWM_13 5
#define SD0_D0__WG0_D0 6
#define SD0_D0__DBG_2 7
#define SD0_D1__SDIO0_D_1 0
#define SD0_D1__IIC1_SDA 1
#define SD0_D1__AUX0 2
#define SD0_D1__XGPIOA_10 3
#define SD0_D1__UART1_TX 4
#define SD0_D1__PWM_12 5
#define SD0_D1__WG0_D1 6
#define SD0_D1__DBG_3 7
#define SD0_D2__SDIO0_D_2 0
#define SD0_D2__IIC1_SCL 1
#define SD0_D2__AUX1 2
#define SD0_D2__XGPIOA_11 3
#define SD0_D2__UART1_RX 4
#define SD0_D2__PWM_11 5
#define SD0_D2__WG1_D0 6
#define SD0_D2__DBG_4 7
#define SD0_D3__SDIO0_D_3 0
#define SD0_D3__CAM_MCLK0 1
#define SD0_D3__SPI0_CS_X 2
#define SD0_D3__XGPIOA_12 3
#define SD0_D3__UART3_RX 4
#define SD0_D3__PWM_10 5
#define SD0_D3__WG1_D1 6
#define SD0_D3__DBG_5 7
#define SD0_CD__SDIO0_CD 0
#define SD0_CD__XGPIOA_13 3
#define SD0_PWR_EN__SDIO0_PWR_EN 0
#define SD0_PWR_EN__XGPIOA_14 3
#define SPK_EN__XGPIOA_15 3
#define UART0_TX__UART0_TX 0
#define UART0_TX__CAM_MCLK1 1
#define UART0_TX__PWM_4 2
#define UART0_TX__XGPIOA_16 3
#define UART0_TX__UART1_TX 4
#define UART0_TX__AUX1 5
#define UART0_TX__JTAG_TMS 6
#define UART0_TX__DBG_6 7
#define UART0_RX__UART0_RX 0
#define UART0_RX__CAM_MCLK0 1
#define UART0_RX__PWM_5 2
#define UART0_RX__XGPIOA_17 3
#define UART0_RX__UART1_RX 4
#define UART0_RX__AUX0 5
#define UART0_RX__JTAG_TCK 6
#define UART0_RX__DBG_7 7
#define SPINOR_HOLD_X__SPINOR_HOLD_X 1
#define SPINOR_HOLD_X__SPINAND_HOLD 2
#define SPINOR_HOLD_X__XGPIOA_26 3
#define SPINOR_SCK__SPINOR_SCK 1
#define SPINOR_SCK__SPINAND_CLK 2
#define SPINOR_SCK__XGPIOA_22 3
#define SPINOR_MOSI__SPINOR_MOSI 1
#define SPINOR_MOSI__SPINAND_MOSI 2
#define SPINOR_MOSI__XGPIOA_25 3
#define SPINOR_WP_X__SPINOR_WP_X 1
#define SPINOR_WP_X__SPINAND_WP 2
#define SPINOR_WP_X__XGPIOA_27 3
#define SPINOR_MISO__SPINOR_MISO 1
#define SPINOR_MISO__SPINAND_MISO 2
#define SPINOR_MISO__XGPIOA_23 3
#define SPINOR_CS_X__SPINOR_CS_X 1
#define SPINOR_CS_X__SPINAND_CS 2
#define SPINOR_CS_X__XGPIOA_24 3
#define JTAG_CPU_TMS__JTAG_TMS 0
#define JTAG_CPU_TMS__CAM_MCLK0 1
#define JTAG_CPU_TMS__PWM_7 2
#define JTAG_CPU_TMS__XGPIOA_19 3
#define JTAG_CPU_TMS__UART1_RTS 4
#define JTAG_CPU_TMS__AUX0 5
#define JTAG_CPU_TMS__UART1_TX 6
#define JTAG_CPU_TCK__JTAG_TCK 0
#define JTAG_CPU_TCK__CAM_MCLK1 1
#define JTAG_CPU_TCK__PWM_6 2
#define JTAG_CPU_TCK__XGPIOA_18 3
#define JTAG_CPU_TCK__UART1_CTS 4
#define JTAG_CPU_TCK__AUX1 5
#define JTAG_CPU_TCK__UART1_RX 6
#define IIC0_SCL__CV_SCL0__CR_4WTDI 0
#define IIC0_SDA__CV_SDA0__CR_4WTDO 0
#define IIC0_SCL__JTAG_TDI 0
#define IIC0_SCL__UART1_TX 1
#define IIC0_SCL__UART2_TX 2
#define IIC0_SCL__XGPIOA_28 3
#define IIC0_SCL__IIC0_SCL 4
#define IIC0_SCL__WG0_D0 5
#define IIC0_SCL__DBG_10 7
#define IIC0_SDA__JTAG_TDO 0
#define IIC0_SDA__UART1_RX 1
#define IIC0_SDA__UART2_RX 2
#define IIC0_SDA__XGPIOA_29 3
#define IIC0_SDA__IIC0_SDA 4
#define IIC0_SDA__WG0_D1 5
#define IIC0_SDA__WG1_D0 6
#define IIC0_SDA__DBG_11 7
#define AUX0__AUX0 0
#define AUX0__XGPIOA_30 3
#define AUX0__IIS1_MCLK 4
#define AUX0__WG1_D1 6
#define AUX0__DBG_12 7
#define GPIO_ZQ__PWR_GPIO_24 3
#define GPIO_ZQ__PWM_2 4
#define PWR_VBAT_DET__PWR_VBAT_DET 0
#define PWR_RSTN__PWR_RSTN 0
#define PWR_SEQ1__PWR_SEQ1 0
#define PWR_SEQ1__PWR_GPIO_3 3
#define PWR_SEQ2__PWR_SEQ2 0
#define PWR_SEQ2__PWR_GPIO_4 3
#define PTEST__PWR_PTEST 0
#define PWR_WAKEUP0__PWR_WAKEUP0 0
#define PWR_WAKEUP0__PWR_IR0 1
#define PWR_WAKEUP0__PWR_UART0_TX 2
#define PWR_WAKEUP0__PWR_GPIO_6 3
#define PWR_WAKEUP0__UART1_TX 4
#define PWR_WAKEUP0__IIC4_SCL 5
#define PWR_WAKEUP0__EPHY_LNK_LED 6
#define PWR_WAKEUP0__WG2_D0 7
#define PWR_BUTTON1__PWR_BUTTON1 0
#define PWR_BUTTON1__PWR_GPIO_8 3
#define PWR_BUTTON1__UART1_RX 4
#define PWR_BUTTON1__IIC4_SDA 5
#define PWR_BUTTON1__EPHY_SPD_LED 6
#define PWR_BUTTON1__WG2_D1 7
#define XTAL_XIN__PWR_XTAL_CLKIN 0
#define PWR_GPIO0__PWR_GPIO_0 0
#define PWR_GPIO0__UART2_TX 1
#define PWR_GPIO0__PWR_UART0_RX 2
#define PWR_GPIO0__PWM_8 4
#define PWR_GPIO1__PWR_GPIO_1 0
#define PWR_GPIO1__UART2_RX 1
#define PWR_GPIO1__EPHY_LNK_LED 3
#define PWR_GPIO1__PWM_9 4
#define PWR_GPIO1__PWR_IIC_SCL 5
#define PWR_GPIO1__IIC2_SCL 6
#define PWR_GPIO1__IIC0_SDA 7
#define PWR_GPIO2__PWR_GPIO_2 0
#define PWR_GPIO2__PWR_SECTICK 2
#define PWR_GPIO2__EPHY_SPD_LED 3
#define PWR_GPIO2__PWM_10 4
#define PWR_GPIO2__PWR_IIC_SDA 5
#define PWR_GPIO2__IIC2_SDA 6
#define PWR_GPIO2__IIC0_SCL 7
#define SD1_GPIO1__UART4_TX 1
#define SD1_GPIO1__PWR_GPIO_26 3
#define SD1_GPIO1__PWM_10 7
#define SD1_GPIO0__UART4_RX 1
#define SD1_GPIO0__PWR_GPIO_25 3
#define SD1_GPIO0__PWM_11 7
#define SD1_D3__PWR_SD1_D3 0
#define SD1_D3__SPI2_CS_X 1
#define SD1_D3__IIC1_SCL 2
#define SD1_D3__PWR_GPIO_18 3
#define SD1_D3__CAM_MCLK0 4
#define SD1_D3__UART3_CTS 5
#define SD1_D3__PWR_SPINOR1_CS_X 6
#define SD1_D3__PWM_4 7
#define SD1_D2__PWR_SD1_D2 0
#define SD1_D2__IIC1_SCL 1
#define SD1_D2__UART2_TX 2
#define SD1_D2__PWR_GPIO_19 3
#define SD1_D2__CAM_MCLK0 4
#define SD1_D2__UART3_TX 5
#define SD1_D2__PWR_SPINOR1_HOLD_X 6
#define SD1_D2__PWM_5 7
#define SD1_D1__PWR_SD1_D1 0
#define SD1_D1__IIC1_SDA 1
#define SD1_D1__UART2_RX 2
#define SD1_D1__PWR_GPIO_20 3
#define SD1_D1__CAM_MCLK1 4
#define SD1_D1__UART3_RX 5
#define SD1_D1__PWR_SPINOR1_WP_X 6
#define SD1_D1__PWM_6 7
#define SD1_D0__PWR_SD1_D0 0
#define SD1_D0__SPI2_SDI 1
#define SD1_D0__IIC1_SDA 2
#define SD1_D0__PWR_GPIO_21 3
#define SD1_D0__CAM_MCLK1 4
#define SD1_D0__UART3_RTS 5
#define SD1_D0__PWR_SPINOR1_MISO 6
#define SD1_D0__PWM_7 7
#define SD1_CMD__PWR_SD1_CMD 0
#define SD1_CMD__SPI2_SDO 1
#define SD1_CMD__IIC3_SCL 2
#define SD1_CMD__PWR_GPIO_22 3
#define SD1_CMD__CAM_VS0 4
#define SD1_CMD__EPHY_LNK_LED 5
#define SD1_CMD__PWR_SPINOR1_MOSI 6
#define SD1_CMD__PWM_8 7
#define SD1_CLK__PWR_SD1_CLK 0
#define SD1_CLK__SPI2_SCK 1
#define SD1_CLK__IIC3_SDA 2
#define SD1_CLK__PWR_GPIO_23 3
#define SD1_CLK__CAM_HS0 4
#define SD1_CLK__EPHY_SPD_LED 5
#define SD1_CLK__PWR_SPINOR1_SCK 6
#define SD1_CLK__PWM_9 7
#define PWM0_BUCK__PWM_0 0
#define PWM0_BUCK__XGPIOB_0 3
#define ADC1__XGPIOB_3 3
#define ADC1__KEY_COL2 4
#define ADC1__PWM_3 6
#define USB_VBUS_DET__USB_VBUS_DET 0
#define USB_VBUS_DET__XGPIOB_6 3
#define USB_VBUS_DET__CAM_MCLK0 4
#define USB_VBUS_DET__CAM_MCLK1 5
#define USB_VBUS_DET__PWM_4 6
#define MUX_SPI1_MISO__UART3_RTS 1
#define MUX_SPI1_MISO__IIC1_SDA 2
#define MUX_SPI1_MISO__XGPIOB_8 3
#define MUX_SPI1_MISO__PWM_9 4
#define MUX_SPI1_MISO__KEY_COL1 5
#define MUX_SPI1_MISO__SPI1_SDI 6
#define MUX_SPI1_MISO__DBG_14 7
#define MUX_SPI1_MOSI__UART3_RX 1
#define MUX_SPI1_MOSI__IIC1_SCL 2
#define MUX_SPI1_MOSI__XGPIOB_7 3
#define MUX_SPI1_MOSI__PWM_8 4
#define MUX_SPI1_MOSI__KEY_COL0 5
#define MUX_SPI1_MOSI__SPI1_SDO 6
#define MUX_SPI1_MOSI__DBG_13 7
#define MUX_SPI1_CS__UART3_CTS 1
#define MUX_SPI1_CS__CAM_MCLK0 2
#define MUX_SPI1_CS__XGPIOB_10 3
#define MUX_SPI1_CS__PWM_11 4
#define MUX_SPI1_CS__KEY_ROW3 5
#define MUX_SPI1_CS__SPI1_CS_X 6
#define MUX_SPI1_CS__DBG_16 7
#define MUX_SPI1_SCK__UART3_TX 1
#define MUX_SPI1_SCK__CAM_MCLK1 2
#define MUX_SPI1_SCK__XGPIOB_9 3
#define MUX_SPI1_SCK__PWM_10 4
#define MUX_SPI1_SCK__KEY_ROW2 5
#define MUX_SPI1_SCK__SPI1_SCK 6
#define MUX_SPI1_SCK__DBG_15 7
#define PAD_ETH_TXP__UART3_RX 1
#define PAD_ETH_TXP__IIC1_SCL 2
#define PAD_ETH_TXP__XGPIOB_25 3
#define PAD_ETH_TXP__PWM_13 4
#define PAD_ETH_TXP__CAM_MCLK0 5
#define PAD_ETH_TXP__SPI1_SDO 6
#define PAD_ETH_TXP__IIS2_LRCK 7
#define PAD_ETH_TXM__UART3_RTS 1
#define PAD_ETH_TXM__IIC1_SDA 2
#define PAD_ETH_TXM__XGPIOB_24 3
#define PAD_ETH_TXM__PWM_12 4
#define PAD_ETH_TXM__CAM_MCLK1 5
#define PAD_ETH_TXM__SPI1_SDI 6
#define PAD_ETH_TXM__IIS2_BCLK 7
#define PAD_ETH_RXP__UART3_TX 1
#define PAD_ETH_RXP__CAM_MCLK1 2
#define PAD_ETH_RXP__XGPIOB_27 3
#define PAD_ETH_RXP__PWM_15 4
#define PAD_ETH_RXP__CAM_HS0 5
#define PAD_ETH_RXP__SPI1_SCK 6
#define PAD_ETH_RXP__IIS2_DO 7
#define PAD_ETH_RXM__UART3_CTS 1
#define PAD_ETH_RXM__CAM_MCLK0 2
#define PAD_ETH_RXM__XGPIOB_26 3
#define PAD_ETH_RXM__PWM_14 4
#define PAD_ETH_RXM__CAM_VS0 5
#define PAD_ETH_RXM__SPI1_CS_X 6
#define PAD_ETH_RXM__IIS2_DI 7
#define GPIO_RTX__VI0_D_15 1
#define GPIO_RTX__XGPIOB_23 3
#define GPIO_RTX__PWM_1 4
#define GPIO_RTX__CAM_MCLK0 5
#define GPIO_RTX__IIS2_MCLK 7
#define PAD_MIPIRX4N__VI0_CLK 1
#define PAD_MIPIRX4N__IIC0_SCL 2
#define PAD_MIPIRX4N__XGPIOC_2 3
#define PAD_MIPIRX4N__IIC1_SDA 4
#define PAD_MIPIRX4N__CAM_MCLK0 5
#define PAD_MIPIRX4N__KEY_ROW0 6
#define PAD_MIPIRX4N__MUX_SPI1_SCK 7
#define PAD_MIPIRX4P__VI0_D_0 1
#define PAD_MIPIRX4P__IIC0_SDA 2
#define PAD_MIPIRX4P__XGPIOC_3 3
#define PAD_MIPIRX4P__IIC1_SCL 4
#define PAD_MIPIRX4P__CAM_MCLK1 5
#define PAD_MIPIRX4P__KEY_ROW1 6
#define PAD_MIPIRX4P__MUX_SPI1_CS 7
#define PAD_MIPIRX3N__VI0_D_1 1
#define PAD_MIPIRX3N__XGPIOC_4 3
#define PAD_MIPIRX3N__CAM_MCLK0 4
#define PAD_MIPIRX3N__MUX_SPI1_MISO 7
#define PAD_MIPIRX3P__VI0_D_2 1
#define PAD_MIPIRX3P__XGPIOC_5 3
#define PAD_MIPIRX3P__MUX_SPI1_MOSI 7
#define PAD_MIPIRX2N__VI0_D_3 1
#define PAD_MIPIRX2N__XGPIOC_6 3
#define PAD_MIPIRX2N__IIC4_SCL 5
#define PAD_MIPIRX2N__DBG_6 7
#define PAD_MIPIRX2P__VI0_D_4 1
#define PAD_MIPIRX2P__XGPIOC_7 3
#define PAD_MIPIRX2P__IIC4_SDA 5
#define PAD_MIPIRX2P__DBG_7 7
#define PAD_MIPIRX1N__VI0_D_5 1
#define PAD_MIPIRX1N__XGPIOC_8 3
#define PAD_MIPIRX1N__KEY_ROW3 6
#define PAD_MIPIRX1N__DBG_8 7
#define PAD_MIPIRX1P__VI0_D_6 1
#define PAD_MIPIRX1P__XGPIOC_9 3
#define PAD_MIPIRX1P__IIC1_SDA 4
#define PAD_MIPIRX1P__KEY_ROW2 6
#define PAD_MIPIRX1P__DBG_9 7
#define PAD_MIPIRX0N__VI0_D_7 1
#define PAD_MIPIRX0N__XGPIOC_10 3
#define PAD_MIPIRX0N__IIC1_SCL 4
#define PAD_MIPIRX0N__CAM_MCLK1 5
#define PAD_MIPIRX0N__DBG_10 7
#define PAD_MIPIRX0P__VI0_D_8 1
#define PAD_MIPIRX0P__XGPIOC_11 3
#define PAD_MIPIRX0P__CAM_MCLK0 4
#define PAD_MIPIRX0P__DBG_11 7
#define PAD_MIPI_TXM2__VI0_D_13 1
#define PAD_MIPI_TXM2__IIC0_SDA 2
#define PAD_MIPI_TXM2__XGPIOC_16 3
#define PAD_MIPI_TXM2__IIC1_SDA 4
#define PAD_MIPI_TXM2__PWM_8 5
#define PAD_MIPI_TXM2__SPI0_SCK 6
#define PAD_MIPI_TXP2__VI0_D_14 1
#define PAD_MIPI_TXP2__IIC0_SCL 2
#define PAD_MIPI_TXP2__XGPIOC_17 3
#define PAD_MIPI_TXP2__IIC1_SCL 4
#define PAD_MIPI_TXP2__PWM_9 5
#define PAD_MIPI_TXP2__SPI0_CS_X 6
#define PAD_MIPI_TXP2__IIS1_MCLK 7
#define PAD_MIPI_TXM1__SPI3_SDO 0
#define PAD_MIPI_TXM1__VI0_D_11 1
#define PAD_MIPI_TXM1__IIS1_LRCK 2
#define PAD_MIPI_TXM1__XGPIOC_14 3
#define PAD_MIPI_TXM1__IIC2_SDA 4
#define PAD_MIPI_TXM1__PWM_10 5
#define PAD_MIPI_TXM1__SPI0_SDO 6
#define PAD_MIPI_TXM1__DBG_14 7
#define PAD_MIPI_TXP1__SPI3_SDI 0
#define PAD_MIPI_TXP1__VI0_D_12 1
#define PAD_MIPI_TXP1__IIS1_DO 2
#define PAD_MIPI_TXP1__XGPIOC_15 3
#define PAD_MIPI_TXP1__IIC2_SCL 4
#define PAD_MIPI_TXP1__PWM_11 5
#define PAD_MIPI_TXP1__SPI0_SDI 6
#define PAD_MIPI_TXP1__DBG_15 7
#define PAD_MIPI_TXM0__SPI3_SCK 0
#define PAD_MIPI_TXM0__VI0_D_9 1
#define PAD_MIPI_TXM0__IIS1_DI 2
#define PAD_MIPI_TXM0__XGPIOC_12 3
#define PAD_MIPI_TXM0__CAM_MCLK1 4
#define PAD_MIPI_TXM0__PWM_14 5
#define PAD_MIPI_TXM0__CAM_VS0 6
#define PAD_MIPI_TXM0__DBG_12 7
#define PAD_MIPI_TXP0__SPI3_CS_X 0
#define PAD_MIPI_TXP0__VI0_D_10 1
#define PAD_MIPI_TXP0__IIS1_BCLK 2
#define PAD_MIPI_TXP0__XGPIOC_13 3
#define PAD_MIPI_TXP0__CAM_MCLK0 4
#define PAD_MIPI_TXP0__PWM_15 5
#define PAD_MIPI_TXP0__CAM_HS0 6
#define PAD_MIPI_TXP0__DBG_13 7
#define PKG_TYPE0__PKG_TYPE0 0
#define PKG_TYPE1__PKG_TYPE1 0
#define PKG_TYPE2__PKG_TYPE2 0
#define PAD_AUD_AINL_MIC__XGPIOC_23 3
#define PAD_AUD_AINL_MIC__IIS1_BCLK 4
#define PAD_AUD_AINL_MIC__IIS2_BCLK 5
#define PAD_AUD_AINR_MIC__XGPIOC_22 3
#define PAD_AUD_AINR_MIC__IIS1_DO 4
#define PAD_AUD_AINR_MIC__IIS2_DI 5
#define PAD_AUD_AINR_MIC__IIS1_DI 6
#define PAD_AUD_AOUTL__XGPIOC_25 3
#define PAD_AUD_AOUTL__IIS1_LRCK 4
#define PAD_AUD_AOUTL__IIS2_LRCK 5
#define PAD_AUD_AOUTR__XGPIOC_24 3
#define PAD_AUD_AOUTR__IIS1_DI 4
#define PAD_AUD_AOUTR__IIS2_DO 5
#define PAD_AUD_AOUTR__IIS1_DO 6

#define PINMUX_BASE 0x03001000
#define PINMUX_MASK(PIN_NAME) FMUX_GPIO_FUNCSEL_##PIN_NAME##_MASK
#define PINMUX_OFFSET(PIN_NAME) FMUX_GPIO_FUNCSEL_##PIN_NAME##_OFFSET
#define PINMUX_VALUE(PIN_NAME, FUNC_NAME) PIN_NAME##__##FUNC_NAME
#define PINMUX_CONFIG(PIN_NAME, FUNC_NAME) \
        mmio_clrsetbits_32(PINMUX_BASE + FMUX_GPIO_FUNCSEL_##PIN_NAME, \
            FMUX_GPIO_FUNCSEL_##PIN_NAME##_MASK << FMUX_GPIO_FUNCSEL_##PIN_NAME##_OFFSET, \
            PIN_NAME##__##FUNC_NAME)

#endif

#define MTIMER_FREQ 25000000
