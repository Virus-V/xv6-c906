#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "defs.h"

volatile static int started = 0;

// start() jumps here in supervisor mode on all CPUs.
void
main()
{
#if 0
  sbi_console_putchar('a');
  sbi_console_putchar('a');
  sbi_console_putchar('a');
  sbi_console_putchar('a');
#endif

#if 1
  /* enable jtag */
  PINMUX_CONFIG(UART0_TX, JTAG_TMS);
  PINMUX_CONFIG(UART0_RX, JTAG_TCK);
  PINMUX_CONFIG(IIC0_SCL, JTAG_TDI);
  PINMUX_CONFIG(IIC0_SDA, JTAG_TDO);
#endif

  if(cpuid() == 0){
    consoleinit();
    printfinit();
    printf("\n");
    printf("xv6 kernel is booting\n");
    printf("\n");
    printf("helloworld!\n");
    kinit();         // physical page allocator
    kvminit();       // create kernel page table
    kvminithart();   // turn on paging
    procinit();      // process table
    trapinit();      // trap vectors
    trapinithart();  // install kernel trap vector
    plicinit();      // set up interrupt controller
    plicinithart();  // ask PLIC for device interrupts
    binit();         // buffer cache
    iinit();         // inode table
    fileinit();      // file table
                     //
#if 0
    virtio_disk_init(); // emulated hard disk
#else
    ramdisk_init(); // fs.img in ram
#endif

    userinit();      // first user process
    __sync_synchronize();
    started = 1;
  } else {
    while(started == 0)
      ;
    __sync_synchronize();
    printf("hart %d starting\n", cpuid());
    kvminithart();    // turn on paging
    trapinithart();   // install kernel trap vector
    plicinithart();   // ask PLIC for device interrupts
  }

  scheduler();
}
