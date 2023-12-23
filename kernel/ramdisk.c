//
// ramdisk that uses the disk image loaded by qemu -initrd fs.img
//

#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "fs.h"
#include "buf.h"

static struct spinlock ramdisk_lock;
void
ramdisk_init(void)
{
  initlock(&ramdisk_lock, "ramdisk");
}

// If B_DIRTY is set, write buf to disk, clear B_DIRTY, set B_VALID.
// Else if B_VALID is not set, read buf from disk, set B_VALID.
void
ramdisk_rw(struct buf *b, int write)
{
  acquire(&ramdisk_lock);

  if(b->blockno >= FSSIZE)
    panic("ramdiskrw: blockno too big");

  /* we hold the buffer */
  b->disk = 1;

  uint64 diskaddr = b->blockno * BSIZE;
  char *addr = (char *)RAMDISK + diskaddr;

  if(write){
    // write
    memmove(addr, b->data, BSIZE);
  } else {
    // read
    memmove(b->data, addr, BSIZE);
  }

  b->disk = 0;   // disk is done with buf
  release(&ramdisk_lock);
}
