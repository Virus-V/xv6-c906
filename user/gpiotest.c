// init: The initial user-level program

#include "kernel/types.h"
#include "kernel/stat.h"
#include "kernel/spinlock.h"
#include "kernel/sleeplock.h"
#include "kernel/fs.h"
#include "kernel/file.h"
#include "user/user.h"
#include "kernel/fcntl.h"

char *argv[] = { "sh", 0 };

int
main(void)
{
  int gpio_fd;

  if((gpio_fd = open("gpio", O_RDWR)) < 0){
    mknod("gpio", GPIO, 0);
    gpio_fd = open("gpio", O_RDWR);
  }

  if (gpio_fd < 0) {
    printf("gpio open failed!\n");
    exit(1);
  }

  printf("gpio open success! fd:%d\n", gpio_fd);

  uint32 value;

  read(gpio_fd, &value, sizeof(value));
  printf("read value: %x\n", value);

  value ++;
  write(gpio_fd, &value, sizeof(value));

  exit(0);
}
