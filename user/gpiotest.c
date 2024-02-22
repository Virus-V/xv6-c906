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

  uint8_t value;
  int i;

  for (i = 0; i < 300; i++) {
    value = 1;
    write(gpio_fd, &value, sizeof(value));
    sleep(250);
    value = 0;
    write(gpio_fd, &value, sizeof(value));
    sleep(250);
  }

  close(gpio_fd);

  printf("gpio test done!\n");

  exit(0);
}
