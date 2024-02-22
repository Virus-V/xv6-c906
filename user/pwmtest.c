// init: The initial user-level program

#include "kernel/types.h"
#include "kernel/stat.h"
#include "kernel/spinlock.h"
#include "kernel/sleeplock.h"
#include "kernel/fs.h"
#include "kernel/file.h"
#include "user/user.h"
#include "kernel/fcntl.h"

struct pwm_ctrl {
  uint32_t period;
  uint32_t low_period;
  uint8_t enable;
};

int
main(void)
{
  int pwm_fd;

  if((pwm_fd = open("pwm", O_RDWR)) < 0){
    mknod("pwm", PWM, 0);
    pwm_fd = open("pwm", O_RDWR);
  }

  if (pwm_fd < 0) {
    printf("pwm open failed!\n");
    exit(1);
  }

  printf("pwm open success! fd:%d\n", pwm_fd);

  struct pwm_ctrl value;
  int i;

  value.enable = 0;
  value.period = 100;

  for (i = 0; i < 100; i++) {
    value.low_period = i;
    value.enable = 1;
    write(pwm_fd, &value, sizeof(value));
    printf("pwm H/L:%d/%d\n", value.period - value.low_period, value.low_period);
    sleep(250);
    value.enable = 0;
    write(pwm_fd, &value, sizeof(value));
  }

  close(pwm_fd);

  printf("pwm test done!\n");

  exit(0);
}
