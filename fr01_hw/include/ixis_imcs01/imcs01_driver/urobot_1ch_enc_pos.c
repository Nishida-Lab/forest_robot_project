#include <fcntl.h>          /* for open */
#include <stdio.h>
#include <unistd.h>         /* for read, close */
#include <sys/ioctl.h>      /* for ioctl */
#include <sys/types.h>      /* for open */
#include <sys/stat.h>       /* for open */

#include <signal.h>

#include "driver/urbtc.h"          /* Linux specific part */
#include "driver/urobotc.h"        /* OS independent part */

#undef __BIG_ENDIAN

void exit_program();

int quit_flag = 1;
int fd;

int main(int argc, char **argv)
{
  struct uin buf;
  struct uout obuf;
  struct ccmd cmd;
  int i,j=0;

  char *dev = "/dev/urbtc0";
  
  signal(SIGINT, exit_program);

  if (argc>1)
	dev = argv[1];

  if ((fd = open(dev, O_RDWR)) == -1) {
    fprintf(stderr, "%s: Open error\n", dev);
    exit(1);
  }

  cmd.retval = 0 /* RETURN_VAL */;
  cmd.setoffset  = CH0 | CH1 | CH2 | CH3;
  cmd.setcounter = CH0 | CH1 | CH2 | CH3;
  cmd.resetint   = CH0 | CH1 | CH2 | CH3;

  /*CH0 をエンコーダ入力, CH2を CH0 のエンコーダ入力に設定*/
  cmd.selin = SET_SELECT | SET_CH2_HIN;
  cmd.dout = 0;

  /*CH0をDA出力，CH2をPWM出力に設定*/
  cmd.selout = SET_SELECT | CH2;

  cmd.offset[0] = cmd.offset[1] = cmd.offset[2] = cmd.offset[3] = 32768;
  cmd.counter[0] = cmd.counter[1] = cmd.counter[2] = cmd.counter[3] = 0;
  cmd.posneg = SET_POSNEG | CH0| CH1 | CH2 | CH3; /*POS PWM out*/
  cmd.breaks = SET_BREAKS | CH0 | CH1 | CH2 | CH3; /*No Brake*/
  cmd.magicno = 0x00;

  cmd.wrrom = 0; /* WR_MAGIC | WR_OFFSET | WR_SELOUT;*/

  if (ioctl(fd, URBTC_COUNTER_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    exit(1);
  }

  if (write(fd, &cmd, sizeof(cmd)) < 0) {
    fprintf(stderr, "write error\n");
    exit(1);
  }

  for (i=0; i<4; i++) {
    obuf.ch[i].x = 0;
    obuf.ch[i].d = 0;
    obuf.ch[i].kp = 10;
    obuf.ch[i].kpx = 1;
    obuf.ch[i].kd = 0;
    obuf.ch[i].kdx = 1;
    obuf.ch[i].ki = 0;
    obuf.ch[i].kix = 1;
  }

  /*set scmd mode*/
  if (ioctl(fd, URBTC_DESIRE_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_DESIRE_SET error\n");
    exit(1);
  }

  if (ioctl(fd, URBTC_CONTINUOUS_READ) < 0){
    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    exit(1);
  }

  if (ioctl(fd, URBTC_BUFREAD) < 0){
    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    exit(1);
  }

  while(quit_flag) {

    unsigned short x_d= 100 * sin(j*3.14/655.360);
    x_d <<= 5;

    /*CH2の制御ループを用いる*/
    obuf.ch[2].x = x_d;

    if (write(fd, &obuf, sizeof(obuf)) > 0) {
      j++;
      //printf("OK\n");
    } else {
      printf("write err\n");
      break;
    }
  }

  close(fd);

  return 0;
}

void exit_program(sig, code, scp, addr)
int sig;
int code;
struct sigcontext *scp;
char *addr;
{
  quit_flag = 0;
  fprintf(stderr, "kill signal is received\n");
}
