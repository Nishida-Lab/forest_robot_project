#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>         /* for read, write, close */
#include <sys/ioctl.h>      /* for ioctl */
#include <sys/types.h>      /* for open */
#include <sys/stat.h>       /* for open */
#include <fcntl.h>          /* for open */

#include <signal.h>

#include "driver/urbtc.h"          /* Linux specific part */
#include "driver/urobotc.h"        /* OS independent part */

void exit_program();

int quit_flag = 1;
int fd;

int main(int argc, char **argv)
{
  struct uin ibuf;
  struct uout obuf;
  struct ccmd cmd;
  int i;

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

  cmd.selin = CH0 | CH1 | CH2 | CH3 | SET_SELECT;

#if 0
  cmd.dout = DOUT0 | DOUT1 | DOUT2 | DOUT3; /*digital out (ON/OFF out)*/
#endif
  cmd.dout = 0;

  cmd.selout = SET_SELECT | CH0 | CH1 | CH2 | CH3; /* EEPROM データが正しければ不要 */

  /*---各チャンネルのオフセット値の設定---*/
#if __BYTE_ORDER ==  __LITTLE_ENDIAN
  cmd.offset[0] = cmd.offset[1] = cmd.offset[2] = cmd.offset[3] = 0x7fff;
  cmd.counter[0] = cmd.counter[1] = cmd.counter[2] = cmd.counter[3] = 0;
  cmd.magicno = 0x00;
#else
  cmd.offset[0] = cmd.offset[1] = cmd.offset[2] = cmd.offset[3] = 0xff7f;
  cmd.counter[0] = cmd.counter[1] = cmd.counter[2] = cmd.counter[3] = 0;
  cmd.magicno = 0x00;
#endif

  cmd.posneg = SET_POSNEG | CH0 | CH1 | CH2 | CH3;
  cmd.breaks = SET_BREAKS | CH0 | CH1 | CH2 | CH3;

  cmd.wrrom = 0; /* WR_MAGIC | WR_OFFSET | WR_SELOUT;*/

  /*---ccmdのモードに切り替える---*/
  /*---以後，ccmdモードに切り替えられるまで，モードは維持される---*/
  if (ioctl(fd, URBTC_COUNTER_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    exit(1);
  }
  // printf("sizeof(cmd) %d\n", sizeof(cmd));

  /*---ccmdの値をH8に書き込む---*/
  if (write(fd, &cmd, sizeof(cmd)) < 0) {
    fprintf(stderr, "write error\n");
    exit(1);
  }

#if 0
  /*set scmd mode*/
  if (ioctl(fd, URBTC_DESIRE_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_DESIRE_SET error\n");
    exit(1);
  }
#endif

  for (i=0; i<4; i++) {
#if __BYTE_ORDER ==  __LITTLE_ENDIAN
    obuf.ch[i].x = 0;
    obuf.ch[i].d = 0;
    obuf.ch[i].kp = 0;
    obuf.ch[i].kpx = 1;
    obuf.ch[i].kd = 0;
    obuf.ch[i].kdx = 1;
    obuf.ch[i].ki = 0;
    obuf.ch[i].kix = 1;
#else
    obuf.ch[i].x = 0;
    obuf.ch[i].d = 0;
    obuf.ch[i].kp = 0;
    obuf.ch[i].kpx = 0x0100;
    obuf.ch[i].kd = 0;
    obuf.ch[i].kdx = 0x0100;
    obuf.ch[i].ki = 0;
    obuf.ch[i].kix = 0x0100;
#endif
  }

  i = 0;
  while(quit_flag) {
    unsigned short a = 300.0*sin(i*3.14/655.360) + 512.0;
    a <<= 5;

    /*---各チャンネルのオフセット値の設定---*/
    cmd.offset[0] = cmd.offset[1] = cmd.offset[2] = cmd.offset[3] = a;

    if (ioctl(fd, URBTC_COUNTER_SET) < 0){
      fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
      exit(1);
    }

    /*---ccmdの値をH8に書き込む---*/
    if (write(fd, &cmd, sizeof(cmd)) > 0) {
      i += 1;
    } else {
      fprintf(stderr, "write cmd error\n");
      exit(1);
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





