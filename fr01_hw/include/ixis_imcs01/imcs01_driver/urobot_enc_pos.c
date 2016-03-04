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

#define numOfControllers 1
static char *devfiles[] = {"/dev/urbtc0"};

int main(int argc, char **argv)
{
  struct uin ibuf;
  struct uout obuf[numOfControllers];
  struct ccmd cmd;
  int fd, fds[numOfControllers];
  int i, j;
  
  signal(SIGINT, exit_program);

  for (i=0; i<numOfControllers; i++) {
    if ((fd = open(devfiles[i], O_RDWR)) == -1) {
      fprintf(stderr, "%s: Open error\n", devfiles[i]);
      exit(1);
    }
    if (ioctl(fd, URBTC_CONTINUOUS_READ) < 0){
      fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
      exit(1);
    }
    if (read(fd, &ibuf, sizeof(ibuf)) != sizeof(ibuf)) {
      fprintf(stderr, "Read size mismatch.\n");
      exit(1);
    }

#if __BYTE_ORDER == __BIG_ENDIAN
    ibuf.magicno = (0xff & ibuf.magicno)<<8 | (0xff00 & ibuf.magicno)>>8;
#endif

    if (ibuf.magicno == 0) {
      fds[0] = fd;
      fprintf(stderr, "Found controller #0.\n");
    } else if (ibuf.magicno == 1) {
      fds[1] = fd;
      fprintf(stderr, "Found controller #1.\n");
    } else {
      fprintf(stderr, "Wrong magic no: %d.\n", ibuf.magicno);
      exit(1);
    }

    cmd.retval = 0 /* RETURN_VAL */;
    cmd.setoffset  = CH0 | CH1 | CH2 | CH3;
    cmd.setcounter = CH0 | CH1 | CH2 | CH3;
    cmd.resetint   = CH0 | CH1 | CH2 | CH3;

    cmd.selin = CH0 | CH1 | SET_SELECT; /* AD in:ch0,ch1    ENC in:ch2,ch3*/
    cmd.selout = SET_SELECT | CH0 | CH1 | CH2 | CH3; /*  PWM out:ch0,ch1,ch2,ch3*/

#if __BYTE_ORDER == __LITTLE_ENDIAN
    cmd.offset[0] = cmd.offset[1] = cmd.offset[2] = cmd.offset[3] = 0x7fff;
    cmd.counter[0] = cmd.counter[1] = cmd.counter[2] = cmd.counter[3] = 0;
#else
    cmd.offset[0] = cmd.offset[1] = cmd.offset[2] = cmd.offset[3] = 0xff7f;
    cmd.counter[0] = cmd.counter[1] = cmd.counter[2] = cmd.counter[3] = 0;
#endif

    cmd.posneg = SET_POSNEG | CH0| CH1 | CH2 | CH3; /*POS PWM out*/
    cmd.breaks = SET_BREAKS | CH0 | CH1 | CH2 | CH3; /*No Brake*/

    if (ioctl(fd, URBTC_COUNTER_SET) < 0){
      fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
      exit(1);
    }
    if (write(fd, &cmd, sizeof(cmd)) < 0) {
      fprintf(stderr, "write error\n");
      exit(1);
    }
    if (ioctl(fd, URBTC_DESIRE_SET) < 0){
      fprintf(stderr, "ioctl: URBTC_DESIRE_SET error\n");
      exit(1);
    }
  }

  for (j=0; j<numOfControllers; j++) {
    for (i=0; i<4; i++) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
      obuf[j].ch[i].x = 0;
      obuf[j].ch[i].d = 0;
      obuf[j].ch[i].kp = 0;
      obuf[j].ch[i].kpx = 1;
      obuf[j].ch[i].kd = 0;
      obuf[j].ch[i].kdx = 1;
      obuf[j].ch[i].ki = 0;
      obuf[j].ch[i].kix = 1;
#else
      obuf[j].ch[i].x = 0;
      obuf[j].ch[i].d = 0;
      obuf[j].ch[i].kp = 0;
      obuf[j].ch[i].kpx = 0x0100;
      obuf[j].ch[i].kd = 0;
      obuf[j].ch[i].kdx = 0x0100;
      obuf[j].ch[i].ki = 0;
      obuf[j].ch[i].kix = 0x0100;
#endif
    }
#if __BYTE_ORDER == __LITTLE_ENDIAN
    obuf[j].ch[2].kp = 0x10;
    obuf[j].ch[3].kp = 0x10;
#else
    obuf[j].ch[2].kp = 0x1000;
    obuf[j].ch[3].kp = 0x1000;
#endif
  }

  i = 0;
  while(quit_flag) {
    unsigned short a = 300.0*sin(i*3.14/655.360);// + 512.0;
    a <<= 5;
    for (j=0; j<numOfControllers; j++) {

#if __BYTE_ORDER == __LITTLE_ENDIAN
      obuf[j].ch[2].x = obuf[j].ch[3].x = a;
#else
      obuf[j].ch[2].x = obuf[j].ch[3].x = ((a & 0xff) << 8 | (a & 0xff00) >> 8);
#endif
	printf("%x\r\n",obuf[j].ch[3].x);

      if (write(fds[j], &obuf[j], sizeof(obuf[j])) > 0) {
	i++;
      } else {
	printf("write err\n");
	break;
      }
    }
  }

  for (i=0; i<numOfControllers; i++)
    close(fds[i]);

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
