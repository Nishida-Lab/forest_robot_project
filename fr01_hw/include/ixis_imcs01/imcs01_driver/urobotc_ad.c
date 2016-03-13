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

  cmd.dout = 0;

  cmd.selout = SET_SELECT | CH0 | CH1 | CH2 | CH3; /* EEPROM $B%G!<%?$,@5$7$1$l$PITMW(B */

  /*---$B3F%A%c%s%M%k$N%*%U%;%C%HCM$N@_Dj(B---*/
  cmd.offset[0] = cmd.offset[1] = cmd.offset[2] = cmd.offset[3] = 32768;

  cmd.counter[0] = cmd.counter[1] = cmd.counter[2] = cmd.counter[3] = 0;

  cmd.posneg = SET_POSNEG | CH0 | CH1 | CH2 | CH3;
  cmd.breaks = SET_BREAKS | CH0 | CH1 | CH2 | CH3;

  cmd.magicno = 0x00;

  cmd.wrrom = 0; /* WR_MAGIC | WR_OFFSET | WR_SELOUT;*/

  /*---ccmd$B$N%b!<%I$K@Z$jBX$($k(B---*/
  if (ioctl(fd, URBTC_COUNTER_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    exit(1);
  }

  /*---ccmd$B$NCM$r(BH8$B$K=q$-9~$`(B---*/
  if (write(fd, &cmd, sizeof(cmd)) < 0) {
    fprintf(stderr, "write error\n");
    exit(1);
  }

  /*---scmd$B%b!<%I$K@Z$jBX$($k(B---*/
  /*---$B0J8e!$(Bccmd$B%b!<%I$K@Z$jBX$($i$l$k$^$G!$%b!<%I$O0];}$5$l$k(B---*/
  if (ioctl(fd, URBTC_DESIRE_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_DESIRE_SET error\n");
    exit(1);
  }

  /*---$B3F%A%c%s%M%k$N(Bscmd$B$N=i4|2=(B($BHfNc%2%$%s$@$1M?$($k(B)---*/
  for (i=0; i<4; i++) {
    obuf.ch[i].x = 0;
    obuf.ch[i].d = 0;
    obuf.ch[i].kp = 0x2;
    obuf.ch[i].kpx = 1;
    obuf.ch[i].kd = 0;
    obuf.ch[i].kdx = 1;
    obuf.ch[i].ki = 0;
    obuf.ch[i].kix = 1;
  }

  /*---$BO"B3$7$F(Buin$B$,<B9T$G$-$k$h$&$K%;%C%H(B---*/
  if (ioctl(fd, URBTC_CONTINUOUS_READ) < 0){
    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    exit(1);
  }

  if (ioctl(fd, URBTC_BUFREAD) < 0){
    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    exit(1);
  }

  while(quit_flag) {

    /*---$BL\I8CM$N@_Dj(B(0-1023)---*/
    unsigned short x_d= 511.0;

    /*---H8$B$+$i$NCM$r<h$j9~$`(B---*/
    if ((i = read(fd, &buf, sizeof(buf))) != sizeof(buf)) {
      fprintf(stderr, "Warning: read size mismatch (%d!=%d).\n", i, sizeof(buf));
      continue;
    }

    for (i=0; i<4; i++) {
      buf.ad[i] >>= 5;
    }

    printf("current=%d, delta=%d\n", buf.ad[3], x_d - buf.ad[3]);

    x_d <<= 5;

    /*---$B3F%A%c%s%M%k$NL\I8CM$r%;%C%H(B---*/
    obuf.ch[0].x = obuf.ch[1].x = obuf.ch[2].x = obuf.ch[3].x = x_d;

    /*---scmd$B%b!<%I$G(Bobuf$B$NCM$r=q$-9~$`(B---*/
    if (write(fd, &obuf, sizeof(obuf)) > 0) {
      i++;
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
