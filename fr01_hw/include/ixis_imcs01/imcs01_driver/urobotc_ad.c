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

  cmd.selout = SET_SELECT | CH0 | CH1 | CH2 | CH3; /* EEPROM データが正しければ不要 */

  /*---各チャンネルのオフセット値の設定---*/
  cmd.offset[0] = cmd.offset[1] = cmd.offset[2] = cmd.offset[3] = 32768;

  cmd.counter[0] = cmd.counter[1] = cmd.counter[2] = cmd.counter[3] = 0;

  cmd.posneg = SET_POSNEG | CH0 | CH1 | CH2 | CH3;
  cmd.breaks = SET_BREAKS | CH0 | CH1 | CH2 | CH3;

  cmd.magicno = 0x00;

  cmd.wrrom = 0; /* WR_MAGIC | WR_OFFSET | WR_SELOUT;*/

  /*---ccmdのモードに切り替える---*/
  if (ioctl(fd, URBTC_COUNTER_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    exit(1);
  }

  /*---ccmdの値をH8に書き込む---*/
  if (write(fd, &cmd, sizeof(cmd)) < 0) {
    fprintf(stderr, "write error\n");
    exit(1);
  }

  /*---scmdモードに切り替える---*/
  /*---以後，ccmdモードに切り替えられるまで，モードは維持される---*/
  if (ioctl(fd, URBTC_DESIRE_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_DESIRE_SET error\n");
    exit(1);
  }

  /*---各チャンネルのscmdの初期化(比例ゲインだけ与える)---*/
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

  /*---連続してuinが実行できるようにセット---*/
  if (ioctl(fd, URBTC_CONTINUOUS_READ) < 0){
    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    exit(1);
  }

  if (ioctl(fd, URBTC_BUFREAD) < 0){
    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    exit(1);
  }

  while(quit_flag) {

    /*---目標値の設定(0-1023)---*/
    unsigned short x_d= 511.0;

    /*---H8からの値を取り込む---*/
    if ((i = read(fd, &buf, sizeof(buf))) != sizeof(buf)) {
      fprintf(stderr, "Warning: read size mismatch (%d!=%d).\n", i, sizeof(buf));
      continue;
    }

    for (i=0; i<4; i++) {
      buf.ad[i] >>= 5;
    }

    printf("current=%d, delta=%d\n", buf.ad[3], x_d - buf.ad[3]);

    x_d <<= 5;

    /*---各チャンネルの目標値をセット---*/
    obuf.ch[0].x = obuf.ch[1].x = obuf.ch[2].x = obuf.ch[3].x = x_d;

    /*---scmdモードでobufの値を書き込む---*/
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
