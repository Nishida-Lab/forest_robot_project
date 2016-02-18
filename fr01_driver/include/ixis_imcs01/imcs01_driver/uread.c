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
  int i;
  char *dev = "/dev/urbtc0";
  
  signal(SIGINT, exit_program);

  if (argc>1)
	dev = argv[1];

  if ((fd = open(dev, O_RDWR)) == -1) {
    fprintf(stderr, "%s: Open error\n", dev);
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
    if ((i = read(fd, &buf, sizeof(buf))) != sizeof(buf)) {
      fprintf(stderr, "Warning: read size mismatch (%d!=%d).\n", i, sizeof(buf));
      continue;
    }
    for (i=0; i<4; i++) {
#if __BYTE_ORDER == __BIG_ENDIAN
      buf.ad[i] = (0xff & buf.ad[i])<<8 | (0xff00 & buf.ad[i])>>8;
#endif
      buf.ad[i] = buf.ad[i] >> 5;
    }

#if __BYTE_ORDER == __BIG_ENDIAN
    buf.time = (0xff & buf.time)<<8 | (0xff00 & buf.time)>>8;
#endif

    printf("%d %d %d %d %d %d %d %d %d %d %d %d %d %x %x %d %d %x\n",
	   buf.time, buf.ad[0], buf.ad[1], buf.ad[2], buf.ad[3],
	   buf.ct[0], buf.ct[1], buf.ct[2], buf.ct[3],
	   buf.da[0], buf.da[1], buf.da[2], buf.da[3],
	   buf.din, buf.dout, buf.intmax, buf.interval, buf.magicno);
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
