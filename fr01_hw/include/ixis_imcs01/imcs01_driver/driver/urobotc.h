/* $B%S%C%H$H%A%c%s%M%k$NBP1~(B */
#define CH0 1
#define CH1 2
#define CH2 4
#define CH3 8

/* $B%G%8%?%kF~NOMQ$N%^%9%/(B */
#define DIN0 0x10
#define DIN1 0x20

/* $B%G%8%?%k=PNOMQ$N%^%9%/(B */
#define DO_DOUT 0x1
#define DOUT0 0x08
#define DOUT1 0x20
#define DOUT2 0x40
#define DOUT3 0x80

struct uin {
  unsigned short time; /* $BFbIt%+%&%s%?(B (1ms$B<~4|(B)$B$NCM(B */
  unsigned short ad[4];/* A/D $B%3%s%P!<%?$NCM(B(10bit) */
  short ct[4];         /* 2$BAj%+%&%s%?$NCM(B(16bit) */
  unsigned short da[4];/* D/A $B=PNO$NCM(B($B>e0L(B 8/12bit$B$,M-8z(B) */
  unsigned char din;   /* $B%G%8%?%kF~NO(B */
  unsigned char dout;  /* $B%G%8%?%k=PNO(B */
  unsigned short intmax;  /* $B%=%U%H%&%'%"%+%&%s%?$N:GBg<~4|(B */
  unsigned short interval;/* $B%=%U%H%&%'%"%+%&%s%?$N:G6a$N<~4|(B */
  unsigned short magicno; /* EEPROM $BFb$N?t;z(B */
  char dmy[30];
};

struct scmd {
  short x;             /* $BL\I80LCV(B */
  short d;             /* $BL\I8B.EY(B */
  signed short kp;     /* $B0LCV8m:9%2%$%s(B($BJ,;R(B) */
  unsigned short kpx;  /* $B0LCV8m:9%2%$%s(B($BJ,Jl(B) */
  signed short kd;     /* $BB.EY8m:9%2%$%s(B($BJ,;R(B) */
  unsigned short kdx;  /* $BB.EY8m:9%2%$%s(B($BJ,Jl(B) */
  signed short ki;     /* $B@QJ,8m:9%2%$%s(B($BJ,;R(B) */
  unsigned short kix;  /* $B@QJ,8m:9%2%$%s(B($BJ,Jl(B) */
};

struct uout {
  struct scmd ch[4];
};

#define RETURN_VAL 1
#define SET_SELECT 0x80
#define SET_POSNEG 0x80
#define SET_BREAKS 0x80
#define SET_CH2_HIN 0x40
#define WR_MAGIC 0x80
#define WR_SELOUT 0x40
#define WR_OFFSET 0x20
#define WR_BREAKS 0x10

struct ccmd {
  unsigned char retval;    /* $B<!$KFbIt%+%&%s%?$,>e$,$C$?$H$-$NCM$r(B EP1 $B$+$iJV$9(B */
  unsigned char setoffset; /* $B%*%U%;%C%H$r%;%C%H$9$k%A%c%s%M%k$N;XDj(B */
  unsigned char setcounter;/* $B%+%&%s%?$NCM$r%;%C%H$9$k%A%c%s%M%k$N;XDj(B */
  unsigned char resetint;  /* $B@QJ,CM$r%j%;%C%H$9$k%A%c%s%M%k$N;XDj(B */
  unsigned char selin;     /* counter(0)/ADC(1) $B$NA*Br(B */
  unsigned char dout;      /* $B%G%8%?%k=PNO(B */
  unsigned short offset[4];         /* $B%*%U%;%C%H$NCM(B */
  short counter[4];        /* $B%+%&%s%?$NCM(B */
  unsigned char selout;    /* PWM $B$r(B D/A $B8~$1(B(0)$B$H$9$k$+(B
			      H $B%V%j%C%88~$1(B(1)$B$H$9$k$+(B */
  unsigned char wrrom;
  unsigned short magicno;
  unsigned char posneg;    /* PWM $B%Q%k%9$N@5(B/$BIi(B */
  unsigned char breaks;    /* break $B=PNO(B */
  char dummy[36];
};
