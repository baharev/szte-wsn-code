#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/usr/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
}  ;
#line 14
struct __nesc_attr_one_nok {
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
}  ;
# 38 "/usr/msp430/include/sys/inttypes.h" 3
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;





static __inline int8_t __nesc_ntoh_int8(const void * source)  ;
#line 257
static __inline int8_t __nesc_hton_int8(void * target, int8_t value)  ;
#line 276
static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;
#line 294
static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
#line 385
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 41 "/usr/msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 39 "/usr/msp430/include/string.h" 3
extern int memcmp(const void *arg_0x4029ca18, const void *arg_0x4029cbb0, size_t arg_0x4029cd48);
extern void *memcpy(void *arg_0x4029f200, const void *arg_0x4029f398, size_t arg_0x4029f530);

extern void *memset(void *arg_0x4029e220, int arg_0x4029e378, size_t arg_0x4029e510);
#line 63
extern void *memset(void *arg_0x402ac118, int arg_0x402ac270, size_t arg_0x402ac408);
# 59 "/usr/msp430/include/stdlib.h" 3
#line 56
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;







#line 64
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/usr/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 28 "/usr/msp430/include/sys/reent.h" 3
typedef __uint32_t __ULong;


struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x402c9510);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x402cdb88);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 23 "/home/ali/workspace/TinyOS-main/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;






struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 51 "/home/ali/workspace/TinyOS-main/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 39 "/usr/msp430/include/msp430/iostructures.h" 3
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4249 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
#line 108
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};









struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 120 "/usr/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1IFG __asm ("0x0023");

volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");










volatile unsigned char P2IN __asm ("0x0028");





volatile unsigned char P2IFG __asm ("0x002B");

volatile unsigned char P2IES __asm ("0x002C");

volatile unsigned char P2IE __asm ("0x002D");
# 92 "/usr/msp430/include/msp430/usart.h" 3
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");
#line 232
volatile unsigned char I2CIFG __asm ("0x0051");

volatile unsigned char I2CNDAT __asm ("0x0052");

volatile unsigned char I2CTCTL __asm ("0x0071");

volatile unsigned char I2CDCTL __asm ("0x0072");

volatile unsigned char I2CPSC __asm ("0x0073");

volatile unsigned char I2CSCLH __asm ("0x0074");

volatile unsigned char I2CSCLL __asm ("0x0075");







volatile unsigned int I2CSA __asm ("0x011A");

volatile unsigned int I2CIV __asm ("0x011C");





volatile unsigned char I2CDR __asm ("0x0076");
#line 275
volatile unsigned char U1CTL __asm ("0x0078");

volatile unsigned char U1TCTL __asm ("0x0079");



volatile unsigned char U1MCTL __asm ("0x007B");

volatile unsigned char U1BR0 __asm ("0x007C");

volatile unsigned char U1BR1 __asm ("0x007D");

volatile unsigned char U1RXBUF __asm ("0x007E");
# 27 "/usr/msp430/include/msp430/timera.h" 3
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
#line 70
volatile unsigned int TA0CCTL2 __asm ("0x0166");
#line 127
#line 118
typedef struct __nesc_unnamed4250 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 143
#line 129
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;


struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned dummy[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};



struct timera_t;
# 26 "/usr/msp430/include/msp430/timerb.h" 3
volatile unsigned int TBR __asm ("0x0190");


volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBCCR0 __asm ("0x0192");
#line 76
#line 64
typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 20 "/usr/msp430/include/msp430/basic_clock.h" 3
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/usr/msp430/include/msp430/adc12.h" 3
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
#line 42
#line 30
typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;
#line 54
#line 44
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;
#line 74
#line 56
typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 56 "/usr/msp430/include/msp430x16x.h" 3
volatile unsigned char IE1 __asm ("0x0000");
#line 83
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 338 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 368
static inline void brief_pause(unsigned int n);
#line 399
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4257 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 454
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
#line 469
enum __nesc_unnamed4258 {
  MSP430_PORT_RESISTOR_INVALID, 
  MSP430_PORT_RESISTOR_OFF, 
  MSP430_PORT_RESISTOR_PULLDOWN, 
  MSP430_PORT_RESISTOR_PULLUP
};
# 50 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/hardware.h"
static inline void TOSH_SET_RED_LED_PIN()  ;
#line 50
static inline void TOSH_MAKE_RED_LED_OUTPUT()  ;
#line 50
static inline void TOSH_SEL_RED_LED_IOFUNC()  ;
static inline void TOSH_SET_YELLOW_LED_PIN()  ;
#line 51
static inline void TOSH_MAKE_YELLOW_LED_OUTPUT()  ;
#line 51
static inline void TOSH_SEL_YELLOW_LED_IOFUNC()  ;
static inline void TOSH_SET_GREEN_LED_PIN()  ;
#line 52
static inline void TOSH_MAKE_GREEN_LED_OUTPUT()  ;
#line 52
static inline void TOSH_SEL_GREEN_LED_IOFUNC()  ;


static inline void TOSH_MAKE_RADIO_FIFO_INPUT()  ;
#line 55
static inline void TOSH_SEL_RADIO_FIFO_IOFUNC()  ;
static inline void TOSH_MAKE_RADIO_FIFOP_INPUT()  ;
#line 56
static inline void TOSH_SEL_RADIO_FIFOP_IOFUNC()  ;
static inline void TOSH_MAKE_RADIO_CCA_INPUT()  ;
#line 57
static inline void TOSH_SEL_RADIO_CCA_IOFUNC()  ;


static inline void TOSH_CLR_RADIO_VREF_PIN()  ;
#line 60
static inline void TOSH_MAKE_RADIO_VREF_OUTPUT()  ;
#line 60
static inline void TOSH_SEL_RADIO_VREF_IOFUNC()  ;

static inline void TOSH_MAKE_RADIO_SFD_INPUT()  ;
#line 62
static inline void TOSH_SEL_RADIO_SFD_IOFUNC()  ;


static inline void TOSH_SET_RADIO_CSN_PIN()  ;
#line 65
static inline void TOSH_MAKE_RADIO_CSN_OUTPUT()  ;
#line 65
static inline void TOSH_SEL_RADIO_CSN_IOFUNC()  ;
static inline void TOSH_CLR_RADIO_RESET_PIN()  ;
#line 66
static inline void TOSH_MAKE_RADIO_RESET_OUTPUT()  ;
#line 66
static inline void TOSH_SEL_RADIO_RESET_IOFUNC()  ;









static inline void TOSH_MAKE_BT_PIO_INPUT()  ;
#line 76
static inline void TOSH_SEL_BT_PIO_IOFUNC()  ;
static inline void TOSH_MAKE_BT_RTS_INPUT()  ;
#line 77
static inline void TOSH_SEL_BT_RTS_IOFUNC()  ;
static inline void TOSH_MAKE_BT_CTS_OUTPUT()  ;
#line 78
static inline void TOSH_SEL_BT_CTS_IOFUNC()  ;
static inline void TOSH_MAKE_BT_TXD_OUTPUT()  ;
#line 79
static inline void TOSH_SEL_BT_TXD_IOFUNC()  ;
static inline void TOSH_MAKE_BT_RXD_INPUT()  ;
#line 80
static inline void TOSH_SEL_BT_RXD_IOFUNC()  ;
static inline void TOSH_CLR_BT_RESET_PIN()  ;
#line 81
static inline void TOSH_MAKE_BT_RESET_OUTPUT()  ;
#line 81
static inline void TOSH_SEL_BT_RESET_IOFUNC()  ;


static inline void TOSH_SET_PROG_OUT_PIN()  ;
#line 84
static inline void TOSH_CLR_PROG_OUT_PIN()  ;
#line 84
static inline void TOSH_MAKE_PROG_OUT_OUTPUT()  ;
#line 84
static inline void TOSH_SEL_PROG_OUT_IOFUNC()  ;
static inline void TOSH_MAKE_PROG_IN_OUTPUT()  ;
#line 85
static inline void TOSH_SEL_PROG_IN_IOFUNC()  ;


static inline void TOSH_SET_SD_CS_N_PIN()  ;
#line 88
static inline void TOSH_CLR_SD_CS_N_PIN()  ;
#line 88
static inline void TOSH_MAKE_SD_CS_N_OUTPUT()  ;
#line 88
static inline void TOSH_SEL_SD_CS_N_IOFUNC()  ;

static inline void TOSH_MAKE_TILT_INPUT()  ;
#line 90
static inline void TOSH_SEL_TILT_IOFUNC()  ;


static inline void TOSH_MAKE_ADC_0_OUTPUT()  ;
#line 93
static inline void TOSH_SEL_ADC_0_IOFUNC()  ;
static inline void TOSH_MAKE_ADC_1_OUTPUT()  ;
#line 94
static inline void TOSH_SEL_ADC_1_IOFUNC()  ;
static inline void TOSH_MAKE_ADC_2_OUTPUT()  ;
#line 95
static inline void TOSH_SEL_ADC_2_IOFUNC()  ;



static inline void TOSH_MAKE_ADC_6_OUTPUT()  ;
#line 99
static inline void TOSH_SEL_ADC_6_IOFUNC()  ;
static inline void TOSH_MAKE_ADC_7_OUTPUT()  ;
#line 100
static inline void TOSH_SEL_ADC_7_IOFUNC()  ;

static inline void TOSH_MAKE_ADC_ACCELZ_INPUT()  ;
#line 102
static inline void TOSH_SEL_ADC_ACCELZ_IOFUNC()  ;
static inline void TOSH_MAKE_ADC_ACCELY_INPUT()  ;
#line 103
static inline void TOSH_SEL_ADC_ACCELY_IOFUNC()  ;
static inline void TOSH_MAKE_ADC_ACCELX_INPUT()  ;
#line 104
static inline void TOSH_SEL_ADC_ACCELX_IOFUNC()  ;








static inline void TOSH_SET_SW_SD_PWR_N_PIN()  ;
#line 113
static inline void TOSH_CLR_SW_SD_PWR_N_PIN()  ;
#line 113
static inline void TOSH_MAKE_SW_SD_PWR_N_OUTPUT()  ;
#line 113
static inline void TOSH_SEL_SW_SD_PWR_N_IOFUNC()  ;
static inline void TOSH_SET_SW_BT_PWR_N_PIN()  ;
#line 114
static inline void TOSH_MAKE_SW_BT_PWR_N_OUTPUT()  ;
#line 114
static inline void TOSH_SEL_SW_BT_PWR_N_IOFUNC()  ;


static inline void TOSH_CLR_SER0_RTS_PIN()  ;
#line 117
static inline void TOSH_MAKE_SER0_RTS_OUTPUT()  ;
#line 117
static inline void TOSH_SEL_SER0_RTS_IOFUNC()  ;
static inline void TOSH_MAKE_SER0_CTS_OUTPUT()  ;
#line 118
static inline void TOSH_SEL_SER0_CTS_IOFUNC()  ;




static inline void TOSH_MAKE_UTXD0_OUTPUT()  ;
#line 123
static inline void TOSH_SEL_UTXD0_IOFUNC()  ;
static inline void TOSH_SET_URXD0_PIN()  ;
#line 124
static inline void TOSH_MAKE_URXD0_OUTPUT()  ;
#line 124
static inline void TOSH_SEL_URXD0_IOFUNC()  ;


static inline void TOSH_SET_SD_DI_PIN()  ;
#line 127
static inline void TOSH_CLR_SD_DI_PIN()  ;
#line 127
static inline void TOSH_MAKE_SD_DI_OUTPUT()  ;
#line 127
static inline void TOSH_SEL_SD_DI_IOFUNC()  ;
static inline void TOSH_CLR_SD_DO_PIN()  ;
#line 128
static inline void TOSH_MAKE_SD_DO_INPUT()  ;
#line 128
static inline void TOSH_SEL_SD_DO_IOFUNC()  ;
static inline void TOSH_SET_SD_CLK_PIN()  ;
#line 129
static inline void TOSH_CLR_SD_CLK_PIN()  ;
#line 129
static inline void TOSH_MAKE_SD_CLK_OUTPUT()  ;
#line 129
static inline void TOSH_SEL_SD_CLK_IOFUNC()  ;


static inline void TOSH_SET_SIMO1_PIN()  ;
#line 132
static inline void TOSH_MAKE_SIMO1_OUTPUT()  ;
#line 132
static inline void TOSH_SEL_SIMO1_IOFUNC()  ;
static inline void TOSH_MAKE_SOMI1_INPUT()  ;
#line 133
static inline void TOSH_SEL_SOMI1_IOFUNC()  ;
static inline void TOSH_SET_UCLK1_PIN()  ;
#line 134
static inline void TOSH_MAKE_UCLK1_OUTPUT()  ;
#line 134
static inline void TOSH_SEL_UCLK1_IOFUNC()  ;
static inline void TOSH_MAKE_UTXD1_OUTPUT()  ;
#line 135
static inline void TOSH_SEL_UTXD1_IOFUNC()  ;
static inline void TOSH_MAKE_URXD1_OUTPUT()  ;
#line 136
static inline void TOSH_SEL_URXD1_IOFUNC()  ;

static inline void TOSH_CLR_GIO0_PIN()  ;
#line 138
static inline void TOSH_MAKE_GIO0_OUTPUT()  ;
#line 138
static inline void TOSH_SEL_GIO0_IOFUNC()  ;
static inline void TOSH_MAKE_GIO1_OUTPUT()  ;
#line 139
static inline void TOSH_SEL_GIO1_IOFUNC()  ;



static inline void TOSH_SET_ONEWIRE_PIN()  ;
#line 143
static inline void TOSH_CLR_ONEWIRE_PIN()  ;
#line 143
static inline uint8_t TOSH_READ_ONEWIRE_PIN()  ;
#line 143
static inline void TOSH_MAKE_ONEWIRE_OUTPUT()  ;
#line 143
static inline void TOSH_MAKE_ONEWIRE_INPUT()  ;
#line 143
static inline void TOSH_SEL_ONEWIRE_IOFUNC()  ;


static inline void TOSH_MAKE_ROSC_INPUT()  ;
#line 146
static inline void TOSH_SEL_ROSC_IOFUNC()  ;


static inline void TOSH_SET_DOCK_N_PIN()  ;
#line 149
static inline uint8_t TOSH_READ_DOCK_N_PIN()  ;
#line 149
static inline void TOSH_MAKE_DOCK_N_OUTPUT()  ;
#line 149
static inline void TOSH_MAKE_DOCK_N_INPUT()  ;
#line 149
static inline void TOSH_SEL_DOCK_N_IOFUNC()  ;


static inline void TOSH_CLR_PWRMUX_SEL_PIN()  ;
#line 152
static inline void TOSH_MAKE_PWRMUX_SEL_OUTPUT()  ;
#line 152
static inline void TOSH_SEL_PWRMUX_SEL_IOFUNC()  ;
static inline void TOSH_CLR_ACCEL_SEL0_PIN()  ;
#line 153
static inline void TOSH_MAKE_ACCEL_SEL0_OUTPUT()  ;
#line 153
static inline void TOSH_SEL_ACCEL_SEL0_IOFUNC()  ;
static inline void TOSH_CLR_ACCEL_SLEEP_N_PIN()  ;
#line 154
static inline void TOSH_MAKE_ACCEL_SLEEP_N_OUTPUT()  ;
#line 154
static inline void TOSH_SEL_ACCEL_SLEEP_N_IOFUNC()  ;
# 43 "/home/ali/workspace/TinyOS-main/tos/types/Leds.h"
enum __nesc_unnamed4259 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 96 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SD.h"
typedef uint8_t mmcerror_t;
# 41 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4260 {
#line 41
  int notUsed;
} 
#line 41
TSecond;
typedef struct __nesc_unnamed4261 {
#line 42
  int notUsed;
} 
#line 42
TMilli;
typedef struct __nesc_unnamed4262 {
#line 43
  int notUsed;
} 
#line 43
T32khz;
typedef struct __nesc_unnamed4263 {
#line 44
  int notUsed;
} 
#line 44
TMicro;
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4264 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 75
#line 62
typedef struct __nesc_unnamed4265 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 87
#line 77
typedef struct __nesc_unnamed4266 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 102
#line 89
typedef struct __nesc_unnamed4267 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 56 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4268 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4269 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4270 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4271 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4272 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4273 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4274 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t msp430_spi_default_config = { 
{ 
.ubr = 0x0002, 
.ssel = 0x02, 
.clen = 1, 
.listen = 0, 
.mm = 1, 
.ckph = 1, 
.ckpl = 0, 
.stc = 1 } };
#line 171
#line 150
typedef enum __nesc_unnamed4275 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55, 

  UBR_4MHZ_115200 = 0x0024, UMCTL_4MHZ_115200 = 0x29
} msp430_uart_rate_t;
#line 202
#line 173
typedef struct __nesc_unnamed4276 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 204
typedef struct __nesc_unnamed4277 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 213
typedef union __nesc_unnamed4278 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
#line 250
#line 242
typedef struct __nesc_unnamed4279 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 278
#line 255
typedef struct __nesc_unnamed4280 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 280
typedef struct __nesc_unnamed4281 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 289
typedef union __nesc_unnamed4282 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 311
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4283 {
  TOS_UART_1200 = 0, 
  TOS_UART_1800 = 1, 
  TOS_UART_2400 = 2, 
  TOS_UART_4800 = 3, 
  TOS_UART_9600 = 4, 
  TOS_UART_19200 = 5, 
  TOS_UART_38400 = 6, 
  TOS_UART_57600 = 7, 
  TOS_UART_76800 = 8, 
  TOS_UART_115200 = 9, 
  TOS_UART_230400 = 10
};

enum __nesc_unnamed4284 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4285 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 33 "/home/ali/workspace/TinyOS-main/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 38 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Magnetometer.h"
enum __nesc_unnamed4286 {
  ZERO_7_GAUSS, 
  ONE_GAUSS, 
  ONE_5_GAUSS, 
  TWO_GAUSS, 
  THREE_2_GAUSS, 
  THREE_8_GAUSS, 
  FOUR_5_GAUSS
};

enum __nesc_unnamed4287 {
  ZERO_5_HZ, 
  ONE_HZ, 
  TWO_HZ, 
  FIVE_HZ, 
  TEN_HZ, 
  TWENTY_HZ, 
  FIFTY_HZ
};
# 40 "/home/ali/workspace/TinyOS-main/tos/types/I2C.h"
typedef struct __nesc_unnamed4288 {
} 
#line 40
TI2CExtdAddr;
typedef struct __nesc_unnamed4289 {
} 
#line 41
TI2CBasicAddr;

typedef uint8_t i2c_flags_t;

enum __nesc_unnamed4290 {
  I2C_START = 0x01, 
  I2C_STOP = 0x02, 
  I2C_ACK_END = 0x04
};
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4291 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 96 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4292 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 6 "/home/ali/workspace/TinyOS-main/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4293 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4294 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 83 "/home/ali/workspace/TinyOS-main/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4295 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4296 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4297 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 121
#line 109
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 123
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 131
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 136
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 59 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/platform_message.h"
#line 56
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 61
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 65
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/home/ali/workspace/TinyOS-main/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[96];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 40 "/home/ali/workspace/TinyOS-main/tos/types/IeeeEui64.h"
enum __nesc_unnamed4298 {
#line 40
  IEEE_EUI64_LENGTH = 8
};


#line 42
typedef struct ieee_eui64 {
  uint8_t data[IEEE_EUI64_LENGTH];
} ieee_eui64_t;
# 35 "/home/ali/workspace/TinyOS-main/tos/types/Ieee154.h"
typedef uint16_t ieee154_panid_t;
typedef uint16_t ieee154_saddr_t;
typedef ieee_eui64_t ieee154_laddr_t;







#line 39
typedef struct __nesc_unnamed4299 {
  uint8_t ieee_mode : 2;
  union __nesc_unnamed4300 {
    ieee154_saddr_t saddr;
    ieee154_laddr_t laddr;
  } ieee_addr;
} ieee154_addr_t;



enum __nesc_unnamed4301 {
  IEEE154_BROADCAST_ADDR = 0xffff, 
  IEEE154_LINK_MTU = 127
};

struct ieee154_frame_addr {
  ieee154_addr_t ieee_src;
  ieee154_addr_t ieee_dst;
  ieee154_panid_t ieee_dstpan;
};

enum __nesc_unnamed4302 {
  IEEE154_MIN_HDR_SZ = 6
};
#line 74
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3
};

enum iee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3
};
# 28 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/ds2411/PlatformIeeeEui64.h"
enum __nesc_unnamed4303 {

  IEEE_EUI64_COMPANY_ID_0 = 0x00, 
  IEEE_EUI64_COMPANY_ID_1 = 0x12, 
  IEEE_EUI64_COMPANY_ID_2 = 0x6d, 

  IEEE_EUI64_SERIAL_ID_0 = 'L', 
  IEEE_EUI64_SERIAL_ID_1 = 'O'
};
# 43 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420TimeSyncMessage.h"
typedef nx_uint32_t timesync_radio_t;





#line 45
typedef nx_struct timesync_footer_t {

  nx_am_id_t type;
  timesync_radio_t timestamp;
} __attribute__((packed)) timesync_footer_t;
typedef TMilli AppP__MagTimer__precision_tag;
enum /*SDC.Msp430Usart0C*/Msp430Usart0C__0____nesc_unnamed4304 {
  Msp430Usart0C__0__CLIENT_ID = 0U
};
typedef TMilli Hmc5843P__testTimer__precision_tag;
typedef TI2CBasicAddr Hmc5843P__I2CPacket__addr_size;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4305 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
enum MagnetometerC____nesc_unnamed4306 {
  MagnetometerC__CLIENT_ID = 0U
};
typedef TI2CBasicAddr Msp430I2CP__I2CBasicAddr__addr_size;
enum CC2420ActiveMessageC____nesc_unnamed4307 {
  CC2420ActiveMessageC__CC2420_AM_SEND_ID = 0U
};
typedef T32khz CC2420ControlP__StartupTimer__precision_tag;
typedef uint32_t CC2420ControlP__StartupTimer__size_type;
typedef uint16_t CC2420ControlP__ReadRssi__val_t;
enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4308 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__1__from_precision_tag;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__from_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__to_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_precision_tag;
typedef uint32_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__size_type;
typedef TMicro /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__LocalTimeMicro__precision_tag;
typedef T32khz /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__LocalTime32khz__precision_tag;
typedef T32khz /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag;
typedef /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__precision_tag;
typedef /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__precision_tag;
typedef uint32_t /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type;
typedef TMicro /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag;
typedef /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__precision_tag;
typedef uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type;
typedef TMicro /*CounterMicro32C.Transform*/TransformCounterC__2__to_precision_tag;
typedef uint32_t /*CounterMicro32C.Transform*/TransformCounterC__2__to_size_type;
typedef TMicro /*CounterMicro32C.Transform*/TransformCounterC__2__from_precision_tag;
typedef uint16_t /*CounterMicro32C.Transform*/TransformCounterC__2__from_size_type;
typedef uint16_t /*CounterMicro32C.Transform*/TransformCounterC__2__upper_count_type;
typedef /*CounterMicro32C.Transform*/TransformCounterC__2__from_precision_tag /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__precision_tag;
typedef /*CounterMicro32C.Transform*/TransformCounterC__2__from_size_type /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__size_type;
typedef /*CounterMicro32C.Transform*/TransformCounterC__2__to_precision_tag /*CounterMicro32C.Transform*/TransformCounterC__2__Counter__precision_tag;
typedef /*CounterMicro32C.Transform*/TransformCounterC__2__to_size_type /*CounterMicro32C.Transform*/TransformCounterC__2__Counter__size_type;
typedef TMicro /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__precision_tag;
typedef /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__precision_tag /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__LocalTime__precision_tag;
typedef /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__precision_tag /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__Counter__precision_tag;
typedef uint32_t /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__Counter__size_type;
enum /*CC2420ControlC.Spi*/CC2420SpiC__0____nesc_unnamed4309 {
  CC2420SpiC__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0____nesc_unnamed4310 {
  Msp430Spi1C__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart1C__0____nesc_unnamed4311 {
  Msp430Usart1C__0__CLIENT_ID = 0U
};
enum /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1____nesc_unnamed4312 {
  CC2420SpiC__1__CLIENT_ID = 1U
};
enum /*CC2420ControlC.RssiResource*/CC2420SpiC__2____nesc_unnamed4313 {
  CC2420SpiC__2__CLIENT_ID = 2U
};
typedef T32khz CC2420TransmitP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420TransmitP__PacketTimeStamp__size_type;
typedef T32khz CC2420TransmitP__BackoffTimer__precision_tag;
typedef uint32_t CC2420TransmitP__BackoffTimer__size_type;
enum /*CC2420TransmitC.Spi*/CC2420SpiC__3____nesc_unnamed4314 {
  CC2420SpiC__3__CLIENT_ID = 3U
};
typedef T32khz CC2420ReceiveP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420ReceiveP__PacketTimeStamp__size_type;
typedef T32khz CC2420PacketP__PacketTimeStamp32khz__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStamp32khz__size_type;
typedef T32khz CC2420PacketP__LocalTime32khz__precision_tag;
typedef TMilli CC2420PacketP__LocalTimeMilli__precision_tag;
typedef TMilli CC2420PacketP__PacketTimeStampMilli__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStampMilli__size_type;
typedef T32khz /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__3__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__3__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__3__LocalTime__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__3__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__3__Counter__precision_tag;
typedef uint32_t /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__3__Counter__size_type;
enum /*CC2420ReceiveC.Spi*/CC2420SpiC__4____nesc_unnamed4315 {
  CC2420SpiC__4__CLIENT_ID = 4U
};
typedef uint16_t RandomMlcgC__SeedInit__parameter;
enum CC2420TinyosNetworkC____nesc_unnamed4316 {
  CC2420TinyosNetworkC__TINYOS_N_NETWORKS = 1U
};
enum DiagMsgC____nesc_unnamed4317 {
  DiagMsgC__AM_DIAG_MSG = 0xB1
};
enum AMQueueP____nesc_unnamed4318 {
  AMQueueP__NUM_CLIENTS = 1U
};
# 71 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Magnetometer.nc"
static void AppP__Magnetometer__writeDone(error_t success);
#line 69
static void AppP__Magnetometer__readDone(uint8_t *data, error_t success);
# 60 "/home/ali/workspace/TinyOS-main/tos/interfaces/Boot.nc"
static void AppP__Boot__booted(void );
# 113 "/home/ali/workspace/TinyOS-main/tos/interfaces/SplitControl.nc"
static void AppP__AMControl__startDone(error_t error);
#line 138
static void AppP__AMControl__stopDone(error_t error);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void AppP__SDtest__runTask(void );
# 83 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
static void AppP__MagTimer__fired(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SD.nc"
static void AppP__SD__unavailable(void );
#line 69
static void AppP__SD__available(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 62
static error_t MotePlatformC__Init__init(void );
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 43
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 45
static void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4076bcf8);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4076bcf8);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 68
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__clearOverflow(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControlAsCapture(uint8_t cm);
#line 42
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__clearPendingInterrupt(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 41 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t time);
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 41 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 76 "/home/ali/workspace/TinyOS-main/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 56 "/home/ali/workspace/TinyOS-main/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x406939e0);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 56 "/home/ali/workspace/TinyOS-main/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x406939e0);
# 57 "/home/ali/workspace/TinyOS-main/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 72
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 65
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/Leds.nc"
static void LedsP__Leds__led0Off(void );










static void LedsP__Leds__led1On(void );










static void LedsP__Leds__led1Toggle(void );
#line 78
static void LedsP__Leds__led1Off(void );
#line 57
static void LedsP__Leds__led0On(void );
# 73 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__get(void );
#line 99
static void /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__selectIOFunc(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__getRaw(void );
#line 92
static void /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__selectModuleFunc(void );
#line 73
static bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__makeInput(void );
#line 99
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__makeInput(void );
#line 99
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__set(void );




static void /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__clr(void );




static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__toggle(void );
#line 85
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );




static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P43*/HplMsp430GeneralIOP__27__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P43*/HplMsp430GeneralIOP__27__IO__set(void );
#line 99
static void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectModuleFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );




static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__set(void );




static void /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__clr(void );
# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void SDP__DockInterrupt__fired(void );
# 54 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SD.nc"
static error_t SDP__SD__readBlock(const uint32_t sector, uint8_t *buffer);
# 95 "/home/ali/workspace/TinyOS-main/tos/interfaces/StdControl.nc"
static error_t SDP__StdControl__start(void );
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a05df8, 
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a05df8);
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );
#line 39
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(
# 40 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a02698);
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a42148);
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22310);
# 90 "/home/ali/workspace/TinyOS-main/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 213 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
static bool HplMsp430Usart0P__Usart__isTxEmpty(void );
#line 180
static void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 97
static void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart0P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart0P__Usart__setUmctl(uint8_t umctl);
#line 207
static void HplMsp430Usart0P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart0P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart0P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart0P__Usart__disableUart(void );
#line 153
static void HplMsp430Usart0P__Usart__enableSpi(void );
#line 168
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t HplMsp430Usart0P__Usart__rx(void );
#line 192
static bool HplMsp430Usart0P__Usart__isRxIntrPending(void );
#line 158
static void HplMsp430Usart0P__Usart__disableSpi(void );
# 91 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
static bool HplMsp430I2C0P__HplI2C__isGeneralCallPending(void );
#line 7
static void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430I2C0P__HplI2C__isI2C(void );
#line 94
static bool HplMsp430I2C0P__HplI2C__isAccessReadyPending(void );


static bool HplMsp430I2C0P__HplI2C__isArbitrationLostPending(void );
#line 15
static void HplMsp430I2C0P__HplI2C__disableI2C(void );
#line 95
static bool HplMsp430I2C0P__HplI2C__isOwnAddressPending(void );
#line 8
static void HplMsp430I2C0P__HplI2C__setModeI2C(msp430_i2c_union_config_t *config);
#line 90
static bool HplMsp430I2C0P__HplI2C__isStartDetectPending(void );





static bool HplMsp430I2C0P__HplI2C__isNoAckPending(void );
# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__clear(void );
#line 72
static void HplMsp430InterruptP__Port14__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port26__clear(void );
#line 72
static void HplMsp430InterruptP__Port26__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port17__clear(void );
#line 72
static void HplMsp430InterruptP__Port17__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port21__clear(void );
#line 72
static void HplMsp430InterruptP__Port21__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port12__clear(void );
#line 72
static void HplMsp430InterruptP__Port12__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port24__clear(void );
#line 72
static void HplMsp430InterruptP__Port24__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port15__clear(void );
#line 72
static void HplMsp430InterruptP__Port15__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port27__clear(void );
#line 47
static void HplMsp430InterruptP__Port27__disable(void );
#line 67
static void HplMsp430InterruptP__Port27__edge(bool low_to_high);
#line 42
static void HplMsp430InterruptP__Port27__enable(void );









static void HplMsp430InterruptP__Port10__clear(void );
#line 47
static void HplMsp430InterruptP__Port10__disable(void );
#line 67
static void HplMsp430InterruptP__Port10__edge(bool low_to_high);
#line 42
static void HplMsp430InterruptP__Port10__enable(void );









static void HplMsp430InterruptP__Port22__clear(void );
#line 72
static void HplMsp430InterruptP__Port22__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port13__clear(void );
#line 72
static void HplMsp430InterruptP__Port13__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port25__clear(void );
#line 72
static void HplMsp430InterruptP__Port25__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port16__clear(void );
#line 72
static void HplMsp430InterruptP__Port16__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port20__clear(void );
#line 72
static void HplMsp430InterruptP__Port20__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port11__clear(void );
#line 72
static void HplMsp430InterruptP__Port11__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port23__clear(void );
#line 67
static void HplMsp430InterruptP__Port23__edge(bool low_to_high);
#line 42
static void HplMsp430InterruptP__Port23__enable(void );
#line 59
static bool HplMsp430InterruptP__Port23__getValue(void );
# 41 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Magnetometer.nc"
static void Hmc5843P__Magnetometer__disableBus(void );
#line 39
static void Hmc5843P__Magnetometer__enableBus(void );
#line 66
static void Hmc5843P__Magnetometer__convertRegistersToData(uint8_t *readBuf, int16_t *data);
#line 63
static void Hmc5843P__Magnetometer__selfTest(void );
#line 57
static error_t Hmc5843P__Magnetometer__readData(void );
#line 54
static error_t Hmc5843P__Magnetometer__runContinuousConversion(void );
# 83 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
static void Hmc5843P__testTimer__fired(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t Hmc5843P__Init__init(void );
# 112 "/home/ali/workspace/TinyOS-main/tos/interfaces/I2CPacket.nc"
static void Hmc5843P__I2CPacket__writeDone(error_t error, uint16_t addr, uint8_t length, 
#line 109
uint8_t * data);
#line 102
static void Hmc5843P__I2CPacket__readDone(error_t error, uint16_t addr, uint8_t length, 
#line 99
uint8_t * data);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 103 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 64
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 109 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 136 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 129
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 83 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 83
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 48 "/home/ali/workspace/TinyOS-main/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40cac9f0);
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 48 "/home/ali/workspace/TinyOS-main/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40cac9f0, 
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 48 "/home/ali/workspace/TinyOS-main/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40cac9f0);
# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void Msp430I2CP__writeDone__runTask(void );
# 76 "/home/ali/workspace/TinyOS-main/tos/interfaces/I2CPacket.nc"
static error_t Msp430I2CP__I2CBasicAddr__read(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 72
uint8_t * data);
#line 92
static error_t Msp430I2CP__I2CBasicAddr__write(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 88
uint8_t * data);
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void Msp430I2CP__I2CInterrupts__fired(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void Msp430I2CP__readDone__runTask(void );
# 104 "/home/ali/workspace/TinyOS-main/tos/interfaces/SplitControl.nc"
static error_t CC2420CsmaP__SplitControl__start(void );
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t * msg);
# 73 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420CsmaP__CC2420Transmit__sendDone(message_t * p_msg, error_t error);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static error_t CC2420CsmaP__Send__send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t CC2420CsmaP__Send__maxPayloadLength(void );
# 76 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420CsmaP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420CsmaP__CC2420Power__startVRegDone(void );
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void CC2420CsmaP__Resource__granted(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void CC2420CsmaP__sendDone_task__runTask(void );
#line 75
static void CC2420CsmaP__stopDone_task__runTask(void );
#line 75
static void CC2420CsmaP__startDone_task__runTask(void );
# 91 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
static bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 115
static bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );
#line 110
static bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );
#line 64
static ieee_eui64_t CC2420ControlP__CC2420Config__getExtAddr(void );




static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );
#line 52
static error_t CC2420ControlP__CC2420Config__sync(void );
#line 75
static uint16_t CC2420ControlP__CC2420Config__getPanAddr(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__fired(void );
# 63 "/home/ali/workspace/TinyOS-main/tos/interfaces/Read.nc"
static void CC2420ControlP__ReadRssi__default__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__syncDone__runTask(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t CC2420ControlP__Init__init(void );
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void CC2420ControlP__SpiResource__granted(void );
#line 102
static void CC2420ControlP__SyncResource__granted(void );
# 71 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420ControlP__CC2420Power__stopVReg(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__sync__runTask(void );
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__Resource__release(void );
#line 88
static error_t CC2420ControlP__Resource__request(void );
# 68 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
static void CC2420ControlP__InterruptCCA__fired(void );
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void CC2420ControlP__RssiResource__granted(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 103 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);
#line 73
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
#line 64
static /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void );
# 109 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 103
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__size_type dt);
#line 66
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__size_type dt);






static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );


static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );

static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
#line 43
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );


static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );


static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );


static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void );
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 54 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioCapture.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__captureFallingEdge(void );
#line 66
static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__disable(void );
#line 53
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__captureRisingEdge(void );
# 61 "/home/ali/workspace/TinyOS-main/tos/lib/timer/LocalTime.nc"
static uint32_t /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void );
# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static void /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );






static bool /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__isOverflowPending(void );










static void /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__overflow(void );
#line 64
static /*CounterMicro32C.Transform*/TransformCounterC__2__Counter__size_type /*CounterMicro32C.Transform*/TransformCounterC__2__Counter__get(void );
# 61 "/home/ali/workspace/TinyOS-main/tos/lib/timer/LocalTime.nc"
static uint32_t /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__LocalTime__get(void );
# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static void /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__Counter__overflow(void );
# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );
#line 53
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );
# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );
#line 54
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );
# 82 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiPacket.nc"
static void CC2420SpiP__SpiPacket__sendDone(
#line 75
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 62 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420SpiP__Fifo__continueRead(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5b38, 
# 62 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 91
static void CC2420SpiP__Fifo__default__writeDone(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5b38, 
# 91 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 82
static cc2420_status_t CC2420SpiP__Fifo__write(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5b38, 
# 82 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420SpiP__Fifo__beginRead(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5b38, 
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 71
static void CC2420SpiP__Fifo__default__readDone(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5b38, 
# 71 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 31 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__abortRelease(void );







static error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void CC2420SpiP__SpiResource__granted(void );
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420SpiP__Ram__write(
# 47 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint16_t arg_0x40ef45b0, 
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Ram.nc"
uint8_t offset, uint8_t * data, uint8_t length);
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420SpiP__Reg__read(
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef4d58, 
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t *data);







static cc2420_status_t CC2420SpiP__Reg__write(
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef4d58, 
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t data);
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__release(
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5108);
# 97 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__immediateRequest(
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5108);
# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__request(
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5108);
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void CC2420SpiP__Resource__default__granted(
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5108);
# 128 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static bool CC2420SpiP__Resource__isOwner(
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5108);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void CC2420SpiP__grant__runTask(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef3540);
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t StateImplP__Init__init(void );
# 56 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
static void StateImplP__State__toIdle(
# 67 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
uint8_t arg_0x40f56940);
# 66 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
static bool StateImplP__State__isState(
# 67 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
uint8_t arg_0x40f56940, 
# 66 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP__State__isIdle(
# 67 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
uint8_t arg_0x40f56940);
# 45 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
static error_t StateImplP__State__requestState(
# 67 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
uint8_t arg_0x40f56940, 
# 45 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
uint8_t reqState);





static void StateImplP__State__forceState(
# 67 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
uint8_t arg_0x40f56940, 
# 51 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
uint8_t reqState);
# 65 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(
# 76 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f6ca58);
# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(
# 76 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f6ca58);
# 70 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiPacket.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(
# 79 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f6abe0, 
# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
#line 82
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(
# 79 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f6abe0, 
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(
# 82 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f69eb0);
# 45 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiByte.nc"
static uint8_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f69460);
# 97 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f69460);
# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f69460);
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f69460);
# 128 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f69460);
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(
# 75 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f6c010);
# 97 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(
# 75 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f6c010);
# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(
# 75 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f6c010);
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(
# 75 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f6c010);
# 128 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(
# 75 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f6c010);
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );
# 180 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart1P__Usart__enableRxIntr(void );
#line 197
static void HplMsp430Usart1P__Usart__clrRxIntr(void );
#line 97
static void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart1P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart1P__Usart__setUmctl(uint8_t umctl);
#line 177
static void HplMsp430Usart1P__Usart__disableRxIntr(void );
#line 207
static void HplMsp430Usart1P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart1P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart1P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart1P__Usart__disableUart(void );
#line 153
static void HplMsp430Usart1P__Usart__enableSpi(void );
#line 168
static void HplMsp430Usart1P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t HplMsp430Usart1P__Usart__rx(void );
#line 192
static bool HplMsp430Usart1P__Usart__isRxIntrPending(void );
#line 158
static void HplMsp430Usart1P__Usart__disableSpi(void );
# 95 "/home/ali/workspace/TinyOS-main/tos/interfaces/AsyncStdControl.nc"
static error_t HplMsp430Usart1P__AsyncStdControl__start(void );









static error_t HplMsp430Usart1P__AsyncStdControl__stop(void );
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a05df8, 
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a05df8);
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init(void );
# 79 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void );








static bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(
# 55 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22d38);
# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(
# 55 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22d38);
# 65 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(
# 60 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a42148);
# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(
# 60 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a42148);
# 56 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(
# 54 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22310);
# 97 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(
# 54 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22310);
# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(
# 54 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22310);
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(
# 54 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22310);
# 128 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(
# 54 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22310);
# 90 "/home/ali/workspace/TinyOS-main/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 73 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 50 "/home/ali/workspace/TinyOS-main/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );




static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/ds2411/IDChip.nc"
static error_t HplDs2411C__IDChip__read(uint8_t *id_buf);
# 48 "/home/ali/workspace/TinyOS-main/tos/interfaces/LocalIeeeEui64.nc"
static ieee_eui64_t DallasId48ToIeeeEui64C__LocalIeeeEui64__getId(void );
# 66 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime);
# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioCapture.nc"
static void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static void CC2420TransmitP__BackoffTimer__fired(void );
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t * message);
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca);
# 24 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__releasing(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t CC2420TransmitP__Init__init(void );
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void CC2420TransmitP__SpiResource__granted(void );
# 95 "/home/ali/workspace/TinyOS-main/tos/interfaces/StdControl.nc"
static error_t CC2420TransmitP__StdControl__start(void );









static error_t CC2420TransmitP__StdControl__stop(void );
# 91 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420TransmitP__TXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420TransmitP__TXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ReceiveP__CC2420Config__syncDone(error_t error);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void CC2420ReceiveP__receiveDone_task__runTask(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t CC2420ReceiveP__Init__init(void );
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void CC2420ReceiveP__SpiResource__granted(void );
# 91 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420ReceiveP__RXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420ReceiveP__RXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 68 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
static void CC2420ReceiveP__InterruptFIFOP__fired(void );
# 95 "/home/ali/workspace/TinyOS-main/tos/interfaces/StdControl.nc"
static error_t CC2420ReceiveP__StdControl__start(void );









static error_t CC2420ReceiveP__StdControl__stop(void );
# 77 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static void CC2420PacketP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId);
#line 75
static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg);
# 70 "/home/ali/workspace/TinyOS-main/tos/interfaces/PacketTimeStamp.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(
#line 66
message_t * msg);
#line 78
static void CC2420PacketP__PacketTimeStamp32khz__set(
#line 73
message_t * msg, 




CC2420PacketP__PacketTimeStamp32khz__size_type value);
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420PacketP__CC2420PacketBody__getMetadata(message_t * msg);
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420PacketP__PacketTimeSyncOffset__get(
#line 53
message_t * msg);
#line 50
static bool CC2420PacketP__PacketTimeSyncOffset__isSet(
#line 46
message_t * msg);
# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__3__Counter__overflow(void );
# 52 "/home/ali/workspace/TinyOS-main/tos/interfaces/Random.nc"
static uint16_t RandomMlcgC__Random__rand16(void );
#line 46
static uint32_t RandomMlcgC__Random__rand32(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static void UniqueSendP__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
#line 75
static error_t UniqueSendP__Send__send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t UniqueSendP__Send__maxPayloadLength(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t UniqueSendP__Init__init(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t UniqueReceiveP__Init__init(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__DuplicateReceive__default__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void CC2420TinyosNetworkP__grantTask__runTask(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__ActiveSend__send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__BareReceive__default__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__release(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x4122b2d0);
# 97 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__immediateRequest(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x4122b2d0);
# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__request(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x4122b2d0);
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__default__granted(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x4122b2d0);
# 125 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static 
#line 123
void * 

CC2420TinyosNetworkP__BareSend__getPayload(
#line 122
message_t * msg, 


uint8_t len);
#line 100
static void CC2420TinyosNetworkP__BareSend__default__sendDone(
#line 96
message_t * msg, 



error_t error);
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );
# 79 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceQueue.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
#line 53
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );








static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static void CC2420ActiveMessageP__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error);
# 95 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP__RadioBackoff__default__requestCca(
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x4125fdb8, 
# 95 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
#line 81
static void CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x4125fdb8, 
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);






static void CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x4125fdb8, 
# 88 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/SendNotifier.nc"
static void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x4125f740, 
# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 95 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP__SubBackoff__requestCca(message_t * msg);
#line 81
static void CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(message_t * msg);






static void CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(message_t * msg);
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Packet.nc"
static uint8_t CC2420ActiveMessageP__Packet__payloadLength(
#line 74
message_t * msg);
#line 106
static uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void );
#line 94
static void CC2420ActiveMessageP__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 80 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
static error_t CC2420ActiveMessageP__AMSend__send(
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x41264b88, 
# 80 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__Snoop__default__receive(
# 50 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x41261cc0, 
# 71 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
#line 78
static 
#line 74
message_t * 



CC2420ActiveMessageP__Receive__default__receive(
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x41261628, 
# 71 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 68 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMPacket.nc"
static am_addr_t CC2420ActiveMessageP__AMPacket__address(void );









static am_addr_t CC2420ActiveMessageP__AMPacket__destination(
#line 74
message_t * amsg);
#line 103
static void CC2420ActiveMessageP__AMPacket__setDestination(
#line 99
message_t * amsg, 



am_addr_t addr);
#line 147
static am_id_t CC2420ActiveMessageP__AMPacket__type(
#line 143
message_t * amsg);
#line 162
static void CC2420ActiveMessageP__AMPacket__setType(
#line 158
message_t * amsg, 



am_id_t t);
#line 136
static bool CC2420ActiveMessageP__AMPacket__isForMe(
#line 133
message_t * amsg);
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void CC2420ActiveMessageP__RadioResource__granted(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void DiagMsgP__send__runTask(void );
# 110 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
static void DiagMsgP__AMSend__sendDone(
#line 103
message_t * msg, 






error_t error);
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t DiagMsgP__Init__init(void );
# 92 "/home/ali/workspace/TinyOS-main/tos/lib/diagmsg/DiagMsg.nc"
static void DiagMsgP__DiagMsg__str(const char *value);




static void DiagMsgP__DiagMsg__send(void );
#line 49
static bool DiagMsgP__DiagMsg__record(void );
#line 78
static void DiagMsgP__DiagMsg__int16s(const int16_t *value, uint8_t len);








static void DiagMsgP__DiagMsg__chrs(const char *value, uint8_t len);
# 80 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
static error_t /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static void /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 110 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(
# 48 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
am_id_t arg_0x41343490, 
# 103 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(
# 46 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
uint8_t arg_0x41346a58, 
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 100
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 46 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
uint8_t arg_0x41346a58, 
# 96 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );
#line 75
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
# 41 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Magnetometer.nc"
static void AppP__Magnetometer__disableBus(void );
#line 39
static void AppP__Magnetometer__enableBus(void );
#line 66
static void AppP__Magnetometer__convertRegistersToData(uint8_t *readBuf, int16_t *data);
#line 57
static error_t AppP__Magnetometer__readData(void );
#line 54
static error_t AppP__Magnetometer__runContinuousConversion(void );
# 104 "/home/ali/workspace/TinyOS-main/tos/interfaces/SplitControl.nc"
static error_t AppP__AMControl__start(void );
# 95 "/home/ali/workspace/TinyOS-main/tos/interfaces/StdControl.nc"
static error_t AppP__SDStdControl__start(void );
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t AppP__SDtest__postTask(void );
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
static void AppP__MagTimer__startPeriodic(uint32_t dt);
# 62 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/Leds.nc"
static void AppP__Leds__led0Off(void );










static void AppP__Leds__led1On(void );










static void AppP__Leds__led1Toggle(void );
#line 78
static void AppP__Leds__led1Off(void );
#line 57
static void AppP__Leds__led0On(void );
# 97 "/home/ali/workspace/TinyOS-main/tos/lib/diagmsg/DiagMsg.nc"
static void AppP__DiagMsg__send(void );
#line 49
static bool AppP__DiagMsg__record(void );
#line 78
static void AppP__DiagMsg__int16s(const int16_t *value, uint8_t len);
# 54 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SD.nc"
static error_t AppP__SD__readBlock(const uint32_t sector, uint8_t *buffer);
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t AppP__MagInit__init(void );
# 58 "AppP.nc"
enum AppP____nesc_unnamed4319 {
#line 58
  AppP__SDtest = 0U
};
#line 58
typedef int AppP____nesc_sillytask_SDtest[AppP__SDtest];
#line 55
uint8_t AppP__buffer[512];
int16_t AppP__mag[3];

static inline void AppP__SDtest__runTask(void );
#line 86
static inline void AppP__SD__available(void );




static inline void AppP__SD__unavailable(void );

static inline void AppP__Boot__booted(void );






static inline void AppP__AMControl__startDone(error_t error);
#line 114
static inline void AppP__AMControl__stopDone(error_t error);

static inline void AppP__MagTimer__fired(void );







static inline void AppP__sendData(void );








static inline void AppP__Magnetometer__readDone(uint8_t *data, error_t error);
#line 149
static inline void AppP__Magnetometer__writeDone(error_t success);
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 62
static error_t PlatformP__MoteClockInit__init(void );
#line 62
static error_t PlatformP__LedsInit__init(void );
# 10 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/MotePlatformC.nc"
static inline error_t MotePlatformC__Init__init(void );
# 43 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4320 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 79
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 100
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 115
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 130
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 163
static inline void Msp430ClockP__startTimerB(void );
#line 175
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 204
static inline void Msp430ClockP__busyCalibrateDco(void );
#line 229
static inline error_t Msp430ClockP__Init__init(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4076bcf8);
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
#line 81
static inline bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4076bcf8);
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 81
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 72
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__captureControl(uint8_t l_cm);
#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__clearPendingInterrupt(void );
#line 110
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControlAsCapture(uint8_t cm);
#line 130
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 175
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__clearOverflow(void );




static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
#line 192
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__get(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCompare(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(12)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(10)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(26)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(24)))  ;
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 59 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 112
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 60 "/home/ali/workspace/TinyOS-main/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 57 "/home/ali/workspace/TinyOS-main/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 72
static void RealMainP__Scheduler__taskLoop(void );
#line 65
static bool RealMainP__Scheduler__runNextTask(void );
# 63 "/home/ali/workspace/TinyOS-main/tos/system/RealMainP.nc"
int main(void )   ;
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 56 "/home/ali/workspace/TinyOS-main/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x406939e0);
# 76 "/home/ali/workspace/TinyOS-main/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 61 "/home/ali/workspace/TinyOS-main/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4321 {

  SchedulerBasicP__NUM_TASKS = 19U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 97
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 124
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 170
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__makeOutput(void );
#line 40
static void LedsP__Led0__set(void );
static void LedsP__Led0__clr(void );
static void LedsP__Led1__toggle(void );



static void LedsP__Led1__makeOutput(void );
#line 40
static void LedsP__Led1__set(void );
static void LedsP__Led1__clr(void );




static void LedsP__Led2__makeOutput(void );
#line 40
static void LedsP__Led2__set(void );
# 57 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 75
static inline void LedsP__Leds__led0On(void );




static inline void LedsP__Leds__led0Off(void );









static inline void LedsP__Leds__led1On(void );




static inline void LedsP__Leds__led1Off(void );




static inline void LedsP__Leds__led1Toggle(void );
# 59 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__selectIOFunc(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__makeInput(void );
#line 61
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__makeInput(void );



static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 61
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__makeInput(void );



static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 56
static void /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__set(void );
static void /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__makeOutput(void );
#line 56
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P43*/HplMsp430GeneralIOP__27__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P43*/HplMsp430GeneralIOP__27__IO__makeOutput(void );

static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 56
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 56
static void /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__set(void );
static void /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__makeOutput(void );
# 85 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void );
#line 85
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 85 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void SDP__DockInterrupt__clear(void );
#line 67
static void SDP__DockInterrupt__edge(bool low_to_high);
#line 42
static void SDP__DockInterrupt__enable(void );
#line 59
static bool SDP__DockInterrupt__getValue(void );
# 213 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
static bool SDP__Usart__isTxEmpty(void );
#line 180
static void SDP__Usart__enableRxIntr(void );
#line 90
static void SDP__Usart__setUmctl(uint8_t umctl);
#line 80
static void SDP__Usart__setUbr(uint16_t ubr);
#line 224
static void SDP__Usart__tx(uint8_t data);
#line 168
static void SDP__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t SDP__Usart__rx(void );
#line 192
static bool SDP__Usart__isRxIntrPending(void );
#line 158
static void SDP__Usart__disableSpi(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SD.nc"
static void SDP__SD__unavailable(void );
#line 69
static void SDP__SD__available(void );
# 111 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SDP.nc"
static error_t SDP__cardInit(void );










static void SDP__powerCycle(void );
#line 151
static inline error_t SDP__StdControl__start(void );
#line 189
static inline void SDP__DockInterrupt__fired(void );
#line 206
static inline void SDP__initSPI(void );
#line 236
static uint8_t SDP__spiSendByte(const uint8_t data);
#line 248
static void SDP__sendCmd(const uint8_t cmd, uint32_t data, const uint8_t crc);
#line 266
static uint8_t SDP__getResponse(void );
#line 279
static inline uint8_t SDP__getXXResponse(const uint8_t resp);
#line 325
static inline error_t SDP__setIdle(void );
#line 349
static error_t SDP__cardInit(void );
#line 371
static inline error_t SDP__setBlockLength(const uint16_t len);
#line 396
static inline error_t SDP__read_block(const uint32_t address, const uint16_t count, uint8_t *buffer);
#line 445
static inline error_t SDP__SD__readBlock(const uint32_t sector, uint8_t *buffer);
# 90 "/home/ali/workspace/TinyOS-main/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a05df8, 
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a05df8);
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(
# 40 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a02698);








static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id);
# 49 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1____nesc_unnamed4322 {
#line 49
  FcfsResourceQueueC__1__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[1U];



static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a42148);
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22310);
#line 75
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4323 {
#line 75
  ArbiterP__0__grantedTask = 1U
};
#line 75
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4324 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4325 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4326 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
#line 153
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
#line 190
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);
#line 216
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);
# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__UCLK__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__UCLK__selectModuleFunc(void );
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart0P__Interrupts__txDone(void );
# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__URXD__selectIOFunc(void );
#line 99
static void HplMsp430Usart0P__UTXD__selectIOFunc(void );
# 7 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430Usart0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430Usart0P__HplI2C__isI2C(void );
# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SOMI__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__SOMI__selectModuleFunc(void );
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void HplMsp430Usart0P__I2CInterrupts__fired(void );
# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SIMO__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__SIMO__selectModuleFunc(void );
# 91 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static volatile uint8_t HplMsp430Usart0P__IE1 __asm ("0x0000");
static volatile uint8_t HplMsp430Usart0P__ME1 __asm ("0x0004");
static volatile uint8_t HplMsp430Usart0P__IFG1 __asm ("0x0002");
static volatile uint8_t HplMsp430Usart0P__U0TCTL __asm ("0x0071");

static volatile uint8_t HplMsp430Usart0P__U0TXBUF __asm ("0x0077");

void sig_UART0RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(18)))  ;





void sig_UART0TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(16)))  ;
#line 139
static void HplMsp430Usart0P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 214
static void HplMsp430Usart0P__Usart__disableUart(void );
#line 245
static inline void HplMsp430Usart0P__Usart__enableSpi(void );








static void HplMsp430Usart0P__Usart__disableSpi(void );








static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config);








static inline void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 330
static inline bool HplMsp430Usart0P__Usart__isTxEmpty(void );






static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void );
#line 352
static inline void HplMsp430Usart0P__Usart__clrIntr(void );
#line 364
static inline void HplMsp430Usart0P__Usart__disableIntr(void );



static inline void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 389
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data);



static inline uint8_t HplMsp430Usart0P__Usart__rx(void );
# 97 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset);
#line 128
static void HplMsp430I2C0P__HplUsart__disableUart(void );
#line 158
static void HplMsp430I2C0P__HplUsart__disableSpi(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430I2C0P__UCLK__makeInput(void );
#line 92
static void HplMsp430I2C0P__UCLK__selectModuleFunc(void );
#line 78
static void HplMsp430I2C0P__SIMO__makeInput(void );
#line 92
static void HplMsp430I2C0P__SIMO__selectModuleFunc(void );
# 49 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430I2C0P.nc"
static volatile uint8_t HplMsp430I2C0P__U0CTL __asm ("0x0070");
static volatile uint8_t HplMsp430I2C0P__I2CTCTL __asm ("0x0071");


static volatile uint8_t HplMsp430I2C0P__I2CIE __asm ("0x0050");

static inline bool HplMsp430I2C0P__HplI2C__isI2C(void );



static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void );






static void HplMsp430I2C0P__HplI2C__setModeI2C(msp430_i2c_union_config_t *config);
#line 116
static inline void HplMsp430I2C0P__HplI2C__disableI2C(void );
#line 223
static inline bool HplMsp430I2C0P__HplI2C__isStartDetectPending(void );







static inline bool HplMsp430I2C0P__HplI2C__isGeneralCallPending(void );
#line 247
static inline bool HplMsp430I2C0P__HplI2C__isAccessReadyPending(void );







static inline bool HplMsp430I2C0P__HplI2C__isOwnAddressPending(void );







static inline bool HplMsp430I2C0P__HplI2C__isNoAckPending(void );







static inline bool HplMsp430I2C0P__HplI2C__isArbitrationLostPending(void );
# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__fired(void );
#line 72
static void HplMsp430InterruptP__Port26__fired(void );
#line 72
static void HplMsp430InterruptP__Port17__fired(void );
#line 72
static void HplMsp430InterruptP__Port21__fired(void );
#line 72
static void HplMsp430InterruptP__Port12__fired(void );
#line 72
static void HplMsp430InterruptP__Port24__fired(void );
#line 72
static void HplMsp430InterruptP__Port15__fired(void );
#line 72
static void HplMsp430InterruptP__Port27__fired(void );
#line 72
static void HplMsp430InterruptP__Port10__fired(void );
#line 72
static void HplMsp430InterruptP__Port22__fired(void );
#line 72
static void HplMsp430InterruptP__Port13__fired(void );
#line 72
static void HplMsp430InterruptP__Port25__fired(void );
#line 72
static void HplMsp430InterruptP__Port16__fired(void );
#line 72
static void HplMsp430InterruptP__Port20__fired(void );
#line 72
static void HplMsp430InterruptP__Port11__fired(void );
#line 72
static void HplMsp430InterruptP__Port23__fired(void );
# 64 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(8)))  ;
#line 79
static inline void HplMsp430InterruptP__Port11__default__fired(void );
static inline void HplMsp430InterruptP__Port12__default__fired(void );
static inline void HplMsp430InterruptP__Port13__default__fired(void );
static inline void HplMsp430InterruptP__Port14__default__fired(void );
static inline void HplMsp430InterruptP__Port15__default__fired(void );
static inline void HplMsp430InterruptP__Port16__default__fired(void );
static inline void HplMsp430InterruptP__Port17__default__fired(void );
static inline void HplMsp430InterruptP__Port10__enable(void );







static inline void HplMsp430InterruptP__Port10__disable(void );







static inline void HplMsp430InterruptP__Port10__clear(void );
static inline void HplMsp430InterruptP__Port11__clear(void );
static inline void HplMsp430InterruptP__Port12__clear(void );
static inline void HplMsp430InterruptP__Port13__clear(void );
static inline void HplMsp430InterruptP__Port14__clear(void );
static inline void HplMsp430InterruptP__Port15__clear(void );
static inline void HplMsp430InterruptP__Port16__clear(void );
static inline void HplMsp430InterruptP__Port17__clear(void );








static inline void HplMsp430InterruptP__Port10__edge(bool l2h);
#line 169
void sig_PORT2_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(2)))  ;
#line 182
static inline void HplMsp430InterruptP__Port20__default__fired(void );
static inline void HplMsp430InterruptP__Port21__default__fired(void );
static inline void HplMsp430InterruptP__Port22__default__fired(void );

static inline void HplMsp430InterruptP__Port24__default__fired(void );
static inline void HplMsp430InterruptP__Port25__default__fired(void );
static inline void HplMsp430InterruptP__Port26__default__fired(void );




static inline void HplMsp430InterruptP__Port23__enable(void );



static inline void HplMsp430InterruptP__Port27__enable(void );







static inline void HplMsp430InterruptP__Port27__disable(void );
static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );



static inline bool HplMsp430InterruptP__Port23__getValue(void );
#line 240
static void HplMsp430InterruptP__Port23__edge(bool l2h);
#line 264
static inline void HplMsp430InterruptP__Port27__edge(bool l2h);
# 71 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Magnetometer.nc"
static void Hmc5843P__Magnetometer__writeDone(error_t success);
#line 69
static void Hmc5843P__Magnetometer__readDone(uint8_t *data, error_t success);
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
static void Hmc5843P__testTimer__startPeriodic(uint32_t dt);
#line 78
static void Hmc5843P__testTimer__stop(void );
# 15 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void Hmc5843P__HplI2C__disableI2C(void );
#line 8
static void Hmc5843P__HplI2C__setModeI2C(msp430_i2c_union_config_t *config);
# 76 "/home/ali/workspace/TinyOS-main/tos/interfaces/I2CPacket.nc"
static error_t Hmc5843P__I2CPacket__read(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 72
uint8_t * data);
#line 92
static error_t Hmc5843P__I2CPacket__write(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 88
uint8_t * data);
# 67 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
uint8_t Hmc5843P__readbuff[7];
#line 67
uint8_t Hmc5843P__testPhase;
uint8_t Hmc5843P__packet[2];
#line 68
uint8_t Hmc5843P__readSize;
#line 68
uint8_t *Hmc5843P__readDataBuffer;

msp430_i2c_union_config_t Hmc5843P__msp430_i2c_my_config = { 
{ 
.rxdmaen = 0, 
.txdmaen = 0, 
.xa = 0, 
.listen = 0, 
.mst = 1, 
.i2cword = 0, 
.i2crm = 1, 
.i2cssel = 0x2, 
.i2cpsc = 0, 
.i2csclh = 0x3, 
.i2cscll = 0x3, 
.i2coa = 0 } };



static inline error_t Hmc5843P__Init__init(void );
#line 106
static inline void Hmc5843P__Magnetometer__enableBus(void );



static inline void Hmc5843P__Magnetometer__disableBus(void );



static error_t Hmc5843P__writeRegValue(uint8_t reg_addr, uint8_t val);










static error_t Hmc5843P__readValues(uint8_t size, uint8_t *data);
#line 182
static inline void Hmc5843P__Magnetometer__selfTest(void );
#line 204
static inline void Hmc5843P__testTimer__fired(void );
#line 260
static inline error_t Hmc5843P__Magnetometer__runContinuousConversion(void );








static inline error_t Hmc5843P__Magnetometer__readData(void );
#line 293
static inline int16_t Hmc5843P__twos_comp_pack(uint8_t up, uint8_t low);
#line 308
static inline void Hmc5843P__Magnetometer__convertRegistersToData(uint8_t *readBuf, int16_t *data);
#line 333
static inline void Hmc5843P__I2CPacket__readDone(error_t success, uint16_t addr, uint8_t length, uint8_t *data);



static inline void Hmc5843P__I2CPacket__writeDone(error_t success, uint16_t addr, uint8_t length, uint8_t *data);
# 41 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 47
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 65
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 67 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4327 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 133
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 77 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4328 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 5, 
  TransformAlarmC__0__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 147
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt);
#line 162
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 177
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 109 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 83 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 74 "/home/ali/workspace/TinyOS-main/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4329 {
#line 74
  AlarmToTimerC__0__fired = 2U
};
#line 74
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 55
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 71
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 136 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 129
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 48 "/home/ali/workspace/TinyOS-main/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40cac9f0);
#line 71
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4330 {
#line 71
  VirtualizeTimerC__0__updateFromTimer = 3U
};
#line 71
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 53
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4331 {

  VirtualizeTimerC__0__NUM_TIMERS = 3U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 59
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4332 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 100
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 139
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);









static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);
#line 204
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 58 "/home/ali/workspace/TinyOS-main/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t Msp430I2CP__writeDone__postTask(void );
# 112 "/home/ali/workspace/TinyOS-main/tos/interfaces/I2CPacket.nc"
static void Msp430I2CP__I2CBasicAddr__writeDone(error_t error, uint16_t addr, uint8_t length, 
#line 109
uint8_t * data);
#line 102
static void Msp430I2CP__I2CBasicAddr__readDone(error_t error, uint16_t addr, uint8_t length, 
#line 99
uint8_t * data);
# 91 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
static bool Msp430I2CP__HplI2C__isGeneralCallPending(void );


static bool Msp430I2CP__HplI2C__isAccessReadyPending(void );


static bool Msp430I2CP__HplI2C__isArbitrationLostPending(void );
#line 95
static bool Msp430I2CP__HplI2C__isOwnAddressPending(void );
#line 90
static bool Msp430I2CP__HplI2C__isStartDetectPending(void );





static bool Msp430I2CP__HplI2C__isNoAckPending(void );
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t Msp430I2CP__readDone__postTask(void );
# 72 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/Msp430I2CP.nc"
enum Msp430I2CP____nesc_unnamed4333 {
#line 72
  Msp430I2CP__readDone = 4U
};
#line 72
typedef int Msp430I2CP____nesc_sillytask_readDone[Msp430I2CP__readDone];
#line 88
enum Msp430I2CP____nesc_unnamed4334 {
#line 88
  Msp430I2CP__writeDone = 5U
};
#line 88
typedef int Msp430I2CP____nesc_sillytask_writeDone[Msp430I2CP__writeDone];
#line 51
static volatile uint8_t Msp430I2CP__I2CIE __asm ("0x0050");

enum Msp430I2CP____nesc_unnamed4335 {
  Msp430I2CP__OFF = 1, 
  Msp430I2CP__IDLE, 
  Msp430I2CP__PACKET_WRITE, 
  Msp430I2CP__PACKET_READ
};







uint8_t Msp430I2CP__stateI2C = Msp430I2CP__IDLE;
uint8_t Msp430I2CP__length;
uint8_t Msp430I2CP__ptr;
error_t Msp430I2CP__result;
uint8_t *Msp430I2CP__data;

static inline void Msp430I2CP__readDone__runTask(void );
#line 88
static inline void Msp430I2CP__writeDone__runTask(void );
#line 108
static inline error_t Msp430I2CP__I2CBasicAddr__read(i2c_flags_t flags, 
uint16_t _addr, uint8_t _length, 
uint8_t *_data);
#line 160
static inline void Msp430I2CP__localRxData(void );
#line 187
static inline error_t Msp430I2CP__I2CBasicAddr__write(i2c_flags_t flags, 
uint16_t _addr, uint8_t _length, 
uint8_t *_data);
#line 239
static inline void Msp430I2CP__localTxData(void );
#line 266
static void Msp430I2CP__localNoAck(void );
#line 287
static inline void Msp430I2CP__I2CInterrupts__fired(void );
# 113 "/home/ali/workspace/TinyOS-main/tos/interfaces/SplitControl.nc"
static void CC2420CsmaP__SplitControl__startDone(error_t error);
#line 138
static void CC2420CsmaP__SplitControl__stopDone(error_t error);
# 95 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420CsmaP__RadioBackoff__requestCca(message_t * msg);
#line 81
static void CC2420CsmaP__RadioBackoff__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__RadioBackoff__requestCongestionBackoff(message_t * msg);
#line 66
static void CC2420CsmaP__SubBackoff__setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420CsmaP__SubBackoff__setInitialBackoff(uint16_t backoffTime);
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg, bool useCca);
# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static void CC2420CsmaP__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 52 "/home/ali/workspace/TinyOS-main/tos/interfaces/Random.nc"
static uint16_t CC2420CsmaP__Random__rand16(void );
# 95 "/home/ali/workspace/TinyOS-main/tos/interfaces/StdControl.nc"
static error_t CC2420CsmaP__SubControl__start(void );









static error_t CC2420CsmaP__SubControl__stop(void );
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg);
# 71 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420CsmaP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420CsmaP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420CsmaP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420CsmaP__CC2420Power__stopVReg(void );
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420CsmaP__Resource__release(void );
#line 88
static error_t CC2420CsmaP__Resource__request(void );
# 66 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState);
#line 45
static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState);





static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState);
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t CC2420CsmaP__sendDone_task__postTask(void );
#line 67
static error_t CC2420CsmaP__stopDone_task__postTask(void );
#line 67
static error_t CC2420CsmaP__startDone_task__postTask(void );
# 74 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
enum CC2420CsmaP____nesc_unnamed4336 {
#line 74
  CC2420CsmaP__startDone_task = 6U
};
#line 74
typedef int CC2420CsmaP____nesc_sillytask_startDone_task[CC2420CsmaP__startDone_task];
enum CC2420CsmaP____nesc_unnamed4337 {
#line 75
  CC2420CsmaP__stopDone_task = 7U
};
#line 75
typedef int CC2420CsmaP____nesc_sillytask_stopDone_task[CC2420CsmaP__stopDone_task];
enum CC2420CsmaP____nesc_unnamed4338 {
#line 76
  CC2420CsmaP__sendDone_task = 8U
};
#line 76
typedef int CC2420CsmaP____nesc_sillytask_sendDone_task[CC2420CsmaP__sendDone_task];
#line 58
enum CC2420CsmaP____nesc_unnamed4339 {
  CC2420CsmaP__S_STOPPED, 
  CC2420CsmaP__S_STARTING, 
  CC2420CsmaP__S_STARTED, 
  CC2420CsmaP__S_STOPPING, 
  CC2420CsmaP__S_TRANSMITTING
};

message_t * CC2420CsmaP__m_msg;

error_t CC2420CsmaP__sendErr = SUCCESS;


bool CC2420CsmaP__ccaOn;






static inline void CC2420CsmaP__shutdown(void );


static inline error_t CC2420CsmaP__SplitControl__start(void );
#line 122
static inline error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len);
#line 173
static inline uint8_t CC2420CsmaP__Send__maxPayloadLength(void );
#line 205
static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err);




static inline void CC2420CsmaP__CC2420Power__startVRegDone(void );



static inline void CC2420CsmaP__Resource__granted(void );



static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void );




static inline void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t *msg);






static inline void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t *msg);
#line 244
static inline void CC2420CsmaP__sendDone_task__runTask(void );
#line 257
static inline void CC2420CsmaP__startDone_task__runTask(void );







static inline void CC2420CsmaP__stopDone_task__runTask(void );









static inline void CC2420CsmaP__shutdown(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ControlP__CC2420Config__syncDone(error_t error);
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data);
# 48 "/home/ali/workspace/TinyOS-main/tos/interfaces/LocalIeeeEui64.nc"
static ieee_eui64_t CC2420ControlP__LocalIeeeEui64__getId(void );
# 66 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt);
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data);
# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__RSTN__makeOutput(void );
#line 40
static void CC2420ControlP__RSTN__set(void );
static void CC2420ControlP__RSTN__clr(void );
# 63 "/home/ali/workspace/TinyOS-main/tos/interfaces/Read.nc"
static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val);
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t CC2420ControlP__syncDone__postTask(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data);







static cc2420_status_t CC2420ControlP__TXCTRL__write(uint16_t data);
#line 63
static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data);
# 50 "/home/ali/workspace/TinyOS-main/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void );




static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void );
# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__CSN__makeOutput(void );
#line 40
static void CC2420ControlP__CSN__set(void );
static void CC2420ControlP__CSN__clr(void );




static void CC2420ControlP__VREN__makeOutput(void );
#line 40
static void CC2420ControlP__VREN__set(void );
static void CC2420ControlP__VREN__clr(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void );
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__SpiResource__release(void );
#line 88
static error_t CC2420ControlP__SpiResource__request(void );
#line 120
static error_t CC2420ControlP__SyncResource__release(void );
#line 88
static error_t CC2420ControlP__SyncResource__request(void );
# 76 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420ControlP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420ControlP__CC2420Power__startVRegDone(void );
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data);
#line 63
static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data);
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRXON__strobe(void );
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void CC2420ControlP__Resource__granted(void );
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420ControlP__IEEEADR__write(uint8_t offset, uint8_t * data, uint8_t length);
# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ControlP__InterruptCCA__disable(void );
#line 53
static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void );
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__RssiResource__release(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void );
# 125 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
enum CC2420ControlP____nesc_unnamed4340 {
#line 125
  CC2420ControlP__sync = 9U
};
#line 125
typedef int CC2420ControlP____nesc_sillytask_sync[CC2420ControlP__sync];
enum CC2420ControlP____nesc_unnamed4341 {
#line 126
  CC2420ControlP__syncDone = 10U
};
#line 126
typedef int CC2420ControlP____nesc_sillytask_syncDone[CC2420ControlP__syncDone];
#line 90
#line 84
typedef enum CC2420ControlP____nesc_unnamed4342 {
  CC2420ControlP__S_VREG_STOPPED, 
  CC2420ControlP__S_VREG_STARTING, 
  CC2420ControlP__S_VREG_STARTED, 
  CC2420ControlP__S_XOSC_STARTING, 
  CC2420ControlP__S_XOSC_STARTED
} CC2420ControlP__cc2420_control_state_t;

uint8_t CC2420ControlP__m_channel;

uint8_t CC2420ControlP__m_tx_power;

uint16_t CC2420ControlP__m_pan;

uint16_t CC2420ControlP__m_short_addr;

ieee_eui64_t CC2420ControlP__m_ext_addr;

bool CC2420ControlP__m_sync_busy;


bool CC2420ControlP__autoAckEnabled;


bool CC2420ControlP__hwAutoAckDefault;


bool CC2420ControlP__addressRecognition;


bool CC2420ControlP__hwAddressRecognition;

CC2420ControlP__cc2420_control_state_t CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;



static void CC2420ControlP__writeFsctrl(void );
static void CC2420ControlP__writeMdmctrl0(void );
static void CC2420ControlP__writeId(void );
static inline void CC2420ControlP__writeTxctrl(void );





static inline error_t CC2420ControlP__Init__init(void );
#line 180
static inline error_t CC2420ControlP__Resource__request(void );







static inline error_t CC2420ControlP__Resource__release(void );







static inline error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 208
static inline error_t CC2420ControlP__CC2420Power__stopVReg(void );







static inline error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 260
static inline error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 290
static inline ieee_eui64_t CC2420ControlP__CC2420Config__getExtAddr(void );



static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );







static inline uint16_t CC2420ControlP__CC2420Config__getPanAddr(void );
#line 315
static inline error_t CC2420ControlP__CC2420Config__sync(void );
#line 347
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 374
static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );






static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );









static inline void CC2420ControlP__SyncResource__granted(void );
#line 405
static inline void CC2420ControlP__SpiResource__granted(void );




static inline void CC2420ControlP__RssiResource__granted(void );
#line 423
static inline void CC2420ControlP__StartupTimer__fired(void );









static inline void CC2420ControlP__InterruptCCA__fired(void );
#line 457
static inline void CC2420ControlP__sync__runTask(void );



static inline void CC2420ControlP__syncDone__runTask(void );









static void CC2420ControlP__writeFsctrl(void );
#line 488
static void CC2420ControlP__writeMdmctrl0(void );
#line 507
static void CC2420ControlP__writeId(void );
#line 524
static inline void CC2420ControlP__writeTxctrl(void );
#line 536
static inline void CC2420ControlP__ReadRssi__default__readDone(error_t error, uint16_t data);
# 41 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
#line 47
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void );










static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 44
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
#line 65
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get(void );






static bool /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow(void );
# 67 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformCounterC.nc"
/*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper;

enum /*Counter32khz32C.Transform*/TransformCounterC__1____nesc_unnamed4343 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type ) - /*Counter32khz32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type ) + 0, 



  TransformCounterC__1__OVERFLOW_MASK = /*Counter32khz32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type )2 << (/*Counter32khz32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void );
#line 133
static inline void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__fired(void );
#line 103
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt);
#line 73
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void );
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__get(void );
# 77 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0;
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt;

enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1____nesc_unnamed4344 {

  TransformAlarmC__1__MAX_DELAY_LOG2 = 8 * sizeof(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_size_type ) - 1 - 0, 
  TransformAlarmC__1__MAX_DELAY = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type )1 << /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__MAX_DELAY_LOG2
};

static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 102
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__set_alarm(void );
#line 147
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type dt);









static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type dt);




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
#line 177
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void );
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
# 85 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
# 73 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void );
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
# 73 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void );
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );
# 85 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void );
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
# 85 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr(void );
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
# 68 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430Capture__clearOverflow(void );
# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioCapture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__captured(uint16_t time);
# 61 "/home/ali/workspace/TinyOS-main/tos/lib/timer/LocalTime.nc"
static uint32_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__LocalTimeMicro__get(void );
#line 61
static uint32_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__LocalTime32khz__get(void );
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm);

static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__enableEvents(void );
static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__GeneralIO__selectIOFunc(void );
#line 92
static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__GeneralIO__selectModuleFunc(void );
# 56 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/chips/cc2420/CC2420GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__enableCapture(uint8_t mode);
#line 68
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__captureRisingEdge(void );



static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__captureFallingEdge(void );



static inline void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__disable(void );






static inline void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430Capture__captured(uint16_t capturedMicro);
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void );




static inline void /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void );
static bool /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__isOverflowPending(void );
# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void );
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );




static inline bool /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__isOverflowPending(void );









static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__size_type /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__get(void );






static bool /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__isOverflowPending(void );










static void /*CounterMicro32C.Transform*/TransformCounterC__2__Counter__overflow(void );
# 67 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformCounterC.nc"
/*CounterMicro32C.Transform*/TransformCounterC__2__upper_count_type /*CounterMicro32C.Transform*/TransformCounterC__2__m_upper;

enum /*CounterMicro32C.Transform*/TransformCounterC__2____nesc_unnamed4345 {

  TransformCounterC__2__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__2__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMicro32C.Transform*/TransformCounterC__2__from_size_type ) - /*CounterMicro32C.Transform*/TransformCounterC__2__LOW_SHIFT_RIGHT, 
  TransformCounterC__2__NUM_UPPER_BITS = 8 * sizeof(/*CounterMicro32C.Transform*/TransformCounterC__2__to_size_type ) - 8 * sizeof(/*CounterMicro32C.Transform*/TransformCounterC__2__from_size_type ) + 0, 



  TransformCounterC__2__OVERFLOW_MASK = /*CounterMicro32C.Transform*/TransformCounterC__2__NUM_UPPER_BITS ? ((/*CounterMicro32C.Transform*/TransformCounterC__2__upper_count_type )2 << (/*CounterMicro32C.Transform*/TransformCounterC__2__NUM_UPPER_BITS - 1)) - 1 : 0
};

static inline /*CounterMicro32C.Transform*/TransformCounterC__2__to_size_type /*CounterMicro32C.Transform*/TransformCounterC__2__Counter__get(void );
#line 133
static void /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__overflow(void );
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
static /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__Counter__size_type /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__Counter__get(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__LocalTime__get(void );




static inline void /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__Counter__overflow(void );
# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void );
#line 47
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void );
#line 67
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high);
#line 42
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void );
# 68 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void );
# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising);








static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );







static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void );
#line 47
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void );
#line 67
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high);
#line 42
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void );
# 68 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void );
# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising);
#line 65
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );



static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 70 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiPacket.nc"
static error_t CC2420SpiP__SpiPacket__send(
#line 59
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
# 91 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420SpiP__Fifo__writeDone(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5b38, 
# 91 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420SpiP__Fifo__readDone(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5b38, 
# 71 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 24 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__releasing(void );
# 45 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiByte.nc"
static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx);
# 56 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
static void CC2420SpiP__WorkingState__toIdle(void );




static bool CC2420SpiP__WorkingState__isIdle(void );
#line 45
static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState);
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__SpiResource__release(void );
#line 97
static error_t CC2420SpiP__SpiResource__immediateRequest(void );
#line 88
static error_t CC2420SpiP__SpiResource__request(void );
#line 128
static bool CC2420SpiP__SpiResource__isOwner(void );
#line 102
static void CC2420SpiP__Resource__granted(
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40ef5108);
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t CC2420SpiP__grant__postTask(void );
# 88 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
enum CC2420SpiP____nesc_unnamed4346 {
#line 88
  CC2420SpiP__grant = 11U
};
#line 88
typedef int CC2420SpiP____nesc_sillytask_grant[CC2420SpiP__grant];
#line 63
enum CC2420SpiP____nesc_unnamed4347 {
  CC2420SpiP__RESOURCE_COUNT = 5U, 
  CC2420SpiP__NO_HOLDER = 0xFF
};


enum CC2420SpiP____nesc_unnamed4348 {
  CC2420SpiP__S_IDLE, 
  CC2420SpiP__S_BUSY
};


uint16_t CC2420SpiP__m_addr;


uint8_t CC2420SpiP__m_requests = 0;


uint8_t CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;


bool CC2420SpiP__release;


static error_t CC2420SpiP__attemptRelease(void );







static inline void CC2420SpiP__ChipSpiResource__abortRelease(void );






static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );




static error_t CC2420SpiP__Resource__request(uint8_t id);
#line 126
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id);
#line 149
static error_t CC2420SpiP__Resource__release(uint8_t id);
#line 178
static inline bool CC2420SpiP__Resource__isOwner(uint8_t id);





static inline void CC2420SpiP__SpiResource__granted(void );




static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 209
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len);



static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 260
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len);
#line 287
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data);
#line 305
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data);
#line 318
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr);










static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);








static error_t CC2420SpiP__attemptRelease(void );
#line 358
static inline void CC2420SpiP__grant__runTask(void );








static inline void CC2420SpiP__Resource__default__granted(uint8_t id);


static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 74 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
uint8_t StateImplP__state[4U];

enum StateImplP____nesc_unnamed4349 {
  StateImplP__S_IDLE = 0
};


static inline error_t StateImplP__Init__init(void );
#line 96
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState);
#line 111
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState);






static inline void StateImplP__State__toIdle(uint8_t id);







static inline bool StateImplP__State__isIdle(uint8_t id);






static bool StateImplP__State__isState(uint8_t id, uint8_t myState);
# 82 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(
# 79 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f6abe0, 
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(
# 82 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f69eb0);
# 180 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void );
#line 197
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void );
#line 97
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset);
#line 177
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void );
#line 224
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data);
#line 168
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void );
#line 192
static bool /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void );
#line 158
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void );
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f69460);
# 97 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f69460);
# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f69460);
# 128 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f69460);
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(
# 75 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40f6c010);
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void );
# 102 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
enum /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4350 {
#line 102
  Msp430SpiNoDmaP__0__signalDone_task = 12U
};
#line 102
typedef int /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0____nesc_sillytask_signalDone_task[/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task];
#line 91
enum /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4351 {
  Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE = 2
};

uint16_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_len;
uint8_t * /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf;
uint8_t * /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf;
uint16_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_pos;
uint8_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_client;

static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );


static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id);



static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id);



static inline bool /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id);



static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id);



static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id);



static uint8_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
#line 173
static inline bool /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id);
static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id);
static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id);

static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void );
#line 206
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
#line 228
static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 245
static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );




static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );

static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__UCLK__selectIOFunc(void );
#line 92
static void HplMsp430Usart1P__UCLK__selectModuleFunc(void );
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart1P__Interrupts__txDone(void );
# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__URXD__selectIOFunc(void );
#line 99
static void HplMsp430Usart1P__UTXD__selectIOFunc(void );
#line 99
static void HplMsp430Usart1P__SOMI__selectIOFunc(void );
#line 92
static void HplMsp430Usart1P__SOMI__selectModuleFunc(void );






static void HplMsp430Usart1P__SIMO__selectIOFunc(void );
#line 92
static void HplMsp430Usart1P__SIMO__selectModuleFunc(void );
# 87 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static volatile uint8_t HplMsp430Usart1P__IE2 __asm ("0x0001");
static volatile uint8_t HplMsp430Usart1P__ME2 __asm ("0x0005");
static volatile uint8_t HplMsp430Usart1P__IFG2 __asm ("0x0003");
static volatile uint8_t HplMsp430Usart1P__U1TCTL __asm ("0x0079");

static volatile uint8_t HplMsp430Usart1P__U1TXBUF __asm ("0x007F");



void sig_UART1RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(6)))  ;




void sig_UART1TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(4)))  ;



static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void );



static inline error_t HplMsp430Usart1P__AsyncStdControl__stop(void );
#line 140
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 211
static inline void HplMsp430Usart1P__Usart__disableUart(void );
#line 242
static inline void HplMsp430Usart1P__Usart__enableSpi(void );








static void HplMsp430Usart1P__Usart__disableSpi(void );








static inline void HplMsp430Usart1P__configSpi(msp430_spi_union_config_t *config);








static void HplMsp430Usart1P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 332
static inline bool HplMsp430Usart1P__Usart__isRxIntrPending(void );










static inline void HplMsp430Usart1P__Usart__clrRxIntr(void );



static inline void HplMsp430Usart1P__Usart__clrIntr(void );



static inline void HplMsp430Usart1P__Usart__disableRxIntr(void );







static inline void HplMsp430Usart1P__Usart__disableIntr(void );



static inline void HplMsp430Usart1P__Usart__enableRxIntr(void );
#line 384
static inline void HplMsp430Usart1P__Usart__tx(uint8_t data);



static inline uint8_t HplMsp430Usart1P__Usart__rx(void );
# 90 "/home/ali/workspace/TinyOS-main/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(void );
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a05df8, 
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(
# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a05df8);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 49 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2____nesc_unnamed4352 {
#line 49
  FcfsResourceQueueC__2__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[1U];
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init(void );




static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void );
#line 82
static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
# 53 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(
# 55 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22d38);
# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(
# 55 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22d38);
# 65 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(
# 60 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a42148);
# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(
# 60 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a42148);
# 79 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void );
# 73 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void );
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(
# 54 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
uint8_t arg_0x40a22310);
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4353 {
#line 75
  ArbiterP__1__grantedTask = 13U
};
#line 75
typedef int /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1____nesc_sillytask_grantedTask[/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask];
#line 67
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4354 {
#line 67
  ArbiterP__1__RES_CONTROLLED, ArbiterP__1__RES_GRANTING, ArbiterP__1__RES_IMM_GRANTING, ArbiterP__1__RES_BUSY
};
#line 68
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4355 {
#line 68
  ArbiterP__1__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4356 {
#line 69
  ArbiterP__1__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;



static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(uint8_t id);
#line 93
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id);
#line 111
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id);
#line 133
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );










static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id);
#line 190
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id);

static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id);









static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id);
# 62 "/home/ali/workspace/TinyOS-main/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void );
# 56 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void );
# 95 "/home/ali/workspace/TinyOS-main/tos/interfaces/AsyncStdControl.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void );









static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop(void );
# 69 "/home/ali/workspace/TinyOS-main/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__requested(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/system/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC__addr = TOS_AM_ADDRESS;


am_group_t ActiveMessageAddressC__group = TOS_AM_GROUP;






static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
#line 93
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
#line 106
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 65 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/ds2411/HplDs2411C.nc"
enum HplDs2411C____nesc_unnamed4357 {

  HplDs2411C__STD_A = 6, 
  HplDs2411C__STD_B = 64, 
  HplDs2411C__STD_C = 60, 
  HplDs2411C__STD_D = 10, 
  HplDs2411C__STD_E = 9, 
  HplDs2411C__STD_F = 55, 
  HplDs2411C__STD_G = 0, 
  HplDs2411C__STD_H = 480, 
  HplDs2411C__STD_I = 90, 
  HplDs2411C__STD_J = 220
};

static inline void HplDs2411C__init_pins(void );
#line 91
static inline void HplDs2411C__clear_pins(void );







static inline bool HplDs2411C__reset(void );
#line 111
static inline void HplDs2411C__write_bit_one(void );







static inline void HplDs2411C__write_bit_zero(void );







static inline void HplDs2411C__write_bit(int is_one);







static inline bool HplDs2411C__read_bit(void );
#line 147
static inline void HplDs2411C__write_byte(uint8_t byte);






static inline uint8_t HplDs2411C__read_byte(void );
#line 166
static inline uint8_t HplDs2411C__crc8_byte(uint8_t crc, uint8_t byte);
#line 188
static inline error_t HplDs2411C__IDChip__read(uint8_t *id_buf);
# 44 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/ds2411/IDChip.nc"
static error_t DallasId48ToIeeeEui64C__IDChip__read(uint8_t *id_buf);
# 8 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/ds2411/DallasId48ToIeeeEui64C.nc"
static ieee_eui64_t DallasId48ToIeeeEui64C__LocalIeeeEui64__getId(void );
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420TransmitP__RadioBackoff__requestInitialBackoff(message_t * msg);






static void CC2420TransmitP__RadioBackoff__requestCongestionBackoff(message_t * msg);
# 70 "/home/ali/workspace/TinyOS-main/tos/interfaces/PacketTimeStamp.nc"
static void CC2420TransmitP__PacketTimeStamp__clear(
#line 66
message_t * msg);
#line 78
static void CC2420TransmitP__PacketTimeStamp__set(
#line 73
message_t * msg, 




CC2420TransmitP__PacketTimeStamp__size_type value);
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void );
# 54 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioCapture.nc"
static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void );
#line 66
static void CC2420TransmitP__CaptureSFD__disable(void );
#line 53
static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void );
# 109 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void );
#line 66
static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt);






static void CC2420TransmitP__BackoffTimer__stop(void );
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length);
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data);
# 55 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time);
# 73 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error);
# 31 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__abortRelease(void );







static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void );
# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CSN__makeOutput(void );
#line 40
static void CC2420TransmitP__CSN__set(void );
static void CC2420TransmitP__CSN__clr(void );
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg);
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(
#line 53
message_t * msg);
#line 50
static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(
#line 46
message_t * msg);
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420TransmitP__SpiResource__release(void );
#line 97
static error_t CC2420TransmitP__SpiResource__immediateRequest(void );
#line 88
static error_t CC2420TransmitP__SpiResource__request(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CCA__makeInput(void );
#line 43
static bool CC2420TransmitP__CCA__get(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SNOP__strobe(void );
# 44 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__SFD__makeInput(void );
#line 43
static bool CC2420TransmitP__SFD__get(void );
# 82 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length);
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXON__strobe(void );
# 99 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
#line 89
typedef enum CC2420TransmitP____nesc_unnamed4358 {
  CC2420TransmitP__S_STOPPED, 
  CC2420TransmitP__S_STARTED, 
  CC2420TransmitP__S_LOAD, 
  CC2420TransmitP__S_SAMPLE_CCA, 
  CC2420TransmitP__S_BEGIN_TRANSMIT, 
  CC2420TransmitP__S_SFD, 
  CC2420TransmitP__S_EFD, 
  CC2420TransmitP__S_ACK_WAIT, 
  CC2420TransmitP__S_CANCEL
} CC2420TransmitP__cc2420_transmit_state_t;





enum CC2420TransmitP____nesc_unnamed4359 {
  CC2420TransmitP__CC2420_ABORT_PERIOD = 320
};
#line 120
message_t * CC2420TransmitP__m_msg;

bool CC2420TransmitP__m_cca;

uint8_t CC2420TransmitP__m_tx_power;

CC2420TransmitP__cc2420_transmit_state_t CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;

bool CC2420TransmitP__m_receiving = FALSE;

uint16_t CC2420TransmitP__m_prev_time;


bool CC2420TransmitP__sfdHigh;


bool CC2420TransmitP__abortSpiRelease;


int8_t CC2420TransmitP__totalCcaChecks;


uint16_t CC2420TransmitP__myInitialBackoff;


uint16_t CC2420TransmitP__myCongestionBackoff;



static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);

static void CC2420TransmitP__loadTXFIFO(void );
static void CC2420TransmitP__attemptSend(void );
static void CC2420TransmitP__congestionBackoff(void );
static error_t CC2420TransmitP__acquireSpiResource(void );
static inline error_t CC2420TransmitP__releaseSpiResource(void );
static void CC2420TransmitP__signalDone(error_t err);



static inline error_t CC2420TransmitP__Init__init(void );







static inline error_t CC2420TransmitP__StdControl__start(void );










static inline error_t CC2420TransmitP__StdControl__stop(void );
#line 192
static inline error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca);
#line 243
static inline void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime);







static inline void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime);







static __inline uint32_t CC2420TransmitP__getTime32(uint16_t captured_time);
#line 280
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
#line 377
static inline void CC2420TransmitP__ChipSpiResource__releasing(void );
#line 389
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg);
#line 416
static inline void CC2420TransmitP__SpiResource__granted(void );
#line 454
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);
#line 486
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);










static inline void CC2420TransmitP__BackoffTimer__fired(void );
#line 547
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);
#line 737
static void CC2420TransmitP__attemptSend(void );
#line 782
static void CC2420TransmitP__congestionBackoff(void );






static error_t CC2420TransmitP__acquireSpiResource(void );







static inline error_t CC2420TransmitP__releaseSpiResource(void );
#line 819
static void CC2420TransmitP__loadTXFIFO(void );
#line 844
static void CC2420TransmitP__signalDone(error_t err);
# 43 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFO__get(void );
# 91 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void );
#line 115
static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void );
#line 110
static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void );
#line 64
static ieee_eui64_t CC2420ReceiveP__CC2420Config__getExtAddr(void );




static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void );
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t CC2420ReceiveP__receiveDone_task__postTask(void );
# 70 "/home/ali/workspace/TinyOS-main/tos/interfaces/PacketTimeStamp.nc"
static void CC2420ReceiveP__PacketTimeStamp__clear(
#line 66
message_t * msg);
#line 78
static void CC2420ReceiveP__PacketTimeStamp__set(
#line 73
message_t * msg, 




CC2420ReceiveP__PacketTimeStamp__size_type value);
# 43 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFOP__get(void );
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message);
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SACK__strobe(void );
# 40 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
static void CC2420ReceiveP__CSN__set(void );
static void CC2420ReceiveP__CSN__clr(void );
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg);
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ReceiveP__Receive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420ReceiveP__SpiResource__release(void );
#line 97
static error_t CC2420ReceiveP__SpiResource__immediateRequest(void );
#line 88
static error_t CC2420ReceiveP__SpiResource__request(void );
#line 128
static bool CC2420ReceiveP__SpiResource__isOwner(void );
# 62 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length);
# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ReceiveP__InterruptFIFOP__disable(void );
#line 54
static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void );
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void );
# 148 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
enum CC2420ReceiveP____nesc_unnamed4360 {
#line 148
  CC2420ReceiveP__receiveDone_task = 14U
};
#line 148
typedef int CC2420ReceiveP____nesc_sillytask_receiveDone_task[CC2420ReceiveP__receiveDone_task];
#line 89
#line 81
typedef enum CC2420ReceiveP____nesc_unnamed4361 {
  CC2420ReceiveP__S_STOPPED, 
  CC2420ReceiveP__S_STARTED, 
  CC2420ReceiveP__S_RX_LENGTH, 
  CC2420ReceiveP__S_RX_DEC, 
  CC2420ReceiveP__S_RX_DEC_WAIT, 
  CC2420ReceiveP__S_RX_FCF, 
  CC2420ReceiveP__S_RX_PAYLOAD
} CC2420ReceiveP__cc2420_receive_state_t;

enum CC2420ReceiveP____nesc_unnamed4362 {
  CC2420ReceiveP__RXFIFO_SIZE = 128, 
  CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE = 8, 
  CC2420ReceiveP__SACK_HEADER_LENGTH = 7
};

uint32_t CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE];

uint8_t CC2420ReceiveP__m_timestamp_head;

uint8_t CC2420ReceiveP__m_timestamp_size;





uint8_t CC2420ReceiveP__m_missed_packets;



bool CC2420ReceiveP__receivingPacket;


uint8_t CC2420ReceiveP__rxFrameLength;

uint8_t CC2420ReceiveP__m_bytes_left;

message_t * CC2420ReceiveP__m_p_rx_buf;

message_t CC2420ReceiveP__m_rx_buf;
#line 137
CC2420ReceiveP__cc2420_receive_state_t CC2420ReceiveP__m_state;



static void CC2420ReceiveP__reset_state(void );
static void CC2420ReceiveP__beginReceive(void );
static void CC2420ReceiveP__receive(void );
static void CC2420ReceiveP__waitForNextPacket(void );
static void CC2420ReceiveP__flush(void );
static inline bool CC2420ReceiveP__passesAddressCheck(message_t * msg);




static inline error_t CC2420ReceiveP__Init__init(void );





static inline error_t CC2420ReceiveP__StdControl__start(void );
#line 171
static inline error_t CC2420ReceiveP__StdControl__stop(void );
#line 186
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);








static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );
#line 212
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void );
#line 508
static inline void CC2420ReceiveP__SpiResource__granted(void );
#line 525
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error);
#line 663
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);







static inline void CC2420ReceiveP__receiveDone_task__runTask(void );
#line 704
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error);






static void CC2420ReceiveP__beginReceive(void );
#line 728
static void CC2420ReceiveP__flush(void );
#line 754
static void CC2420ReceiveP__receive(void );









static void CC2420ReceiveP__waitForNextPacket(void );
#line 808
static void CC2420ReceiveP__reset_state(void );










static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg);
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline int CC2420PacketP__getAddressLength(int type);








static uint8_t *CC2420PacketP__getNetwork(message_t *msg);
#line 119
static inline uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg);








static inline void CC2420PacketP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId);








static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);
#line 152
static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg);
#line 171
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg);





static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value);
#line 210
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg);








static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg);
# 58 "/home/ali/workspace/TinyOS-main/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__3__Counter__overflow(void );
# 52 "/home/ali/workspace/TinyOS-main/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
#line 69
static uint32_t RandomMlcgC__Random__rand32(void );
#line 89
static inline uint16_t RandomMlcgC__Random__rand16(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static error_t UniqueSendP__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t UniqueSendP__SubSend__maxPayloadLength(void );
#line 100
static void UniqueSendP__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 52 "/home/ali/workspace/TinyOS-main/tos/interfaces/Random.nc"
static uint16_t UniqueSendP__Random__rand16(void );
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg);
# 56 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
static void UniqueSendP__State__toIdle(void );
#line 45
static error_t UniqueSendP__State__requestState(uint8_t reqState);
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/unique/UniqueSendP.nc"
uint8_t UniqueSendP__localSendId;

enum UniqueSendP____nesc_unnamed4363 {
  UniqueSendP__S_IDLE, 
  UniqueSendP__S_SENDING
};


static inline error_t UniqueSendP__Init__init(void );
#line 75
static inline error_t UniqueSendP__Send__send(message_t *msg, uint8_t len);
#line 95
static inline uint8_t UniqueSendP__Send__maxPayloadLength(void );








static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error);
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__Receive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg);
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__DuplicateReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 59 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/unique/UniqueReceiveP.nc"
#line 56
struct UniqueReceiveP____nesc_unnamed4364 {
  uint16_t source;
  uint8_t dsn;
} UniqueReceiveP__receivedMessages[4];

uint8_t UniqueReceiveP__writeIndex = 0;


uint8_t UniqueReceiveP__recycleSourceElement;

enum UniqueReceiveP____nesc_unnamed4365 {
  UniqueReceiveP__INVALID_ELEMENT = 0xFF
};


static inline error_t UniqueReceiveP__Init__init(void );









static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);
static inline uint16_t UniqueReceiveP__getSourceKey(message_t *msg);


static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len);
#line 112
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
#line 138
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);
#line 165
static inline uint16_t UniqueReceiveP__getSourceKey(message_t *msg);
#line 192
static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t CC2420TinyosNetworkP__SubSend__maxPayloadLength(void );
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t CC2420TinyosNetworkP__grantTask__postTask(void );
# 77 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static void CC2420TinyosNetworkP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId);
#line 75
static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t *p_msg);
# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__ActiveSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 79 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceQueue.nc"
static error_t CC2420TinyosNetworkP__Queue__enqueue(resource_client_id_t id);
#line 53
static bool CC2420TinyosNetworkP__Queue__isEmpty(void );
#line 70
static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void );
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg);
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__BareReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__granted(
# 46 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x4122b2d0);
# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__BareSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__ActiveReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 180 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
enum CC2420TinyosNetworkP____nesc_unnamed4366 {
#line 180
  CC2420TinyosNetworkP__grantTask = 15U
};
#line 180
typedef int CC2420TinyosNetworkP____nesc_sillytask_grantTask[CC2420TinyosNetworkP__grantTask];
#line 68
enum CC2420TinyosNetworkP____nesc_unnamed4367 {
  CC2420TinyosNetworkP__OWNER_NONE = 0xff, 
  CC2420TinyosNetworkP__TINYOS_N_NETWORKS = 1U
};




#line 73
enum CC2420TinyosNetworkP____nesc_unnamed4368 {
  CC2420TinyosNetworkP__CLIENT_AM, 
  CC2420TinyosNetworkP__CLIENT_BARE
} CC2420TinyosNetworkP__m_busy_client;

uint8_t CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
#line 78
uint8_t CC2420TinyosNetworkP__next_owner;

static error_t CC2420TinyosNetworkP__ActiveSend__send(message_t *msg, uint8_t len);









static inline uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void );
#line 138
static inline void *CC2420TinyosNetworkP__BareSend__getPayload(message_t *msg, uint8_t len);









static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error);








static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 180
static inline void CC2420TinyosNetworkP__grantTask__runTask(void );
#line 199
static inline error_t CC2420TinyosNetworkP__Resource__request(uint8_t id);
#line 215
static inline error_t CC2420TinyosNetworkP__Resource__immediateRequest(uint8_t id);
#line 229
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id);
#line 241
static inline message_t *CC2420TinyosNetworkP__BareReceive__default__receive(message_t *msg, void *payload, uint8_t len);


static inline void CC2420TinyosNetworkP__BareSend__default__sendDone(message_t *msg, error_t error);








static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client);
# 49 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
enum /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0____nesc_unnamed4369 {
#line 49
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[1];
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );




static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );



static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
#line 82
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static error_t CC2420ActiveMessageP__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t CC2420ActiveMessageP__SubSend__maxPayloadLength(void );
# 75 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
static uint16_t CC2420ActiveMessageP__CC2420Config__getPanAddr(void );
# 95 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP__RadioBackoff__requestCca(
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x4125fdb8, 
# 95 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
#line 81
static void CC2420ActiveMessageP__RadioBackoff__requestInitialBackoff(
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x4125fdb8, 
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);






static void CC2420ActiveMessageP__RadioBackoff__requestCongestionBackoff(
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x4125fdb8, 
# 88 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/SendNotifier.nc"
static void CC2420ActiveMessageP__SendNotifier__aboutToSend(
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x4125f740, 
# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 110 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
static void CC2420ActiveMessageP__AMSend__sendDone(
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x41264b88, 
# 103 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__Snoop__receive(
# 50 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x41261cc0, 
# 71 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 50 "/home/ali/workspace/TinyOS-main/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ActiveMessageP__ActiveMessageAddress__amAddress(void );
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ActiveMessageP__CC2420PacketBody__getHeader(message_t * msg);
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__Receive__receive(
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x41261628, 
# 71 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
static error_t CC2420ActiveMessageP__RadioResource__release(void );
#line 97
static error_t CC2420ActiveMessageP__RadioResource__immediateRequest(void );
#line 88
static error_t CC2420ActiveMessageP__RadioResource__request(void );
# 71 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
uint16_t CC2420ActiveMessageP__pending_length;
message_t *CC2420ActiveMessageP__pending_message = (void *)0;

static void CC2420ActiveMessageP__RadioResource__granted(void );
#line 87
static error_t CC2420ActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len);
#line 135
static inline am_addr_t CC2420ActiveMessageP__AMPacket__address(void );



static am_addr_t CC2420ActiveMessageP__AMPacket__destination(message_t *amsg);









static inline void CC2420ActiveMessageP__AMPacket__setDestination(message_t *amsg, am_addr_t addr);









static inline bool CC2420ActiveMessageP__AMPacket__isForMe(message_t *amsg);




static inline am_id_t CC2420ActiveMessageP__AMPacket__type(message_t *amsg);




static inline void CC2420ActiveMessageP__AMPacket__setType(message_t *amsg, am_id_t type);
#line 194
static inline uint8_t CC2420ActiveMessageP__Packet__payloadLength(message_t *msg);



static inline void CC2420ActiveMessageP__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void );









static inline void CC2420ActiveMessageP__SubSend__sendDone(message_t *msg, error_t result);






static inline message_t *CC2420ActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 235
static inline void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error);





static inline void CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(message_t *msg);




static inline void CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(message_t *msg);



static inline void CC2420ActiveMessageP__SubBackoff__requestCca(message_t *msg);
#line 279
static inline message_t *CC2420ActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t *CC2420ActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);







static inline void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg);

static inline void CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(am_id_t id, 
message_t *msg);


static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(am_id_t id, 
message_t *msg);


static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCca(am_id_t id, 
message_t *msg);
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t DiagMsgP__send__postTask(void );
# 80 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
static error_t DiagMsgP__AMSend__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 252 "/home/ali/workspace/TinyOS-main/tos/lib/diagmsg/DiagMsgP.nc"
enum DiagMsgP____nesc_unnamed4370 {
#line 252
  DiagMsgP__send = 16U
};
#line 252
typedef int DiagMsgP____nesc_sillytask_send[DiagMsgP__send];
#line 66
enum DiagMsgP____nesc_unnamed4371 {

  DiagMsgP__STATE_READY = 1, 
  DiagMsgP__STATE_RECORDING_FIRST = 2, 
  DiagMsgP__STATE_RECORDING_SECOND = 3, 
  DiagMsgP__STATE_MSG_FULL = 4, 
  DiagMsgP__STATE_BUFFER_FULL = 5
};

volatile uint8_t DiagMsgP__state;

message_t DiagMsgP__msgs[10];

message_t *DiagMsgP__recording;
message_t *DiagMsgP__sending;

uint8_t DiagMsgP__nextData;
uint8_t DiagMsgP__prevType;
uint8_t DiagMsgP__retries;

static inline error_t DiagMsgP__Init__init(void );









enum DiagMsgP____nesc_unnamed4372 {

  DiagMsgP__TYPE_END = 0, 
  DiagMsgP__TYPE_INT8 = 1, 
  DiagMsgP__TYPE_UINT8 = 2, 
  DiagMsgP__TYPE_HEX8 = 3, 
  DiagMsgP__TYPE_INT16 = 4, 
  DiagMsgP__TYPE_UINT16 = 5, 
  DiagMsgP__TYPE_HEX16 = 6, 
  DiagMsgP__TYPE_INT32 = 7, 
  DiagMsgP__TYPE_UINT32 = 8, 
  DiagMsgP__TYPE_HEX32 = 9, 
  DiagMsgP__TYPE_FLOAT = 10, 
  DiagMsgP__TYPE_CHAR = 11, 
  DiagMsgP__TYPE_INT64 = 12, 
  DiagMsgP__TYPE_UINT64 = 13, 
  DiagMsgP__TYPE_ARRAY = 15
};
#line 136
static bool DiagMsgP__DiagMsg__record(void );
#line 157
static inline int8_t DiagMsgP__allocate(uint8_t size, uint8_t type);
#line 199
static void DiagMsgP__copyArray(uint8_t size, uint8_t type2, const void *data, uint8_t len);
#line 221
static inline void DiagMsgP__DiagMsg__int16s(const int16_t *value, uint8_t len);








static inline void DiagMsgP__DiagMsg__chrs(const char *value, uint8_t len);

static inline void DiagMsgP__DiagMsg__str(const char *str);









static __inline void DiagMsgP__setPayloadLength(message_t *msg, uint8_t length);




static __inline uint8_t DiagMsgP__getPayloadLength(message_t *msg);




static inline void DiagMsgP__send__runTask(void );










static __inline message_t *DiagMsgP__nextPointer(message_t *ptr);







static void DiagMsgP__DiagMsg__send(void );
#line 298
static void DiagMsgP__AMSend__sendDone(message_t *p, error_t error);
# 110 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
static void /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(
#line 103
message_t * msg, 






error_t error);
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static error_t /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(
#line 67
message_t * msg, 







uint8_t len);
# 103 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMPacket.nc"
static void /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(
#line 99
message_t * amsg, 



am_addr_t addr);
#line 162
static void /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(
#line 158
message_t * amsg, 



am_id_t t);
# 53 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueEntryP.nc"
static inline error_t /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err);
# 80 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(
# 48 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
am_id_t arg_0x41343490, 
# 80 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 46 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
uint8_t arg_0x41346a58, 
# 96 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Packet.nc"
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(
#line 74
message_t * msg);
#line 94
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void );
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMPacket.nc"
static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(
#line 74
message_t * amsg);
#line 147
static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(
#line 143
message_t * amsg);
# 126 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4373 {
#line 126
  AMQueueImplP__0__CancelTask = 17U
};
#line 126
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 169
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4374 {
#line 169
  AMQueueImplP__0__errorTask = 18U
};
#line 169
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 57
#line 55
typedef struct /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4375 {
  message_t * msg;
} /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t;

uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[1];
uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[1 / 8 + 1];

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void );
#line 90
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 126
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
#line 163
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );




static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );
#line 189
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 215
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 418 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 196 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void ){
#line 82
  /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__overflow();
#line 82
}
#line 82
# 64 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void )
{
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow();
}

# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 48
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 48
}
#line 48
# 137 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x4076bcf8){
#line 39
  switch (arg_0x4076bcf8) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x4076bcf8);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 126 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 39
}
#line 39
# 58 "/home/ali/workspace/TinyOS-main/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__Counter__overflow(void )
{
}

# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static void /*CounterMicro32C.Transform*/TransformCounterC__2__Counter__overflow(void ){
#line 82
  /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__Counter__overflow();
#line 82
}
#line 82
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4376 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 45
}
#line 45
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4377 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

# 276 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 276
{
  const uint8_t *base = source;

#line 278
  return ((uint16_t )base[1] << 8) | base[0];
}

#line 301
static __inline  uint32_t __nesc_hton_uint32(void * target, uint32_t value)
#line 301
{
  uint8_t *base = target;

#line 303
  base[3] = value;
  base[2] = value >> 8;
  base[1] = value >> 16;
  base[0] = value >> 24;
  return value;
}

#line 294
static __inline  uint32_t __nesc_ntoh_uint32(const void * source)
#line 294
{
  const uint8_t *base = source;

#line 296
  return ((((uint32_t )base[0] << 24) | (
  (uint32_t )base[1] << 16)) | (
  (uint32_t )base[2] << 8)) | base[3];
}

# 70 "/home/ali/workspace/TinyOS-main/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__clear(message_t * msg){
#line 70
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 70
}
#line 70
# 195 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void )
#line 195
{
  if (CC2420ReceiveP__m_timestamp_size) {
      CC2420ReceiveP__m_timestamp_size--;
    }
}

# 55 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd_dropped(void ){
#line 55
  CC2420ReceiveP__CC2420Receive__sfd_dropped();
#line 55
}
#line 55
# 68 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/chips/cc2420/CC2420GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__captureRisingEdge(void )
#line 68
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__enableCapture(MSP430TIMER_CM_RISING);
}

# 53 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__captureRisingEdge();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 59 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 2);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__getRaw() != 0;
}

# 73 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get();
}

# 43 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__SFD__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 186 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time)
#line 186
{
  if (CC2420ReceiveP__m_timestamp_size < CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE) {
      uint8_t tail = (CC2420ReceiveP__m_timestamp_head + CC2420ReceiveP__m_timestamp_size) % 
      CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;

#line 190
      CC2420ReceiveP__m_timestamp_queue[tail] = time;
      CC2420ReceiveP__m_timestamp_size++;
    }
}

# 49 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time){
#line 49
  CC2420ReceiveP__CC2420Receive__sfd(time);
#line 49
}
#line 49
# 72 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/chips/cc2420/CC2420GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__captureFallingEdge(void )
#line 72
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__enableCapture(MSP430TIMER_CM_FALLING);
}

# 54 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__captureFallingEdge();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__getNow(void )
{
  return /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__get();
}

#line 157
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type dt)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__getNow(), dt);
}

# 66 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt){
#line 66
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__start(dt);
#line 66
}
#line 66
# 137 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg)
#line 137
{
  return (cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
}

# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 135 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void )
{
  * (volatile uint16_t * )388U &= ~0x0010;
}

# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents();
#line 58
}
#line 58
# 65 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
}

# 73 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void ){
#line 73
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop();
#line 73
}
#line 73
# 102 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__stop();
}

# 73 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__stop(void ){
#line 73
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__stop();
#line 73
}
#line 73
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 797 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__releaseSpiResource(void )
#line 797
{
  CC2420TransmitP__SpiResource__release();
  return SUCCESS;
}

# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 48
}
#line 48
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set();
}

# 40 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__set(void ){
#line 40
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 40
}
#line 40
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_TXFIFO, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr();
#line 53
}
#line 53
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr();
}

# 41 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__clr(void ){
#line 41
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 41
}
#line 41
# 246 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 246
{
  const uint8_t *base = source;

#line 248
  return base[0];
}

# 219 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg)
{
  return __nesc_ntoh_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(msg)->length.data)
   + (sizeof(cc2420_header_t ) - MAC_HEADER_SIZE)
   - MAC_FOOTER_SIZE
   - sizeof(timesync_radio_t );
}

# 58 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(message_t * msg){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = CC2420PacketP__PacketTimeSyncOffset__get(msg);
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 235
{
  const uint8_t *base = source;

#line 237
  return base[0];
}

#line 257
static __inline  int8_t __nesc_ntoh_int8(const void * source)
#line 257
{
#line 257
  return __nesc_ntoh_uint8(source);
}

# 152 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg)
#line 152
{
  return (cc2420_metadata_t *)msg->metadata;
}

#line 210
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg)
{
  return __nesc_ntoh_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.data);
}

# 50 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(message_t * msg){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = CC2420PacketP__PacketTimeSyncOffset__isSet(msg);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 177 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value)
{
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.data, value);
}

# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__set(message_t * msg, CC2420TransmitP__PacketTimeStamp__size_type value){
#line 78
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 78
}
#line 78
# 109 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 259 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static __inline uint32_t CC2420TransmitP__getTime32(uint16_t captured_time)
{
  uint32_t now = CC2420TransmitP__BackoffTimer__getNow();


  return now - (uint16_t )(now - captured_time);
}

#line 280
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time)
#line 280
{
  unsigned char *__nesc_temp45;
  unsigned char *__nesc_temp44;
#line 281
  uint32_t time32;
  uint8_t sfd_state = 0;

  /* atomic removed: atomic calls only */
#line 283
  {
    time32 = CC2420TransmitP__getTime32(time);
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SFD: 
          CC2420TransmitP__m_state = CC2420TransmitP__S_EFD;
        CC2420TransmitP__sfdHigh = TRUE;


        CC2420TransmitP__m_receiving = FALSE;
        CC2420TransmitP__CaptureSFD__captureFallingEdge();
        CC2420TransmitP__PacketTimeStamp__set(CC2420TransmitP__m_msg, time32);
        if (CC2420TransmitP__PacketTimeSyncOffset__isSet(CC2420TransmitP__m_msg)) {
            uint8_t absOffset = sizeof(message_header_t ) - sizeof(cc2420_header_t ) + CC2420TransmitP__PacketTimeSyncOffset__get(CC2420TransmitP__m_msg);
            timesync_radio_t *timesync = (timesync_radio_t *)((nx_uint8_t *)CC2420TransmitP__m_msg + absOffset);

            (__nesc_temp44 = (*timesync).data, __nesc_hton_uint32(__nesc_temp44, __nesc_ntoh_uint32(__nesc_temp44) - time32));
            CC2420TransmitP__CSN__clr();
            CC2420TransmitP__TXFIFO_RAM__write(absOffset, (uint8_t *)timesync, sizeof(timesync_radio_t ));
            CC2420TransmitP__CSN__set();

            (__nesc_temp45 = (*timesync).data, __nesc_hton_uint32(__nesc_temp45, __nesc_ntoh_uint32(__nesc_temp45) + time32));
          }

        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ)) {

            CC2420TransmitP__abortSpiRelease = TRUE;
          }
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__BackoffTimer__stop();

        if (CC2420TransmitP__SFD__get()) {
            break;
          }


        case CC2420TransmitP__S_EFD: 
          CC2420TransmitP__sfdHigh = FALSE;
        CC2420TransmitP__CaptureSFD__captureRisingEdge();

        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ)) {
            CC2420TransmitP__m_state = CC2420TransmitP__S_ACK_WAIT;
            CC2420TransmitP__BackoffTimer__start(CC2420_ACK_WAIT_DELAY);
          }
        else 
#line 326
          {
            CC2420TransmitP__signalDone(SUCCESS);
          }

        if (!CC2420TransmitP__SFD__get()) {
            break;
          }


        default: 

          if (!CC2420TransmitP__m_receiving && CC2420TransmitP__sfdHigh == FALSE) {
              CC2420TransmitP__sfdHigh = TRUE;
              CC2420TransmitP__CaptureSFD__captureFallingEdge();

              sfd_state = CC2420TransmitP__SFD__get();
              CC2420TransmitP__CC2420Receive__sfd(time32);
              CC2420TransmitP__m_receiving = TRUE;
              CC2420TransmitP__m_prev_time = time;
              if (CC2420TransmitP__SFD__get()) {

                  return;
                }
            }



        if (CC2420TransmitP__sfdHigh == TRUE) {
            CC2420TransmitP__sfdHigh = FALSE;
            CC2420TransmitP__CaptureSFD__captureRisingEdge();
            CC2420TransmitP__m_receiving = FALSE;








            if (sfd_state == 0 && time - CC2420TransmitP__m_prev_time < 10) {
                CC2420TransmitP__CC2420Receive__sfd_dropped();
                if (CC2420TransmitP__m_msg) {
                  CC2420TransmitP__PacketTimeStamp__clear(CC2420TransmitP__m_msg);
                  }
              }
#line 370
            break;
          }
      }
  }
}

# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioCapture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__captured(uint16_t time){
#line 61
  CC2420TransmitP__CaptureSFD__captured(time);
#line 61
}
#line 61
# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 53 "/home/ali/workspace/TinyOS-main/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void )
{
  return /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get();
}

# 61 "/home/ali/workspace/TinyOS-main/tos/lib/timer/LocalTime.nc"
inline static uint32_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__LocalTime32khz__get(void ){
#line 61
  unsigned long __nesc_result;
#line 61

#line 61
  __nesc_result = /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 62 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void )
{




  if (0) {
      /* atomic removed: atomic calls only */
#line 69
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )368U;

#line 72
        do {
#line 72
            t0 = t1;
#line 72
            t1 = * (volatile uint16_t * )368U;
          }
        while (
#line 72
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 73
          t1;

#line 73
          return __nesc_temp;
        }
      }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )368U;
    }
}

# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void )
{
  return /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get();
}

# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__size_type /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )352U & 1U;
}

# 46 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__isOverflowPending(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__isOverflowPending(void )
{
  return /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__isOverflowPending();
}

# 71 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static bool /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 80 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformCounterC.nc"
static inline /*CounterMicro32C.Transform*/TransformCounterC__2__to_size_type /*CounterMicro32C.Transform*/TransformCounterC__2__Counter__get(void )
{
  /*CounterMicro32C.Transform*/TransformCounterC__2__to_size_type rv = 0;

  /* atomic removed: atomic calls only */
#line 84
  {
    /*CounterMicro32C.Transform*/TransformCounterC__2__upper_count_type high = /*CounterMicro32C.Transform*/TransformCounterC__2__m_upper;
    /*CounterMicro32C.Transform*/TransformCounterC__2__from_size_type low = /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__get();

#line 87
    if (/*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__isOverflowPending()) 
      {






        high++;
        low = /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__get();
      }
    {
      /*CounterMicro32C.Transform*/TransformCounterC__2__to_size_type high_to = high;
      /*CounterMicro32C.Transform*/TransformCounterC__2__to_size_type low_to = low >> /*CounterMicro32C.Transform*/TransformCounterC__2__LOW_SHIFT_RIGHT;

#line 101
      rv = (high_to << /*CounterMicro32C.Transform*/TransformCounterC__2__HIGH_SHIFT_LEFT) | low_to;
    }
  }
  return rv;
}

# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__Counter__size_type /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*CounterMicro32C.Transform*/TransformCounterC__2__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 53 "/home/ali/workspace/TinyOS-main/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__LocalTime__get(void )
{
  return /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__Counter__get();
}

# 61 "/home/ali/workspace/TinyOS-main/tos/lib/timer/LocalTime.nc"
inline static uint32_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__LocalTimeMicro__get(void ){
#line 61
  unsigned long __nesc_result;
#line 61

#line 61
  __nesc_result = /*HplCC2420InterruptsC.CounterMicroToLocalTime*/CounterToLocalTimeC__2__LocalTime__get();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 83 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/chips/cc2420/CC2420GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430Capture__captured(uint16_t capturedMicro)
#line 83
{
  uint16_t elapsedMicro;
  uint16_t captured32khz;

  /* atomic removed: atomic calls only */
#line 87
  {
    elapsedMicro = /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__LocalTimeMicro__get() - capturedMicro;
    captured32khz = /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__LocalTime32khz__get();
  }

  captured32khz -= elapsedMicro >> 5;

  /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__captured(captured32khz);
}

# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 86
  /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430Capture__captured(time);
#line 86
}
#line 86
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 46 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 71 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static bool /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 65 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )38U |= 0x01 << 2;
}

# 92 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__GeneralIO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__selectModuleFunc();
#line 92
}
#line 92
# 95 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )356U &= ~0x0001;
}

# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 175 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__clearOverflow(void )
{
  * (volatile uint16_t * )356U &= ~0x0002;
}

# 68 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430Capture__clearOverflow(void ){
#line 68
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__clearOverflow();
#line 68
}
#line 68
# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4378 {
#line 57
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

#line 72
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__captureControl(uint8_t l_cm)
{
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(x);
}

#line 110
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControlAsCapture(uint8_t cm)
{
  * (volatile uint16_t * )356U = /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__captureControl(cm);
}

# 55 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm){
#line 55
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControlAsCapture(cm);
#line 55
}
#line 55
# 130 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__enableEvents(void )
{
  * (volatile uint16_t * )356U |= 0x0010;
}

# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__enableEvents();
#line 57
}
#line 57
# 384 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__tx(uint8_t data)
#line 384
{
  HplMsp430Usart1P__U1TXBUF = data;
}

# 224 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart1P__Usart__tx(data);
#line 224
}
#line 224
# 332 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline bool HplMsp430Usart1P__Usart__isRxIntrPending(void )
#line 332
{
  if (HplMsp430Usart1P__IFG2 & (1 << 4)) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void ){
#line 192
  unsigned char __nesc_result;
#line 192

#line 192
  __nesc_result = HplMsp430Usart1P__Usart__isRxIntrPending();
#line 192

#line 192
  return __nesc_result;
#line 192
}
#line 192
# 343 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__clrRxIntr(void )
#line 343
{
  HplMsp430Usart1P__IFG2 &= ~(1 << 4);
}

# 197 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void ){
#line 197
  HplMsp430Usart1P__Usart__clrRxIntr();
#line 197
}
#line 197
# 388 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline uint8_t HplMsp430Usart1P__Usart__rx(void )
#line 388
{
  return U1RXBUF;
}

# 231 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static uint8_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void ){
#line 231
  unsigned char __nesc_result;
#line 231

#line 231
  __nesc_result = HplMsp430Usart1P__Usart__rx();
#line 231

#line 231
  return __nesc_result;
#line 231
}
#line 231
# 118 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
static inline void StateImplP__State__toIdle(uint8_t id)
#line 118
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    StateImplP__state[id] = StateImplP__S_IDLE;
#line 119
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
inline static void CC2420SpiP__WorkingState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(0U);
#line 56
}
#line 56
# 95 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__ChipSpiResource__abortRelease(void )
#line 95
{
  /* atomic removed: atomic calls only */
#line 96
  CC2420SpiP__release = FALSE;
}

# 31 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420TransmitP__ChipSpiResource__abortRelease(void ){
#line 31
  CC2420SpiP__ChipSpiResource__abortRelease();
#line 31
}
#line 31
# 377 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__ChipSpiResource__releasing(void )
#line 377
{
  if (CC2420TransmitP__abortSpiRelease) {
      CC2420TransmitP__ChipSpiResource__abortRelease();
    }
}

# 24 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420SpiP__ChipSpiResource__releasing(void ){
#line 24
  CC2420TransmitP__ChipSpiResource__releasing();
#line 24
}
#line 24
# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 7);
}

# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 6);
}

# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc();
#line 99
}
#line 99
# 211 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableUart(void )
#line 211
{
  /* atomic removed: atomic calls only */
#line 212
  {
    HplMsp430Usart1P__ME2 &= ~((1 << 5) | (1 << 4));
    HplMsp430Usart1P__UTXD__selectIOFunc();
    HplMsp430Usart1P__URXD__selectIOFunc();
  }
}

#line 109
static inline error_t HplMsp430Usart1P__AsyncStdControl__stop(void )
#line 109
{
  HplMsp430Usart1P__Usart__disableSpi();
  HplMsp430Usart1P__Usart__disableUart();
  return SUCCESS;
}

# 105 "/home/ali/workspace/TinyOS-main/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = HplMsp430Usart1P__AsyncStdControl__stop();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 84 "/home/ali/workspace/TinyOS-main/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void )
#line 84
{
}

# 62 "/home/ali/workspace/TinyOS-main/tos/lib/power/PowerDownCleanup.nc"
inline static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void ){
#line 62
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup();
#line 62
}
#line 62
# 79 "/home/ali/workspace/TinyOS-main/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void )
#line 79
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop();
}

# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted();
#line 46
}
#line 46
# 159 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset)
#line 159
{
  if (reset) {
    U1CTL = 0x01;
    }
  else {
#line 163
    U1CTL &= ~0x01;
    }
}

# 97 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart1P__Usart__resetUsart(reset);
#line 97
}
#line 97
#line 158
inline static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void ){
#line 158
  HplMsp430Usart1P__Usart__disableSpi();
#line 158
}
#line 158
# 124 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 124
{
  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(TRUE);
  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi();
  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(FALSE);
}

# 218 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id)
#line 218
{
}

# 65 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(uint8_t arg_0x40a42148){
#line 65
  switch (arg_0x40a42148) {
#line 65
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 65
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID);
#line 65
      break;
#line 65
    default:
#line 65
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(arg_0x40a42148);
#line 65
      break;
#line 65
    }
#line 65
}
#line 65
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 68 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void )
#line 68
{
  /* atomic removed: atomic calls only */
#line 69
  {
    if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead != /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead;

#line 72
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead];
        if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
          /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
          }
#line 75
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[id] = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 76
          id;

#line 76
          return __nesc_temp;
        }
      }
#line 78
    {
      unsigned char __nesc_temp = 
#line 78
      /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

#line 78
      return __nesc_temp;
    }
  }
}

# 70 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 60 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 61
  {
    unsigned char __nesc_temp = 
#line 61
    /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

#line 61
    return __nesc_temp;
  }
}

# 53 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 111 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id)
#line 111
{
  /* atomic removed: atomic calls only */
#line 112
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
        if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty() == FALSE) {
            /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue();
            /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;
            /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING;
            /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
            /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
          }
        else {
            /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
            /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
            /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
            /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted();
          }
        {
          unsigned char __nesc_temp = 
#line 127
          SUCCESS;

#line 127
          return __nesc_temp;
        }
      }
  }
#line 130
  return FAIL;
}

# 176 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id)
#line 176
{
#line 176
  return FAIL;
}

# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(uint8_t arg_0x40f69460){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x40f69460) {
#line 120
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID:
#line 120
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(arg_0x40f69460);
#line 120
      break;
#line 120
    }
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 116 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id)
#line 116
{
  return /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(id);
}

# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 97 "/home/ali/workspace/TinyOS-main/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 1);
}

# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SIMO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 2);
}

# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SOMI__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 3);
}

# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UCLK__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc();
#line 99
}
#line 99
# 130 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void )
{
  * (volatile uint16_t * )388U |= 0x0010;
}

# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents();
#line 57
}
#line 57
# 95 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )388U &= ~0x0001;
}

# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )404U = x;
}

# 41 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )404U = /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__get() + x;
}

# 43 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
  }
}

# 103 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt){
#line 103
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 102 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void )
#line 102
{
  return CC2420SpiP__attemptRelease();
}

# 39 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void ){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = CC2420SpiP__ChipSpiResource__attemptRelease();
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 150 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 45
}
#line 45
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4379 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 45
}
#line 45
# 131 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 134
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 39
}
#line 39
# 126 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 39
}
#line 39
# 196 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 114 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 58 "/home/ali/workspace/TinyOS-main/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 177 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 82
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 82
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 82
}
#line 82
# 133 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__0__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 177 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__overflow(void )
{
}

# 58 "/home/ali/workspace/TinyOS-main/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void )
{
}

#line 58
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__3__Counter__overflow(void )
{
}

# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static void /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow(void ){
#line 82
  /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__3__Counter__overflow();
#line 82
  /*HplCC2420InterruptsC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow();
#line 82
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__overflow();
#line 82
}
#line 82
# 133 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformCounterC.nc"
static inline void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper++;
    if ((/*Counter32khz32C.Transform*/TransformCounterC__1__m_upper & /*Counter32khz32C.Transform*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
      /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow();
      }
  }
}

# 82 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 82
  /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow();
#line 82
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 82
}
#line 82
# 64 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 48
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 48
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 48
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 48
}
#line 48
# 137 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 81 "/home/ali/workspace/TinyOS-main/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 82
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 78
}
#line 78
# 162 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 45
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4380 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64







inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 130 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 57
}
#line 57
# 95 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 44 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 41 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 43 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 103 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 180 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__request(void )
#line 180
{
  return CC2420ControlP__SpiResource__request();
}

# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420ControlP__Resource__request();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 210 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startVRegDone(void )
#line 210
{
  CC2420CsmaP__Resource__request();
}

# 56 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startVRegDone(void ){
#line 56
  CC2420CsmaP__CC2420Power__startVRegDone();
#line 56
}
#line 56
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__set();
#line 48
}
#line 48
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set();
}

# 40 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__set(void ){
#line 40
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set();
#line 40
}
#line 40
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__clr();
#line 53
}
#line 53
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr();
}

# 41 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__clr(void ){
#line 41
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr();
#line 41
}
#line 41
# 423 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__StartupTimer__fired(void )
#line 423
{
  if (CC2420ControlP__m_state == CC2420ControlP__S_VREG_STARTING) {
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTED;
      CC2420ControlP__RSTN__clr();
      CC2420ControlP__RSTN__set();
      CC2420ControlP__CC2420Power__startVRegDone();
    }
}

# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHTX);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 59 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )40U & (0x01 << 7);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__getRaw() != 0;
}

# 73 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get();
}

# 43 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__CCA__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 498 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__BackoffTimer__fired(void )
#line 498
{
  /* atomic removed: atomic calls only */
#line 499
  {
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SAMPLE_CCA: 


          if (CC2420TransmitP__CCA__get()) {
              CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
              CC2420TransmitP__BackoffTimer__start(CC2420_TIME_ACK_TURNAROUND);
            }
          else {
              CC2420TransmitP__congestionBackoff();
            }
        break;

        case CC2420TransmitP__S_BEGIN_TRANSMIT: 
          case CC2420TransmitP__S_CANCEL: 
            if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
                CC2420TransmitP__attemptSend();
              }
        break;

        case CC2420TransmitP__S_ACK_WAIT: 
          CC2420TransmitP__signalDone(SUCCESS);
        break;

        case CC2420TransmitP__S_SFD: 


          CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CaptureSFD__captureRisingEdge();
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__signalDone(ERETRY);
        break;

        default: 
          break;
      }
  }
}

# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__fired(void ){
#line 78
  CC2420TransmitP__BackoffTimer__fired();
#line 78
  CC2420ControlP__StartupTimer__fired();
#line 78
}
#line 78
# 162 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt == 0) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__fired();
      }
    else 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__set_alarm();
      }
  }
}

# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 78
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__fired();
#line 78
}
#line 78
# 70 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 45
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4381 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}

# 297 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(am_id_t id, 
message_t *msg)
#line 298
{
}

# 88 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP__RadioBackoff__requestCongestionBackoff(am_id_t arg_0x4125fdb8, message_t * msg){
#line 88
    CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(arg_0x4125fdb8, msg);
#line 88
}
#line 88
# 246 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(message_t *msg)
#line 246
{
  CC2420ActiveMessageP__RadioBackoff__requestCongestionBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

# 88 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestCongestionBackoff(message_t * msg){
#line 88
  CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(msg);
#line 88
}
#line 88
# 89 "/home/ali/workspace/TinyOS-main/tos/system/RandomMlcgC.nc"
static inline uint16_t RandomMlcgC__Random__rand16(void )
#line 89
{
  return (uint16_t )RandomMlcgC__Random__rand32();
}

# 52 "/home/ali/workspace/TinyOS-main/tos/interfaces/Random.nc"
inline static uint16_t CC2420CsmaP__Random__rand16(void ){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = RandomMlcgC__Random__rand16();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 251 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime)
#line 251
{
  CC2420TransmitP__myCongestionBackoff = backoffTime + 1;
}

# 66 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__SubBackoff__setCongestionBackoff(uint16_t backoffTime){
#line 66
  CC2420TransmitP__RadioBackoff__setCongestionBackoff(backoffTime);
#line 66
}
#line 66
# 230 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t *msg)
#line 230
{
  CC2420CsmaP__SubBackoff__setCongestionBackoff(CC2420CsmaP__Random__rand16()
   % (0x7 * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP__RadioBackoff__requestCongestionBackoff(msg);
}

# 88 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP__RadioBackoff__requestCongestionBackoff(message_t * msg){
#line 88
  CC2420CsmaP__SubBackoff__requestCongestionBackoff(msg);
#line 88
}
#line 88
# 97 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 45 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
inline static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(0U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 173 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline bool /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id)
#line 173
{
#line 173
  return FALSE;
}

# 128 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static bool /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(uint8_t arg_0x40f69460){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  switch (arg_0x40f69460) {
#line 128
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID:
#line 128
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 128
      break;
#line 128
    default:
#line 128
      __nesc_result = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(arg_0x40f69460);
#line 128
      break;
#line 128
    }
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 112 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline bool /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id)
#line 112
{
  return /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(id);
}

# 128 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static bool CC2420SpiP__SpiResource__isOwner(void ){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  __nesc_result = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID);
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 177 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id)
#line 177
{
  return &msp430_spi_default_config;
}

# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x40f69eb0){
#line 39
  union __nesc_unnamed4274 *__nesc_result;
#line 39

#line 39
    __nesc_result = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(arg_0x40f69eb0);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 168 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config){
#line 168
  HplMsp430Usart1P__Usart__setModeSpi(config);
#line 168
}
#line 168
# 120 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id)
#line 120
{
  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(id));
}

# 216 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(uint8_t arg_0x40a42148){
#line 59
  switch (arg_0x40a42148) {
#line 59
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 59
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(arg_0x40a42148);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 56 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceDefaultOwner.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 105 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void )
#line 105
{
  return SUCCESS;
}

# 95 "/home/ali/workspace/TinyOS-main/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = HplMsp430Usart1P__AsyncStdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 74 "/home/ali/workspace/TinyOS-main/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void )
#line 74
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release();
}

# 81 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested();
#line 81
}
#line 81
# 206 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id)
#line 206
{
}

# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(uint8_t arg_0x40a22d38){
#line 61
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(arg_0x40a22d38);
#line 61
}
#line 61
# 93 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id)
#line 93
{
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /* atomic removed: atomic calls only */
#line 95
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING;
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 100
        FAIL;

#line 100
        return __nesc_temp;
      }
  }
#line 102
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 107
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
  return FAIL;
}

# 175 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 175
{
#line 175
  return FAIL;
}

# 97 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(uint8_t arg_0x40f69460){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  switch (arg_0x40f69460) {
#line 97
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID:
#line 97
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 97
      break;
#line 97
    default:
#line 97
      __nesc_result = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(arg_0x40f69460);
#line 97
      break;
#line 97
    }
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 104 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id)
#line 104
{
  return /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(id);
}

# 97 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 151 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control)
#line 151
{
  U1MCTL = control;
}

#line 140
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control)
#line 140
{
  /* atomic removed: atomic calls only */
#line 141
  {
    U1BR0 = control & 0x00FF;
    U1BR1 = (control >> 8) & 0x00FF;
  }
}

#line 260
static inline void HplMsp430Usart1P__configSpi(msp430_spi_union_config_t *config)
#line 260
{
  U1CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  HplMsp430Usart1P__U1TCTL = config->spiRegisters.utctl;

  HplMsp430Usart1P__Usart__setUbr(config->spiRegisters.ubr);
  HplMsp430Usart1P__Usart__setUmctl(0x00);
}

# 65 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )51U |= 0x01 << 3;
}

# 92 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UCLK__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectModuleFunc();
#line 92
}
#line 92
# 65 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )51U |= 0x01 << 2;
}

# 92 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SOMI__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectModuleFunc();
#line 92
}
#line 92
# 65 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )51U |= 0x01 << 1;
}

# 92 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SIMO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectModuleFunc();
#line 92
}
#line 92
# 242 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableSpi(void )
#line 242
{
  /* atomic removed: atomic calls only */
#line 243
  {
    HplMsp430Usart1P__SIMO__selectModuleFunc();
    HplMsp430Usart1P__SOMI__selectModuleFunc();
    HplMsp430Usart1P__UCLK__selectModuleFunc();
  }
  HplMsp430Usart1P__ME2 |= 1 << 4;
}

#line 347
static inline void HplMsp430Usart1P__Usart__clrIntr(void )
#line 347
{
  HplMsp430Usart1P__IFG2 &= ~((1 << 5) | (1 << 4));
}









static inline void HplMsp430Usart1P__Usart__disableIntr(void )
#line 359
{
  HplMsp430Usart1P__IE2 &= ~((1 << 5) | (1 << 4));
}

# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 69 "/home/ali/workspace/TinyOS-main/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__requested(void )
#line 69
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release();
}

# 73 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__requested();
#line 73
}
#line 73
# 64 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[id] != /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY || /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id)
#line 82
{
  /* atomic removed: atomic calls only */
#line 83
  {
    if (!/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
          /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead = id;
          }
        else {
#line 88
          /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail] = id;
          }
#line 89
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 90
          SUCCESS;

#line 90
          return __nesc_temp;
        }
      }
#line 92
    {
      unsigned char __nesc_temp = 
#line 92
      EBUSY;

#line 92
      return __nesc_temp;
    }
  }
}

# 79 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 204 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id)
#line 204
{
}

# 53 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(uint8_t arg_0x40a22d38){
#line 53
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(arg_0x40a22d38);
#line 53
}
#line 53
# 77 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /* atomic removed: atomic calls only */
#line 79
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING;
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = id;
      }
    else {
#line 84
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId == id) {
          {
            unsigned char __nesc_temp = 
#line 85
            SUCCESS;

#line 85
            return __nesc_temp;
          }
        }
      else 
#line 87
        {
          unsigned char __nesc_temp = 
#line 87
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(id);

#line 87
          return __nesc_temp;
        }
      }
  }
#line 89
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 174 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id)
#line 174
{
#line 174
  return FAIL;
}

# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(uint8_t arg_0x40f69460){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  switch (arg_0x40f69460) {
#line 88
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID:
#line 88
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 88
      break;
#line 88
    default:
#line 88
      __nesc_result = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(arg_0x40f69460);
#line 88
      break;
#line 88
    }
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 108 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id)
#line 108
{
  return /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(id);
}

# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXONCCA);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
inline static cc2420_status_t CC2420TransmitP__STXON__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
inline static cc2420_status_t CC2420TransmitP__SNOP__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SNOP);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 192 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void )
{
}

# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4382 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4383 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4384 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4385 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 45 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4386 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 131 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 134
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 39
}
#line 39
# 124 "/home/ali/workspace/TinyOS-main/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 57 "/home/ali/workspace/TinyOS-main/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 57
  SchedulerBasicP__Scheduler__init();
#line 57
}
#line 57
# 56 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P43*/HplMsp430GeneralIOP__27__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )29U |= 0x01 << 3;
}

# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P43*/HplMsp430GeneralIOP__27__IO__set();
#line 48
}
#line 48
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 40 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 40
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 40
}
#line 40
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set();
#line 48
}
#line 48
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 40 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 40
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 40
}
#line 40
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__set();
#line 48
}
#line 48
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 40 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 40
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 40
}
#line 40
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P43*/HplMsp430GeneralIOP__27__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 3;
}

# 85 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P43*/HplMsp430GeneralIOP__27__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 2;
}

# 85 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 0;
}

# 85 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 46
}
#line 46
# 57 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 58
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = LedsP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 149 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/hardware.h"
static inline  void TOSH_MAKE_DOCK_N_INPUT()
#line 149
{
#line 149
  static volatile uint8_t r __asm ("0x002A");

#line 149
  r &= ~(1 << 3);
}

#line 134
static inline  void TOSH_SET_UCLK1_PIN()
#line 134
{
#line 134
  static volatile uint8_t r __asm ("0x0031");

#line 134
  r |= 1 << 3;
}

#line 132
static inline  void TOSH_SET_SIMO1_PIN()
#line 132
{
#line 132
  static volatile uint8_t r __asm ("0x0031");

#line 132
  r |= 1 << 1;
}

#line 129
static inline  void TOSH_SET_SD_CLK_PIN()
#line 129
{
#line 129
  static volatile uint8_t r __asm ("0x0019");

#line 129
  r |= 1 << 3;
}

#line 127
static inline  void TOSH_SET_SD_DI_PIN()
#line 127
{
#line 127
  static volatile uint8_t r __asm ("0x0019");

#line 127
  r |= 1 << 1;
}

#line 88
static inline  void TOSH_SET_SD_CS_N_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x0019");

#line 88
  r |= 1 << 0;
}

#line 133
static inline  void TOSH_MAKE_SOMI1_INPUT()
#line 133
{
#line 133
  static volatile uint8_t r __asm ("0x0032");

#line 133
  r &= ~(1 << 2);
}

#line 133
static inline  void TOSH_SEL_SOMI1_IOFUNC()
#line 133
{
#line 133
  static volatile uint8_t r __asm ("0x0033");

#line 133
  r &= ~(1 << 2);
}

#line 132
static inline  void TOSH_MAKE_SIMO1_OUTPUT()
#line 132
{
#line 132
  static volatile uint8_t r __asm ("0x0032");

#line 132
  r |= 1 << 1;
}

#line 132
static inline  void TOSH_SEL_SIMO1_IOFUNC()
#line 132
{
#line 132
  static volatile uint8_t r __asm ("0x0033");

#line 132
  r &= ~(1 << 1);
}

#line 134
static inline  void TOSH_MAKE_UCLK1_OUTPUT()
#line 134
{
#line 134
  static volatile uint8_t r __asm ("0x0032");

#line 134
  r |= 1 << 3;
}

#line 134
static inline  void TOSH_SEL_UCLK1_IOFUNC()
#line 134
{
#line 134
  static volatile uint8_t r __asm ("0x0033");

#line 134
  r &= ~(1 << 3);
}

#line 136
static inline  void TOSH_MAKE_URXD1_OUTPUT()
#line 136
{
#line 136
  static volatile uint8_t r __asm ("0x001A");

#line 136
  r |= 1 << 7;
}

#line 136
static inline  void TOSH_SEL_URXD1_IOFUNC()
#line 136
{
#line 136
  static volatile uint8_t r __asm ("0x001B");

#line 136
  r &= ~(1 << 7);
}

#line 135
static inline  void TOSH_MAKE_UTXD1_OUTPUT()
#line 135
{
#line 135
  static volatile uint8_t r __asm ("0x001A");

#line 135
  r |= 1 << 6;
}

#line 135
static inline  void TOSH_SEL_UTXD1_IOFUNC()
#line 135
{
#line 135
  static volatile uint8_t r __asm ("0x001B");

#line 135
  r &= ~(1 << 6);
}

#line 124
static inline  void TOSH_SET_URXD0_PIN()
#line 124
{
#line 124
  static volatile uint8_t r __asm ("0x0019");

#line 124
  r |= 1 << 5;
}

#line 124
static inline  void TOSH_MAKE_URXD0_OUTPUT()
#line 124
{
#line 124
  static volatile uint8_t r __asm ("0x001A");

#line 124
  r |= 1 << 5;
}

#line 124
static inline  void TOSH_SEL_URXD0_IOFUNC()
#line 124
{
#line 124
  static volatile uint8_t r __asm ("0x001B");

#line 124
  r &= ~(1 << 5);
}

#line 123
static inline  void TOSH_MAKE_UTXD0_OUTPUT()
#line 123
{
#line 123
  static volatile uint8_t r __asm ("0x001A");

#line 123
  r |= 1 << 4;
}

#line 123
static inline  void TOSH_SEL_UTXD0_IOFUNC()
#line 123
{
#line 123
  static volatile uint8_t r __asm ("0x001B");

#line 123
  r &= ~(1 << 4);
}

#line 139
static inline  void TOSH_MAKE_GIO1_OUTPUT()
#line 139
{
#line 139
  static volatile uint8_t r __asm ("0x002A");

#line 139
  r |= 1 << 1;
}

#line 139
static inline  void TOSH_SEL_GIO1_IOFUNC()
#line 139
{
#line 139
  static volatile uint8_t r __asm ("0x002E");

#line 139
  r &= ~(1 << 1);
}

#line 138
static inline  void TOSH_CLR_GIO0_PIN()
#line 138
{
#line 138
  static volatile uint8_t r __asm ("0x0029");

#line 138
  r &= ~(1 << 0);
}

#line 138
static inline  void TOSH_MAKE_GIO0_OUTPUT()
#line 138
{
#line 138
  static volatile uint8_t r __asm ("0x002A");

#line 138
  r |= 1 << 0;
}

#line 138
static inline  void TOSH_SEL_GIO0_IOFUNC()
#line 138
{
#line 138
  static volatile uint8_t r __asm ("0x002E");

#line 138
  r &= ~(1 << 0);
}

#line 117
static inline  void TOSH_CLR_SER0_RTS_PIN()
#line 117
{
#line 117
  static volatile uint8_t r __asm ("0x0021");

#line 117
  r &= ~(1 << 3);
}

#line 117
static inline  void TOSH_MAKE_SER0_RTS_OUTPUT()
#line 117
{
#line 117
  static volatile uint8_t r __asm ("0x0022");

#line 117
  r |= 1 << 3;
}

#line 117
static inline  void TOSH_SEL_SER0_RTS_IOFUNC()
#line 117
{
#line 117
  static volatile uint8_t r __asm ("0x0026");

#line 117
  r &= ~(1 << 3);
}

#line 118
static inline  void TOSH_MAKE_SER0_CTS_OUTPUT()
#line 118
{
#line 118
  static volatile uint8_t r __asm ("0x0022");

#line 118
  r |= 1 << 4;
}

#line 118
static inline  void TOSH_SEL_SER0_CTS_IOFUNC()
#line 118
{
#line 118
  static volatile uint8_t r __asm ("0x0026");

#line 118
  r &= ~(1 << 4);
}

#line 152
static inline  void TOSH_CLR_PWRMUX_SEL_PIN()
#line 152
{
#line 152
  static volatile uint8_t r __asm ("0x001D");

#line 152
  r &= ~(1 << 1);
}

#line 152
static inline  void TOSH_MAKE_PWRMUX_SEL_OUTPUT()
#line 152
{
#line 152
  static volatile uint8_t r __asm ("0x001E");

#line 152
  r |= 1 << 1;
}

#line 152
static inline  void TOSH_SEL_PWRMUX_SEL_IOFUNC()
#line 152
{
#line 152
  static volatile uint8_t r __asm ("0x001F");

#line 152
  r &= ~(1 << 1);
}

#line 154
static inline  void TOSH_CLR_ACCEL_SLEEP_N_PIN()
#line 154
{
#line 154
  static volatile uint8_t r __asm ("0x0031");

#line 154
  r &= ~(1 << 0);
}

#line 154
static inline  void TOSH_MAKE_ACCEL_SLEEP_N_OUTPUT()
#line 154
{
#line 154
  static volatile uint8_t r __asm ("0x0032");

#line 154
  r |= 1 << 0;
}

#line 154
static inline  void TOSH_SEL_ACCEL_SLEEP_N_IOFUNC()
#line 154
{
#line 154
  static volatile uint8_t r __asm ("0x0033");

#line 154
  r &= ~(1 << 0);
}

#line 153
static inline  void TOSH_CLR_ACCEL_SEL0_PIN()
#line 153
{
#line 153
  static volatile uint8_t r __asm ("0x001D");

#line 153
  r &= ~(1 << 4);
}

#line 153
static inline  void TOSH_MAKE_ACCEL_SEL0_OUTPUT()
#line 153
{
#line 153
  static volatile uint8_t r __asm ("0x001E");

#line 153
  r |= 1 << 4;
}

#line 153
static inline  void TOSH_SEL_ACCEL_SEL0_IOFUNC()
#line 153
{
#line 153
  static volatile uint8_t r __asm ("0x001F");

#line 153
  r &= ~(1 << 4);
}

#line 143
static inline  void TOSH_SET_ONEWIRE_PIN()
#line 143
{
#line 143
  static volatile uint8_t r __asm ("0x001D");

#line 143
  r |= 1 << 7;
}

#line 143
static inline  void TOSH_MAKE_ONEWIRE_OUTPUT()
#line 143
{
#line 143
  static volatile uint8_t r __asm ("0x001E");

#line 143
  r |= 1 << 7;
}

#line 143
static inline  void TOSH_SEL_ONEWIRE_IOFUNC()
#line 143
{
#line 143
  static volatile uint8_t r __asm ("0x001F");

#line 143
  r &= ~(1 << 7);
}

static inline  void TOSH_MAKE_ROSC_INPUT()
#line 146
{
#line 146
  static volatile uint8_t r __asm ("0x002A");

#line 146
  r &= ~(1 << 5);
}

#line 146
static inline  void TOSH_SEL_ROSC_IOFUNC()
#line 146
{
#line 146
  static volatile uint8_t r __asm ("0x002E");

#line 146
  r &= ~(1 << 5);
}

#line 104
static inline  void TOSH_MAKE_ADC_ACCELX_INPUT()
#line 104
{
#line 104
  static volatile uint8_t r __asm ("0x0036");

#line 104
  r &= ~(1 << 5);
}

#line 104
static inline  void TOSH_SEL_ADC_ACCELX_IOFUNC()
#line 104
{
#line 104
  static volatile uint8_t r __asm ("0x0037");

#line 104
  r &= ~(1 << 5);
}

#line 103
static inline  void TOSH_MAKE_ADC_ACCELY_INPUT()
#line 103
{
#line 103
  static volatile uint8_t r __asm ("0x0036");

#line 103
  r &= ~(1 << 4);
}

#line 103
static inline  void TOSH_SEL_ADC_ACCELY_IOFUNC()
#line 103
{
#line 103
  static volatile uint8_t r __asm ("0x0037");

#line 103
  r &= ~(1 << 4);
}

#line 102
static inline  void TOSH_MAKE_ADC_ACCELZ_INPUT()
#line 102
{
#line 102
  static volatile uint8_t r __asm ("0x0036");

#line 102
  r &= ~(1 << 3);
}

#line 102
static inline  void TOSH_SEL_ADC_ACCELZ_IOFUNC()
#line 102
{
#line 102
  static volatile uint8_t r __asm ("0x0037");

#line 102
  r &= ~(1 << 3);
}

#line 100
static inline  void TOSH_MAKE_ADC_7_OUTPUT()
#line 100
{
#line 100
  static volatile uint8_t r __asm ("0x0036");

#line 100
  r |= 1 << 7;
}

#line 100
static inline  void TOSH_SEL_ADC_7_IOFUNC()
#line 100
{
#line 100
  static volatile uint8_t r __asm ("0x0037");

#line 100
  r &= ~(1 << 7);
}

#line 99
static inline  void TOSH_MAKE_ADC_6_OUTPUT()
#line 99
{
#line 99
  static volatile uint8_t r __asm ("0x0036");

#line 99
  r |= 1 << 6;
}

#line 99
static inline  void TOSH_SEL_ADC_6_IOFUNC()
#line 99
{
#line 99
  static volatile uint8_t r __asm ("0x0037");

#line 99
  r &= ~(1 << 6);
}

#line 95
static inline  void TOSH_MAKE_ADC_2_OUTPUT()
#line 95
{
#line 95
  static volatile uint8_t r __asm ("0x0036");

#line 95
  r |= 1 << 2;
}

#line 95
static inline  void TOSH_SEL_ADC_2_IOFUNC()
#line 95
{
#line 95
  static volatile uint8_t r __asm ("0x0037");

#line 95
  r &= ~(1 << 2);
}

#line 94
static inline  void TOSH_MAKE_ADC_1_OUTPUT()
#line 94
{
#line 94
  static volatile uint8_t r __asm ("0x0036");

#line 94
  r |= 1 << 1;
}

#line 94
static inline  void TOSH_SEL_ADC_1_IOFUNC()
#line 94
{
#line 94
  static volatile uint8_t r __asm ("0x0037");

#line 94
  r &= ~(1 << 1);
}

#line 93
static inline  void TOSH_MAKE_ADC_0_OUTPUT()
#line 93
{
#line 93
  static volatile uint8_t r __asm ("0x0036");

#line 93
  r |= 1 << 0;
}

#line 93
static inline  void TOSH_SEL_ADC_0_IOFUNC()
#line 93
{
#line 93
  static volatile uint8_t r __asm ("0x0037");

#line 93
  r &= ~(1 << 0);
}

#line 84
static inline  void TOSH_SEL_PROG_OUT_IOFUNC()
#line 84
{
#line 84
  static volatile uint8_t r __asm ("0x0026");

#line 84
  r &= ~(1 << 1);
}

#line 85
static inline  void TOSH_SEL_PROG_IN_IOFUNC()
#line 85
{
#line 85
  static volatile uint8_t r __asm ("0x002E");

#line 85
  r &= ~(1 << 2);
}

#line 84
static inline  void TOSH_SET_PROG_OUT_PIN()
#line 84
{
#line 84
  static volatile uint8_t r __asm ("0x0021");

#line 84
  r |= 1 << 1;
}

#line 84
static inline  void TOSH_MAKE_PROG_OUT_OUTPUT()
#line 84
{
#line 84
  static volatile uint8_t r __asm ("0x0022");

#line 84
  r |= 1 << 1;
}

#line 85
static inline  void TOSH_MAKE_PROG_IN_OUTPUT()
#line 85
{
#line 85
  static volatile uint8_t r __asm ("0x002A");

#line 85
  r |= 1 << 2;
}

#line 80
static inline  void TOSH_SEL_BT_RXD_IOFUNC()
#line 80
{
#line 80
  static volatile uint8_t r __asm ("0x001B");

#line 80
  r &= ~(1 << 7);
}

#line 80
static inline  void TOSH_MAKE_BT_RXD_INPUT()
#line 80
{
#line 80
  static volatile uint8_t r __asm ("0x001A");

#line 80
  r &= ~(1 << 7);
}

#line 79
static inline  void TOSH_SEL_BT_TXD_IOFUNC()
#line 79
{
#line 79
  static volatile uint8_t r __asm ("0x001B");

#line 79
  r &= ~(1 << 6);
}

#line 79
static inline  void TOSH_MAKE_BT_TXD_OUTPUT()
#line 79
{
#line 79
  static volatile uint8_t r __asm ("0x001A");

#line 79
  r |= 1 << 6;
}

#line 78
static inline  void TOSH_SEL_BT_CTS_IOFUNC()
#line 78
{
#line 78
  static volatile uint8_t r __asm ("0x0026");

#line 78
  r &= ~(1 << 7);
}

#line 78
static inline  void TOSH_MAKE_BT_CTS_OUTPUT()
#line 78
{
#line 78
  static volatile uint8_t r __asm ("0x0022");

#line 78
  r |= 1 << 7;
}

#line 76
static inline  void TOSH_SEL_BT_PIO_IOFUNC()
#line 76
{
#line 76
  static volatile uint8_t r __asm ("0x002E");

#line 76
  r &= ~(1 << 6);
}

#line 76
static inline  void TOSH_MAKE_BT_PIO_INPUT()
#line 76
{
#line 76
  static volatile uint8_t r __asm ("0x002A");

#line 76
  r &= ~(1 << 6);
}

#line 77
static inline  void TOSH_SEL_BT_RTS_IOFUNC()
#line 77
{
#line 77
  static volatile uint8_t r __asm ("0x0026");

#line 77
  r &= ~(1 << 6);
}

#line 77
static inline  void TOSH_MAKE_BT_RTS_INPUT()
#line 77
{
#line 77
  static volatile uint8_t r __asm ("0x0022");

#line 77
  r &= ~(1 << 6);
}


static inline  void TOSH_CLR_BT_RESET_PIN()
#line 81
{
#line 81
  static volatile uint8_t r __asm ("0x0031");

#line 81
  r &= ~(1 << 5);
}

#line 81
static inline  void TOSH_SEL_BT_RESET_IOFUNC()
#line 81
{
#line 81
  static volatile uint8_t r __asm ("0x0033");

#line 81
  r &= ~(1 << 5);
}

#line 81
static inline  void TOSH_MAKE_BT_RESET_OUTPUT()
#line 81
{
#line 81
  static volatile uint8_t r __asm ("0x0032");

#line 81
  r |= 1 << 5;
}







static inline  void TOSH_MAKE_TILT_INPUT()
#line 90
{
#line 90
  static volatile uint8_t r __asm ("0x002A");

#line 90
  r &= ~(1 << 4);
}

#line 90
static inline  void TOSH_SEL_TILT_IOFUNC()
#line 90
{
#line 90
  static volatile uint8_t r __asm ("0x002E");

#line 90
  r &= ~(1 << 4);
}

#line 62
static inline  void TOSH_MAKE_RADIO_SFD_INPUT()
#line 62
{
#line 62
  static volatile uint8_t r __asm ("0x0022");

#line 62
  r &= ~(1 << 2);
}

#line 62
static inline  void TOSH_SEL_RADIO_SFD_IOFUNC()
#line 62
{
#line 62
  static volatile uint8_t r __asm ("0x0026");

#line 62
  r &= ~(1 << 2);
}

#line 56
static inline  void TOSH_MAKE_RADIO_FIFOP_INPUT()
#line 56
{
#line 56
  static volatile uint8_t r __asm ("0x0022");

#line 56
  r &= ~(1 << 0);
}

#line 56
static inline  void TOSH_SEL_RADIO_FIFOP_IOFUNC()
#line 56
{
#line 56
  static volatile uint8_t r __asm ("0x0026");

#line 56
  r &= ~(1 << 0);
}

#line 55
static inline  void TOSH_MAKE_RADIO_FIFO_INPUT()
#line 55
{
#line 55
  static volatile uint8_t r __asm ("0x0022");

#line 55
  r &= ~(1 << 5);
}

#line 55
static inline  void TOSH_SEL_RADIO_FIFO_IOFUNC()
#line 55
{
#line 55
  static volatile uint8_t r __asm ("0x0026");

#line 55
  r &= ~(1 << 5);
}

#line 57
static inline  void TOSH_MAKE_RADIO_CCA_INPUT()
#line 57
{
#line 57
  static volatile uint8_t r __asm ("0x002A");

#line 57
  r &= ~(1 << 7);
}

#line 57
static inline  void TOSH_SEL_RADIO_CCA_IOFUNC()
#line 57
{
#line 57
  static volatile uint8_t r __asm ("0x002E");

#line 57
  r &= ~(1 << 7);
}







static inline  void TOSH_CLR_RADIO_RESET_PIN()
#line 66
{
#line 66
  static volatile uint8_t r __asm ("0x0031");

#line 66
  r &= ~(1 << 7);
}

#line 66
static inline  void TOSH_SEL_RADIO_RESET_IOFUNC()
#line 66
{
#line 66
  static volatile uint8_t r __asm ("0x0033");

#line 66
  r &= ~(1 << 7);
}

#line 66
static inline  void TOSH_MAKE_RADIO_RESET_OUTPUT()
#line 66
{
#line 66
  static volatile uint8_t r __asm ("0x0032");

#line 66
  r |= 1 << 7;
}

#line 65
static inline  void TOSH_SET_RADIO_CSN_PIN()
#line 65
{
#line 65
  static volatile uint8_t r __asm ("0x0031");

#line 65
  r |= 1 << 4;
}

#line 65
static inline  void TOSH_SEL_RADIO_CSN_IOFUNC()
#line 65
{
#line 65
  static volatile uint8_t r __asm ("0x0033");

#line 65
  r &= ~(1 << 4);
}

#line 65
static inline  void TOSH_MAKE_RADIO_CSN_OUTPUT()
#line 65
{
#line 65
  static volatile uint8_t r __asm ("0x0032");

#line 65
  r |= 1 << 4;
}

#line 60
static inline  void TOSH_CLR_RADIO_VREF_PIN()
#line 60
{
#line 60
  static volatile uint8_t r __asm ("0x0031");

#line 60
  r &= ~(1 << 6);
}

#line 60
static inline  void TOSH_SEL_RADIO_VREF_IOFUNC()
#line 60
{
#line 60
  static volatile uint8_t r __asm ("0x0033");

#line 60
  r &= ~(1 << 6);
}

#line 60
static inline  void TOSH_MAKE_RADIO_VREF_OUTPUT()
#line 60
{
#line 60
  static volatile uint8_t r __asm ("0x0032");

#line 60
  r |= 1 << 6;
}

#line 52
static inline  void TOSH_SET_GREEN_LED_PIN()
#line 52
{
#line 52
  static volatile uint8_t r __asm ("0x001D");

#line 52
  r |= 1 << 3;
}

#line 51
static inline  void TOSH_SET_YELLOW_LED_PIN()
#line 51
{
#line 51
  static volatile uint8_t r __asm ("0x001D");

#line 51
  r |= 1 << 2;
}

#line 50
static inline  void TOSH_SET_RED_LED_PIN()
#line 50
{
#line 50
  static volatile uint8_t r __asm ("0x001D");

#line 50
  r |= 1 << 0;
}

#line 52
static inline  void TOSH_MAKE_GREEN_LED_OUTPUT()
#line 52
{
#line 52
  static volatile uint8_t r __asm ("0x001E");

#line 52
  r |= 1 << 3;
}

#line 52
static inline  void TOSH_SEL_GREEN_LED_IOFUNC()
#line 52
{
#line 52
  static volatile uint8_t r __asm ("0x001F");

#line 52
  r &= ~(1 << 3);
}

#line 51
static inline  void TOSH_MAKE_YELLOW_LED_OUTPUT()
#line 51
{
#line 51
  static volatile uint8_t r __asm ("0x001E");

#line 51
  r |= 1 << 2;
}

#line 51
static inline  void TOSH_SEL_YELLOW_LED_IOFUNC()
#line 51
{
#line 51
  static volatile uint8_t r __asm ("0x001F");

#line 51
  r &= ~(1 << 2);
}

#line 50
static inline  void TOSH_MAKE_RED_LED_OUTPUT()
#line 50
{
#line 50
  static volatile uint8_t r __asm ("0x001E");

#line 50
  r |= 1 << 0;
}

#line 50
static inline  void TOSH_SEL_RED_LED_IOFUNC()
#line 50
{
#line 50
  static volatile uint8_t r __asm ("0x001F");

#line 50
  r &= ~(1 << 0);
}

#line 114
static inline  void TOSH_SET_SW_BT_PWR_N_PIN()
#line 114
{
#line 114
  static volatile uint8_t r __asm ("0x001D");

#line 114
  r |= 1 << 6;
}

#line 114
static inline  void TOSH_MAKE_SW_BT_PWR_N_OUTPUT()
#line 114
{
#line 114
  static volatile uint8_t r __asm ("0x001E");

#line 114
  r |= 1 << 6;
}

#line 114
static inline  void TOSH_SEL_SW_BT_PWR_N_IOFUNC()
#line 114
{
#line 114
  static volatile uint8_t r __asm ("0x001F");

#line 114
  r &= ~(1 << 6);
}

#line 129
static inline  void TOSH_CLR_SD_CLK_PIN()
#line 129
{
#line 129
  static volatile uint8_t r __asm ("0x0019");

#line 129
  r &= ~(1 << 3);
}

#line 127
static inline  void TOSH_CLR_SD_DI_PIN()
#line 127
{
#line 127
  static volatile uint8_t r __asm ("0x0019");

#line 127
  r &= ~(1 << 1);
}

#line 88
static inline  void TOSH_CLR_SD_CS_N_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x0019");

#line 88
  r &= ~(1 << 0);
}

#line 113
static inline  void TOSH_SET_SW_SD_PWR_N_PIN()
#line 113
{
#line 113
  static volatile uint8_t r __asm ("0x001D");

#line 113
  r |= 1 << 5;
}

#line 127
static inline  void TOSH_MAKE_SD_DI_OUTPUT()
#line 127
{
#line 127
  static volatile uint8_t r __asm ("0x001A");

#line 127
  r |= 1 << 1;
}

#line 127
static inline  void TOSH_SEL_SD_DI_IOFUNC()
#line 127
{
#line 127
  static volatile uint8_t r __asm ("0x001B");

#line 127
  r &= ~(1 << 1);
}

#line 128
static inline  void TOSH_MAKE_SD_DO_INPUT()
#line 128
{
#line 128
  static volatile uint8_t r __asm ("0x001A");

#line 128
  r &= ~(1 << 2);
}

#line 128
static inline  void TOSH_SEL_SD_DO_IOFUNC()
#line 128
{
#line 128
  static volatile uint8_t r __asm ("0x001B");

#line 128
  r &= ~(1 << 2);
}

#line 129
static inline  void TOSH_MAKE_SD_CLK_OUTPUT()
#line 129
{
#line 129
  static volatile uint8_t r __asm ("0x001A");

#line 129
  r |= 1 << 3;
}

#line 129
static inline  void TOSH_SEL_SD_CLK_IOFUNC()
#line 129
{
#line 129
  static volatile uint8_t r __asm ("0x001B");

#line 129
  r &= ~(1 << 3);
}

#line 88
static inline  void TOSH_MAKE_SD_CS_N_OUTPUT()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001A");

#line 88
  r |= 1 << 0;
}

#line 88
static inline  void TOSH_SEL_SD_CS_N_IOFUNC()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001B");

#line 88
  r &= ~(1 << 0);
}

#line 113
static inline  void TOSH_MAKE_SW_SD_PWR_N_OUTPUT()
#line 113
{
#line 113
  static volatile uint8_t r __asm ("0x001E");

#line 113
  r |= 1 << 5;
}

#line 113
static inline  void TOSH_SEL_SW_SD_PWR_N_IOFUNC()
#line 113
{
#line 113
  static volatile uint8_t r __asm ("0x001F");

#line 113
  r &= ~(1 << 5);
}

#line 149
static inline  void TOSH_SET_DOCK_N_PIN()
#line 149
{
#line 149
  static volatile uint8_t r __asm ("0x0029");

#line 149
  r |= 1 << 3;
}

#line 149
static inline  void TOSH_MAKE_DOCK_N_OUTPUT()
#line 149
{
#line 149
  static volatile uint8_t r __asm ("0x002A");

#line 149
  r |= 1 << 3;
}

#line 149
static inline  void TOSH_SEL_DOCK_N_IOFUNC()
#line 149
{
#line 149
  static volatile uint8_t r __asm ("0x002E");

#line 149
  r &= ~(1 << 3);
}

# 44 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/MotePlatformC.nc"
static inline error_t MotePlatformC__Init__init(void )
#line 44
{


  TOSH_SEL_DOCK_N_IOFUNC();
  TOSH_MAKE_DOCK_N_OUTPUT();
  TOSH_SET_DOCK_N_PIN();


  TOSH_SEL_SW_SD_PWR_N_IOFUNC();
  TOSH_MAKE_SW_SD_PWR_N_OUTPUT();

  TOSH_SEL_SD_CS_N_IOFUNC();
  TOSH_MAKE_SD_CS_N_OUTPUT();

  TOSH_SEL_SD_CLK_IOFUNC();
  TOSH_MAKE_SD_CLK_OUTPUT();
  TOSH_SEL_SD_DO_IOFUNC();
  TOSH_MAKE_SD_DO_INPUT();
  TOSH_SEL_SD_DI_IOFUNC();
  TOSH_MAKE_SD_DI_OUTPUT();






  TOSH_SET_SW_SD_PWR_N_PIN();
  TOSH_CLR_SD_CS_N_PIN();
  TOSH_CLR_SD_DI_PIN();
  TOSH_CLR_SD_CLK_PIN();

  TOSH_SEL_SW_BT_PWR_N_IOFUNC();
  TOSH_MAKE_SW_BT_PWR_N_OUTPUT();
  TOSH_SET_SW_BT_PWR_N_PIN();


  TOSH_SEL_RED_LED_IOFUNC();
  TOSH_MAKE_RED_LED_OUTPUT();
  TOSH_SEL_YELLOW_LED_IOFUNC();
  TOSH_MAKE_YELLOW_LED_OUTPUT();
  TOSH_SEL_GREEN_LED_IOFUNC();
  TOSH_MAKE_GREEN_LED_OUTPUT();

  TOSH_SET_RED_LED_PIN();
  TOSH_SET_YELLOW_LED_PIN();
  TOSH_SET_GREEN_LED_PIN();



  TOSH_MAKE_RADIO_VREF_OUTPUT();
  TOSH_SEL_RADIO_VREF_IOFUNC();
  TOSH_CLR_RADIO_VREF_PIN();

  TOSH_MAKE_RADIO_CSN_OUTPUT();
  TOSH_SEL_RADIO_CSN_IOFUNC();
  TOSH_SET_RADIO_CSN_PIN();


  TOSH_MAKE_RADIO_RESET_OUTPUT();
  TOSH_SEL_RADIO_RESET_IOFUNC();
  TOSH_CLR_RADIO_RESET_PIN();

  TOSH_SEL_RADIO_CCA_IOFUNC();
  TOSH_MAKE_RADIO_CCA_INPUT();
  TOSH_SEL_RADIO_FIFO_IOFUNC();
  TOSH_MAKE_RADIO_FIFO_INPUT();
  TOSH_SEL_RADIO_FIFOP_IOFUNC();
  TOSH_MAKE_RADIO_FIFOP_INPUT();
  TOSH_SEL_RADIO_SFD_IOFUNC();
  TOSH_MAKE_RADIO_SFD_INPUT();

  TOSH_SEL_TILT_IOFUNC();
  TOSH_MAKE_TILT_INPUT();


  TOSH_MAKE_BT_RESET_OUTPUT();
  TOSH_SEL_BT_RESET_IOFUNC();
  TOSH_CLR_BT_RESET_PIN();

  TOSH_MAKE_BT_RTS_INPUT();
  TOSH_SEL_BT_RTS_IOFUNC();

  TOSH_MAKE_BT_PIO_INPUT();
  TOSH_SEL_BT_PIO_IOFUNC();

  TOSH_MAKE_BT_CTS_OUTPUT();
  TOSH_SEL_BT_CTS_IOFUNC();

  TOSH_MAKE_BT_TXD_OUTPUT();
  TOSH_SEL_BT_TXD_IOFUNC();

  TOSH_MAKE_BT_RXD_INPUT();
  TOSH_SEL_BT_RXD_IOFUNC();


  TOSH_MAKE_PROG_IN_OUTPUT();
  TOSH_MAKE_PROG_OUT_OUTPUT();
  TOSH_SET_PROG_OUT_PIN();
  TOSH_SEL_PROG_IN_IOFUNC();
  TOSH_SEL_PROG_OUT_IOFUNC();


  TOSH_SEL_ADC_0_IOFUNC();
  TOSH_MAKE_ADC_0_OUTPUT();
  TOSH_SEL_ADC_1_IOFUNC();
  TOSH_MAKE_ADC_1_OUTPUT();
  TOSH_SEL_ADC_2_IOFUNC();
  TOSH_MAKE_ADC_2_OUTPUT();
  TOSH_SEL_ADC_6_IOFUNC();
  TOSH_MAKE_ADC_6_OUTPUT();
  TOSH_SEL_ADC_7_IOFUNC();
  TOSH_MAKE_ADC_7_OUTPUT();

  TOSH_SEL_ADC_ACCELZ_IOFUNC();
  TOSH_MAKE_ADC_ACCELZ_INPUT();
  TOSH_SEL_ADC_ACCELY_IOFUNC();
  TOSH_MAKE_ADC_ACCELY_INPUT();
  TOSH_SEL_ADC_ACCELX_IOFUNC();
  TOSH_MAKE_ADC_ACCELX_INPUT();

  TOSH_SEL_ROSC_IOFUNC();
  TOSH_MAKE_ROSC_INPUT();


  TOSH_SEL_ONEWIRE_IOFUNC();
  TOSH_MAKE_ONEWIRE_OUTPUT();
  TOSH_SET_ONEWIRE_PIN();







  TOSH_SEL_ACCEL_SEL0_IOFUNC();
  TOSH_MAKE_ACCEL_SEL0_OUTPUT();
  TOSH_CLR_ACCEL_SEL0_PIN();
  TOSH_SEL_ACCEL_SLEEP_N_IOFUNC();
  TOSH_MAKE_ACCEL_SLEEP_N_OUTPUT();
  TOSH_CLR_ACCEL_SLEEP_N_PIN();





  TOSH_SEL_PWRMUX_SEL_IOFUNC();
  TOSH_MAKE_PWRMUX_SEL_OUTPUT();
  TOSH_CLR_PWRMUX_SEL_PIN();


  TOSH_SEL_SER0_CTS_IOFUNC();
  TOSH_MAKE_SER0_CTS_OUTPUT();
  TOSH_SEL_SER0_RTS_IOFUNC();
  TOSH_MAKE_SER0_RTS_OUTPUT();
  TOSH_CLR_SER0_RTS_PIN();


  TOSH_SEL_GIO0_IOFUNC();
  TOSH_MAKE_GIO0_OUTPUT();
  TOSH_CLR_GIO0_PIN();

  TOSH_SEL_GIO1_IOFUNC();
  TOSH_MAKE_GIO1_OUTPUT();

  TOSH_SEL_UTXD0_IOFUNC();
  TOSH_MAKE_UTXD0_OUTPUT();
  TOSH_SEL_URXD0_IOFUNC();
  TOSH_MAKE_URXD0_OUTPUT();
  TOSH_SET_URXD0_PIN();

  TOSH_SEL_UTXD1_IOFUNC();
  TOSH_MAKE_UTXD1_OUTPUT();
  TOSH_SEL_URXD1_IOFUNC();
  TOSH_MAKE_URXD1_OUTPUT();
  TOSH_SEL_UCLK1_IOFUNC();
  TOSH_MAKE_UCLK1_OUTPUT();
  TOSH_SEL_SIMO1_IOFUNC();
  TOSH_MAKE_SIMO1_OUTPUT();
  TOSH_SEL_SOMI1_IOFUNC();
  TOSH_MAKE_SOMI1_INPUT();

  TOSH_SET_SD_CS_N_PIN();
  TOSH_SET_SD_DI_PIN();
  TOSH_SET_SD_CLK_PIN();


  TOSH_SET_SIMO1_PIN();
  TOSH_SET_UCLK1_PIN();


  TOSH_MAKE_DOCK_N_INPUT();

  return SUCCESS;
}

# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = MotePlatformC__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 163 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 151
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TA0CTL = 0x0020 | (Msp430ClockP__TA0CTL & ~(0x0020 | 0x0010));
}

#line 115
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 145
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 43 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 43
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 43
}
#line 43
# 100 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TA0R = 0;









  Msp430ClockP__TA0CTL = 0x0200 | 0x0002;
}

#line 140
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 42
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 42
}
#line 42
# 79 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP__IE1 &= ~(1 << 1);
}

#line 135
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 41 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 41
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 41
}
#line 41
# 181 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP__set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP__ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP__busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP__test_calib_busywait_delta(calib | step) <= Msp430ClockP__TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP__set_dco_calib(calib);
}

#line 67
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TA0CTL = 0x0200 | 0x0020;
  Msp430ClockP__TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 130
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 40 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 40
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 40
}
#line 40
# 229 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TA0CTL = 0x0004;
  Msp430ClockP__TA0IV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP__Msp430ClockInit__setupDcoCalibrate();
    Msp430ClockP__busyCalibrateDco();
    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteClockInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 10 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 65 "/home/ali/workspace/TinyOS-main/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 110 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
inline static void /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(message_t * msg, error_t error){
#line 110
  DiagMsgP__AMSend__sendDone(msg, error);
#line 110
}
#line 110
# 65 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueEntryP.nc"
static inline void /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err)
#line 65
{
  /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(m, err);
}

# 215 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 215
{
}

# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0x41346a58, message_t * msg, error_t error){
#line 100
  switch (arg_0x41346a58) {
#line 100
    case 0U:
#line 100
      /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 100
      break;
#line 100
    default:
#line 100
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0x41346a58, msg, error);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 126 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void )
#line 126
{
  uint8_t i;
#line 127
  uint8_t j;
#line 127
  uint8_t mask;
#line 127
  uint8_t last;
  message_t *msg;

#line 129
  for (i = 0; i < 1 / 8 + 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] &= ~mask;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

# 263 "/home/ali/workspace/TinyOS-main/tos/lib/diagmsg/DiagMsgP.nc"
static __inline message_t *DiagMsgP__nextPointer(message_t *ptr)
{
  if (++ptr >= DiagMsgP__msgs + 10) {
    return DiagMsgP__msgs;
    }
  else {
#line 268
    return ptr;
    }
}

#line 230
static inline void DiagMsgP__DiagMsg__chrs(const char *value, uint8_t len)
#line 230
{
#line 230
  DiagMsgP__copyArray(sizeof(char ), DiagMsgP__TYPE_CHAR, value, len);
}

#line 232
static inline void DiagMsgP__DiagMsg__str(const char *str)
{
  int8_t len = 0;

#line 235
  while (str[len] != 0 && len < 15) 
    ++len;

  DiagMsgP__DiagMsg__chrs(str, len);
}

#line 157
static inline int8_t DiagMsgP__allocate(uint8_t size, uint8_t type)
{
  int8_t ret = -1;

  if (DiagMsgP__state == DiagMsgP__STATE_RECORDING_FIRST) 
    {
      if (DiagMsgP__nextData + 1 + size <= 96) 
        {
          DiagMsgP__state = DiagMsgP__STATE_RECORDING_SECOND;

          DiagMsgP__prevType = DiagMsgP__nextData++;
          ((uint8_t *)& DiagMsgP__recording->data)[DiagMsgP__prevType] = type;
          ret = DiagMsgP__nextData;
          DiagMsgP__nextData += size;
        }
      else {
        DiagMsgP__state = DiagMsgP__STATE_MSG_FULL;
        }
    }
  else {
#line 175
    if (DiagMsgP__state == DiagMsgP__STATE_RECORDING_SECOND) 
      {
        if (DiagMsgP__nextData + size <= 96) 
          {
            DiagMsgP__state = DiagMsgP__STATE_RECORDING_FIRST;
            (
            (uint8_t *)& DiagMsgP__recording->data)[DiagMsgP__prevType] += type << 4;
            ret = DiagMsgP__nextData;
            DiagMsgP__nextData += size;
          }
        else {
          DiagMsgP__state = DiagMsgP__STATE_MSG_FULL;
          }
      }
    }
#line 189
  return ret;
}

#line 242
static __inline void DiagMsgP__setPayloadLength(message_t *msg, uint8_t length)
{
  * (uint8_t *)& msg->header = length;
}

# 169 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void )
#line 169
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg, FAIL);
}

# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 80 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0x41343490, am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = CC2420ActiveMessageP__AMSend__send(arg_0x41343490, addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ActiveMessageP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 194 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint8_t CC2420ActiveMessageP__Packet__payloadLength(message_t *msg)
#line 194
{
  return __nesc_ntoh_leuint8(CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg)->length.data) - CC2420_SIZE;
}

# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Packet.nc"
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(message_t * msg){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ActiveMessageP__Packet__payloadLength(msg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(message_t * amsg){
#line 78
  unsigned int __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ActiveMessageP__AMPacket__destination(amsg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 164 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline am_id_t CC2420ActiveMessageP__AMPacket__type(message_t *amsg)
#line 164
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 166
  return __nesc_ntoh_leuint8(header->type.data);
}

# 147 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMPacket.nc"
inline static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(message_t * amsg){
#line 147
  unsigned char __nesc_result;
#line 147

#line 147
  __nesc_result = CC2420ActiveMessageP__AMPacket__type(amsg);
#line 147

#line 147
  return __nesc_result;
#line 147
}
#line 147
# 65 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void )
#line 65
{
  uint8_t i;

#line 67
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == (void *)0 || 
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current / 8] & (1 << /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current % 8)) 
        {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 78
    /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
    }
}

#line 174
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void )
#line 174
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket();
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current < 1) {
      error_t nextErr;
      message_t *nextMsg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg;
      am_id_t nextId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(nextMsg);
      am_addr_t nextDest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(nextMsg);
      uint8_t len = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(nextMsg);

#line 182
      nextErr = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask();
        }
    }
}

# 173 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline uint8_t CC2420CsmaP__Send__maxPayloadLength(void )
#line 173
{
  return 96;
}

# 112 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static uint8_t UniqueSendP__SubSend__maxPayloadLength(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = CC2420CsmaP__Send__maxPayloadLength();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 95 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline uint8_t UniqueSendP__Send__maxPayloadLength(void )
#line 95
{
  return UniqueSendP__SubSend__maxPayloadLength();
}

# 112 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static uint8_t CC2420TinyosNetworkP__SubSend__maxPayloadLength(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = UniqueSendP__Send__maxPayloadLength();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 90 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void )
#line 90
{
  return CC2420TinyosNetworkP__SubSend__maxPayloadLength();
}

# 112 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static uint8_t CC2420ActiveMessageP__SubSend__maxPayloadLength(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = CC2420TinyosNetworkP__ActiveSend__maxPayloadLength();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 202 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void )
#line 202
{
  return CC2420ActiveMessageP__SubSend__maxPayloadLength();
}

# 302 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline uint16_t CC2420ControlP__CC2420Config__getPanAddr(void )
#line 302
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 303
    {
      unsigned int __nesc_temp = 
#line 303
      CC2420ControlP__m_pan;

      {
#line 303
        __nesc_atomic_end(__nesc_atomic); 
#line 303
        return __nesc_temp;
      }
    }
#line 305
    __nesc_atomic_end(__nesc_atomic); }
}

# 75 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420ActiveMessageP__CC2420Config__getPanAddr(void ){
#line 75
  unsigned int __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420ControlP__CC2420Config__getPanAddr();
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 53 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceQueue.nc"
inline static bool CC2420TinyosNetworkP__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 215 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__immediateRequest(uint8_t id)
#line 215
{
  if (CC2420TinyosNetworkP__resource_owner == id) {
#line 216
    return EALREADY;
    }
  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP__resource_owner == CC2420TinyosNetworkP__OWNER_NONE && CC2420TinyosNetworkP__Queue__isEmpty()) {
          CC2420TinyosNetworkP__resource_owner = id;
          return SUCCESS;
        }
      return FAIL;
    }
  else 
#line 224
    {
      CC2420TinyosNetworkP__resource_owner = id;
      return SUCCESS;
    }
}

# 97 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = CC2420TinyosNetworkP__Resource__immediateRequest(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 291 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg)
#line 291
{
}

# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/SendNotifier.nc"
inline static void CC2420ActiveMessageP__SendNotifier__aboutToSend(am_id_t arg_0x4125f740, am_addr_t dest, message_t * msg){
#line 59
    CC2420ActiveMessageP__SendNotifier__default__aboutToSend(arg_0x4125f740, dest, msg);
#line 59
}
#line 59
# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static error_t CC2420ActiveMessageP__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420TinyosNetworkP__ActiveSend__send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 128 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline void CC2420PacketP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId)
#line 128
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    *CC2420PacketP__getNetwork(p_msg) = networkId;
#line 131
    __nesc_atomic_end(__nesc_atomic); }
}

# 77 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static void CC2420TinyosNetworkP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId){
#line 77
  CC2420PacketP__CC2420Packet__setNetwork(p_msg, networkId);
#line 77
}
#line 77
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline int CC2420PacketP__getAddressLength(int type)
#line 81
{
  switch (type) {
      case IEEE154_ADDR_SHORT: return 2;
      case IEEE154_ADDR_EXT: return 8;
      case IEEE154_ADDR_NONE: return 0;
      default: return -100;
    }
}

# 251 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 251
{
  uint8_t *base = target;

#line 253
  base[0] = value;
  return value;
}

#line 240
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 240
{
  uint8_t *base = target;

#line 242
  base[0] = value;
  return value;
}

#line 257
static __inline  int8_t __nesc_hton_int8(void * target, int8_t value)
#line 257
{
#line 257
  __nesc_hton_uint8(target, value);
#line 257
  return value;
}

#line 281
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 281
{
  uint8_t *base = target;

#line 283
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

# 547 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca)
#line 547
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 548
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          {
            unsigned char __nesc_temp = 
#line 550
            ECANCEL;

            {
#line 550
              __nesc_atomic_end(__nesc_atomic); 
#line 550
              return __nesc_temp;
            }
          }
        }
#line 553
      if (CC2420TransmitP__m_state != CC2420TransmitP__S_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 554
            FAIL;

            {
#line 554
              __nesc_atomic_end(__nesc_atomic); 
#line 554
              return __nesc_temp;
            }
          }
        }


      CC2420TransmitP__m_state = CC2420TransmitP__S_LOAD;
      CC2420TransmitP__m_cca = cca;
      CC2420TransmitP__m_msg = p_msg;
      CC2420TransmitP__totalCcaChecks = 0;
    }
#line 564
    __nesc_atomic_end(__nesc_atomic); }

  if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
      CC2420TransmitP__loadTXFIFO();
    }

  return SUCCESS;
}

#line 192
static inline error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca)
#line 192
{
  return CC2420TransmitP__send(p_msg, useCca);
}

# 51 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg, bool useCca){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420TransmitP__Send__send(p_msg, useCca);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 301 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCca(am_id_t id, 
message_t *msg)
#line 302
{
}

# 95 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP__RadioBackoff__requestCca(am_id_t arg_0x4125fdb8, message_t * msg){
#line 95
    CC2420ActiveMessageP__RadioBackoff__default__requestCca(arg_0x4125fdb8, msg);
#line 95
}
#line 95
# 250 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubBackoff__requestCca(message_t *msg)
#line 250
{

  CC2420ActiveMessageP__RadioBackoff__requestCca(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

# 95 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestCca(message_t * msg){
#line 95
  CC2420ActiveMessageP__SubBackoff__requestCca(msg);
#line 95
}
#line 95
# 111 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    StateImplP__state[id] = reqState;
#line 112
    __nesc_atomic_end(__nesc_atomic); }
}

# 51 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
inline static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState){
#line 51
  StateImplP__State__forceState(1U, reqState);
#line 51
}
#line 51
#line 66
inline static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = StateImplP__State__isState(1U, myState);
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
#line 42
inline static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 122 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len)
#line 122
{
  unsigned char *__nesc_temp43;
  unsigned char *__nesc_temp42;
#line 124
  cc2420_header_t *header = CC2420CsmaP__CC2420PacketBody__getHeader(p_msg);
  cc2420_metadata_t *metadata = CC2420CsmaP__CC2420PacketBody__getMetadata(p_msg);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 127
    {
      if (!CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
          {
            unsigned char __nesc_temp = 
#line 129
            FAIL;

            {
#line 129
              __nesc_atomic_end(__nesc_atomic); 
#line 129
              return __nesc_temp;
            }
          }
        }
#line 132
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_TRANSMITTING);
      CC2420CsmaP__m_msg = p_msg;
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }








  (__nesc_temp42 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp42, __nesc_ntoh_leuint16(__nesc_temp42) & (((1 << IEEE154_FCF_ACK_REQ) | (
  0x3 << IEEE154_FCF_SRC_ADDR_MODE)) | (
  0x3 << IEEE154_FCF_DEST_ADDR_MODE))));

  (__nesc_temp43 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp43, __nesc_ntoh_leuint16(__nesc_temp43) | ((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE) | (
  1 << IEEE154_FCF_INTRAPAN))));

  __nesc_hton_int8(metadata->ack.data, FALSE);
  __nesc_hton_uint8(metadata->rssi.data, 0);
  __nesc_hton_uint8(metadata->lqi.data, 0);

  __nesc_hton_uint32(metadata->timestamp.data, CC2420_INVALID_TIMESTAMP);

  CC2420CsmaP__ccaOn = TRUE;
  CC2420CsmaP__RadioBackoff__requestCca(CC2420CsmaP__m_msg);

  CC2420CsmaP__CC2420Transmit__send(CC2420CsmaP__m_msg, CC2420CsmaP__ccaOn);
  return SUCCESS;
}

# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static error_t UniqueSendP__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420CsmaP__Send__send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 45 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
inline static error_t UniqueSendP__State__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(2U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 75 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline error_t UniqueSendP__Send__send(message_t *msg, uint8_t len)
#line 75
{
  error_t error;

#line 77
  if (UniqueSendP__State__requestState(UniqueSendP__S_SENDING) == SUCCESS) {
      __nesc_hton_leuint8(UniqueSendP__CC2420PacketBody__getHeader(msg)->dsn.data, UniqueSendP__localSendId++);

      if ((error = UniqueSendP__SubSend__send(msg, len)) != SUCCESS) {
          UniqueSendP__State__toIdle();
        }

      return error;
    }

  return EBUSY;
}

# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static error_t CC2420TinyosNetworkP__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = UniqueSendP__Send__send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_TXCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 70 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiPacket.nc"
inline static error_t CC2420SpiP__SpiPacket__send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID, txBuf, rxBuf, len);
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 45 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiByte.nc"
inline static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(tx);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 126 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
static inline bool StateImplP__State__isIdle(uint8_t id)
#line 126
{
  return StateImplP__State__isState(id, StateImplP__S_IDLE);
}

# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
inline static bool CC2420SpiP__WorkingState__isIdle(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = StateImplP__State__isIdle(0U);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 214 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 215
{

  uint8_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 219
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 221
            status;

            {
#line 221
              __nesc_atomic_end(__nesc_atomic); 
#line 221
              return __nesc_temp;
            }
          }
        }
    }
#line 225
    __nesc_atomic_end(__nesc_atomic); }
#line 225
  CC2420SpiP__m_addr = addr;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__SpiPacket__send(data, (void *)0, len);

  return status;
}

# 82 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length){
#line 82
  unsigned char __nesc_result;
#line 82

#line 82
  __nesc_result = CC2420SpiP__Fifo__write(CC2420_TXFIFO, data, length);
#line 82

#line 82
  return __nesc_result;
#line 82
}
#line 82
# 363 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableRxIntr(void )
#line 363
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 364
    {
      HplMsp430Usart1P__IFG2 &= ~(1 << 4);
      HplMsp430Usart1P__IE2 |= 1 << 4;
    }
#line 367
    __nesc_atomic_end(__nesc_atomic); }
}

# 180 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void ){
#line 180
  HplMsp430Usart1P__Usart__enableRxIntr();
#line 180
}
#line 180
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 64 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[id] != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY || /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id)
#line 82
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 83
    {
      if (!/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(id)) {
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = id;
            }
          else {
#line 88
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail] = id;
            }
#line 89
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = id;
          {
            unsigned char __nesc_temp = 
#line 90
            SUCCESS;

            {
#line 90
              __nesc_atomic_end(__nesc_atomic); 
#line 90
              return __nesc_temp;
            }
          }
        }
#line 92
      {
        unsigned char __nesc_temp = 
#line 92
        EBUSY;

        {
#line 92
          __nesc_atomic_end(__nesc_atomic); 
#line 92
          return __nesc_temp;
        }
      }
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }
}

# 79 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceQueue.nc"
inline static error_t CC2420TinyosNetworkP__Queue__enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420TinyosNetworkP__grantTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420TinyosNetworkP__grantTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 199 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__request(uint8_t id)
#line 199
{

  CC2420TinyosNetworkP__grantTask__postTask();

  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      return CC2420TinyosNetworkP__Queue__enqueue(id);
    }
  else 
#line 205
    {
      if (id == CC2420TinyosNetworkP__resource_owner) {
          return EALREADY;
        }
      else 
#line 208
        {
          CC2420TinyosNetworkP__next_owner = id;
          return SUCCESS;
        }
    }
}

# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420TinyosNetworkP__Resource__request(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t DiagMsgP__send__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(DiagMsgP__send);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 247 "/home/ali/workspace/TinyOS-main/tos/lib/diagmsg/DiagMsgP.nc"
static __inline uint8_t DiagMsgP__getPayloadLength(message_t *msg)
{
  return * (uint8_t *)& msg->header;
}

# 198 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 198
{
  __nesc_hton_leuint8(CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg)->length.data, len + CC2420_SIZE);
}

# 94 "/home/ali/workspace/TinyOS-main/tos/interfaces/Packet.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 94
  CC2420ActiveMessageP__Packet__setPayloadLength(msg, len);
#line 94
}
#line 94
# 90 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 91
{
  if (clientId >= 1) {
      return FAIL;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = msg;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(msg, len);

  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      error_t err;
      am_id_t amId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(msg);
      am_addr_t dest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(msg);

      ;
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = clientId;

      err = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static error_t /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(0U, msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 169 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__AMPacket__setType(message_t *amsg, am_id_t type)
#line 169
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 171
  __nesc_hton_leuint8(header->type.data, type);
}

# 162 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMPacket.nc"
inline static void /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(message_t * amsg, am_id_t t){
#line 162
  CC2420ActiveMessageP__AMPacket__setType(amsg, t);
#line 162
}
#line 162
# 149 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__AMPacket__setDestination(message_t *amsg, am_addr_t addr)
#line 149
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 151
  __nesc_hton_leuint16(header->dest.data, addr);
}

# 103 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMPacket.nc"
inline static void /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 103
  CC2420ActiveMessageP__AMPacket__setDestination(amsg, addr);
#line 103
}
#line 103
# 53 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueEntryP.nc"
static inline error_t /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 55
{
  /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(msg, dest);
  /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(msg, 177);
  return /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(msg, len);
}

# 80 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
inline static error_t DiagMsgP__AMSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*DiagMsgC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 252 "/home/ali/workspace/TinyOS-main/tos/lib/diagmsg/DiagMsgP.nc"
static inline void DiagMsgP__send__runTask(void )
{
  message_t *msg;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 256
    msg = DiagMsgP__sending;
#line 256
    __nesc_atomic_end(__nesc_atomic); }

  if (DiagMsgP__AMSend__send(AM_BROADCAST_ADDR, msg, DiagMsgP__getPayloadLength(msg)) != SUCCESS) {
    DiagMsgP__send__postTask();
    }
}

# 229 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id)
#line 229
{
  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      CC2420TinyosNetworkP__grantTask__postTask();
    }
  CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
  return SUCCESS;
}

#line 253
static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client)
#line 253
{
  CC2420TinyosNetworkP__Resource__release(client);
}

# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static void CC2420TinyosNetworkP__Resource__granted(uint8_t arg_0x4122b2d0){
#line 102
  switch (arg_0x4122b2d0) {
#line 102
    case CC2420ActiveMessageC__CC2420_AM_SEND_ID:
#line 102
      CC2420ActiveMessageP__RadioResource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      CC2420TinyosNetworkP__Resource__default__granted(arg_0x4122b2d0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 68 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 68
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 69
    {
      if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
          uint8_t id = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead;

#line 72
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead];
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
            }
#line 75
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[id] = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
          {
            unsigned char __nesc_temp = 
#line 76
            id;

            {
#line 76
              __nesc_atomic_end(__nesc_atomic); 
#line 76
              return __nesc_temp;
            }
          }
        }
#line 78
      {
        unsigned char __nesc_temp = 
#line 78
        /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

        {
#line 78
          __nesc_atomic_end(__nesc_atomic); 
#line 78
          return __nesc_temp;
        }
      }
    }
#line 81
    __nesc_atomic_end(__nesc_atomic); }
}

# 70 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 180 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__grantTask__runTask(void )
#line 180
{


  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP__resource_owner == CC2420TinyosNetworkP__OWNER_NONE && !CC2420TinyosNetworkP__Queue__isEmpty()) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__Queue__dequeue();

          if (CC2420TinyosNetworkP__resource_owner != CC2420TinyosNetworkP__OWNER_NONE) {
              CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
            }
        }
    }
  else 
#line 191
    {
      if (CC2420TinyosNetworkP__next_owner != CC2420TinyosNetworkP__resource_owner) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__next_owner;
          CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
        }
    }
}

# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 138 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void *CC2420TinyosNetworkP__BareSend__getPayload(message_t *msg, uint8_t len)
#line 138
{

  cc2420_header_t *hdr = CC2420TinyosNetworkP__CC2420PacketBody__getHeader(msg);

#line 141
  return hdr;
}

#line 241
static inline message_t *CC2420TinyosNetworkP__BareReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 241
{
  return msg;
}

# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__BareReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420TinyosNetworkP__BareReceive__default__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 283 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 283
{
  return msg;
}

# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP__Snoop__receive(am_id_t arg_0x41261cc0, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
    __nesc_result = CC2420ActiveMessageP__Snoop__default__receive(arg_0x41261cc0, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 279 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 279
{
  return msg;
}

# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP__Receive__receive(am_id_t arg_0x41261628, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
    __nesc_result = CC2420ActiveMessageP__Receive__default__receive(arg_0x41261628, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 72 "/home/ali/workspace/TinyOS-main/tos/system/ActiveMessageAddressC.nc"
static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void )
#line 72
{
  return ActiveMessageAddressC__amAddress();
}

# 50 "/home/ali/workspace/TinyOS-main/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t CC2420ActiveMessageP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 135 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline am_addr_t CC2420ActiveMessageP__AMPacket__address(void )
#line 135
{
  return CC2420ActiveMessageP__ActiveMessageAddress__amAddress();
}

#line 159
static inline bool CC2420ActiveMessageP__AMPacket__isForMe(message_t *amsg)
#line 159
{
  return CC2420ActiveMessageP__AMPacket__destination(amsg) == CC2420ActiveMessageP__AMPacket__address() || 
  CC2420ActiveMessageP__AMPacket__destination(amsg) == AM_BROADCAST_ADDR;
}

#line 219
static inline message_t *CC2420ActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 219
{

  if (CC2420ActiveMessageP__AMPacket__isForMe(msg)) {
      return CC2420ActiveMessageP__Receive__receive(CC2420ActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
  else {
      return CC2420ActiveMessageP__Snoop__receive(CC2420ActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
}

# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__ActiveReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ActiveMessageP__SubReceive__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 119 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg)
#line 119
{



  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      unsigned char __nesc_temp = 
#line 124
      *CC2420PacketP__getNetwork(p_msg);

      {
#line 124
        __nesc_atomic_end(__nesc_atomic); 
#line 124
        return __nesc_temp;
      }
    }
#line 126
    __nesc_atomic_end(__nesc_atomic); }
}

# 75 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t *p_msg){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420PacketP__CC2420Packet__getNetwork(p_msg);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 157 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 157
{
  uint8_t network = CC2420TinyosNetworkP__CC2420Packet__getNetwork(msg);

  if (! __nesc_ntoh_int8(CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(msg)->crc.data)) {
      return msg;
    }

  if (network == 0x3f) {
      return CC2420TinyosNetworkP__ActiveReceive__receive(msg, payload, len);
    }
  else 
#line 166
    {
      return CC2420TinyosNetworkP__BareReceive__receive(msg, 
      CC2420TinyosNetworkP__BareSend__getPayload(msg, len), 
      len + sizeof(cc2420_header_t ));
    }
}

# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420TinyosNetworkP__SubReceive__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 138 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn)
#line 138
{
  uint8_t element = UniqueReceiveP__recycleSourceElement;
  bool increment = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 142
    {
      if (element == UniqueReceiveP__INVALID_ELEMENT || UniqueReceiveP__writeIndex == element) {

          element = UniqueReceiveP__writeIndex;
          increment = TRUE;
        }

      UniqueReceiveP__receivedMessages[element].source = msgSource;
      UniqueReceiveP__receivedMessages[element].dsn = msgDsn;
      if (increment) {
          UniqueReceiveP__writeIndex++;
          UniqueReceiveP__writeIndex %= 4;
        }
    }
#line 155
    __nesc_atomic_end(__nesc_atomic); }
}

#line 192
static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 192
{
  return msg;
}

# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__DuplicateReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = UniqueReceiveP__DuplicateReceive__default__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 112 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn)
#line 112
{
  int i;

#line 114
  UniqueReceiveP__recycleSourceElement = UniqueReceiveP__INVALID_ELEMENT;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 116
    {
      for (i = 0; i < 4; i++) {
          if (UniqueReceiveP__receivedMessages[i].source == msgSource) {
              if (UniqueReceiveP__receivedMessages[i].dsn == msgDsn) {

                  {
                    unsigned char __nesc_temp = 
#line 121
                    TRUE;

                    {
#line 121
                      __nesc_atomic_end(__nesc_atomic); 
#line 121
                      return __nesc_temp;
                    }
                  }
                }
#line 124
              UniqueReceiveP__recycleSourceElement = i;
            }
        }
    }
#line 127
    __nesc_atomic_end(__nesc_atomic); }

  return FALSE;
}

# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 165 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline uint16_t UniqueReceiveP__getSourceKey(message_t *msg)
#line 165
{
  cc2420_header_t *hdr = UniqueReceiveP__CC2420PacketBody__getHeader(msg);
  int s_mode = (__nesc_ntoh_leuint16(hdr->fcf.data) >> IEEE154_FCF_SRC_ADDR_MODE) & 0x3;
  int d_mode = (__nesc_ntoh_leuint16(hdr->fcf.data) >> IEEE154_FCF_DEST_ADDR_MODE) & 0x3;
  int s_offset = 2;
#line 169
  int s_len = 2;
  uint16_t key = 0;
  uint8_t *current = (uint8_t *)& hdr->dest;
  int i;

  if (s_mode == IEEE154_ADDR_EXT) {
      s_len = 8;
    }
  if (d_mode == IEEE154_ADDR_EXT) {
      s_offset = 8;
    }

  current += s_offset;

  for (i = 0; i < s_len; i++) {
      key += current[i];
    }
  return key;
}

#line 86
static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len)
#line 87
{

  uint16_t msgSource = UniqueReceiveP__getSourceKey(msg);
  uint8_t msgDsn = __nesc_ntoh_leuint8(UniqueReceiveP__CC2420PacketBody__getHeader(msg)->dsn.data);

  if (UniqueReceiveP__hasSeen(msgSource, msgDsn)) {
      return UniqueReceiveP__DuplicateReceive__receive(msg, payload, len);
    }
  else 
#line 94
    {
      UniqueReceiveP__insert(msgSource, msgDsn);
      return UniqueReceiveP__Receive__receive(msg, payload, len);
    }
}

# 78 "/home/ali/workspace/TinyOS-main/tos/interfaces/Receive.nc"
inline static message_t * CC2420ReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = UniqueReceiveP__SubReceive__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 48 "/home/ali/workspace/TinyOS-main/tos/interfaces/LocalIeeeEui64.nc"
inline static ieee_eui64_t CC2420ControlP__LocalIeeeEui64__getId(void ){
#line 48
  struct ieee_eui64 __nesc_result;
#line 48

#line 48
  __nesc_result = DallasId48ToIeeeEui64C__LocalIeeeEui64__getId();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 290 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline ieee_eui64_t CC2420ControlP__CC2420Config__getExtAddr(void )
#line 290
{
  return CC2420ControlP__LocalIeeeEui64__getId();
}

# 64 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static ieee_eui64_t CC2420ReceiveP__CC2420Config__getExtAddr(void ){
#line 64
  struct ieee_eui64 __nesc_result;
#line 64

#line 64
  __nesc_result = CC2420ControlP__CC2420Config__getExtAddr();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64





inline static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void ){
#line 69
  unsigned int __nesc_result;
#line 69

#line 69
  __nesc_result = CC2420ControlP__CC2420Config__getShortAddr();
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 347 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void )
#line 347
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 348
    {
      unsigned char __nesc_temp = 
#line 348
      CC2420ControlP__addressRecognition;

      {
#line 348
        __nesc_atomic_end(__nesc_atomic); 
#line 348
        return __nesc_temp;
      }
    }
#line 350
    __nesc_atomic_end(__nesc_atomic); }
}

# 91 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void ){
#line 91
  unsigned char __nesc_result;
#line 91

#line 91
  __nesc_result = CC2420ControlP__CC2420Config__isAddressRecognitionEnabled();
#line 91

#line 91
  return __nesc_result;
#line 91
}
#line 91
# 42 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 819 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg)
#line 819
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(msg);
  int mode = (__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_DEST_ADDR_MODE) & 3;
  ieee_eui64_t *ext_addr;

  if (!CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled()) {
      return TRUE;
    }

  if (mode == IEEE154_ADDR_SHORT) {
      return __nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP__CC2420Config__getShortAddr()
       || __nesc_ntoh_leuint16(header->dest.data) == IEEE154_BROADCAST_ADDR;
    }
  else {
#line 831
    if (mode == IEEE154_ADDR_EXT) {
        ieee_eui64_t local_addr = CC2420ReceiveP__CC2420Config__getExtAddr();

#line 833
        ext_addr = (ieee_eui64_t *)& header->dest;
        return memcmp(ext_addr->data, local_addr.data, IEEE_EUI64_LENGTH) == 0;
      }
    else 
#line 835
      {

        return FALSE;
      }
    }
}

# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 671 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__receiveDone_task__runTask(void )
#line 671
{
  cc2420_metadata_t *metadata = CC2420ReceiveP__CC2420PacketBody__getMetadata(CC2420ReceiveP__m_p_rx_buf);
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t length = __nesc_ntoh_leuint8(header->length.data);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

  __nesc_hton_int8(metadata->crc.data, buf[length] >> 7);
  __nesc_hton_uint8(metadata->lqi.data, buf[length] & 0x7f);
  __nesc_hton_uint8(metadata->rssi.data, buf[length - 1]);

  if (CC2420ReceiveP__passesAddressCheck(CC2420ReceiveP__m_p_rx_buf) && length >= CC2420_SIZE) {
#line 696
      CC2420ReceiveP__m_p_rx_buf = CC2420ReceiveP__Receive__receive(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_p_rx_buf->data, 
      length - CC2420_SIZE);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 699
    CC2420ReceiveP__receivingPacket = FALSE;
#line 699
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ReceiveP__waitForNextPacket();
}

# 91 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/ds2411/HplDs2411C.nc"
static inline void HplDs2411C__clear_pins(void )
{
}

# 368 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/msp430hardware.h"
static inline void brief_pause(unsigned int n)
{
   __asm volatile ("1: \n\t"
  "dec	%0 \n\t"
  "jne	1b\n\t" : 
  "+r"(n));}

# 143 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/hardware.h"
static inline  uint8_t TOSH_READ_ONEWIRE_PIN()
#line 143
{
#line 143
  static volatile uint8_t r __asm ("0x001C");

#line 143
  return r & (1 << 7);
}

#line 143
static inline  void TOSH_MAKE_ONEWIRE_INPUT()
#line 143
{
#line 143
  static volatile uint8_t r __asm ("0x001E");

#line 143
  r &= ~(1 << 7);
}

# 135 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/ds2411/HplDs2411C.nc"
static inline bool HplDs2411C__read_bit(void )
{
  int bit;

#line 138
  TOSH_MAKE_ONEWIRE_OUTPUT();
  brief_pause(((unsigned long long )HplDs2411C__STD_A * 4096 * 1024 / 1000000 - 2) / 3);
  TOSH_MAKE_ONEWIRE_INPUT();
  brief_pause(((unsigned long long )HplDs2411C__STD_E * 4096 * 1024 / 1000000 - 2) / 3);
  bit = TOSH_READ_ONEWIRE_PIN();
  brief_pause(((unsigned long long )HplDs2411C__STD_F * 4096 * 1024 / 1000000 - 2) / 3);
  return bit;
}








static inline uint8_t HplDs2411C__read_byte(void )
{
  uint8_t byte = 0;
  uint8_t bit;

#line 158
  for (bit = 0x01; bit != 0; bit <<= 1) 
    {
      if (HplDs2411C__read_bit()) {
        byte |= bit;
        }
    }
#line 163
  return byte;
}

static inline uint8_t HplDs2411C__crc8_byte(uint8_t crc, uint8_t byte)
{
  int i;

#line 169
  crc ^= byte;
  for (i = 0; i < 8; i++) 
    {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0x8c;
        }
      else {
#line 175
        crc >>= 1;
        }
    }
#line 177
  return crc;
}

#line 119
static inline void HplDs2411C__write_bit_zero(void )
{
  TOSH_MAKE_ONEWIRE_OUTPUT();
  brief_pause(((unsigned long long )HplDs2411C__STD_C * 4096 * 1024 / 1000000 - 2) / 3);
  TOSH_MAKE_ONEWIRE_INPUT();
  brief_pause(((unsigned long long )HplDs2411C__STD_D * 4096 * 1024 / 1000000 - 2) / 3);
}

#line 111
static inline void HplDs2411C__write_bit_one(void )
{
  TOSH_MAKE_ONEWIRE_OUTPUT();
  brief_pause(((unsigned long long )HplDs2411C__STD_A * 4096 * 1024 / 1000000 - 2) / 3);
  TOSH_MAKE_ONEWIRE_INPUT();
  brief_pause(((unsigned long long )HplDs2411C__STD_B * 4096 * 1024 / 1000000 - 2) / 3);
}









static inline void HplDs2411C__write_bit(int is_one)
{
  if (is_one) {
    HplDs2411C__write_bit_one();
    }
  else {
#line 132
    HplDs2411C__write_bit_zero();
    }
}

#line 147
static inline void HplDs2411C__write_byte(uint8_t byte)
{
  uint8_t bit;

#line 150
  for (bit = 0x01; bit != 0; bit <<= 1) 
    HplDs2411C__write_bit(byte & bit);
}

#line 99
static inline bool HplDs2411C__reset(void )
{
  int present;

#line 102
  TOSH_MAKE_ONEWIRE_OUTPUT();
  brief_pause(((unsigned long long )HplDs2411C__STD_H * 4096 * 1024 / 1000000 - 2) / 3);
  TOSH_MAKE_ONEWIRE_INPUT();
  brief_pause(((unsigned long long )HplDs2411C__STD_I * 4096 * 1024 / 1000000 - 2) / 3);
  present = TOSH_READ_ONEWIRE_PIN();
  brief_pause(((unsigned long long )HplDs2411C__STD_J * 4096 * 1024 / 1000000 - 2) / 3);
  return present == 0;
}

# 143 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/hardware.h"
static inline  void TOSH_CLR_ONEWIRE_PIN()
#line 143
{
#line 143
  static volatile uint8_t r __asm ("0x001D");

#line 143
  r &= ~(1 << 7);
}

# 79 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/ds2411/HplDs2411C.nc"
static inline void HplDs2411C__init_pins(void )
{





  TOSH_SEL_ONEWIRE_IOFUNC();
  TOSH_MAKE_ONEWIRE_INPUT();
  TOSH_CLR_ONEWIRE_PIN();
}

#line 188
static inline error_t HplDs2411C__IDChip__read(uint8_t *id_buf)
{
  int retry = 5;
  uint8_t id[8];

  HplDs2411C__init_pins();
  brief_pause(((unsigned long long )1200 * 4096 * 1024 / 1000000 - 2) / 3);

  while (retry-- > 0) {
      int crc = 0;

#line 198
      if (HplDs2411C__reset()) {
          uint8_t *byte;

          HplDs2411C__write_byte(0x33);
          for (byte = id + 7; byte != id - 1; byte--) 
            crc = HplDs2411C__crc8_byte(crc, *byte = HplDs2411C__read_byte());

          if (crc == 0) {
              memcpy(id_buf, id + 1, 6);
              HplDs2411C__clear_pins();
              return SUCCESS;
            }
        }
    }

  HplDs2411C__clear_pins();
  return FAIL;
}

# 44 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/ds2411/IDChip.nc"
inline static error_t DallasId48ToIeeeEui64C__IDChip__read(uint8_t *id_buf){
#line 44
  unsigned char __nesc_result;
#line 44

#line 44
  __nesc_result = HplDs2411C__IDChip__read(id_buf);
#line 44

#line 44
  return __nesc_result;
#line 44
}
#line 44
# 178 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline bool CC2420SpiP__Resource__isOwner(uint8_t id)
#line 178
{
  /* atomic removed: atomic calls only */
#line 179
  {
    unsigned char __nesc_temp = 
#line 179
    CC2420SpiP__m_holder == id;

#line 179
    return __nesc_temp;
  }
}

# 128 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static bool CC2420ReceiveP__SpiResource__isOwner(void ){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  __nesc_result = CC2420SpiP__Resource__isOwner(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
#line 97
inline static error_t CC2420ReceiveP__SpiResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
#line 88
inline static error_t CC2420ReceiveP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420SpiP__grant__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420SpiP__grant);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 184 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__SpiResource__granted(void )
#line 184
{
  CC2420SpiP__grant__postTask();
}

# 181 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id)
#line 181
{
}

# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(uint8_t arg_0x40f6c010){
#line 102
  switch (arg_0x40f6c010) {
#line 102
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID:
#line 102
      CC2420SpiP__SpiResource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(arg_0x40f6c010);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 130 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id)
#line 130
{
  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(id);
}

# 202 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(uint8_t arg_0x40a22310){
#line 102
  switch (arg_0x40a22310) {
#line 102
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 102
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(arg_0x40a22310);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 190 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId);
}

# 252 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 252
{
}

# 82 "/home/ali/workspace/TinyOS-main/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(uint8_t arg_0x40f6abe0, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 82
  switch (arg_0x40f6abe0) {
#line 82
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi1C__0__CLIENT_ID:
#line 82
      CC2420SpiP__SpiPacket__sendDone(txBuf, rxBuf, len, error);
#line 82
      break;
#line 82
    default:
#line 82
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(arg_0x40f6abe0, txBuf, rxBuf, len, error);
#line 82
      break;
#line 82
    }
#line 82
}
#line 82
# 245 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void )
#line 245
{
  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_client, /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf, /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf, /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_len, 
  SUCCESS);
}

#line 228
static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void )
#line 228
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 229
    /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
#line 229
    __nesc_atomic_end(__nesc_atomic); }
}

# 486 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 487
{
}

# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 40 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__set(void ){
#line 40
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 40
}
#line 40
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ReceiveP__receiveDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420ReceiveP__receiveDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 389 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg)
#line 389
{
  cc2420_header_t *ack_header;
  cc2420_header_t *msg_header;
  cc2420_metadata_t *msg_metadata;
  uint8_t *ack_buf;
  uint8_t length;

  if (type == IEEE154_TYPE_ACK && CC2420TransmitP__m_msg) {
      ack_header = CC2420TransmitP__CC2420PacketBody__getHeader(ack_msg);
      msg_header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);

      if (CC2420TransmitP__m_state == CC2420TransmitP__S_ACK_WAIT && __nesc_ntoh_leuint8(msg_header->dsn.data) == __nesc_ntoh_leuint8(ack_header->dsn.data)) {
          CC2420TransmitP__BackoffTimer__stop();

          msg_metadata = CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg);
          ack_buf = (uint8_t *)ack_header;
          length = __nesc_ntoh_leuint8(ack_header->length.data);

          __nesc_hton_int8(msg_metadata->ack.data, TRUE);
          __nesc_hton_uint8(msg_metadata->rssi.data, ack_buf[length - 1]);
          __nesc_hton_uint8(msg_metadata->lqi.data, ack_buf[length] & 0x7f);
          CC2420TransmitP__signalDone(SUCCESS);
        }
    }
}

# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message){
#line 63
  CC2420TransmitP__CC2420Receive__receive(type, message);
#line 63
}
#line 63
# 70 "/home/ali/workspace/TinyOS-main/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420ReceiveP__PacketTimeStamp__clear(message_t * msg){
#line 70
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 70
}
#line 70








inline static void CC2420ReceiveP__PacketTimeStamp__set(message_t * msg, CC2420ReceiveP__PacketTimeStamp__size_type value){
#line 78
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 78
}
#line 78
# 59 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 0);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw() != 0;
}

# 73 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get();
}

# 43 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFOP__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 59 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 5);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw() != 0;
}

# 73 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get();
}

# 43 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFO__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 209 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 210
{
  return CC2420SpiP__SpiPacket__send((void *)0, data, len);
}

# 62 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = CC2420SpiP__Fifo__continueRead(CC2420_RXFIFO, data, length);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
#line 51
inline static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420SpiP__Fifo__beginRead(CC2420_RXFIFO, data, length);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 41 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__clr(void ){
#line 41
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 41
}
#line 41
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SACK__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SACK);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 374 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void )
#line 374
{
  /* atomic removed: atomic calls only */
#line 375
  {
    unsigned char __nesc_temp = 
#line 375
    CC2420ControlP__hwAutoAckDefault;

#line 375
    return __nesc_temp;
  }
}

# 110 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420ControlP__CC2420Config__isHwAutoAckDefault();
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 381 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void )
#line 381
{
  /* atomic removed: atomic calls only */
#line 382
  {
    unsigned char __nesc_temp = 
#line 382
    CC2420ControlP__autoAckEnabled;

#line 382
    return __nesc_temp;
  }
}

# 115 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void ){
#line 115
  unsigned char __nesc_result;
#line 115

#line 115
  __nesc_result = CC2420ControlP__CC2420Config__isAutoAckEnabled();
#line 115

#line 115
  return __nesc_result;
#line 115
}
#line 115
# 525 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error)
#line 526
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

#line 530
  CC2420ReceiveP__rxFrameLength = buf[0];

  switch (CC2420ReceiveP__m_state) {

      case CC2420ReceiveP__S_RX_LENGTH: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_FCF;



      if (CC2420ReceiveP__rxFrameLength + 1 > CC2420ReceiveP__m_bytes_left) 



        {

          CC2420ReceiveP__flush();
        }
      else {
          if (!CC2420ReceiveP__FIFO__get() && !CC2420ReceiveP__FIFOP__get()) {
              CC2420ReceiveP__m_bytes_left -= CC2420ReceiveP__rxFrameLength + 1;
            }

          if (CC2420ReceiveP__rxFrameLength <= MAC_PACKET_SIZE) {
              if (CC2420ReceiveP__rxFrameLength > 0) {
                  if (CC2420ReceiveP__rxFrameLength > CC2420ReceiveP__SACK_HEADER_LENGTH) {

                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__SACK_HEADER_LENGTH);
                    }
                  else {

                      CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;
                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__rxFrameLength);
                    }
                }
              else {
                  /* atomic removed: atomic calls only */
                  CC2420ReceiveP__receivingPacket = FALSE;
                  CC2420ReceiveP__CSN__set();
                  CC2420ReceiveP__SpiResource__release();
                  CC2420ReceiveP__waitForNextPacket();
                }
            }
          else {

              CC2420ReceiveP__flush();
            }
        }
      break;

      case CC2420ReceiveP__S_RX_FCF: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;










      if (CC2420ReceiveP__CC2420Config__isAutoAckEnabled() && !CC2420ReceiveP__CC2420Config__isHwAutoAckDefault()) {



          if (((__nesc_ntoh_leuint16(
#line 592
          header->fcf.data) >> IEEE154_FCF_ACK_REQ) & 0x01) == 1
           && (__nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP__CC2420Config__getShortAddr()
           || __nesc_ntoh_leuint16(header->dest.data) == AM_BROADCAST_ADDR)
           && ((__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7) == IEEE154_TYPE_DATA) {

              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__SACK__strobe();
              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__RXFIFO__beginRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
              CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
              return;
            }
        }

      CC2420ReceiveP__RXFIFO__continueRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
      CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
      break;

      case CC2420ReceiveP__S_RX_PAYLOAD: 

        CC2420ReceiveP__CSN__set();
      if (!CC2420ReceiveP__m_missed_packets) {

          CC2420ReceiveP__SpiResource__release();
        }




      if ((((
#line 621
      CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get())
       || !CC2420ReceiveP__m_timestamp_size)
       || CC2420ReceiveP__rxFrameLength <= 10) {
          CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
        }
      else {
          if (CC2420ReceiveP__m_timestamp_size == 1) {
            CC2420ReceiveP__PacketTimeStamp__set(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__m_timestamp_head]);
            }
#line 629
          CC2420ReceiveP__m_timestamp_head = (CC2420ReceiveP__m_timestamp_head + 1) % CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;
          CC2420ReceiveP__m_timestamp_size--;

          if (CC2420ReceiveP__m_timestamp_size > 0) {
              CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
              CC2420ReceiveP__m_timestamp_head = 0;
              CC2420ReceiveP__m_timestamp_size = 0;
            }
        }



      if (buf[CC2420ReceiveP__rxFrameLength] >> 7 && rx_buf) {
          uint8_t type = (__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7;

#line 643
          CC2420ReceiveP__CC2420Receive__receive(type, CC2420ReceiveP__m_p_rx_buf);
          if (type == IEEE154_TYPE_DATA) {
              CC2420ReceiveP__receiveDone_task__postTask();
              return;
            }
        }

      CC2420ReceiveP__waitForNextPacket();
      break;

      default: /* atomic removed: atomic calls only */
        CC2420ReceiveP__receivingPacket = FALSE;
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__SpiResource__release();
      break;
    }
}

# 370 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 370
{
}

# 71 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__readDone(uint8_t arg_0x40ef5b38, uint8_t * data, uint8_t length, error_t error){
#line 71
  switch (arg_0x40ef5b38) {
#line 71
    case CC2420_TXFIFO:
#line 71
      CC2420TransmitP__TXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    case CC2420_RXFIFO:
#line 71
      CC2420ReceiveP__RXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    default:
#line 71
      CC2420SpiP__Fifo__default__readDone(arg_0x40ef5b38, data, length, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHRX);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 293 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(am_id_t id, 
message_t *msg)
#line 294
{
}

# 81 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP__RadioBackoff__requestInitialBackoff(am_id_t arg_0x4125fdb8, message_t * msg){
#line 81
    CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(arg_0x4125fdb8, msg);
#line 81
}
#line 81
# 241 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(message_t *msg)
#line 241
{
  CC2420ActiveMessageP__RadioBackoff__requestInitialBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

# 81 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestInitialBackoff(message_t * msg){
#line 81
  CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(msg);
#line 81
}
#line 81
# 243 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime)
#line 243
{
  CC2420TransmitP__myInitialBackoff = backoffTime + 1;
}

# 60 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__SubBackoff__setInitialBackoff(uint16_t backoffTime){
#line 60
  CC2420TransmitP__RadioBackoff__setInitialBackoff(backoffTime);
#line 60
}
#line 60
# 223 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t *msg)
#line 223
{
  CC2420CsmaP__SubBackoff__setInitialBackoff(CC2420CsmaP__Random__rand16()
   % (0x1F * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP__RadioBackoff__requestInitialBackoff(msg);
}

# 81 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP__RadioBackoff__requestInitialBackoff(message_t * msg){
#line 81
  CC2420CsmaP__SubBackoff__requestInitialBackoff(msg);
#line 81
}
#line 81
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__sendDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__sendDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 205 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err)
#line 205
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 206
    CC2420CsmaP__sendErr = err;
#line 206
    __nesc_atomic_end(__nesc_atomic); }
  CC2420CsmaP__sendDone_task__postTask();
}

# 73 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error){
#line 73
  CC2420CsmaP__CC2420Transmit__sendDone(p_msg, error);
#line 73
}
#line 73
# 454 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 455
{

  CC2420TransmitP__CSN__set();
  if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
      /* atomic removed: atomic calls only */
#line 459
      {
        CC2420TransmitP__CSN__clr();
        CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CSN__set();
      }
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
    }
  else {
#line 468
    if (!CC2420TransmitP__m_cca) {
        /* atomic removed: atomic calls only */
#line 469
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
        }
        CC2420TransmitP__attemptSend();
      }
    else {
        CC2420TransmitP__releaseSpiResource();
        /* atomic removed: atomic calls only */
#line 476
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_SAMPLE_CCA;
        }

        CC2420TransmitP__RadioBackoff__requestInitialBackoff(CC2420TransmitP__m_msg);
        CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myInitialBackoff);
      }
    }
}

# 663 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 663
{
}

# 373 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 373
{
}

# 91 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__writeDone(uint8_t arg_0x40ef5b38, uint8_t * data, uint8_t length, error_t error){
#line 91
  switch (arg_0x40ef5b38) {
#line 91
    case CC2420_TXFIFO:
#line 91
      CC2420TransmitP__TXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    case CC2420_RXFIFO:
#line 91
      CC2420ReceiveP__RXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    default:
#line 91
      CC2420SpiP__Fifo__default__writeDone(arg_0x40ef5b38, data, length, error);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__TXCTRL__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_TXCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 524 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__writeTxctrl(void )
#line 524
{
  /* atomic removed: atomic calls only */
#line 525
  {
    CC2420ControlP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
    3 << CC2420_TXCTRL_PA_CURRENT)) | (
    1 << CC2420_TXCTRL_RESERVED)) | ((
    31 & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
  }
}

# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_RXCTRL1, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
inline static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG0, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SXOSCON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 197 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__enable(void )
#line 197
{
#line 197
  P2IE |= 1 << 7;
}

# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void ){
#line 42
  HplMsp430InterruptP__Port27__enable();
#line 42
}
#line 42
# 264 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__edge(bool l2h)
#line 264
{
  /* atomic removed: atomic calls only */
#line 265
  {
    if (l2h) {
#line 266
      P2IES &= ~(1 << 7);
      }
    else {
#line 267
      P2IES |= 1 << 7;
      }
  }
}

# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high){
#line 67
  HplMsp430InterruptP__Port27__edge(low_to_high);
#line 67
}
#line 67
# 213 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__clear(void )
#line 213
{
#line 213
  P2IFG &= ~(1 << 7);
}

# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 52
  HplMsp430InterruptP__Port27__clear();
#line 52
}
#line 52
# 205 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__disable(void )
#line 205
{
#line 205
  P2IE &= ~(1 << 7);
}

# 47 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void ){
#line 47
  HplMsp430InterruptP__Port27__disable();
#line 47
}
#line 47
# 69 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void )
#line 69
{
  /* atomic removed: atomic calls only */
#line 70
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  }
  return SUCCESS;
}

#line 52
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising)
#line 52
{
  /* atomic removed: atomic calls only */
#line 53
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable();
  }
  return SUCCESS;
}

static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void )
#line 61
{
  return /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(TRUE);
}

# 53 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG1, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 216 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startOscillator(void )
#line 216
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 217
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 219
            FAIL;

            {
#line 219
              __nesc_atomic_end(__nesc_atomic); 
#line 219
              return __nesc_temp;
            }
          }
        }
#line 222
      CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTING;
      CC2420ControlP__IOCFG1__write(CC2420_SFDMUX_XOSC16M_STABLE << 
      CC2420_IOCFG1_CCAMUX);

      CC2420ControlP__InterruptCCA__enableRisingEdge();
      CC2420ControlP__SXOSCON__strobe();

      CC2420ControlP__IOCFG0__write((1 << CC2420_IOCFG0_FIFOP_POLARITY) | (
      127 << CC2420_IOCFG0_FIFOP_THR));

      CC2420ControlP__writeFsctrl();
      CC2420ControlP__writeMdmctrl0();

      CC2420ControlP__RXCTRL1__write(((((((1 << CC2420_RXCTRL1_RXBPF_LOCUR) | (
      1 << CC2420_RXCTRL1_LOW_LOWGAIN)) | (
      1 << CC2420_RXCTRL1_HIGH_HGM)) | (
      1 << CC2420_RXCTRL1_LNA_CAP_ARRAY)) | (
      1 << CC2420_RXCTRL1_RXMIX_TAIL)) | (
      1 << CC2420_RXCTRL1_RXMIX_VCM)) | (
      2 << CC2420_RXCTRL1_RXMIX_CURRENT));

      CC2420ControlP__writeTxctrl();
    }
#line 244
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 71 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startOscillator(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = CC2420ControlP__CC2420Power__startOscillator();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 214 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__Resource__granted(void )
#line 214
{
  CC2420CsmaP__CC2420Power__startOscillator();
}

# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static void CC2420ControlP__Resource__granted(void ){
#line 102
  CC2420CsmaP__Resource__granted();
#line 102
}
#line 102
# 41 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__clr(void ){
#line 41
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 41
}
#line 41
# 405 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SpiResource__granted(void )
#line 405
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__Resource__granted();
}

# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ControlP__syncDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420ControlP__syncDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 40 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__set(void ){
#line 40
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 40
}
#line 40
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SRXON__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRXON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
inline static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRFOFF);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 391 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SyncResource__granted(void )
#line 391
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRFOFF__strobe();
  CC2420ControlP__writeFsctrl();
  CC2420ControlP__writeMdmctrl0();
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRXON__strobe();
  CC2420ControlP__CSN__set();
  CC2420ControlP__SyncResource__release();
  CC2420ControlP__syncDone__postTask();
}

#line 536
static inline void CC2420ControlP__ReadRssi__default__readDone(error_t error, uint16_t data)
#line 536
{
}

# 63 "/home/ali/workspace/TinyOS-main/tos/interfaces/Read.nc"
inline static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val){
#line 63
  CC2420ControlP__ReadRssi__default__readDone(result, val);
#line 63
}
#line 63
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RssiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 287 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data)
#line 287
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 291
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 293
            status;

            {
#line 293
              __nesc_atomic_end(__nesc_atomic); 
#line 293
              return __nesc_temp;
            }
          }
        }
    }
#line 297
    __nesc_atomic_end(__nesc_atomic); }
#line 297
  status = CC2420SpiP__SpiByte__write(addr | 0x40);
  *data = (uint16_t )CC2420SpiP__SpiByte__write(0) << 8;
  *data |= CC2420SpiP__SpiByte__write(0);

  return status;
}

# 55 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__read(CC2420_RSSI, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 410 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__RssiResource__granted(void )
#line 410
{
  uint16_t data = 0;

#line 412
  CC2420ControlP__CSN__clr();
  CC2420ControlP__RSSI__read(&data);
  CC2420ControlP__CSN__set();

  CC2420ControlP__RssiResource__release();
  data += 0x7f;
  data &= 0x00ff;
  CC2420ControlP__ReadRssi__readDone(SUCCESS, data);
}

# 416 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__SpiResource__granted(void )
#line 416
{
  uint8_t cur_state;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 419
    {
      cur_state = CC2420TransmitP__m_state;
    }
#line 421
    __nesc_atomic_end(__nesc_atomic); }

  switch (cur_state) {
      case CC2420TransmitP__S_LOAD: 
        CC2420TransmitP__loadTXFIFO();
      break;

      case CC2420TransmitP__S_BEGIN_TRANSMIT: 
        CC2420TransmitP__attemptSend();
      break;

      case CC2420TransmitP__S_CANCEL: 
        CC2420TransmitP__CSN__clr();
      CC2420TransmitP__SFLUSHTX__strobe();
      CC2420TransmitP__CSN__set();
      CC2420TransmitP__releaseSpiResource();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 437
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
        }
#line 439
        __nesc_atomic_end(__nesc_atomic); }
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
      break;

      default: 
        CC2420TransmitP__releaseSpiResource();
      break;
    }
}

# 508 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__SpiResource__granted(void )
#line 508
{







  CC2420ReceiveP__receive();
}

# 367 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Resource__default__granted(uint8_t id)
#line 367
{
}

# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static void CC2420SpiP__Resource__granted(uint8_t arg_0x40ef5108){
#line 102
  switch (arg_0x40ef5108) {
#line 102
    case /*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID:
#line 102
      CC2420ControlP__SpiResource__granted();
#line 102
      break;
#line 102
    case /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID:
#line 102
      CC2420ControlP__SyncResource__granted();
#line 102
      break;
#line 102
    case /*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID:
#line 102
      CC2420ControlP__RssiResource__granted();
#line 102
      break;
#line 102
    case /*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID:
#line 102
      CC2420TransmitP__SpiResource__granted();
#line 102
      break;
#line 102
    case /*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID:
#line 102
      CC2420ReceiveP__SpiResource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      CC2420SpiP__Resource__default__granted(arg_0x40ef5108);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 358 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__grant__runTask(void )
#line 358
{
  uint8_t holder;

#line 360
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
    {
      holder = CC2420SpiP__m_holder;
    }
#line 362
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP__Resource__granted(holder);
}

# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_FSCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
inline static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_MDMCTRL0, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420ControlP__IEEEADR__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_IEEEADR, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 235 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error)
#line 235
{
}

# 704 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error)
#line 704
{
}

# 53 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static void CC2420ControlP__CC2420Config__syncDone(error_t error){
#line 53
  CC2420ReceiveP__CC2420Config__syncDone(error);
#line 53
  CC2420ActiveMessageP__CC2420Config__syncDone(error);
#line 53
}
#line 53
# 461 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__syncDone__runTask(void )
#line 461
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 462
    CC2420ControlP__m_sync_busy = FALSE;
#line 462
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__CC2420Config__syncDone(SUCCESS);
}

# 88 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 315 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Config__sync(void )
#line 315
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 316
    {
      if (CC2420ControlP__m_sync_busy) {
          {
            unsigned char __nesc_temp = 
#line 318
            FAIL;

            {
#line 318
              __nesc_atomic_end(__nesc_atomic); 
#line 318
              return __nesc_temp;
            }
          }
        }
#line 321
      CC2420ControlP__m_sync_busy = TRUE;
      if (CC2420ControlP__m_state == CC2420ControlP__S_XOSC_STARTED) {
          CC2420ControlP__SyncResource__request();
        }
      else 
#line 324
        {
          CC2420ControlP__syncDone__postTask();
        }
    }
#line 327
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 457
static inline void CC2420ControlP__sync__runTask(void )
#line 457
{
  CC2420ControlP__CC2420Config__sync();
}

# 244 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__BareSend__default__sendDone(message_t *msg, error_t error)
#line 244
{
}

# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__BareSend__sendDone(message_t * msg, error_t error){
#line 100
  CC2420TinyosNetworkP__BareSend__default__sendDone(msg, error);
#line 100
}
#line 100
# 110 "/home/ali/workspace/TinyOS-main/tos/interfaces/AMSend.nc"
inline static void CC2420ActiveMessageP__AMSend__sendDone(am_id_t arg_0x41264b88, message_t * msg, error_t error){
#line 110
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0x41264b88, msg, error);
#line 110
}
#line 110
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420TinyosNetworkP__Resource__release(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 212 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubSend__sendDone(message_t *msg, error_t result)
#line 212
{
  CC2420ActiveMessageP__RadioResource__release();
  CC2420ActiveMessageP__AMSend__sendDone(CC2420ActiveMessageP__AMPacket__type(msg), msg, result);
}

# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__ActiveSend__sendDone(message_t * msg, error_t error){
#line 100
  CC2420ActiveMessageP__SubSend__sendDone(msg, error);
#line 100
}
#line 100
# 148 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error)
#line 148
{
  if (CC2420TinyosNetworkP__m_busy_client == CC2420TinyosNetworkP__CLIENT_AM) {
      CC2420TinyosNetworkP__ActiveSend__sendDone(msg, error);
    }
  else 
#line 151
    {
      CC2420TinyosNetworkP__BareSend__sendDone(msg, error);
    }
}

# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static void UniqueSendP__Send__sendDone(message_t * msg, error_t error){
#line 100
  CC2420TinyosNetworkP__SubSend__sendDone(msg, error);
#line 100
}
#line 100
# 104 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error)
#line 104
{
  UniqueSendP__State__toIdle();
  UniqueSendP__Send__sendDone(msg, error);
}

# 100 "/home/ali/workspace/TinyOS-main/tos/interfaces/Send.nc"
inline static void CC2420CsmaP__Send__sendDone(message_t * msg, error_t error){
#line 100
  UniqueSendP__SubSend__sendDone(msg, error);
#line 100
}
#line 100
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__stopDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__stopDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )49U &= ~(0x01 << 6);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 53 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr();
#line 53
}
#line 53
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr();
}

# 41 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__clr(void ){
#line 41
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr();
#line 41
}
#line 41
# 208 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__stopVReg(void )
#line 208
{
  CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;
  CC2420ControlP__RSTN__clr();
  CC2420ControlP__VREN__clr();
  CC2420ControlP__RSTN__set();
  return SUCCESS;
}

# 63 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__stopVReg(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420ControlP__CC2420Power__stopVReg();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 69 "/home/ali/workspace/TinyOS-main/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 102 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 102
{
#line 102
  P1IFG &= ~(1 << 0);
}

# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void ){
#line 52
  HplMsp430InterruptP__Port10__clear();
#line 52
}
#line 52
# 94 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__disable(void )
#line 94
{
#line 94
  P1IE &= ~(1 << 0);
}

# 47 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void ){
#line 47
  HplMsp430InterruptP__Port10__disable();
#line 47
}
#line 47
# 69 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void )
#line 69
{
  /* atomic removed: atomic calls only */
#line 70
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  }
  return SUCCESS;
}

# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__disable(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 171 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__StdControl__stop(void )
#line 171
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STOPPED;
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__InterruptFIFOP__disable();
    }
#line 177
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )38U &= ~(0x01 << 2);
}

# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__GeneralIO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__selectIOFunc();
#line 99
}
#line 99
# 135 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents(void )
{
  * (volatile uint16_t * )356U &= ~0x0010;
}

# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents();
#line 58
}
#line 58
# 76 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/chips/cc2420/CC2420GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__disable(void )
#line 76
{
  /* atomic removed: atomic calls only */
#line 77
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__GeneralIO__selectIOFunc();
  }
}

# 66 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioCapture.nc"
inline static void CC2420TransmitP__CaptureSFD__disable(void ){
#line 66
  /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Capture__disable();
#line 66
}
#line 66
# 179 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__StdControl__stop(void )
#line 179
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {
      CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;
      CC2420TransmitP__BackoffTimer__stop();
      CC2420TransmitP__CaptureSFD__disable();
      CC2420TransmitP__SpiResource__release();
      CC2420TransmitP__CSN__set();
    }
#line 186
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 105 "/home/ali/workspace/TinyOS-main/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP__SubControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = CC2420TransmitP__StdControl__stop();
#line 105
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__StdControl__stop());
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 275 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__shutdown(void )
#line 275
{
  CC2420CsmaP__SubControl__stop();
  CC2420CsmaP__CC2420Power__stopVReg();
  CC2420CsmaP__stopDone_task__postTask();
}

#line 244
static inline void CC2420CsmaP__sendDone_task__runTask(void )
#line 244
{
  error_t packetErr;

#line 246
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 246
    packetErr = CC2420CsmaP__sendErr;
#line 246
    __nesc_atomic_end(__nesc_atomic); }
  if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPING)) {
      CC2420CsmaP__shutdown();
    }
  else {
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
    }

  CC2420CsmaP__Send__sendDone(CC2420CsmaP__m_msg, packetErr);
}

# 114 "AppP.nc"
static inline void AppP__AMControl__stopDone(error_t error)
#line 114
{
}

# 138 "/home/ali/workspace/TinyOS-main/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP__SplitControl__stopDone(error_t error){
#line 138
  AppP__AMControl__stopDone(error);
#line 138
}
#line 138
# 265 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__stopDone_task__runTask(void )
#line 265
{
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPED);
  CC2420CsmaP__SplitControl__stopDone(SUCCESS);
}

# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 109 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 96 "/home/ali/workspace/TinyOS-main/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 97
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 136 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 136
  unsigned long __nesc_result;
#line 136

#line 136
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 154 "/home/ali/workspace/TinyOS-main/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
inline static void AppP__MagTimer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(1U, dt);
#line 64
}
#line 64
# 260 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
static inline error_t Hmc5843P__Magnetometer__runContinuousConversion(void )
#line 260
{
  Hmc5843P__writeRegValue(2, 0x00);
  return SUCCESS;
}

# 54 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Magnetometer.nc"
inline static error_t AppP__Magnetometer__runContinuousConversion(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = Hmc5843P__Magnetometer__runContinuousConversion();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 8 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void Hmc5843P__HplI2C__setModeI2C(msp430_i2c_union_config_t *config){
#line 8
  HplMsp430I2C0P__HplI2C__setModeI2C(config);
#line 8
}
#line 8
# 106 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
static inline void Hmc5843P__Magnetometer__enableBus(void )
#line 106
{
  Hmc5843P__HplI2C__setModeI2C(&Hmc5843P__msp430_i2c_my_config);
}

# 84 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/hardware.h"
static inline  void TOSH_CLR_PROG_OUT_PIN()
#line 84
{
#line 84
  static volatile uint8_t r __asm ("0x0021");

#line 84
  r &= ~(1 << 1);
}

# 87 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
static inline error_t Hmc5843P__Init__init(void )
#line 87
{





  TOSH_MAKE_PROG_OUT_OUTPUT();
  TOSH_SEL_PROG_OUT_IOFUNC();
  TOSH_CLR_PROG_OUT_PIN();

  brief_pause(((unsigned long long )5000 * 4096 * 1024 / 1000000 - 2) / 3);

  Hmc5843P__Magnetometer__enableBus();

  Hmc5843P__testPhase = 0;

  return SUCCESS;
}

# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
inline static error_t AppP__MagInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Hmc5843P__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__clr();
#line 53
}
#line 53
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void )
#line 49
{
#line 49
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr();
}

# 41 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__clr(void ){
#line 41
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr();
#line 41
}
#line 41
# 75 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/LedsP.nc"
static inline void LedsP__Leds__led0On(void )
#line 75
{
  LedsP__Led0__clr();
  ;
#line 77
  ;
}

# 57 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/Leds.nc"
inline static void AppP__Leds__led0On(void ){
#line 57
  LedsP__Leds__led0On();
#line 57
}
#line 57
# 100 "AppP.nc"
static inline void AppP__AMControl__startDone(error_t error)
#line 100
{

  if (error) {
      AppP__Leds__led0On();
      return;
    }

  AppP__MagInit__init();

  AppP__Magnetometer__runContinuousConversion();

  AppP__MagTimer__startPeriodic(1000);
}

# 113 "/home/ali/workspace/TinyOS-main/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP__SplitControl__startDone(error_t error){
#line 113
  AppP__AMControl__startDone(error);
#line 113
}
#line 113
# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 188 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__release(void )
#line 188
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 189
    {
      CC2420ControlP__CSN__set();
      {
        unsigned char __nesc_temp = 
#line 191
        CC2420ControlP__SpiResource__release();

        {
#line 191
          __nesc_atomic_end(__nesc_atomic); 
#line 191
          return __nesc_temp;
        }
      }
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
}

# 120 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420ControlP__Resource__release();
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 260 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__rxOn(void )
#line 260
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 261
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_XOSC_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 263
            FAIL;

            {
#line 263
              __nesc_atomic_end(__nesc_atomic); 
#line 263
              return __nesc_temp;
            }
          }
        }
#line 265
      CC2420ControlP__SRXON__strobe();
    }
#line 266
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 90 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__rxOn(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = CC2420ControlP__CC2420Power__rxOn();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 86 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__enable(void )
#line 86
{
#line 86
  P1IE |= 1 << 0;
}

# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void ){
#line 42
  HplMsp430InterruptP__Port10__enable();
#line 42
}
#line 42
# 118 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__edge(bool l2h)
#line 118
{
  /* atomic removed: atomic calls only */
#line 119
  {
    if (l2h) {
#line 120
      P1IES &= ~(1 << 0);
      }
    else {
#line 121
      P1IES |= 1 << 0;
      }
  }
}

# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high){
#line 67
  HplMsp430InterruptP__Port10__edge(low_to_high);
#line 67
}
#line 67
# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising)
#line 52
{
  /* atomic removed: atomic calls only */
#line 53
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable();
  }
  return SUCCESS;
}





static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void )
#line 65
{
  return /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(FALSE);
}

# 54 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 157 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__StdControl__start(void )
#line 157
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 158
    {
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
      CC2420ReceiveP__receivingPacket = FALSE;




      CC2420ReceiveP__InterruptFIFOP__enableFallingEdge();
    }
#line 167
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 168 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__StdControl__start(void )
#line 168
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 169
    {
      CC2420TransmitP__CaptureSFD__captureRisingEdge();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__m_receiving = FALSE;
      CC2420TransmitP__abortSpiRelease = FALSE;
      CC2420TransmitP__m_tx_power = 0;
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 95 "/home/ali/workspace/TinyOS-main/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP__SubControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = CC2420TransmitP__StdControl__start();
#line 95
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__StdControl__start());
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 257 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__startDone_task__runTask(void )
#line 257
{
  CC2420CsmaP__SubControl__start();
  CC2420CsmaP__CC2420Power__rxOn();
  CC2420CsmaP__Resource__release();
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
  CC2420CsmaP__SplitControl__startDone(SUCCESS);
}

# 128 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__disableUart(void ){
#line 128
  HplMsp430Usart0P__Usart__disableUart();
#line 128
}
#line 128
# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UTXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__URXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 99
}
#line 99
# 158 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__disableSpi(void ){
#line 158
  HplMsp430Usart0P__Usart__disableSpi();
#line 158
}
#line 158
# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

# 99 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc();
#line 99
}
#line 99
# 61 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__makeInput(void )
#line 61
{
#line 61
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
    * (volatile uint8_t * )26U &= ~(0x01 << 1);
#line 61
    __nesc_atomic_end(__nesc_atomic); }
}

# 78 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430I2C0P__SIMO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__makeInput();
#line 78
}
#line 78
#line 92
inline static void HplMsp430I2C0P__SIMO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 92
}
#line 92
# 61 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__makeInput(void )
#line 61
{
#line 61
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
    * (volatile uint8_t * )26U &= ~(0x01 << 3);
#line 61
    __nesc_atomic_end(__nesc_atomic); }
}

# 78 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430I2C0P__UCLK__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__makeInput();
#line 78
}
#line 78
#line 92
inline static void HplMsp430I2C0P__UCLK__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 92
}
#line 92
# 187 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/Msp430I2CP.nc"
static inline error_t Msp430I2CP__I2CBasicAddr__write(i2c_flags_t flags, 
uint16_t _addr, uint8_t _length, 
uint8_t *_data)
#line 189
{

  uint8_t _state;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 193
    {
      _state = Msp430I2CP__stateI2C;
      if (_state == Msp430I2CP__IDLE) {
          Msp430I2CP__stateI2C = Msp430I2CP__PACKET_WRITE;
        }
    }
#line 198
    __nesc_atomic_end(__nesc_atomic); }

  if (_state == Msp430I2CP__IDLE) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 202
        {

          Msp430I2CP__result = FAIL;

          U0CTL &= ~0x01;

          I2CSA = _addr;

          Msp430I2CP__length = _length;
          Msp430I2CP__data = _data;
          Msp430I2CP__ptr = 0;

          U0CTL |= 0x02;

          I2CNDAT = _length;


          U0CTL |= 0x01;


          I2CTCTL |= 0x08;


          Msp430I2CP__I2CIE = 0x20 | 0x02;
          I2CIFG = 0;


          I2CTCTL |= 0x02 | 0x01;
        }
#line 230
        __nesc_atomic_end(__nesc_atomic); }

      return SUCCESS;
    }

  return FAIL;
}

# 92 "/home/ali/workspace/TinyOS-main/tos/interfaces/I2CPacket.nc"
inline static error_t Hmc5843P__I2CPacket__write(i2c_flags_t flags, uint16_t addr, uint8_t length, uint8_t * data){
#line 92
  unsigned char __nesc_result;
#line 92

#line 92
  __nesc_result = Msp430I2CP__I2CBasicAddr__write(flags, addr, length, data);
#line 92

#line 92
  return __nesc_result;
#line 92
}
#line 92
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 149 "AppP.nc"
static inline void AppP__Magnetometer__writeDone(error_t success)
#line 149
{
}

# 71 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Magnetometer.nc"
inline static void Hmc5843P__Magnetometer__writeDone(error_t success){
#line 71
  AppP__Magnetometer__writeDone(success);
#line 71
}
#line 71
# 337 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
static inline void Hmc5843P__I2CPacket__writeDone(error_t success, uint16_t addr, uint8_t length, uint8_t *data)
#line 337
{
  Hmc5843P__Magnetometer__writeDone(success);
}

# 112 "/home/ali/workspace/TinyOS-main/tos/interfaces/I2CPacket.nc"
inline static void Msp430I2CP__I2CBasicAddr__writeDone(error_t error, uint16_t addr, uint8_t length, uint8_t * data){
#line 112
  Hmc5843P__I2CPacket__writeDone(error, addr, length, data);
#line 112
}
#line 112
# 88 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/Msp430I2CP.nc"
static inline void Msp430I2CP__writeDone__runTask(void )
#line 88
{

  error_t _result;
  uint8_t _length;
  uint8_t *_data;
  uint16_t _addr;

  _result = Msp430I2CP__result;
  _length = Msp430I2CP__length;
  _data = Msp430I2CP__data;
  _addr = I2CSA;



  while (I2CDCTL & 0x20) ;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 104
    Msp430I2CP__stateI2C = Msp430I2CP__IDLE;
#line 104
    __nesc_atomic_end(__nesc_atomic); }
  Msp430I2CP__I2CBasicAddr__writeDone(_result, _addr, _length, _data);
}

# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__toggle(void )
#line 58
{
#line 58
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 58
    * (volatile uint8_t * )29U ^= 0x01 << 2;
#line 58
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void ){
#line 58
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__toggle();
#line 58
}
#line 58
# 50 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void )
#line 50
{
#line 50
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle();
}

# 42 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__toggle(void ){
#line 42
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle();
#line 42
}
#line 42
# 100 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/LedsP.nc"
static inline void LedsP__Leds__led1Toggle(void )
#line 100
{
  LedsP__Led1__toggle();
  ;
#line 102
  ;
}

# 84 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/Leds.nc"
inline static void AppP__Leds__led1Toggle(void ){
#line 84
  LedsP__Leds__led1Toggle();
#line 84
}
#line 84
# 97 "/home/ali/workspace/TinyOS-main/tos/lib/diagmsg/DiagMsg.nc"
inline static void AppP__DiagMsg__send(void ){
#line 97
  DiagMsgP__DiagMsg__send();
#line 97
}
#line 97
# 221 "/home/ali/workspace/TinyOS-main/tos/lib/diagmsg/DiagMsgP.nc"
static inline void DiagMsgP__DiagMsg__int16s(const int16_t *value, uint8_t len)
#line 221
{
#line 221
  DiagMsgP__copyArray(sizeof(int16_t ), DiagMsgP__TYPE_INT16, value, len);
}

# 78 "/home/ali/workspace/TinyOS-main/tos/lib/diagmsg/DiagMsg.nc"
inline static void AppP__DiagMsg__int16s(const int16_t *value, uint8_t len){
#line 78
  DiagMsgP__DiagMsg__int16s(value, len);
#line 78
}
#line 78
#line 49
inline static bool AppP__DiagMsg__record(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = DiagMsgP__DiagMsg__record();
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 124 "AppP.nc"
static inline void AppP__sendData(void )
#line 124
{

  if (AppP__DiagMsg__record()) {

      AppP__DiagMsg__int16s(AppP__mag, 3);
      AppP__DiagMsg__send();
    }
}

# 116 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430I2C0P.nc"
static inline void HplMsp430I2C0P__HplI2C__disableI2C(void )
#line 116
{
#line 116
  HplMsp430I2C0P__U0CTL &= ~0x01;
}

# 15 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void Hmc5843P__HplI2C__disableI2C(void ){
#line 15
  HplMsp430I2C0P__HplI2C__disableI2C();
#line 15
}
#line 15
# 110 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
static inline void Hmc5843P__Magnetometer__disableBus(void )
#line 110
{
  Hmc5843P__HplI2C__disableI2C();
}

# 41 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Magnetometer.nc"
inline static void AppP__Magnetometer__disableBus(void ){
#line 41
  Hmc5843P__Magnetometer__disableBus();
#line 41
}
#line 41
# 293 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
static inline int16_t Hmc5843P__twos_comp_pack(uint8_t up, uint8_t low)
{
  int16_t out;
  uint16_t uout;

  uout = up;
  uout = uout << 8;
  uout |= low;

  out = (int16_t )uout;

  return out;
}


static inline void Hmc5843P__Magnetometer__convertRegistersToData(uint8_t *readBuf, int16_t *data)
#line 308
{
  uint8_t *src;
  register uint8_t i;

  src = readBuf;


  for (i = 0; i < 3; i++) {
      data[i] = Hmc5843P__twos_comp_pack(*src, *(src + 1));
      src += 2;
    }
}

# 66 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Magnetometer.nc"
inline static void AppP__Magnetometer__convertRegistersToData(uint8_t *readBuf, int16_t *data){
#line 66
  Hmc5843P__Magnetometer__convertRegistersToData(readBuf, data);
#line 66
}
#line 66
# 133 "AppP.nc"
static inline void AppP__Magnetometer__readDone(uint8_t *data, error_t error)
#line 133
{

  if (error) {
      AppP__Leds__led0On();
      return;
    }

  AppP__Magnetometer__convertRegistersToData(data, AppP__mag);

  AppP__Magnetometer__disableBus();

  AppP__sendData();

  AppP__Leds__led1Toggle();
}

# 69 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Magnetometer.nc"
inline static void Hmc5843P__Magnetometer__readDone(uint8_t *data, error_t success){
#line 69
  AppP__Magnetometer__readDone(data, success);
#line 69
}
#line 69
# 333 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
static inline void Hmc5843P__I2CPacket__readDone(error_t success, uint16_t addr, uint8_t length, uint8_t *data)
#line 333
{
  Hmc5843P__Magnetometer__readDone(data, success);
}

# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/I2CPacket.nc"
inline static void Msp430I2CP__I2CBasicAddr__readDone(error_t error, uint16_t addr, uint8_t length, uint8_t * data){
#line 102
  Hmc5843P__I2CPacket__readDone(error, addr, length, data);
#line 102
}
#line 102
# 72 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/Msp430I2CP.nc"
static inline void Msp430I2CP__readDone__runTask(void )
#line 72
{

  error_t _result;
  uint8_t _length;
  uint8_t *_data;
  uint16_t _addr;

  _result = Msp430I2CP__result;
  _length = Msp430I2CP__length;
  _data = Msp430I2CP__data;
  _addr = I2CSA;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 84
    Msp430I2CP__stateI2C = Msp430I2CP__IDLE;
#line 84
    __nesc_atomic_end(__nesc_atomic); }
  Msp430I2CP__I2CBasicAddr__readDone(_result, _addr, _length, _data);
}

# 103 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 58 "/home/ali/workspace/TinyOS-main/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 94
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 129 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 129
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 129
}
#line 129
# 65 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 73 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 102 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 73 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 71 "/home/ali/workspace/TinyOS-main/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 72
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 78
}
#line 78
# 100 "/home/ali/workspace/TinyOS-main/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 135
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

#line 164
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 78 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
inline static void Hmc5843P__testTimer__stop(void ){
#line 78
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(0U);
#line 78
}
#line 78
# 269 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
static inline error_t Hmc5843P__Magnetometer__readData(void )
#line 269
{
  Hmc5843P__readValues(7, Hmc5843P__readbuff);
  return SUCCESS;
}

# 64 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
inline static void Hmc5843P__testTimer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(0U, dt);
#line 64
}
#line 64
# 182 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
static inline void Hmc5843P__Magnetometer__selfTest(void )
#line 182
{
  switch (Hmc5843P__testPhase++) {
      case 0: 
        Hmc5843P__testTimer__startPeriodic(8);
      Hmc5843P__writeRegValue(0, 0x11);
      break;
      case 1: 
        Hmc5843P__writeRegValue(2, 0x01);
      break;
      case 2: 
        Hmc5843P__Magnetometer__readData();
      break;
      case 3: 
        Hmc5843P__writeRegValue(0, 0x10);
      break;
      default: 
        Hmc5843P__testTimer__stop();
      Hmc5843P__testPhase = 0;
      break;
    }
}

static inline void Hmc5843P__testTimer__fired(void )
#line 204
{
  Hmc5843P__Magnetometer__selfTest();
}

# 57 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Magnetometer.nc"
inline static error_t AppP__Magnetometer__readData(void ){
#line 57
  unsigned char __nesc_result;
#line 57

#line 57
  __nesc_result = Hmc5843P__Magnetometer__readData();
#line 57

#line 57
  return __nesc_result;
#line 57
}
#line 57
#line 39
inline static void AppP__Magnetometer__enableBus(void ){
#line 39
  Hmc5843P__Magnetometer__enableBus();
#line 39
}
#line 39
# 116 "AppP.nc"
static inline void AppP__MagTimer__fired(void )
#line 116
{

  AppP__Magnetometer__enableBus();

  AppP__Magnetometer__readData();
}

# 204 "/home/ali/workspace/TinyOS-main/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 83 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x40cac9f0){
#line 83
  switch (arg_0x40cac9f0) {
#line 83
    case 0U:
#line 83
      Hmc5843P__testTimer__fired();
#line 83
      break;
#line 83
    case 1U:
#line 83
      AppP__MagTimer__fired();
#line 83
      break;
#line 83
    default:
#line 83
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x40cac9f0);
#line 83
      break;
#line 83
    }
#line 83
}
#line 83
# 108 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/Msp430I2CP.nc"
static inline error_t Msp430I2CP__I2CBasicAddr__read(i2c_flags_t flags, 
uint16_t _addr, uint8_t _length, 
uint8_t *_data)
#line 110
{
  uint8_t _state;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 113
    {
      _state = Msp430I2CP__stateI2C;
      if (_state == Msp430I2CP__IDLE) {
          Msp430I2CP__stateI2C = Msp430I2CP__PACKET_READ;
        }
    }
#line 118
    __nesc_atomic_end(__nesc_atomic); }

  if (_state == Msp430I2CP__IDLE) {


      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 123
        {
          Msp430I2CP__result = FAIL;


          U0CTL &= ~0x01;

          I2CSA = _addr;

          Msp430I2CP__length = _length;
          Msp430I2CP__data = _data;
          Msp430I2CP__ptr = 0;

          U0CTL |= 0x02;

          I2CNDAT = _length;


          U0CTL |= 0x01;


          I2CTCTL &= ~0x08;


          Msp430I2CP__I2CIE = 0x10 | 0x02;
          I2CIFG = 0;


          I2CTCTL |= 0x02 | 0x01;
        }
#line 151
        __nesc_atomic_end(__nesc_atomic); }

      return SUCCESS;
    }

  return FAIL;
}

# 76 "/home/ali/workspace/TinyOS-main/tos/interfaces/I2CPacket.nc"
inline static error_t Hmc5843P__I2CPacket__read(i2c_flags_t flags, uint16_t addr, uint8_t length, uint8_t * data){
#line 76
  unsigned char __nesc_result;
#line 76

#line 76
  __nesc_result = Msp430I2CP__I2CBasicAddr__read(flags, addr, length, data);
#line 76

#line 76
  return __nesc_result;
#line 76
}
#line 76
# 139 "/home/ali/workspace/TinyOS-main/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 83 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 83
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 83
}
#line 83
# 91 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 93
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type __nesc_temp = 
#line 93
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

      {
#line 93
        __nesc_atomic_end(__nesc_atomic); 
#line 93
        return __nesc_temp;
      }
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }
}

# 116 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 116
  unsigned long __nesc_result;
#line 116

#line 116
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 116

#line 116
  return __nesc_result;
#line 116
}
#line 116
# 74 "/home/ali/workspace/TinyOS-main/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 202 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/home/ali/workspace/TinyOS-main/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x40a22310){
#line 102
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x40a22310);
#line 102
}
#line 102
# 216 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/home/ali/workspace/TinyOS-main/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x40a42148){
#line 59
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x40a42148);
#line 59
}
#line 59
# 190 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 95 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/LedsP.nc"
static inline void LedsP__Leds__led1Off(void )
#line 95
{
  LedsP__Led1__set();
  ;
#line 97
  ;
}

# 78 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/Leds.nc"
inline static void AppP__Leds__led1Off(void ){
#line 78
  LedsP__Leds__led1Off();
#line 78
}
#line 78
# 80 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/LedsP.nc"
static inline void LedsP__Leds__led0Off(void )
#line 80
{
  LedsP__Led0__set();
  ;
#line 82
  ;
}

# 62 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/Leds.nc"
inline static void AppP__Leds__led0Off(void ){
#line 62
  LedsP__Leds__led0Off();
#line 62
}
#line 62
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t AppP__SDtest__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AppP__SDtest);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 330 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isTxEmpty(void )
#line 330
{
  if (HplMsp430Usart0P__U0TCTL & 0x01) {
      return TRUE;
    }
  return FALSE;
}

# 213 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool SDP__Usart__isTxEmpty(void ){
#line 213
  unsigned char __nesc_result;
#line 213

#line 213
  __nesc_result = HplMsp430Usart0P__Usart__isTxEmpty();
#line 213

#line 213
  return __nesc_result;
#line 213
}
#line 213
# 279 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SDP.nc"
static inline uint8_t SDP__getXXResponse(const uint8_t resp)
#line 279
{
  register uint16_t i;
  uint8_t response;

  for (i = 0; i < 1001; i++) 
    if ((response = SDP__spiSendByte(0xff)) == resp) {
      break;
      }
  return response;
}

#line 371
static inline error_t SDP__setBlockLength(const uint16_t len)
#line 371
{
  TOSH_CLR_SD_CS_N_PIN();
#line 372
  ;

  SDP__sendCmd(0x50, len, 0xff);


  if (SDP__getResponse() != 0x00) {
      SDP__cardInit();
      SDP__sendCmd(0x50, len, 0xff);
      SDP__getResponse();
    }

  while (SDP__Usart__isTxEmpty() == FALSE) ;
#line 383
  ;
#line 383
  TOSH_SET_SD_CS_N_PIN();
#line 383
  ;


  SDP__spiSendByte(0xff);

  return 0x00;
}






static inline error_t SDP__read_block(const uint32_t address, const uint16_t count, uint8_t *buffer)
#line 396
{
  register uint16_t i = 0;
  uint8_t rvalue = 0x02;


  if (SDP__setBlockLength(count) == 0x00) {
      TOSH_CLR_SD_CS_N_PIN();
#line 402
      ;

      SDP__sendCmd(0x51, address, 0xff);



      if (SDP__getResponse() == 0x00) {

          if (SDP__getXXResponse(0xfe) == 0xfe) {


              for (i = 0; i < count; i++) 
                buffer[i] = SDP__spiSendByte(0xff);


              SDP__spiSendByte(0xff);
              SDP__spiSendByte(0xff);
              rvalue = 0x00;
            }
          else {

              rvalue = 0x03;
            }
        }
      else {

          rvalue = 0x02;
        }
    }
  else {
      rvalue = 0x01;
    }

  while (SDP__Usart__isTxEmpty() == FALSE) ;
#line 435
  ;
#line 435
  TOSH_SET_SD_CS_N_PIN();
#line 435
  ;
  SDP__spiSendByte(0xff);

  return rvalue;
}

# 149 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/hardware.h"
static inline  uint8_t TOSH_READ_DOCK_N_PIN()
#line 149
{
#line 149
  static volatile uint8_t r __asm ("0x0028");

#line 149
  return r & (1 << 3);
}

# 445 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SDP.nc"
static inline error_t SDP__SD__readBlock(const uint32_t sector, uint8_t *buffer)
#line 445
{
  if (!TOSH_READ_DOCK_N_PIN()) {
    return 0x04;
    }
  return SDP__read_block(sector * 512, 512, buffer);
}

# 54 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SD.nc"
inline static error_t AppP__SD__readBlock(const uint32_t sector, uint8_t *buffer){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = SDP__SD__readBlock(sector, buffer);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 209 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port23__clear(void )
#line 209
{
#line 209
  P2IFG &= ~(1 << 3);
}

# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void SDP__DockInterrupt__clear(void ){
#line 52
  HplMsp430InterruptP__Port23__clear();
#line 52
}
#line 52
# 193 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port23__enable(void )
#line 193
{
#line 193
  P2IE |= 1 << 3;
}

# 42 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void SDP__DockInterrupt__enable(void ){
#line 42
  HplMsp430InterruptP__Port23__enable();
#line 42
}
#line 42
# 86 "AppP.nc"
static inline void AppP__SD__available(void )
#line 86
{
}

# 69 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SD.nc"
inline static void SDP__SD__available(void ){
#line 69
  AppP__SD__available();
#line 69
}
#line 69
# 67 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void SDP__DockInterrupt__edge(bool low_to_high){
#line 67
  HplMsp430InterruptP__Port23__edge(low_to_high);
#line 67
}
#line 67
# 91 "AppP.nc"
static inline void AppP__SD__unavailable(void )
#line 91
{
}

# 75 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SD.nc"
inline static void SDP__SD__unavailable(void ){
#line 75
  AppP__SD__unavailable();
#line 75
}
#line 75
# 113 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/hardware.h"
static inline  void TOSH_CLR_SW_SD_PWR_N_PIN()
#line 113
{
#line 113
  static volatile uint8_t r __asm ("0x001D");

#line 113
  r &= ~(1 << 5);
}

# 151 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SDP.nc"
static inline error_t SDP__StdControl__start(void )
#line 151
{
  TOSH_CLR_SW_SD_PWR_N_PIN();







  if (!TOSH_READ_DOCK_N_PIN()) {
      SDP__DockInterrupt__edge(TRUE);
      SDP__powerCycle();
      SDP__SD__unavailable();
    }
  else {
      SDP__DockInterrupt__edge(FALSE);

      SDP__cardInit();

      SDP__SD__available();
    }

  SDP__DockInterrupt__enable();
  SDP__DockInterrupt__clear();

  return SUCCESS;
}

# 95 "/home/ali/workspace/TinyOS-main/tos/interfaces/StdControl.nc"
inline static error_t AppP__SDStdControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = SDP__StdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )29U &= ~(0x01 << 2);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 53 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr();
#line 53
}
#line 53
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void )
#line 49
{
#line 49
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr();
}

# 41 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__clr(void ){
#line 41
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr();
#line 41
}
#line 41
# 90 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/LedsP.nc"
static inline void LedsP__Leds__led1On(void )
#line 90
{
  LedsP__Led1__clr();
  ;
#line 92
  ;
}

# 73 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/Leds.nc"
inline static void AppP__Leds__led1On(void ){
#line 73
  LedsP__Leds__led1On();
#line 73
}
#line 73
# 58 "AppP.nc"
static inline void AppP__SDtest__runTask(void )
#line 58
{

  error_t error;

  uint32_t size;

  AppP__Leds__led1On();

  error = AppP__SDStdControl__start();

  if (error != SUCCESS) {
      AppP__Leds__led0On();
      AppP__SDtest__postTask();
      return;
    }

  error = AppP__SD__readBlock(0, AppP__buffer);

  if (error) {
      AppP__Leds__led0On();
      AppP__SDtest__postTask();
      return;
    }

  AppP__Leds__led0Off();
  AppP__Leds__led1Off();
}

# 158 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void SDP__Usart__disableSpi(void ){
#line 158
  HplMsp430Usart0P__Usart__disableSpi();
#line 158
}
#line 158
# 128 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/hardware.h"
static inline  void TOSH_CLR_SD_DO_PIN()
#line 128
{
#line 128
  static volatile uint8_t r __asm ("0x0019");

#line 128
  r &= ~(1 << 2);
}

# 368 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableRxIntr(void )
#line 368
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 369
    {
      HplMsp430Usart0P__IFG1 &= ~(1 << 6);
      HplMsp430Usart0P__IE1 |= 1 << 6;
    }
#line 372
    __nesc_atomic_end(__nesc_atomic); }
}

# 180 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void SDP__Usart__enableRxIntr(void ){
#line 180
  HplMsp430Usart0P__Usart__enableRxIntr();
#line 180
}
#line 180
# 150 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control)
#line 150
{
  U0MCTL = control;
}

# 90 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void SDP__Usart__setUmctl(uint8_t umctl){
#line 90
  HplMsp430Usart0P__Usart__setUmctl(umctl);
#line 90
}
#line 90
#line 80
inline static void SDP__Usart__setUbr(uint16_t ubr){
#line 80
  HplMsp430Usart0P__Usart__setUbr(ubr);
#line 80
}
#line 80
# 364 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableIntr(void )
#line 364
{
  HplMsp430Usart0P__IE1 &= ~((1 << 7) | (1 << 6));
}

#line 352
static inline void HplMsp430Usart0P__Usart__clrIntr(void )
#line 352
{
  HplMsp430Usart0P__IFG1 &= ~((1 << 7) | (1 << 6));
}

#line 158
static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset)
#line 158
{
  if (reset) {
      U0CTL = 0x01;
    }
  else {
      U0CTL &= ~0x01;
    }
}

# 92 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 92
}
#line 92
# 65 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 92 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 92
}
#line 92
inline static void HplMsp430Usart0P__SIMO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 92
}
#line 92
# 245 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableSpi(void )
#line 245
{
  /* atomic removed: atomic calls only */
#line 246
  {
    HplMsp430Usart0P__SIMO__selectModuleFunc();
    HplMsp430Usart0P__SOMI__selectModuleFunc();
    HplMsp430Usart0P__UCLK__selectModuleFunc();
  }
  HplMsp430Usart0P__ME1 |= 1 << 6;
}










static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config)
#line 263
{

  U0CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  HplMsp430Usart0P__U0TCTL = config->spiRegisters.utctl;

  HplMsp430Usart0P__Usart__setUbr(config->spiRegisters.ubr);
  HplMsp430Usart0P__Usart__setUmctl(0x00);
}

# 97 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 59 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430I2C0P.nc"
static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void )
#line 59
{
  /* atomic removed: atomic calls only */
#line 60
  {
    HplMsp430I2C0P__U0CTL &= ~((0x20 | 0x04) | 0x01);
    HplMsp430I2C0P__HplUsart__resetUsart(TRUE);
  }
}

# 7 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void HplMsp430Usart0P__HplI2C__clearModeI2C(void ){
#line 7
  HplMsp430I2C0P__HplI2C__clearModeI2C();
#line 7
}
#line 7
# 272 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config)
#line 272
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 274
    {
      HplMsp430Usart0P__Usart__resetUsart(TRUE);
      HplMsp430Usart0P__HplI2C__clearModeI2C();
      HplMsp430Usart0P__Usart__disableUart();
      HplMsp430Usart0P__configSpi(config);
      HplMsp430Usart0P__Usart__enableSpi();
      HplMsp430Usart0P__Usart__resetUsart(FALSE);
      HplMsp430Usart0P__Usart__clrIntr();
      HplMsp430Usart0P__Usart__disableIntr();
    }
#line 283
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 168 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void SDP__Usart__setModeSpi(msp430_spi_union_config_t *config){
#line 168
  HplMsp430Usart0P__Usart__setModeSpi(config);
#line 168
}
#line 168
# 206 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SDP.nc"
static inline void SDP__initSPI(void )
#line 206
{
  msp430_spi_union_config_t *config;

  TOSH_MAKE_SD_CS_N_OUTPUT();
  TOSH_SEL_SD_CS_N_IOFUNC();

  config = &msp430_spi_default_config;

  SDP__Usart__setModeSpi(config);







  SDP__Usart__setUbr(UBR_1MHZ_115200);
  SDP__Usart__setUmctl(UMCTL_1MHZ_115200);





  SDP__Usart__enableRxIntr();

  TOSH_SET_SD_CS_N_PIN();

  while (SDP__Usart__isTxEmpty() == FALSE) ;
}

# 389 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data)
#line 389
{
  /* atomic removed: atomic calls only */
#line 390
  HplMsp430Usart0P__U0TXBUF = data;
}

# 224 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void SDP__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart0P__Usart__tx(data);
#line 224
}
#line 224
# 337 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void )
#line 337
{
  if (HplMsp430Usart0P__IFG1 & (1 << 6)) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool SDP__Usart__isRxIntrPending(void ){
#line 192
  unsigned char __nesc_result;
#line 192

#line 192
  __nesc_result = HplMsp430Usart0P__Usart__isRxIntrPending();
#line 192

#line 192
  return __nesc_result;
#line 192
}
#line 192
# 393 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static inline uint8_t HplMsp430Usart0P__Usart__rx(void )
#line 393
{
  uint8_t value;

#line 395
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 395
    value = U0RXBUF;
#line 395
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

# 231 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static uint8_t SDP__Usart__rx(void ){
#line 231
  unsigned char __nesc_result;
#line 231

#line 231
  __nesc_result = HplMsp430Usart0P__Usart__rx();
#line 231

#line 231
  return __nesc_result;
#line 231
}
#line 231
# 325 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SDP.nc"
static inline error_t SDP__setIdle(void )
#line 325
{
  char response;

#line 327
  TOSH_CLR_SD_CS_N_PIN();
#line 327
  ;


  SDP__sendCmd(0x40, 0, 0x95);


  if ((response = SDP__getResponse()) != 0x01) {
    return 0x04;
    }
  do {
      while (SDP__Usart__isTxEmpty() == FALSE) ;
#line 337
      ;
#line 337
      TOSH_SET_SD_CS_N_PIN();
#line 337
      ;
      SDP__spiSendByte(0xff);
      TOSH_CLR_SD_CS_N_PIN();
#line 339
      ;
      SDP__sendCmd(0x41, 0x00, 0xff);
    }
  while ((
#line 341
  response = SDP__getResponse()) == 0x01);

  while (SDP__Usart__isTxEmpty() == FALSE) ;
#line 343
  ;
#line 343
  TOSH_SET_SD_CS_N_PIN();
#line 343
  ;
  SDP__spiSendByte(0xff);

  return 0x00;
}

# 55 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 55
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4387 {
#line 57
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 47 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 93 "/home/ali/workspace/TinyOS-main/tos/system/ActiveMessageAddressC.nc"
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void )
#line 93
{
  am_group_t myGroup;

  /* atomic removed: atomic calls only */
#line 95
  myGroup = ActiveMessageAddressC__group;
  return myGroup;
}

# 55 "/home/ali/workspace/TinyOS-main/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amGroup();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
#line 50
inline static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 85 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput();
}

# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 7;
}

# 85 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput();
}

# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 85 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput();
}

# 46 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 46
}
#line 46
# 129 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Init__init(void )
#line 129
{
  CC2420ControlP__CSN__makeOutput();
  CC2420ControlP__RSTN__makeOutput();
  CC2420ControlP__VREN__makeOutput();

  CC2420ControlP__m_short_addr = CC2420ControlP__ActiveMessageAddress__amAddress();
  CC2420ControlP__m_ext_addr = CC2420ControlP__LocalIeeeEui64__getId();
  CC2420ControlP__m_pan = CC2420ControlP__ActiveMessageAddress__amGroup();
  CC2420ControlP__m_tx_power = 31;
  CC2420ControlP__m_channel = 11;





  CC2420ControlP__addressRecognition = TRUE;





  CC2420ControlP__hwAddressRecognition = FALSE;






  CC2420ControlP__autoAckEnabled = TRUE;






  CC2420ControlP__hwAutoAckDefault = FALSE;



  return SUCCESS;
}

# 81 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
static inline error_t StateImplP__Init__init(void )
#line 81
{
  int i;

#line 83
  for (i = 0; i < 4U; i++) {
      StateImplP__state[i] = StateImplP__S_IDLE;
    }
  return SUCCESS;
}

# 55 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init(void )
#line 55
{
  memset(/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY, sizeof /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ);
  return SUCCESS;
}

# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4388 {
#line 57
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )388U = /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__compareControl();
}

# 47 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 61 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )34U &= ~(0x01 << 2);
}

# 78 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__makeInput();
#line 78
}
#line 78
# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput();
}

# 44 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__SFD__makeInput(void ){
#line 44
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput();
#line 44
}
#line 44


inline static void CC2420TransmitP__CSN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 46
}
#line 46
# 61 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )42U &= ~(0x01 << 7);
}

# 78 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__makeInput();
#line 78
}
#line 78
# 52 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput();
}

# 44 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CCA__makeInput(void ){
#line 44
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput();
#line 44
}
#line 44
# 160 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__Init__init(void )
#line 160
{
  CC2420TransmitP__CCA__makeInput();
  CC2420TransmitP__CSN__makeOutput();
  CC2420TransmitP__SFD__makeInput();
  return SUCCESS;
}

# 151 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__Init__init(void )
#line 151
{
  CC2420ReceiveP__m_p_rx_buf = &CC2420ReceiveP__m_rx_buf;
  return SUCCESS;
}

# 55 "/home/ali/workspace/TinyOS-main/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC__Init__init(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  RandomMlcgC__seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 52 "/home/ali/workspace/TinyOS-main/tos/interfaces/Random.nc"
inline static uint16_t UniqueSendP__Random__rand16(void ){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = RandomMlcgC__Random__rand16();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 62 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline error_t UniqueSendP__Init__init(void )
#line 62
{
  UniqueSendP__localSendId = UniqueSendP__Random__rand16();
  return SUCCESS;
}

# 71 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline error_t UniqueReceiveP__Init__init(void )
#line 71
{
  int i;

#line 73
  for (i = 0; i < 4; i++) {
      UniqueReceiveP__receivedMessages[i].source = (am_addr_t )0xFFFF;
      UniqueReceiveP__receivedMessages[i].dsn = 0;
    }
  return SUCCESS;
}

# 55 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void )
#line 55
{
  memset(/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 86 "/home/ali/workspace/TinyOS-main/tos/lib/diagmsg/DiagMsgP.nc"
static inline error_t DiagMsgP__Init__init(void )
{
  DiagMsgP__state = DiagMsgP__STATE_READY;
  DiagMsgP__recording = DiagMsgP__msgs;
  DiagMsgP__sending = 0;

  return SUCCESS;
}

# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = DiagMsgP__Init__init();
#line 62
  __nesc_result = ecombine(__nesc_result, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, UniqueReceiveP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, UniqueSendP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, RandomMlcgC__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420TransmitP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, StateImplP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420ControlP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 66 "/home/ali/workspace/TinyOS-main/tos/lib/timer/Alarm.nc"
inline static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt){
#line 66
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__start(dt);
#line 66
}
#line 66
# 56 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )49U |= 0x01 << 6;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 48
}
#line 48
# 48 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set();
}

# 40 "/home/ali/workspace/TinyOS-main/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__set(void ){
#line 40
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set();
#line 40
}
#line 40
# 196 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startVReg(void )
#line 196
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 197
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STOPPED) {
          {
            unsigned char __nesc_temp = 
#line 199
            FAIL;

            {
#line 199
              __nesc_atomic_end(__nesc_atomic); 
#line 199
              return __nesc_temp;
            }
          }
        }
#line 201
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTING;
    }
#line 202
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__VREN__set();
  CC2420ControlP__StartupTimer__start(CC2420_TIME_VREN);
  return SUCCESS;
}

# 51 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startVReg(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420ControlP__CC2420Power__startVReg();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 45 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
inline static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(1U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline error_t CC2420CsmaP__SplitControl__start(void )
#line 81
{
  if (CC2420CsmaP__SplitControlState__requestState(CC2420CsmaP__S_STARTING) == SUCCESS) {
      CC2420CsmaP__CC2420Power__startVReg();
      return SUCCESS;
    }
  else {
#line 86
    if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
        return EALREADY;
      }
    else {
#line 89
      if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTING)) {
          return SUCCESS;
        }
      }
    }
#line 93
  return EBUSY;
}

# 104 "/home/ali/workspace/TinyOS-main/tos/interfaces/SplitControl.nc"
inline static error_t AppP__AMControl__start(void ){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  __nesc_result = CC2420CsmaP__SplitControl__start();
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 93 "AppP.nc"
static inline void AppP__Boot__booted(void )
#line 93
{



  AppP__AMControl__start();
}

# 60 "/home/ali/workspace/TinyOS-main/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 60
  AppP__Boot__booted();
#line 60
}
#line 60
# 412 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 63 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 63
{
  return MSP430_POWER_LPM3;
}

# 62 "/home/ali/workspace/TinyOS-main/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 74 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 74
{
  mcu_power_t pState = MSP430_POWER_LPM4;









  if ((((((
#line 77
  TA0CCTL0 & 0x0010 || 
  TA0CCTL1 & 0x0010) || 
  TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20)) || (
  ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x0010) {
      if (ADC12CTL1 & (2 << 3)) {

          if (ADC12CTL1 & (1 << 3)) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 99
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 100
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 400 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 400
{
  return m1 < m2 ? m1 : m2;
}

# 112 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 112
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 117
{
  uint16_t temp;

#line 119
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 76 "/home/ali/workspace/TinyOS-main/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 76
  McuSleepC__McuSleep__sleep();
#line 76
}
#line 76
# 78 "/home/ali/workspace/TinyOS-main/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 83
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 161
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 72 "/home/ali/workspace/TinyOS-main/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 72
  SchedulerBasicP__Scheduler__taskLoop();
#line 72
}
#line 72
# 98 "/home/ali/workspace/TinyOS-main/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 65 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0x40a05df8, uint8_t data){
#line 54
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0x40a05df8, data);
#line 54
}
#line 54
# 90 "/home/ali/workspace/TinyOS-main/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 55 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isI2C(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  {
    unsigned char __nesc_temp = 
#line 56
    HplMsp430I2C0P__U0CTL & 0x20 && HplMsp430I2C0P__U0CTL & 0x04 && HplMsp430I2C0P__U0CTL & 0x01;

#line 56
    return __nesc_temp;
  }
}

# 6 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool HplMsp430Usart0P__HplI2C__isI2C(void ){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = HplMsp430I2C0P__HplI2C__isI2C();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 66 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id)
#line 66
{
}

# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(uint8_t arg_0x40a02698){
#line 39
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(arg_0x40a02698);
#line 39
}
#line 39
# 59 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void )
#line 59
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 223 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isStartDetectPending(void )
#line 223
{
  if (I2CIFG & 0x80) {
      I2CIFG &= ~0x80;
      return SUCCESS;
    }
  return FAIL;
}

# 90 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool Msp430I2CP__HplI2C__isStartDetectPending(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = HplMsp430I2C0P__HplI2C__isStartDetectPending();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 231 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isGeneralCallPending(void )
#line 231
{
  if (I2CIFG & 0x40) {
      I2CIFG &= ~0x40;
      return SUCCESS;
    }
  return FAIL;
}

# 91 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool Msp430I2CP__HplI2C__isGeneralCallPending(void ){
#line 91
  unsigned char __nesc_result;
#line 91

#line 91
  __nesc_result = HplMsp430I2C0P__HplI2C__isGeneralCallPending();
#line 91

#line 91
  return __nesc_result;
#line 91
}
#line 91
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t Msp430I2CP__writeDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(Msp430I2CP__writeDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 239 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/Msp430I2CP.nc"
static inline void Msp430I2CP__localTxData(void )
#line 239
{
  uint16_t *_data16 = (uint16_t *)Msp430I2CP__data;

  if (Msp430I2CP__stateI2C != Msp430I2CP__PACKET_WRITE) {
    return;
    }


  if (I2CTCTL & 0x80) {
      I2CDR = _data16[(int )Msp430I2CP__ptr >> 1];
      Msp430I2CP__ptr = Msp430I2CP__ptr + 2;
    }
  else {
      I2CDR = Msp430I2CP__data[(int )Msp430I2CP__ptr];
      Msp430I2CP__ptr++;
    }



  if (Msp430I2CP__ptr == Msp430I2CP__length) {
      Msp430I2CP__I2CIE &= ~0x20;
      Msp430I2CP__result = SUCCESS;
      if (!Msp430I2CP__writeDone__postTask()) {
        Msp430I2CP__stateI2C = Msp430I2CP__IDLE;
        }
    }
}

# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t Msp430I2CP__readDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(Msp430I2CP__readDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 160 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/Msp430I2CP.nc"
static inline void Msp430I2CP__localRxData(void )
#line 160
{
  uint16_t *_data16 = (uint16_t *)Msp430I2CP__data;

  if (Msp430I2CP__stateI2C != Msp430I2CP__PACKET_READ) {
    return;
    }


  if (I2CTCTL & 0x80) {
      _data16[(int )Msp430I2CP__ptr >> 1] = I2CDR;
      Msp430I2CP__ptr = Msp430I2CP__ptr + 2;
    }
  else {
      Msp430I2CP__data[(int )Msp430I2CP__ptr] = I2CDR & 0xFF;
      Msp430I2CP__ptr++;
    }



  if (Msp430I2CP__ptr == Msp430I2CP__length) {
      Msp430I2CP__I2CIE &= ~0x10;
      Msp430I2CP__result = SUCCESS;
      if (!Msp430I2CP__readDone__postTask()) {
        Msp430I2CP__stateI2C = Msp430I2CP__IDLE;
        }
    }
}

# 247 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isAccessReadyPending(void )
#line 247
{
  if (I2CIFG & 0x08) {
      I2CIFG &= ~0x08;
      return SUCCESS;
    }
  return FAIL;
}

# 94 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool Msp430I2CP__HplI2C__isAccessReadyPending(void ){
#line 94
  unsigned char __nesc_result;
#line 94

#line 94
  __nesc_result = HplMsp430I2C0P__HplI2C__isAccessReadyPending();
#line 94

#line 94
  return __nesc_result;
#line 94
}
#line 94
# 255 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isOwnAddressPending(void )
#line 255
{
  if (I2CIFG & 0x04) {
      I2CIFG &= ~0x04;
      return SUCCESS;
    }
  return FAIL;
}

# 95 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool Msp430I2CP__HplI2C__isOwnAddressPending(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = HplMsp430I2C0P__HplI2C__isOwnAddressPending();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 263 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isNoAckPending(void )
#line 263
{
  if (I2CIFG & 0x02) {
      I2CIFG &= ~0x02;
      return SUCCESS;
    }
  return FAIL;
}

# 96 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool Msp430I2CP__HplI2C__isNoAckPending(void ){
#line 96
  unsigned char __nesc_result;
#line 96

#line 96
  __nesc_result = HplMsp430I2C0P__HplI2C__isNoAckPending();
#line 96

#line 96
  return __nesc_result;
#line 96
}
#line 96
# 271 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isArbitrationLostPending(void )
#line 271
{
  if (I2CIFG & 0x01) {
      I2CIFG &= ~0x01;
      return SUCCESS;
    }
  return FAIL;
}

# 97 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool Msp430I2CP__HplI2C__isArbitrationLostPending(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = HplMsp430I2C0P__HplI2C__isArbitrationLostPending();
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 287 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/Msp430I2CP.nc"
static inline void Msp430I2CP__I2CInterrupts__fired(void )
#line 287
{
  volatile uint16_t value = I2CIV;


  switch (value) {
      case 0x0000: 
        break;
      case 0x0002: 
        Msp430I2CP__localNoAck();
      Msp430I2CP__HplI2C__isArbitrationLostPending();
      break;
      case 0x0004: 
        Msp430I2CP__localNoAck();
      Msp430I2CP__HplI2C__isNoAckPending();
      break;
      case 0x0006: 
        Msp430I2CP__HplI2C__isOwnAddressPending();
      break;
      case 0x0008: 
        Msp430I2CP__HplI2C__isAccessReadyPending();
      break;
      case 0x000A: 
        Msp430I2CP__localRxData();
      break;
      case 0x000C: 
        Msp430I2CP__localTxData();
      break;
      case 0x000E: 
        Msp430I2CP__HplI2C__isGeneralCallPending();
      break;
      case 0x0010: 
        Msp430I2CP__HplI2C__isStartDetectPending();
      break;
    }
}

# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void HplMsp430Usart0P__I2CInterrupts__fired(void ){
#line 39
  Msp430I2CP__I2CInterrupts__fired();
#line 39
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired();
#line 39
}
#line 39
# 64 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0x40a05df8){
#line 49
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0x40a05df8);
#line 49
}
#line 49
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
# 212 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void )
#line 212
{
  if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STARTED) {

      CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
      CC2420ReceiveP__beginReceive();
    }
  else 



    {
      CC2420ReceiveP__m_missed_packets++;
    }
}

# 68 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void ){
#line 68
  CC2420ReceiveP__InterruptFIFOP__fired();
#line 68
}
#line 68
# 77 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void )
#line 77
{
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 72
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired();
#line 72
}
#line 72
# 103 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port11__clear(void )
#line 103
{
#line 103
  P1IFG &= ~(1 << 1);
}

#line 79
static inline void HplMsp430InterruptP__Port11__default__fired(void )
#line 79
{
#line 79
  HplMsp430InterruptP__Port11__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 72
  HplMsp430InterruptP__Port11__default__fired();
#line 72
}
#line 72
# 104 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__clear(void )
#line 104
{
#line 104
  P1IFG &= ~(1 << 2);
}

#line 80
static inline void HplMsp430InterruptP__Port12__default__fired(void )
#line 80
{
#line 80
  HplMsp430InterruptP__Port12__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 72
  HplMsp430InterruptP__Port12__default__fired();
#line 72
}
#line 72
# 105 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port13__clear(void )
#line 105
{
#line 105
  P1IFG &= ~(1 << 3);
}

#line 81
static inline void HplMsp430InterruptP__Port13__default__fired(void )
#line 81
{
#line 81
  HplMsp430InterruptP__Port13__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 72
  HplMsp430InterruptP__Port13__default__fired();
#line 72
}
#line 72
# 106 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 106
{
#line 106
  P1IFG &= ~(1 << 4);
}

#line 82
static inline void HplMsp430InterruptP__Port14__default__fired(void )
#line 82
{
#line 82
  HplMsp430InterruptP__Port14__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 72
  HplMsp430InterruptP__Port14__default__fired();
#line 72
}
#line 72
# 107 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__clear(void )
#line 107
{
#line 107
  P1IFG &= ~(1 << 5);
}

#line 83
static inline void HplMsp430InterruptP__Port15__default__fired(void )
#line 83
{
#line 83
  HplMsp430InterruptP__Port15__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 72
  HplMsp430InterruptP__Port15__default__fired();
#line 72
}
#line 72
# 108 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__clear(void )
#line 108
{
#line 108
  P1IFG &= ~(1 << 6);
}

#line 84
static inline void HplMsp430InterruptP__Port16__default__fired(void )
#line 84
{
#line 84
  HplMsp430InterruptP__Port16__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 72
  HplMsp430InterruptP__Port16__default__fired();
#line 72
}
#line 72
# 109 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port17__clear(void )
#line 109
{
#line 109
  P1IFG &= ~(1 << 7);
}

#line 85
static inline void HplMsp430InterruptP__Port17__default__fired(void )
#line 85
{
#line 85
  HplMsp430InterruptP__Port17__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 72
  HplMsp430InterruptP__Port17__default__fired();
#line 72
}
#line 72
# 206 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port20__clear(void )
#line 206
{
#line 206
  P2IFG &= ~(1 << 0);
}

#line 182
static inline void HplMsp430InterruptP__Port20__default__fired(void )
#line 182
{
#line 182
  HplMsp430InterruptP__Port20__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 72
  HplMsp430InterruptP__Port20__default__fired();
#line 72
}
#line 72
# 207 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port21__clear(void )
#line 207
{
#line 207
  P2IFG &= ~(1 << 1);
}

#line 183
static inline void HplMsp430InterruptP__Port21__default__fired(void )
#line 183
{
#line 183
  HplMsp430InterruptP__Port21__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 72
  HplMsp430InterruptP__Port21__default__fired();
#line 72
}
#line 72
# 208 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port22__clear(void )
#line 208
{
#line 208
  P2IFG &= ~(1 << 2);
}

#line 184
static inline void HplMsp430InterruptP__Port22__default__fired(void )
#line 184
{
#line 184
  HplMsp430InterruptP__Port22__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 72
  HplMsp430InterruptP__Port22__default__fired();
#line 72
}
#line 72
# 217 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline bool HplMsp430InterruptP__Port23__getValue(void )
#line 217
{
#line 217
  bool b;

  /* atomic removed: atomic calls only */
#line 217
  b = (P2IN >> 3) & 1;
#line 217
  return b;
}

# 59 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static bool SDP__DockInterrupt__getValue(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = HplMsp430InterruptP__Port23__getValue();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 189 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SDP.nc"
static inline void SDP__DockInterrupt__fired(void )
#line 189
{
  if (SDP__DockInterrupt__getValue() == TRUE) {
      SDP__powerCycle();

      SDP__cardInit();

      SDP__DockInterrupt__edge(FALSE);
      SDP__SD__available();
    }
  else {
      SDP__DockInterrupt__edge(TRUE);
      SDP__SD__unavailable();
      SDP__powerCycle();
    }
  SDP__DockInterrupt__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 72
  SDP__DockInterrupt__fired();
#line 72
}
#line 72
# 210 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__clear(void )
#line 210
{
#line 210
  P2IFG &= ~(1 << 4);
}

#line 186
static inline void HplMsp430InterruptP__Port24__default__fired(void )
#line 186
{
#line 186
  HplMsp430InterruptP__Port24__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 72
  HplMsp430InterruptP__Port24__default__fired();
#line 72
}
#line 72
# 211 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port25__clear(void )
#line 211
{
#line 211
  P2IFG &= ~(1 << 5);
}

#line 187
static inline void HplMsp430InterruptP__Port25__default__fired(void )
#line 187
{
#line 187
  HplMsp430InterruptP__Port25__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 72
  HplMsp430InterruptP__Port25__default__fired();
#line 72
}
#line 72
# 212 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port26__clear(void )
#line 212
{
#line 212
  P2IFG &= ~(1 << 6);
}

#line 188
static inline void HplMsp430InterruptP__Port26__default__fired(void )
#line 188
{
#line 188
  HplMsp430InterruptP__Port26__clear();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 72
  HplMsp430InterruptP__Port26__default__fired();
#line 72
}
#line 72
# 67 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__startDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__startDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 218 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void )
#line 218
{
  CC2420CsmaP__startDone_task__postTask();
}

# 76 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startOscillatorDone(void ){
#line 76
  CC2420CsmaP__CC2420Power__startOscillatorDone();
#line 76
}
#line 76
# 61 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__disable(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 433 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__InterruptCCA__fired(void )
#line 433
{
  CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTED;
  CC2420ControlP__InterruptCCA__disable();
  CC2420ControlP__IOCFG1__write(0);
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__CC2420Power__startOscillatorDone();
}

# 68 "/home/ali/workspace/TinyOS-main/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 68
  CC2420ControlP__InterruptCCA__fired();
#line 68
}
#line 68
# 77 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 77
{
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired();
}

# 72 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 72
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired();
#line 72
}
#line 72
# 98 "/home/ali/workspace/TinyOS-main/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 351 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableRxIntr(void )
#line 351
{
  HplMsp430Usart1P__IE2 &= ~(1 << 4);
}

# 177 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void ){
#line 177
  HplMsp430Usart1P__Usart__disableRxIntr();
#line 177
}
#line 177
# 232 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data)
#line 232
{

  if (/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
    /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_pos < /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
    /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else 
#line 239
    {
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr();
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
    }
}

# 65 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(uint8_t arg_0x40a05df8, uint8_t data){
#line 54
  switch (arg_0x40a05df8) {
#line 54
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 54
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(arg_0x40a05df8, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 90 "/home/ali/workspace/TinyOS-main/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 250 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void )
#line 250
{
}

# 64 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(uint8_t arg_0x40a05df8){
#line 49
  switch (arg_0x40a05df8) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(arg_0x40a05df8);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId());
    }
}

# 49 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone();
#line 49
}
#line 49
# 432 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 434
    uint16_t __x;

#line 434
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 434
   & 0x0008) != 0;

#line 435
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(12)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 133 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformCounterC.nc"
static void /*CounterMicro32C.Transform*/TransformCounterC__2__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMicro32C.Transform*/TransformCounterC__2__m_upper++;
    if ((/*CounterMicro32C.Transform*/TransformCounterC__2__m_upper & /*CounterMicro32C.Transform*/TransformCounterC__2__OVERFLOW_MASK) == 0) {
      /*CounterMicro32C.Transform*/TransformCounterC__2__Counter__overflow();
      }
  }
}

# 180 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

# 80 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformCounterC.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void )
{
  /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type rv = 0;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type high = /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper;
      /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type low = /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get();

#line 87
      if (/*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get();
        }
      {
        /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type high_to = high;
        /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type low_to = low >> /*Counter32khz32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT;

#line 101
        rv = (high_to << /*Counter32khz32C.Transform*/TransformCounterC__1__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 62 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 69
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 72
        do {
#line 72
            t0 = t1;
#line 72
            t1 = * (volatile uint16_t * )400U;
          }
        while (
#line 72
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 73
          t1;

#line 73
          return __nesc_temp;
        }
      }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )400U;
    }
}

# 56 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2r/chips/cc2420/CC2420GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__enableCapture(uint8_t mode)
#line 56
{
  /* atomic removed: atomic calls only */
#line 57
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__GeneralIO__selectModuleFunc();
    /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430Capture__clearOverflow();
    /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(mode);
    /*HplCC2420InterruptsC.CaptureSFDC*/CC2420GpioCaptureC__0__Msp430TimerControl__enableEvents();
  }
  return SUCCESS;
}

# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )49U &= ~(0x01 << 4);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 260 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len)
#line 262
{

  cc2420_status_t status = 0;
  uint8_t tmpLen = len;
  uint8_t * tmpData = (uint8_t * )data;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 268
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 270
            status;

            {
#line 270
              __nesc_atomic_end(__nesc_atomic); 
#line 270
              return __nesc_temp;
            }
          }
        }
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
#line 274
  addr += offset;

  status = CC2420SpiP__SpiByte__write(addr | 0x80);
  CC2420SpiP__SpiByte__write((addr >> 1) & 0xc0);
  for (; len; len--) {
      CC2420SpiP__SpiByte__write(tmpData[tmpLen - len]);
    }

  return status;
}

# 133 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
static bool StateImplP__State__isState(uint8_t id, uint8_t myState)
#line 133
{
  bool isState;

#line 135
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    isState = StateImplP__state[id] == myState;
#line 135
    __nesc_atomic_end(__nesc_atomic); }
  return isState;
}

# 134 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static uint8_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx)
#line 134
{
  uint8_t byte;


  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(tx);
  while (!/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr();
  byte = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();

  return byte;
}

# 56 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )49U |= 0x01 << 4;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 149 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__release(uint8_t id)
#line 149
{
  uint8_t i;

#line 151
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    {
      if (CC2420SpiP__m_holder != id) {
          {
            unsigned char __nesc_temp = 
#line 153
            FAIL;

            {
#line 153
              __nesc_atomic_end(__nesc_atomic); 
#line 153
              return __nesc_temp;
            }
          }
        }
#line 156
      CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;
      if (!CC2420SpiP__m_requests) {
          CC2420SpiP__WorkingState__toIdle();
          CC2420SpiP__attemptRelease();
        }
      else {
          for (i = CC2420SpiP__m_holder + 1; ; i++) {
              i %= CC2420SpiP__RESOURCE_COUNT;

              if (CC2420SpiP__m_requests & (1 << i)) {
                  CC2420SpiP__m_holder = i;
                  CC2420SpiP__m_requests &= ~(1 << i);
                  CC2420SpiP__grant__postTask();
                  {
                    unsigned char __nesc_temp = 
#line 169
                    SUCCESS;

                    {
#line 169
                      __nesc_atomic_end(__nesc_atomic); 
#line 169
                      return __nesc_temp;
                    }
                  }
                }
            }
        }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
#line 175
  return SUCCESS;
}

#line 339
static error_t CC2420SpiP__attemptRelease(void )
#line 339
{


  if ((
#line 340
  CC2420SpiP__m_requests > 0
   || CC2420SpiP__m_holder != CC2420SpiP__NO_HOLDER)
   || !CC2420SpiP__WorkingState__isIdle()) {
      return FAIL;
    }
  /* atomic removed: atomic calls only */
  CC2420SpiP__release = TRUE;
  CC2420SpiP__ChipSpiResource__releasing();
  /* atomic removed: atomic calls only */
#line 348
  {
    if (CC2420SpiP__release) {
        CC2420SpiP__SpiResource__release();
        {
          unsigned char __nesc_temp = 
#line 351
          SUCCESS;

#line 351
          return __nesc_temp;
        }
      }
  }
  return EBUSY;
}

# 170 "/home/ali/workspace/TinyOS-main/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
#line 172
      {
        unsigned char __nesc_temp = 
#line 172
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 172
          __nesc_atomic_end(__nesc_atomic); 
#line 172
          return __nesc_temp;
        }
      }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
}

# 251 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static void HplMsp430Usart1P__Usart__disableSpi(void )
#line 251
{
  /* atomic removed: atomic calls only */
#line 252
  {
    HplMsp430Usart1P__ME2 &= ~(1 << 4);
    HplMsp430Usart1P__SIMO__selectIOFunc();
    HplMsp430Usart1P__SOMI__selectIOFunc();
    HplMsp430Usart1P__UCLK__selectIOFunc();
  }
}

# 147 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 = t0;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt = dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

#line 107
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__set_alarm(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__get();
#line 109
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type expires;
#line 109
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type remaining;




  expires = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt;


  remaining = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type )(expires - now);


  if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 <= now) 
    {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__MAX_DELAY) 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 = now + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt = remaining - /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      remaining = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__MAX_DELAY;
    }
  else 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 += /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt = 0;
    }
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt((/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_size_type )now << 0, 
  (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_size_type )remaining << 0);
}

# 844 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__signalDone(error_t err)
#line 844
{
  /* atomic removed: atomic calls only */
#line 845
  CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
  CC2420TransmitP__abortSpiRelease = FALSE;
  CC2420TransmitP__ChipSpiResource__attemptRelease();
  CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, err);
}

# 171 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/packet/CC2420PacketP.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg)
{
  __nesc_hton_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.data, FALSE);
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.data, CC2420_INVALID_TIMESTAMP);
}

# 180 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(10)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(26)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 146 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 39 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x4076bcf8){
#line 39
  switch (arg_0x4076bcf8) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 39
      break;
#line 39
    case 3:
#line 39
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 39
      break;
#line 39
    case 4:
#line 39
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 39
      break;
#line 39
    case 6:
#line 39
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 39
      break;
#line 39
    case 7:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x4076bcf8);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 107 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 5);
}

# 80 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 87
      if (/*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 101
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 782 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__congestionBackoff(void )
#line 782
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 783
    {
      CC2420TransmitP__RadioBackoff__requestCongestionBackoff(CC2420TransmitP__m_msg);
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myCongestionBackoff);
    }
#line 786
    __nesc_atomic_end(__nesc_atomic); }
}

# 69 "/home/ali/workspace/TinyOS-main/tos/system/RandomMlcgC.nc"
static uint32_t RandomMlcgC__Random__rand32(void )
#line 69
{
  uint32_t mlcg;
#line 70
  uint32_t p;
#line 70
  uint32_t q;
  uint64_t tmpseed;

  /* atomic removed: atomic calls only */
#line 73
  {
    tmpseed = (uint64_t )33614U * (uint64_t )RandomMlcgC__seed;
    q = tmpseed;
    q = q >> 1;
    p = tmpseed >> 32;
    mlcg = p + q;
    if (mlcg & 0x80000000) {
        mlcg = mlcg & 0x7FFFFFFF;
        mlcg++;
      }
    RandomMlcgC__seed = mlcg;
  }
  return mlcg;
}

# 789 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static error_t CC2420TransmitP__acquireSpiResource(void )
#line 789
{
  error_t error = CC2420TransmitP__SpiResource__immediateRequest();

#line 791
  if (error != SUCCESS) {
      CC2420TransmitP__SpiResource__request();
    }
  return error;
}

# 126 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id)
#line 126
{
  error_t error;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) != SUCCESS) {
          {
            unsigned char __nesc_temp = 
#line 131
            EBUSY;

            {
#line 131
              __nesc_atomic_end(__nesc_atomic); 
#line 131
              return __nesc_temp;
            }
          }
        }
      if (CC2420SpiP__SpiResource__isOwner()) {
          CC2420SpiP__m_holder = id;
          error = SUCCESS;
        }
      else {
#line 139
        if ((error = CC2420SpiP__SpiResource__immediateRequest()) == SUCCESS) {
            CC2420SpiP__m_holder = id;
          }
        else {
            CC2420SpiP__WorkingState__toIdle();
          }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 146
  return error;
}

# 96 "/home/ali/workspace/TinyOS-main/tos/system/StateImplP.nc"
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState)
#line 96
{
  error_t returnVal = FAIL;

#line 98
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 98
    {
      if (reqState == StateImplP__S_IDLE || StateImplP__state[id] == StateImplP__S_IDLE) {
          StateImplP__state[id] = reqState;
          returnVal = SUCCESS;
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return returnVal;
}

# 177 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id)
#line 177
{
  /* atomic removed: atomic calls only */
#line 178
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId == id && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) {
        unsigned char __nesc_temp = 
#line 179
        TRUE;

#line 179
        return __nesc_temp;
      }
    else 
#line 180
      {
        unsigned char __nesc_temp = 
#line 180
        FALSE;

#line 180
        return __nesc_temp;
      }
  }
}

#line 133
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void )
#line 133
{
  /* atomic removed: atomic calls only */
#line 134
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id) {
        if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING) {
            /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
            {
              unsigned char __nesc_temp = 
#line 138
              SUCCESS;

#line 138
              return __nesc_temp;
            }
          }
        else {
#line 140
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
              {
                unsigned char __nesc_temp = 
#line 143
                SUCCESS;

#line 143
                return __nesc_temp;
              }
            }
          }
      }
  }
#line 147
  return FAIL;
}

# 269 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static void HplMsp430Usart1P__Usart__setModeSpi(msp430_spi_union_config_t *config)
#line 269
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 270
    {
      HplMsp430Usart1P__Usart__resetUsart(TRUE);
      HplMsp430Usart1P__Usart__disableUart();
      HplMsp430Usart1P__configSpi(config);
      HplMsp430Usart1P__Usart__enableSpi();
      HplMsp430Usart1P__Usart__resetUsart(FALSE);
      HplMsp430Usart1P__Usart__clrIntr();
      HplMsp430Usart1P__Usart__disableIntr();
    }
#line 278
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 107 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__request(uint8_t id)
#line 107
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) == SUCCESS) {
          CC2420SpiP__m_holder = id;
          if (CC2420SpiP__SpiResource__isOwner()) {
              CC2420SpiP__grant__postTask();
            }
          else {
              CC2420SpiP__SpiResource__request();
            }
        }
      else {
          CC2420SpiP__m_requests |= 1 << id;
        }
    }
#line 122
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 737 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__attemptSend(void )
#line 737
{
  uint8_t status;
  bool congestion = TRUE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 741
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          CC2420TransmitP__SFLUSHTX__strobe();
          CC2420TransmitP__releaseSpiResource();
          CC2420TransmitP__CSN__set();
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
          CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
          {
#line 748
            __nesc_atomic_end(__nesc_atomic); 
#line 748
            return;
          }
        }





      CC2420TransmitP__CSN__clr();
      status = CC2420TransmitP__m_cca ? CC2420TransmitP__STXONCCA__strobe() : CC2420TransmitP__STXON__strobe();
      if (!(status & CC2420_STATUS_TX_ACTIVE)) {
          status = CC2420TransmitP__SNOP__strobe();
          if (status & CC2420_STATUS_TX_ACTIVE) {
              congestion = FALSE;
            }
        }

      CC2420TransmitP__m_state = congestion ? CC2420TransmitP__S_SAMPLE_CCA : CC2420TransmitP__S_SFD;
      CC2420TransmitP__CSN__set();
    }
#line 767
    __nesc_atomic_end(__nesc_atomic); }

  if (congestion) {
      CC2420TransmitP__totalCcaChecks = 0;
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__congestionBackoff();
    }
  else 
#line 773
    {
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__CC2420_ABORT_PERIOD);
    }
}

# 318 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr)
#line 318
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 319
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 321
            0;

            {
#line 321
              __nesc_atomic_end(__nesc_atomic); 
#line 321
              return __nesc_temp;
            }
          }
        }
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
#line 325
  return CC2420SpiP__SpiByte__write(addr);
}

# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )49U &= ~(0x01 << 7);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

#line 56
static void /*HplMsp430GeneralIOC.P57*/HplMsp430GeneralIOP__39__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )49U |= 0x01 << 7;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 14 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(24)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 63 "/home/ali/workspace/TinyOS-main/tos/system/RealMainP.nc"
  int main(void )
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 71
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 88
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 175 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 56 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )29U |= 0x01 << 0;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

#line 56
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )29U |= 0x01 << 2;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 134 "/home/ali/workspace/TinyOS-main/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 138
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 142
          FALSE;

#line 142
          return __nesc_temp;
        }
      }
  }
#line 145
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

#line 175
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 75 "/home/ali/workspace/TinyOS-main/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x406939e0){
#line 75
  switch (arg_0x406939e0) {
#line 75
    case AppP__SDtest:
#line 75
      AppP__SDtest__runTask();
#line 75
      break;
#line 75
    case /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 75
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 75
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 75
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 75
      break;
#line 75
    case Msp430I2CP__readDone:
#line 75
      Msp430I2CP__readDone__runTask();
#line 75
      break;
#line 75
    case Msp430I2CP__writeDone:
#line 75
      Msp430I2CP__writeDone__runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP__startDone_task:
#line 75
      CC2420CsmaP__startDone_task__runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP__stopDone_task:
#line 75
      CC2420CsmaP__stopDone_task__runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP__sendDone_task:
#line 75
      CC2420CsmaP__sendDone_task__runTask();
#line 75
      break;
#line 75
    case CC2420ControlP__sync:
#line 75
      CC2420ControlP__sync__runTask();
#line 75
      break;
#line 75
    case CC2420ControlP__syncDone:
#line 75
      CC2420ControlP__syncDone__runTask();
#line 75
      break;
#line 75
    case CC2420SpiP__grant:
#line 75
      CC2420SpiP__grant__runTask();
#line 75
      break;
#line 75
    case /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task:
#line 75
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask();
#line 75
      break;
#line 75
    case /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask:
#line 75
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask();
#line 75
      break;
#line 75
    case CC2420ReceiveP__receiveDone_task:
#line 75
      CC2420ReceiveP__receiveDone_task__runTask();
#line 75
      break;
#line 75
    case CC2420TinyosNetworkP__grantTask:
#line 75
      CC2420TinyosNetworkP__grantTask__runTask();
#line 75
      break;
#line 75
    case DiagMsgP__send:
#line 75
      DiagMsgP__send__runTask();
#line 75
      break;
#line 75
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask:
#line 75
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask();
#line 75
      break;
#line 75
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask:
#line 75
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x406939e0);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 298 "/home/ali/workspace/TinyOS-main/tos/lib/diagmsg/DiagMsgP.nc"
static void DiagMsgP__AMSend__sendDone(message_t *p, error_t error)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {

      if (error != SUCCESS && --DiagMsgP__retries > 0) {
        DiagMsgP__send__postTask();
        }
      else {
          p = DiagMsgP__nextPointer(DiagMsgP__sending);
          if (p != DiagMsgP__recording) 
            {
              DiagMsgP__sending = p;
              DiagMsgP__retries = 1;
              DiagMsgP__send__postTask();
            }
          else 
            {
              DiagMsgP__sending = 0;

              if (DiagMsgP__state == DiagMsgP__STATE_BUFFER_FULL) 
                {
                  DiagMsgP__state = DiagMsgP__STATE_READY;
                  if (DiagMsgP__DiagMsg__record()) 
                    {
                      DiagMsgP__DiagMsg__str("DiagMsgOverflow");
                      DiagMsgP__DiagMsg__send();
                    }
                }
            }
        }
    }
#line 329
    __nesc_atomic_end(__nesc_atomic); }
}

#line 136
static bool DiagMsgP__DiagMsg__record(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {

      if (DiagMsgP__state != DiagMsgP__STATE_READY) 
        {
          unsigned char __nesc_temp = 
#line 142
          FALSE;

          {
#line 142
            __nesc_atomic_end(__nesc_atomic); 
#line 142
            return __nesc_temp;
          }
        }
#line 144
      DiagMsgP__state = DiagMsgP__STATE_RECORDING_FIRST;
      DiagMsgP__nextData = 0;
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }

  return TRUE;
}

#line 199
static void DiagMsgP__copyArray(uint8_t size, uint8_t type2, const void *data, uint8_t len)
{
  int8_t start;

  if (len > 15) {
    len = 15;
    }
  start = DiagMsgP__allocate(size * len + 1, DiagMsgP__TYPE_ARRAY);
  if (start >= 0) 
    {
      __nesc_hton_uint8(DiagMsgP__recording->data[start].data, (type2 << 4) + len);
      memcpy(&DiagMsgP__recording->data[start + 1], data, size * len);
    }
}

#line 271
static void DiagMsgP__DiagMsg__send(void )
{

  if (DiagMsgP__state == DiagMsgP__STATE_READY) {
    return;
    }

  DiagMsgP__setPayloadLength(DiagMsgP__recording, DiagMsgP__nextData);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (DiagMsgP__sending == 0) 
        {
          DiagMsgP__sending = DiagMsgP__recording;
          DiagMsgP__retries = 1;
          DiagMsgP__send__postTask();
        }

      DiagMsgP__recording = DiagMsgP__nextPointer(DiagMsgP__recording);

      if (DiagMsgP__recording == DiagMsgP__sending) {
        DiagMsgP__state = DiagMsgP__STATE_BUFFER_FULL;
        }
      else {
#line 294
        DiagMsgP__state = DiagMsgP__STATE_READY;
        }
    }
#line 296
    __nesc_atomic_end(__nesc_atomic); }
}

# 163 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err)
#line 163
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend();
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, err);
}

# 139 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static am_addr_t CC2420ActiveMessageP__AMPacket__destination(message_t *amsg)
#line 139
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 141
  return __nesc_ntoh_leuint16(header->dest.data);
}

#line 87
static error_t CC2420ActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len)
#line 89
{
  unsigned char *__nesc_temp48;
#line 90
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg);

  if (len > CC2420ActiveMessageP__Packet__maxPayloadLength()) {
      return ESIZE;
    }

  __nesc_hton_leuint8(header->type.data, id);
  __nesc_hton_leuint16(header->dest.data, addr);
  __nesc_hton_leuint16(header->destpan.data, CC2420ActiveMessageP__CC2420Config__getPanAddr());
  __nesc_hton_leuint16(header->src.data, CC2420ActiveMessageP__AMPacket__address());
  (__nesc_temp48 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp48, __nesc_ntoh_leuint16(__nesc_temp48) | (((1 << IEEE154_FCF_INTRAPAN) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE))));
  __nesc_hton_leuint8(header->length.data, len + CC2420_SIZE);

  if (CC2420ActiveMessageP__RadioResource__immediateRequest() == SUCCESS) {
      error_t rc;

#line 107
      CC2420ActiveMessageP__SendNotifier__aboutToSend(id, addr, msg);

      rc = CC2420ActiveMessageP__SubSend__send(msg, len);
      if (rc != SUCCESS) {
          CC2420ActiveMessageP__RadioResource__release();
        }

      return rc;
    }
  else 
#line 115
    {
      CC2420ActiveMessageP__pending_length = len;
      CC2420ActiveMessageP__pending_message = msg;
      return CC2420ActiveMessageP__RadioResource__request();
    }
}

# 106 "/home/ali/workspace/TinyOS-main/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void )
#line 106
{
  am_addr_t myAddr;

#line 108
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 108
    myAddr = ActiveMessageAddressC__addr;
#line 108
    __nesc_atomic_end(__nesc_atomic); }
  return myAddr;
}

# 60 "/home/ali/workspace/TinyOS-main/tos/system/FcfsResourceQueueC.nc"
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 60
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
    {
      unsigned char __nesc_temp = 
#line 61
      /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

      {
#line 61
        __nesc_atomic_end(__nesc_atomic); 
#line 61
        return __nesc_temp;
      }
    }
#line 63
    __nesc_atomic_end(__nesc_atomic); }
}

# 80 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static error_t CC2420TinyosNetworkP__ActiveSend__send(message_t *msg, uint8_t len)
#line 80
{
  CC2420TinyosNetworkP__CC2420Packet__setNetwork(msg, 0x3f);
  CC2420TinyosNetworkP__m_busy_client = CC2420TinyosNetworkP__CLIENT_AM;
  return CC2420TinyosNetworkP__SubSend__send(msg, len);
}

# 90 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/packet/CC2420PacketP.nc"
static uint8_t *CC2420PacketP__getNetwork(message_t *msg)
#line 90
{
  cc2420_header_t *hdr = CC2420PacketP__CC2420PacketBody__getHeader(msg);
  int offset;

  offset = CC2420PacketP__getAddressLength((__nesc_ntoh_leuint16(hdr->fcf.data) >> IEEE154_FCF_DEST_ADDR_MODE) & 0x3) + 
  CC2420PacketP__getAddressLength((__nesc_ntoh_leuint16(hdr->fcf.data) >> IEEE154_FCF_SRC_ADDR_MODE) & 0x3) + 
  (size_t )& ((cc2420_header_t *)0)->dest;

  return (uint8_t *)hdr + offset;
}

# 819 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__loadTXFIFO(void )
#line 819
{
  cc2420_header_t *header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);
  uint8_t tx_power = __nesc_ntoh_uint8(CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg)->tx_power.data);

  if (!tx_power) {
      tx_power = 31;
    }

  CC2420TransmitP__CSN__clr();

  if (CC2420TransmitP__m_tx_power != tx_power) {
      CC2420TransmitP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | ((
      tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
    }

  CC2420TransmitP__m_tx_power = tx_power;

  {
    uint8_t tmpLen __attribute((unused))  = __nesc_ntoh_leuint8(header->length.data) - 1;

#line 840
    CC2420TransmitP__TXFIFO__write((uint8_t * )header, __nesc_ntoh_leuint8(header->length.data) - 1);
  }
}

# 305 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data)
#line 305
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 306
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 308
            0;

            {
#line 308
              __nesc_atomic_end(__nesc_atomic); 
#line 308
              return __nesc_temp;
            }
          }
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
#line 311
  CC2420SpiP__SpiByte__write(addr);
  CC2420SpiP__SpiByte__write(data >> 8);
  return CC2420SpiP__SpiByte__write(data & 0xff);
}

# 206 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static error_t /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
#line 208
{

  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_client = id;
  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf = tx_buf;
  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf = rx_buf;
  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_len = len;
  /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_pos = 0;

  if (len) {
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr();
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else {
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask();
    }

  return SUCCESS;
}

#line 183
static void /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void )
#line 183
{

  uint8_t end;
  uint8_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);

      end = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_pos + /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE;
      if (end > /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
        end = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_len;
        }
      while (++/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_pos < end) {
          while (!/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
          tmp = /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();
          if (/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
            /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = tmp;
            }
#line 200
          /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma1P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);
        }
    }
#line 202
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/home/ali/workspace/TinyOS-main/tos/interfaces/State.nc"
static void UniqueSendP__State__toIdle(void ){
#line 56
  StateImplP__State__toIdle(2U);
#line 56
}
#line 56
# 74 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static void CC2420ActiveMessageP__RadioResource__granted(void )
#line 74
{
  uint8_t rc;
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(CC2420ActiveMessageP__pending_message);

  CC2420ActiveMessageP__SendNotifier__aboutToSend(__nesc_ntoh_leuint8(header->type.data), __nesc_ntoh_leuint16(header->dest.data), CC2420ActiveMessageP__pending_message);
  rc = CC2420ActiveMessageP__SubSend__send(CC2420ActiveMessageP__pending_message, CC2420ActiveMessageP__pending_length);
  if (rc != SUCCESS) {
      CC2420ActiveMessageP__RadioResource__release();
      CC2420ActiveMessageP__AMSend__sendDone(__nesc_ntoh_leuint8(header->type.data), CC2420ActiveMessageP__pending_message, rc);
    }
}

# 189 "/home/ali/workspace/TinyOS-main/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 189
{





  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      return;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == msg) {
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, msg, err);
    }
  else {
      ;
    }
}

# 294 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void )
#line 294
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 295
    {
      unsigned int __nesc_temp = 
#line 295
      CC2420ControlP__m_short_addr;

      {
#line 295
        __nesc_atomic_end(__nesc_atomic); 
#line 295
        return __nesc_temp;
      }
    }
#line 297
    __nesc_atomic_end(__nesc_atomic); }
}

# 8 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/ds2411/DallasId48ToIeeeEui64C.nc"
static ieee_eui64_t DallasId48ToIeeeEui64C__LocalIeeeEui64__getId(void )
#line 8
{
  uint8_t id[6];
  ieee_eui64_t eui;

#line 11
  DallasId48ToIeeeEui64C__IDChip__read(id);

  eui.data[0] = IEEE_EUI64_COMPANY_ID_0;
  eui.data[1] = IEEE_EUI64_COMPANY_ID_1;
  eui.data[2] = IEEE_EUI64_COMPANY_ID_2;



  eui.data[3] = IEEE_EUI64_SERIAL_ID_0;
  eui.data[4] = IEEE_EUI64_SERIAL_ID_1;


  eui.data[5] = id[3];
  eui.data[6] = id[4];
  eui.data[7] = id[5];

  return eui;
}

# 764 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP__waitForNextPacket(void )
#line 764
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 765
    {
      if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STOPPED) {
          CC2420ReceiveP__SpiResource__release();
          {
#line 768
            __nesc_atomic_end(__nesc_atomic); 
#line 768
            return;
          }
        }
      CC2420ReceiveP__receivingPacket = FALSE;
#line 783
      if ((CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get()) {

          if (CC2420ReceiveP__m_missed_packets) {
              CC2420ReceiveP__m_missed_packets--;
            }





          CC2420ReceiveP__beginReceive();
        }
      else 
        {

          CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
          CC2420ReceiveP__m_missed_packets = 0;
          CC2420ReceiveP__SpiResource__release();
        }
    }
#line 802
    __nesc_atomic_end(__nesc_atomic); }
}

#line 711
static void CC2420ReceiveP__beginReceive(void )
#line 711
{
  CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
  /* atomic removed: atomic calls only */
#line 713
  CC2420ReceiveP__receivingPacket = TRUE;
  if (CC2420ReceiveP__SpiResource__isOwner()) {
      CC2420ReceiveP__receive();
    }
  else {
#line 717
    if (CC2420ReceiveP__SpiResource__immediateRequest() == SUCCESS) {
        CC2420ReceiveP__receive();
      }
    else {
        CC2420ReceiveP__SpiResource__request();
      }
    }
}

#line 754
static void CC2420ReceiveP__receive(void )
#line 754
{
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__RXFIFO__beginRead((uint8_t *)CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf), 1);
}

# 189 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 190
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 194
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 196
            status;

            {
#line 196
              __nesc_atomic_end(__nesc_atomic); 
#line 196
              return __nesc_temp;
            }
          }
        }
    }
#line 200
    __nesc_atomic_end(__nesc_atomic); }
#line 200
  CC2420SpiP__m_addr = addr | 0x40;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__Fifo__continueRead(addr, data, len);

  return status;
}

#line 329
static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 330
{
  if (CC2420SpiP__m_addr & 0x40) {
      CC2420SpiP__Fifo__readDone(CC2420SpiP__m_addr & ~0x40, rx_buf, len, error);
    }
  else 
#line 333
    {
      CC2420SpiP__Fifo__writeDone(CC2420SpiP__m_addr, tx_buf, len, error);
    }
}

# 728 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP__flush(void )
#line 728
{








  CC2420ReceiveP__reset_state();

  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__SpiResource__release();
  CC2420ReceiveP__waitForNextPacket();
}

#line 808
static void CC2420ReceiveP__reset_state(void )
#line 808
{
  CC2420ReceiveP__m_bytes_left = CC2420ReceiveP__RXFIFO_SIZE;
  /* atomic removed: atomic calls only */
#line 810
  CC2420ReceiveP__receivingPacket = FALSE;
  CC2420ReceiveP__m_timestamp_head = 0;
  CC2420ReceiveP__m_timestamp_size = 0;
  CC2420ReceiveP__m_missed_packets = 0;
}

# 471 "/home/ali/workspace/TinyOS-main/tos/chips/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP__writeFsctrl(void )
#line 471
{
  uint8_t channel;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 474
    {
      channel = CC2420ControlP__m_channel;
    }
#line 476
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__FSCTRL__write((1 << CC2420_FSCTRL_LOCK_THR) | (((
  channel - 11) * 5 + 357) << CC2420_FSCTRL_FREQ));
}







static void CC2420ControlP__writeMdmctrl0(void )
#line 488
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 489
    {
      CC2420ControlP__MDMCTRL0__write((((((((1 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | ((
      CC2420ControlP__addressRecognition && CC2420ControlP__hwAddressRecognition ? 1 : 0) << CC2420_MDMCTRL0_ADR_DECODE)) | (
      2 << CC2420_MDMCTRL0_CCA_HYST)) | (
      3 << CC2420_MDMCTRL0_CCA_MOD)) | (
      1 << CC2420_MDMCTRL0_AUTOCRC)) | ((
      CC2420ControlP__autoAckEnabled && CC2420ControlP__hwAutoAckDefault) << CC2420_MDMCTRL0_AUTOACK)) | (
      0 << CC2420_MDMCTRL0_AUTOACK)) | (
      2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));
    }
#line 498
    __nesc_atomic_end(__nesc_atomic); }
}







static void CC2420ControlP__writeId(void )
#line 507
{
  nxle_uint16_t id[6];

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 510
    {
      * (ieee_eui64_t *)&id[0] = CC2420ControlP__m_ext_addr;
      __nesc_hton_leuint16(id[4].data, CC2420ControlP__m_pan);
      __nesc_hton_leuint16(id[5].data, CC2420ControlP__m_short_addr);
    }
#line 514
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__IEEEADR__write(0, (uint8_t *)&id, 12);
}

# 57 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P40*/HplMsp430GeneralIOP__24__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )29U &= ~(0x01 << 0);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 66 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430I2C0P.nc"
static void HplMsp430I2C0P__HplI2C__setModeI2C(msp430_i2c_union_config_t *config)
#line 66
{

  HplMsp430I2C0P__HplUsart__resetUsart(TRUE);
  HplMsp430I2C0P__HplUsart__disableUart();
  HplMsp430I2C0P__HplUsart__disableSpi();
  HplMsp430I2C0P__SIMO__makeInput();
  HplMsp430I2C0P__SIMO__selectModuleFunc();
  HplMsp430I2C0P__UCLK__makeInput();
  HplMsp430I2C0P__UCLK__selectModuleFunc();

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 76
    {

      IE1 &= ~((1 << 7) | (1 << 6));

      HplMsp430I2C0P__U0CTL &= ~((0x20 | 0x01) | 0x04);
      HplMsp430I2C0P__U0CTL = 0x01;
      HplMsp430I2C0P__U0CTL |= 0x04 | 0x20;
      HplMsp430I2C0P__U0CTL &= ~0x01;

      HplMsp430I2C0P__U0CTL |= 0x02;

      HplMsp430I2C0P__I2CTCTL = 2 << 4;

      I2CPSC = 0x00;
      I2CSCLH = 0x03;
      I2CSCLL = 0x03;

      HplMsp430I2C0P__I2CIE = 0;
      I2CIFG = 0;
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }
}

# 214 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__disableUart(void )
#line 214
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 215
    {
      HplMsp430Usart0P__ME1 &= ~((1 << 7) | (1 << 6));
      HplMsp430Usart0P__UTXD__selectIOFunc();
      HplMsp430Usart0P__URXD__selectIOFunc();
    }
#line 219
    __nesc_atomic_end(__nesc_atomic); }
}

#line 254
static void HplMsp430Usart0P__Usart__disableSpi(void )
#line 254
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 255
    {
      HplMsp430Usart0P__ME1 &= ~(1 << 6);
      HplMsp430Usart0P__SIMO__selectIOFunc();
      HplMsp430Usart0P__SOMI__selectIOFunc();
      HplMsp430Usart0P__UCLK__selectIOFunc();
    }
#line 260
    __nesc_atomic_end(__nesc_atomic); }
}

# 65 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 65
{
#line 65
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 65
    * (volatile uint8_t * )27U |= 0x01 << 1;
#line 65
    __nesc_atomic_end(__nesc_atomic); }
}

#line 65
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 65
{
#line 65
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 65
    * (volatile uint8_t * )27U |= 0x01 << 3;
#line 65
    __nesc_atomic_end(__nesc_atomic); }
}

# 114 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
static error_t Hmc5843P__writeRegValue(uint8_t reg_addr, uint8_t val)
#line 114
{



  Hmc5843P__packet[0] = reg_addr;
  Hmc5843P__packet[1] = val;

  Hmc5843P__I2CPacket__write(I2C_START | I2C_STOP, 0x1e, 2, Hmc5843P__packet);
  return SUCCESS;
}

# 144 "/home/ali/workspace/TinyOS-main/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 147
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

#line 73
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 90
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 125 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer2/chips/hmc5843/Hmc5843P.nc"
static error_t Hmc5843P__readValues(uint8_t size, uint8_t *data)
#line 125
{
  Hmc5843P__readSize = size;
  Hmc5843P__readDataBuffer = data;

  Hmc5843P__I2CPacket__read(I2C_START | I2C_STOP, 0x1e, size, data);
  return SUCCESS;
}

# 147 "/home/ali/workspace/TinyOS-main/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

# 240 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static void HplMsp430InterruptP__Port23__edge(bool l2h)
#line 240
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 241
    {
      if (l2h) {
#line 242
        P2IES &= ~(1 << 3);
        }
      else {
#line 243
        P2IES |= 1 << 3;
        }
    }
#line 245
    __nesc_atomic_end(__nesc_atomic); }
}

# 122 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SDP.nc"
static void SDP__powerCycle(void )
#line 122
{

  while (SDP__Usart__isTxEmpty() == FALSE) ;
#line 124
  ;
#line 124
  TOSH_SET_SD_CS_N_PIN();
#line 124
  ;


  TOSH_MAKE_DOCK_N_OUTPUT();
  TOSH_SET_DOCK_N_PIN();

  TOSH_SET_SW_SD_PWR_N_PIN();
  TOSH_CLR_SD_CS_N_PIN();





  SDP__Usart__disableSpi();
  TOSH_CLR_SD_DI_PIN();
  TOSH_CLR_SD_DO_PIN();
  TOSH_CLR_SD_CLK_PIN();

  brief_pause(((unsigned long long )60000 * 4096 * 1024 / 1000000 - 2) / 3);

  TOSH_SET_SD_CS_N_PIN();
  TOSH_CLR_SW_SD_PWR_N_PIN();


  TOSH_MAKE_DOCK_N_INPUT();
}

#line 349
static error_t SDP__cardInit(void )
#line 349
{
  register uint8_t i;
  uint8_t r;

  SDP__initSPI();

  while (SDP__Usart__isTxEmpty() == FALSE) ;
#line 355
  ;
#line 355
  TOSH_SET_SD_CS_N_PIN();
#line 355
  ;

  for (i = 0; i < 10; i++) 
    SDP__spiSendByte(0xff);

  r = SDP__setIdle();



  SDP__Usart__setUbr(0x0002);
  SDP__Usart__setUmctl(0x00);

  return r;
}

# 139 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__setUbr(uint16_t control)
#line 139
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 140
    {
      U0BR0 = control & 0x00FF;
      U0BR1 = (control >> 8) & 0x00FF;
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 236 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/sd/SDP.nc"
static uint8_t SDP__spiSendByte(const uint8_t data)
#line 236
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 237
    {
      while (SDP__Usart__isTxEmpty() == FALSE) ;

      SDP__Usart__tx(data);

      while (SDP__Usart__isRxIntrPending() == FALSE) ;
    }
#line 243
    __nesc_atomic_end(__nesc_atomic); }

  return SDP__Usart__rx();
}

static void SDP__sendCmd(const uint8_t cmd, uint32_t data, const uint8_t crc)
#line 248
{
  uint8_t frame[6];
  register int8_t i;

  frame[0] = cmd | 0x40;
  for (i = 3; i >= 0; i--) 
    frame[4 - i] = (uint8_t )(data >> 8 * i);

  frame[5] = crc;
  for (i = 0; i < 6; i++) 
    SDP__spiSendByte(frame[i]);
}






static uint8_t SDP__getResponse(void )
{
  register int i = 0;
  uint8_t response;

  for (i = 0; i < 65; i++) {
      if (((response = SDP__spiSendByte(0xff)) == 0x00) | (
      response == 0x01)) {
        break;
        }
    }
#line 276
  return response;
}

# 98 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(18)))  void sig_UART0RX_VECTOR(void )
#line 98
{
  uint8_t temp = U0RXBUF;

  HplMsp430Usart0P__Interrupts__rxDone(temp);
}

# 153 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}






static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 104 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(16)))  void sig_UART0TX_VECTOR(void )
#line 104
{
  if (HplMsp430Usart0P__HplI2C__isI2C()) {

      HplMsp430Usart0P__I2CInterrupts__fired();
    }
  else {

      HplMsp430Usart0P__Interrupts__txDone();
    }
}

# 266 "/home/ali/workspace/TinyOS-main/tos/platforms/shimmer/chips/msp430/Msp430I2CP.nc"
static void Msp430I2CP__localNoAck(void )
#line 266
{
  if (Msp430I2CP__stateI2C != Msp430I2CP__PACKET_WRITE && Msp430I2CP__stateI2C != Msp430I2CP__PACKET_READ) {
    return;
    }
  I2CNDAT = 0;
  Msp430I2CP__I2CIE = 0;


  if (I2CDCTL & 0x01) {
    I2CTCTL |= 0x02;
    }
  if (Msp430I2CP__stateI2C == Msp430I2CP__PACKET_WRITE) {
      if (!Msp430I2CP__writeDone__postTask()) {
        Msp430I2CP__stateI2C = Msp430I2CP__IDLE;
        }
    }
  else {
#line 281
    if (Msp430I2CP__stateI2C == Msp430I2CP__PACKET_READ) {
        if (!Msp430I2CP__readDone__postTask()) {
          Msp430I2CP__stateI2C = Msp430I2CP__IDLE;
          }
      }
    }
}

# 64 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
__attribute((wakeup)) __attribute((interrupt(8)))  void sig_PORT1_VECTOR(void )
{
  volatile int n = P1IFG & P1IE;

  if (n & (1 << 0)) {
#line 68
      HplMsp430InterruptP__Port10__fired();
#line 68
      return;
    }
#line 69
  if (n & (1 << 1)) {
#line 69
      HplMsp430InterruptP__Port11__fired();
#line 69
      return;
    }
#line 70
  if (n & (1 << 2)) {
#line 70
      HplMsp430InterruptP__Port12__fired();
#line 70
      return;
    }
#line 71
  if (n & (1 << 3)) {
#line 71
      HplMsp430InterruptP__Port13__fired();
#line 71
      return;
    }
#line 72
  if (n & (1 << 4)) {
#line 72
      HplMsp430InterruptP__Port14__fired();
#line 72
      return;
    }
#line 73
  if (n & (1 << 5)) {
#line 73
      HplMsp430InterruptP__Port15__fired();
#line 73
      return;
    }
#line 74
  if (n & (1 << 6)) {
#line 74
      HplMsp430InterruptP__Port16__fired();
#line 74
      return;
    }
#line 75
  if (n & (1 << 7)) {
#line 75
      HplMsp430InterruptP__Port17__fired();
#line 75
      return;
    }
}

#line 169
__attribute((wakeup)) __attribute((interrupt(2)))  void sig_PORT2_VECTOR(void )
{
  volatile int n = P2IFG & P2IE;

  if (n & (1 << 0)) {
#line 173
      HplMsp430InterruptP__Port20__fired();
#line 173
      return;
    }
#line 174
  if (n & (1 << 1)) {
#line 174
      HplMsp430InterruptP__Port21__fired();
#line 174
      return;
    }
#line 175
  if (n & (1 << 2)) {
#line 175
      HplMsp430InterruptP__Port22__fired();
#line 175
      return;
    }
#line 176
  if (n & (1 << 3)) {
#line 176
      HplMsp430InterruptP__Port23__fired();
#line 176
      return;
    }
#line 177
  if (n & (1 << 4)) {
#line 177
      HplMsp430InterruptP__Port24__fired();
#line 177
      return;
    }
#line 178
  if (n & (1 << 5)) {
#line 178
      HplMsp430InterruptP__Port25__fired();
#line 178
      return;
    }
#line 179
  if (n & (1 << 6)) {
#line 179
      HplMsp430InterruptP__Port26__fired();
#line 179
      return;
    }
#line 180
  if (n & (1 << 7)) {
#line 180
      HplMsp430InterruptP__Port27__fired();
#line 180
      return;
    }
}

# 96 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(6)))  void sig_UART1RX_VECTOR(void )
#line 96
{
  uint8_t temp = U1RXBUF;

#line 98
  HplMsp430Usart1P__Interrupts__rxDone(temp);
}

# 153 "/home/ali/workspace/TinyOS-main/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}






static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state != /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 101 "/home/ali/workspace/TinyOS-main/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(4)))  void sig_UART1TX_VECTOR(void )
#line 101
{
  HplMsp430Usart1P__Interrupts__txDone();
}

