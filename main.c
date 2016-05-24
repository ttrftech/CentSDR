#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

#include <chprintf.h>
#include <stdlib.h>
#include <math.h>
#include <shell.h>

#include "nanosdr.h"
#include "si5351.h"

#include <stm32f303xc.h>


int count;
int updated;


static THD_WORKING_AREA(waThread1, 128);
static __attribute__((noreturn)) THD_FUNCTION(Thread1, arg)
{
    (void)arg;
    chRegSetThreadName("blink");
    while (1)
    {
    systime_t time = serusbcfg.usbp->state == USB_ACTIVE ? 250 : 500;
	//palClearPad(GPIOC, GPIOC_LED);
	chThdSleepMilliseconds(time);
	//palSetPad(GPIOC, GPIOC_LED);
	chThdSleepMilliseconds(time);
    }
}

static void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    chprintf(chp, "Performing reset\r\n");

    //rccEnableWWDG(FALSE);

    WWDG->CFR = 0x60;
    WWDG->CR = 0xff;

    while (1)
	;
}

static const I2CConfig i2ccfg = {
  0x00902025, //voodoo magic
  //0x00420F13,  // 100kHz @ 72MHz
  0,
  0
};

void I2CWrite(int addr, uint8_t d0, uint8_t d1)
{
    uint8_t buf[] = { d0, d1 };
    i2cAcquireBus(&I2CD1);
    (void)i2cMasterTransmitTimeout(&I2CD1, addr, buf, 2, NULL, 0, 1000);
    i2cReleaseBus(&I2CD1);
}

int I2CRead(int addr, uint8_t d0)
{
    uint8_t buf[] = { d0 };
    i2cAcquireBus(&I2CD1);
    i2cMasterTransmitTimeout(&I2CD1, addr, buf, 1, buf, 1, 1000);
    i2cReleaseBus(&I2CD1);
    return buf[0];
}

static void cmd_freq(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq;
    if (argc != 1) {
        chprintf(chp, "usage: freq {frequency(Hz)}\r\n");
        return;
    }
    freq = atoi(argv[0]);
    si5351_set_frequency(freq);
}


#define AUDIO_BUFFER_LEN 4800

static struct {
  int16_t rms[2];
  int16_t ave[2];
  int callback_count;

  int32_t last_counter_value;
  int32_t interval_cycles;
  int32_t busy_cycles;
} stat;

__attribute__ ( ( always_inline ) ) __STATIC_INLINE float _VSQRTF(float op1) {
  float result;
  __ASM volatile ("vsqrt.f32 %0,%1" : "=w"(result) : "w"(op1) );
  return(result);
}

int16_t rx_buffer[AUDIO_BUFFER_LEN];
int16_t tx_buffer[AUDIO_BUFFER_LEN];

void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
  int32_t cnt_s = port_rt_get_counter_value();
  int32_t cnt_e;
  int16_t *p = &rx_buffer[offset];
  int16_t *q = &tx_buffer[offset];
  uint32_t i;
  (void)i2sp;
  palSetPad(GPIOC, GPIOC_LED);
  stat.callback_count++;

#if 1
  for (i = 0; i < n; i += 2) {
    int32_t x = p[i];
    int32_t y = p[i+1];
    int32_t z;
#define DCOFFSET 16383
    x = x + DCOFFSET;
    y = y + DCOFFSET;
    z = (int16_t)_VSQRTF((float)(x*x+y*y)) - DCOFFSET;
    //z = (int16_t)sqrtf(x*x+y*y) - DCOFFSET;
    q[i] = q[i+1] = z;
    //q[i] = x;
    //q[i+1] = y;
  }
#endif

  cnt_e = port_rt_get_counter_value();
  stat.interval_cycles = cnt_s - stat.last_counter_value;
  stat.busy_cycles = cnt_e - cnt_s;
  stat.last_counter_value = cnt_s;

  palClearPad(GPIOC, GPIOC_LED);
}

static const I2SConfig i2sconfig = {
  tx_buffer, // TX Buffer
  rx_buffer, // RX Buffer
  AUDIO_BUFFER_LEN,
  i2s_end_callback, // tx callback
  NULL, // rx callback
  0, // i2scfgr
  2 // i2spr
};

extern void tlv320aic3204_init(void);

static void cmd_i2sinit(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)chp;
  (void)argc;
  (void)argv;
  tlv320aic3204_init();
}

static void cmd_i2sstart(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)chp;
  (void)argc;
  (void)argv;
  i2sStartExchange(&I2SD2);
}

static void cmd_i2sstop(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)chp;
  (void)argc;
  (void)argv;
  i2sStopExchange(&I2SD2);
}

#define FS 48000

static void tone_generate(int freq)
{
    int i;
    for (i = 0; i < AUDIO_BUFFER_LEN/2; i++) {
      int16_t x = (int16_t)(sin(2*M_PI * i * freq / FS) * 10000);
      tx_buffer[i*2  ] = x;
      tx_buffer[i*2+1] = x;
    }
}

static void cmd_audio(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq = 440;
    if (argc > 1) {
        chprintf(chp, "usage: freq {frequency(kHz)}\r\n");
        return;
    } else if (argc == 1) {
      freq = atoi(argv[0]);
    }
    
    I2SD2.spi->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
    //I2SD2.spi->CR2 = 0;
    tone_generate(freq);
    I2SD2.spi->I2SCFGR |= SPI_I2SCFGR_I2SE;
    //I2SD2.spi->CR2 = SPI_CR2_TXDMAEN;
}

static void cmd_data(BaseSequentialStream *chp, int argc, char *argv[])
{
  int i, j;
  (void)argc;
  (void)argv;
  int16_t *buf = rx_buffer;
  
  if (argc > 0) {
    switch (atoi(argv[0])) {
    case 0:
      break;
    case 1:
      buf = tx_buffer;
      break;
    default:
      chprintf(chp, "unknown source\r\n");
      return;
    }
  }

  i2sStopExchange(&I2SD2);
  for (i = 0; i < AUDIO_BUFFER_LEN/2; ) {
    for (j = 0; j < 16; j++, i++) {
      chprintf(chp, "%04x ", 0xffff & (int)buf[i]);
    }
    chprintf(chp, "\r\n");
  }
  i2sStartExchange(&I2SD2);
}

static void cmd_stat(BaseSequentialStream *chp, int argc, char *argv[])
{
  int16_t *p = &rx_buffer[0];
  int32_t acc0, acc1;
  int32_t ave0, ave1;
  int32_t count = AUDIO_BUFFER_LEN / 2;
  int i;
  (void)argc;
  (void)argv;
  acc0 = acc1 = 0;
  for (i = 0; i < AUDIO_BUFFER_LEN; i += 2) {
    acc0 += p[i];
    acc1 += p[i+1];
  }
  ave0 = acc0 / count;
  ave1 = acc1 / count;
  acc0 = acc1 = 0;
  for (i = 0; i < AUDIO_BUFFER_LEN; i += 2) {
    acc0 += (p[i] - ave0)*(p[i] - ave0);
    acc1 += (p[i+1] - ave1)*(p[i+1] - ave1);
  }
  stat.rms[0] = sqrt(acc0 / count);
  stat.rms[1] = sqrt(acc1 / count);
  stat.ave[0] = ave0;
  stat.ave[1] = ave1;

  chprintf(chp, "average: %d %d\r\n", stat.ave[0], stat.ave[1]);
  chprintf(chp, "rms: %d %d\r\n", stat.rms[0], stat.rms[1]);
  chprintf(chp, "callback count: %d\r\n", stat.callback_count);
  chprintf(chp, "interval cycle: %d\r\n", stat.interval_cycles);
  chprintf(chp, "busy cycle: %d\r\n", stat.busy_cycles);
  chprintf(chp, "load: %d\r\n", stat.busy_cycles * 100 / stat.interval_cycles);
}

extern void tlv320aic3204_set_gain(int gain);
extern void tlv320aic3204_set_volume(int gain);

static void cmd_gain(BaseSequentialStream *chp, int argc, char *argv[])
{
    int gain;
    if (argc != 1) {
        chprintf(chp, "usage: gain {gain(0-95)}\r\n");
        return;
    }

    gain = atoi(argv[0]);
    tlv320aic3204_set_gain(gain);
}

static void cmd_volume(BaseSequentialStream *chp, int argc, char *argv[])
{
    int gain;
    if (argc != 1) {
        chprintf(chp, "usage: volume {gain(-7-29)}\r\n");
        return;
    }

    gain = atoi(argv[0]);
    tlv320aic3204_set_volume(gain);
}

static int ppm = 28430;

void
set_tune(int hz)
{
  si5351_set_frequency(4*hz + (((4*(int32_t)hz/1000)*ppm)/1000000));
}

static void cmd_tune(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq;
    if (argc != 1) {
        chprintf(chp, "usage: tune {frequency(kHz)}\r\n");
        return;
    }
    freq = atoi(argv[0]);
    set_tune(freq);
}

static void cmd_ppm(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "usage: ppm {value}\r\n");
        chprintf(chp, "current: %d\r\n", ppm);
        return;
    }
    ppm = atoi(argv[0]);
}

#define BIT_PUSH	5
#define BIT_DOWN0	4
#define BIT_DOWN1	1
#define BIT_UP0 	7
#define BIT_UP1 	6

static void cmd_port(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    chprintf(chp, "current: %x %d\r\n", palReadPort(GPIOA) & 0b11110010, count);
}

#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

static const ShellCommand commands[] =
{
    { "reset", cmd_reset },
    { "freq", cmd_freq },
    { "tune", cmd_tune },
    { "ppm", cmd_ppm },
    { "i2sinit", cmd_i2sinit },
    { "i2sstart", cmd_i2sstart },
    { "i2sstop", cmd_i2sstop },
    { "audio", cmd_audio },
    { "data", cmd_data },
    { "stat", cmd_stat },
    { "gain", cmd_gain },
    { "volume", cmd_volume },
    { "port", cmd_port },
    { NULL, NULL }
};

static const ShellConfig shell_cfg1 =
{
    (BaseSequentialStream *)&SDU1,
    commands
};

//static condition_variable_t condvar_button;

static THD_WORKING_AREA(waThread2, 512);
static __attribute__((noreturn)) THD_FUNCTION(Thread2, arg)
{
    (void)arg;
    chRegSetThreadName("button");
    while (1)
    {
      ui_process();
      chThdSleepMilliseconds(100);
    }
#if 0
    int prev = palReadPort(GPIOA) & 0b11110010;
    while (1)
    {
      int updated = 0;
      int curr = palReadPort(GPIOA) & 0b11110010;
#define BTN(bit)   ((curr ^ prev) & (1<<(bit))) && ((curr & (1<<(bit))) == 0)
      if (BTN(BIT_PUSH)) {
        count = 0; updated = 1;
      }
      if (BTN(BIT_UP0)) {
        count++; updated = 1;
      }
      if (BTN(BIT_UP1)) {
        count+=10; updated = 1;
      }
      if (BTN(BIT_DOWN0)) {
        count--; updated = 1;
      }
      if (BTN(BIT_DOWN1)) {
        count-=10; updated = 1;
      }

      if (updated) {
        char buf[16];
        itoa(count, buf, 10);
        i2clcd_pos(0, 1);
        i2clcd_str(buf);
        i2clcd_str("        ");
      }
      prev = curr;
      chThdSleepMilliseconds(100);
    }
#endif
}

void ext_callback(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    switch (channel) {
    case BIT_UP0: count++; break;
    case BIT_UP1: count += 10; break;
    case BIT_DOWN0: count--; break;
    case BIT_DOWN1: count -= 10; break;
      //case BIT_PUSH: count = 0; break;
    }
    updated = 1;
    //chCondSignalI(&condvar_button);
#if 0
    BaseSequentialStream *chp = &SDU1;
    if (SDU1.config->usbp->state == USB_ACTIVE) {
      chprintf(chp, "EXTI interrupt: %d\r\n", channel);
    }
#endif
}

static const EXTConfig extconf = {
  {
    { 0, NULL },
    { EXT_MODE_GPIOA | EXT_CH_MODE_FALLING_EDGE, ext_callback },
    { 0, NULL },
    { 0, NULL },
    { EXT_MODE_GPIOA | EXT_CH_MODE_FALLING_EDGE, ext_callback },
    { EXT_MODE_GPIOA | EXT_CH_MODE_FALLING_EDGE, ext_callback },
    { EXT_MODE_GPIOA | EXT_CH_MODE_FALLING_EDGE, ext_callback },
    { EXT_MODE_GPIOA | EXT_CH_MODE_FALLING_EDGE, ext_callback },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL }
  }
};

/*
 * Application entry point.
 */
int __attribute__((noreturn)) main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  i2cStart(&I2CD1, &i2ccfg);
#if 0
  extStart(&EXTD1, &extconf);
  extChannelEnable(&EXTD1, BIT_UP0);
  extChannelEnable(&EXTD1, BIT_UP1);
  extChannelEnable(&EXTD1, BIT_DOWN0);
  extChannelEnable(&EXTD1, BIT_DOWN1);
  extChannelEnable(&EXTD1, BIT_PUSH);
  //chCondObjectInit(&condvar_button);
#endif
  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  //chThdSleepMilliseconds(200);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /*
   * I2S Initialize
   */
  tlv320aic3204_init();
  i2sInit();
  i2sObjectInit(&I2SD2);
  i2sStart(&I2SD2, &i2sconfig);
  i2sStartExchange(&I2SD2);

  //tone_generate(440);

  //si5351_set_frequency(48001);
  //si5351_set_frequency(567*4); // NHK1
  set_tune(567000); // NHK1

  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
#if 0
  i2clcd_init();
  i2clcd_str("FriskSDR");
  i2clcd_pos(0, 1);
  i2clcd_str("Hello");
#endif
  ui_init();

  /*
   * Creates the button thread.
   */
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  /*
   * Normal main() thread activity, spawning shells.
   */
  while (true) {
    if (SDU1.config->usbp->state == USB_ACTIVE) {
      thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                              "shell", NORMALPRIO + 1,
                                              shellThread, (void *)&shell_cfg1);
      chThdWait(shelltp);               /* Waiting termination.             */
    }
    chThdSleepMilliseconds(1000);
  }
}
