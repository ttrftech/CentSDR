#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "ch_test.h"

#include <chprintf.h>
#include <stdlib.h>
#include <math.h>
#include <shell.h>

#include "si5351.h"

#include <stm32f303xc.h>

static THD_WORKING_AREA(waThread1, 128);
static __attribute__((noreturn)) THD_FUNCTION(Thread1, arg)
{
    (void)arg;
    chRegSetThreadName("blink");
    while (1)
    {
    systime_t time = serusbcfg.usbp->state == USB_ACTIVE ? 250 : 500;
	palClearPad(GPIOC, GPIOC_LED);
	chThdSleepMilliseconds(time);
	palSetPad(GPIOC, GPIOC_LED);
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
        chprintf(chp, "usage: freq {frequency(kHz)}\r\n");
        return;
    }
    freq = atoi(argv[0]);
    si5351_set_frequency(freq);
}


#define AUDIO_BUFFER_LEN 4096

int16_t audio_buffer[AUDIO_BUFFER_LEN];
void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
}

static const I2SConfig i2sconfig = {
  audio_buffer, // TX Buffer
  audio_buffer, // RX Buffer
  AUDIO_BUFFER_LEN,
  i2s_end_callback,
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

static void cmd_audio(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq = 440;
    int i;
    if (argc > 1) {
        chprintf(chp, "usage: freq {frequency(kHz)}\r\n");
        return;
    } else if (argc == 1) {
      freq = atoi(argv[0]);
    }

    for (i = 0; i < AUDIO_BUFFER_LEN/2; i++) {
      int16_t x = (int16_t)(sin(6.28 * i * freq / FS) * 10000);
      audio_buffer[i*2  ] = x;
      audio_buffer[i*2+1] = x;
    }
}


#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

static const ShellCommand commands[] =
{
    { "reset", cmd_reset },
    { "freq", cmd_freq },
    { "i2sinit", cmd_i2sinit },
    { "i2sstart", cmd_i2sstart },
    { "i2sstop", cmd_i2sstop },
    { "audio", cmd_audio },
    { NULL, NULL }
};

static const ShellConfig shell_cfg1 =
{
    (BaseSequentialStream *)&SDU1,
    commands
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
  chThdSleepMilliseconds(1500);
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

  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

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
