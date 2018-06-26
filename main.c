#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

#include <chprintf.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <shell.h>

#include "nanosdr.h"
#include "si5351.h"

#include <stm32f303xc.h>


static struct {
  int32_t rms[2];
  int16_t ave[2];
  int16_t min[2];
  int16_t max[2];

  uint32_t callback_count;
  int32_t last_counter_value;
  int32_t interval_cycles;
  int32_t busy_cycles;

  uint16_t fps_count;
  uint16_t fps;
  uint16_t overflow_count;
  uint16_t overflow;
} stat;

static void calc_stat(void);
static void measure_power_dbm(void);

static THD_WORKING_AREA(waThread1, 128);
static __attribute__((noreturn)) THD_FUNCTION(Thread1, arg)
{
    (void)arg;
    int count;
    chRegSetThreadName("blink");
    while (1) {
      systime_t time = 100;
      chThdSleepMilliseconds(time);

      calc_stat();      
      measure_power_dbm();
      disp_update_power();

      if (++count == 10) {
        stat.fps = stat.fps_count;
        stat.fps_count = 0;
        stat.overflow = stat.overflow_count;
        stat.overflow_count = 0;
        count = 0;
      }
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

static void cmd_tune(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq;
    if (argc != 1) {
        chprintf(chp, "usage: tune {frequency(Hz)}\r\n");
        return;
    }
    freq = atoi(argv[0]);
    set_tune(freq);

    uistat.freq = freq;
    uistat.mode = FREQ;
    disp_update();
}


int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];
int16_t tx_buffer[AUDIO_BUFFER_LEN * 2];

const buffer_ref_t buffers_table[BUFFERS_MAX] = {
  { BT_C_INTERLEAVE, AUDIO_BUFFER_LEN, rx_buffer,  NULL },
  { BT_IQ,           AUDIO_BUFFER_LEN, buffer[0],  buffer[1] },
  { BT_IQ,           AUDIO_BUFFER_LEN, buffer2[0], buffer2[1] },
  { BT_R_INTERLEAVE, AUDIO_BUFFER_LEN, tx_buffer,  NULL }
};

const char *agcmode_table[] = {
  "manual", "slow", "mid", "fast"
};

signal_process_func_t signal_process = am_demod;
int16_t mode_freq_offset = AM_FREQ_OFFSET;
int16_t mode_freqoffset_phasestep;
int16_t cw_tone_phasestep = PHASESTEP(800);
int32_t center_frequency;

// restored from/into flash memory
config_t config = {
  .magic = CONFIG_MAGIC,
  .dac_value = 1080,
  .agc = {
    .target_level = 6,
    .maximum_gain = 127
  },
  .uistat = {
    .mode = CHANNEL,
	.channel = 0,
    .freq = 567000,
	.digit = 3,
	.modulation = MOD_AM,
	.volume = 0,
	.rfgain = 40, // 0 ~ 95
	//.agcmode = AGC_MANUAL,
    .agcmode = AGC_MID,
    .cw_tone_freq = 800
  },
  .channels = {
    /*    freq, modulation */
    {   567000, MOD_AM },
    {   747000, MOD_AM },
    {  1287000, MOD_AM },
    {  1440000, MOD_AM },
    {  7100000, MOD_LSB },
    { 14100000, MOD_USB },
    { 21100000, MOD_USB },
    { 26800050, MOD_FM },
    { 27500050, MOD_FM },
    { 28400050, MOD_FM },
    {  2932000, MOD_USB },
    {  5628000, MOD_USB },
    {  6655000, MOD_USB },
    {  8951000, MOD_USB },
    { 10048000, MOD_USB },
    { 11330000, MOD_USB },
    { 13273000, MOD_USB },
    { 17904000, MOD_USB }
  },
  .button_polarity = 0x01,
  .freq_inverse = -1
};

struct {
  signal_process_func_t demod_func;
  int16_t freq_offset;
  int16_t fs;
  const char *name;
} mod_table[] = {
  { cw_demod, AM_FREQ_OFFSET,  48, "cw" },
  { lsb_demod,             0,  48, "lsb" },
  { usb_demod,             0,  48, "usb" },
  { am_demod, AM_FREQ_OFFSET,  48, "am" },
  { fm_demod,              0, 192, "fm" },
  { fm_demod_stereo,       0, 192, "fms" },
};

void set_modulation(modulation_t mod)
{
  if (mod >= MOD_MAX)
    return;

  set_fs(mod_table[mod].fs);
  signal_process = mod_table[mod].demod_func;

  mode_freq_offset = mod_table[mod].freq_offset;
  mode_freqoffset_phasestep = PHASESTEP(mode_freq_offset);
  cw_tone_phasestep = PHASESTEP(uistat.cw_tone_freq);
  
  uistat.modulation = mod;
  disp_update();
}

void
set_tune(int hz)
{
  center_frequency = hz - mode_freq_offset;
  si5351_set_frequency(center_frequency * 4);
}

static int current_fs = 48;

void
set_fs(int fs)
{
  if (fs != 48 && fs != 96 && fs != 192)
    return;

  if (fs != current_fs) {
    current_fs = fs;
    // stop WCLK,BCLK
    tlv320aic3204_stop();

    // then stop I2S
    i2sStopExchange(&I2SD2);
    // wait a second (not enough in 20ms)
    chThdSleepMilliseconds(40);
    // re-prepare I2S
    i2sStartExchange(&I2SD2);

    // enable WCLK,BCLK
    tlv320aic3204_set_fs(fs);
  }
}

void
save_config_current_channel(void)
{
  int channel = uistat.channel;
  config.channels[channel].freq = uistat.freq;
  config.channels[channel].modulation = uistat.modulation;
  
  config.uistat = uistat;
  config_save();
}

void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
  int32_t cnt_s = port_rt_get_counter_value();
  int32_t cnt_e;
  int16_t *p = &rx_buffer[offset];
  int16_t *q = &tx_buffer[offset];
  (void)i2sp;
  palSetPad(GPIOC, GPIOC_LED);

  (*signal_process)(p, q, n);

  cnt_e = port_rt_get_counter_value();
  stat.interval_cycles = cnt_s - stat.last_counter_value;
  stat.busy_cycles = cnt_e - cnt_s;
  stat.last_counter_value = cnt_s;

  stat.callback_count++;
  palClearPad(GPIOC, GPIOC_LED);
}

static const I2SConfig i2sconfig = {
  tx_buffer, // TX Buffer
  rx_buffer, // RX Buffer
  AUDIO_BUFFER_LEN * 2,
  i2s_end_callback, // tx callback
  NULL, // rx callback
  0, // i2scfgr
  2 // i2spr
};

static void tone_generate(int freq)
{
    int i;
    for (i = 0; i < AUDIO_BUFFER_LEN; i++) {
      int16_t x = (int16_t)(sin(2*M_PI * i * freq / FS) * 10000);
      tx_buffer[i*2  ] = x;
      tx_buffer[i*2+1] = x;
    }
}

static void cmd_tone(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq = 440;
    if (argc > 1) {
        chprintf(chp, "usage: tone {audio frequency(Hz)}\r\n");
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
    case 2:
      buf = buffer[0];
      break;
    case 3:
      buf = buffer2[0];
      break;
    default:
      chprintf(chp, "unknown source\r\n");
      return;
    }
  }

  //i2sStopExchange(&I2SD2);
  for (i = 0; i < AUDIO_BUFFER_LEN; ) {
    for (j = 0; j < 16; j++, i++) {
      chprintf(chp, "%04x ", 0xffff & (int)buf[i]);
    }
    chprintf(chp, "\r\n");
  }
  //i2sStartExchange(&I2SD2);
}

static void
calc_stat(void)
{
  int16_t *p = &rx_buffer[0];
  int16_t min0 = 0, min1 = 0;
  int16_t max0 = 0, max1 = 0;
  int32_t count = AUDIO_BUFFER_LEN;
  int i;
  float accx0 = 0, accx1 = 0;
  for (i = 0; i < AUDIO_BUFFER_LEN*2; i += 2) {
    if (min0 > p[i]) min0 = p[i];
    if (min1 > p[i+1]) min1 = p[i+1];
    if (max0 < p[i]) max0 = p[i];
    if (max1 < p[i+1]) max1 = p[i+1];
    float x0 = p[i];
    float x1 = p[i+1];
    accx0 += x0 * x0;
    accx1 += x1 * x1;
  }
  stat.rms[0] = sqrtf(accx0 / count);
  stat.rms[1] = sqrtf(accx1 / count);
  stat.min[0] = min0;
  stat.min[1] = min1;
  stat.max[0] = max0;
  stat.max[1] = max1;
}  

int16_t measured_power_dbm;

static void
measure_power_dbm(void)
{
  extern int log2_q31(int32_t x);
  int agcgain = uistat.rfgain;
  if (uistat.agcmode != AGC_MANUAL)
    agcgain = tlv320aic3204_get_left_agc_gain();
  
  int dbm =                    // fixed point 8.8 format
    6 * log2_q31(stat.rms[0])  // 6dB/bit
    - (agcgain << 7);          // 0.5dB/agcgain
  dbm -= 116 << 8;
  measured_power_dbm = dbm;
}

uint16_t adc_single_read(ADC_TypeDef *adc, uint32_t chsel)
{
  /* ADC setup */
  adc->ISR    = adc->ISR;
  adc->IER    = 0;
  adc->SMPR1  = ADC_SMPR1_SMP0_2; // 19.5 cycle
  adc->CFGR   = 0; // 12bit
  adc->SQR1   = chsel << 6;

  /* ADC conversion start.*/
  adc->CR |= ADC_CR_ADSTART;
  while (adc->CR & ADC_CR_ADSTART)
    ;

  return adc->DR;
}

#define ADC1_CHANNEL_TEMP 16
#define ADC1_CHANNEL_BAT  17
#define ADC1_CHANNEL_VREF 18

static void cmd_stat(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  chprintf(chp, "average: %d %d\r\n", stat.ave[0], stat.ave[1]);
  chprintf(chp, "rms: %d %d\r\n", stat.rms[0], stat.rms[1]);
  chprintf(chp, "min: %d %d\r\n", stat.min[0], stat.min[1]);
  chprintf(chp, "max: %d %d\r\n", stat.max[0], stat.max[1]);
  chprintf(chp, "callback count: %d\r\n", stat.callback_count);
  chprintf(chp, "load: %d%% (%d/%d)\r\n", stat.busy_cycles * 100 / stat.interval_cycles, stat.busy_cycles, stat.interval_cycles);
  chprintf(chp, "fps: %d\r\n", stat.fps);
  chprintf(chp, "overflow: %d\r\n", stat.overflow);
  int gain0 = tlv320aic3204_get_left_agc_gain();
  int gain1 = tlv320aic3204_get_right_agc_gain();
  chprintf(chp, "agc gain: %d %d\r\n", gain0, gain1);

  chprintf(chp, "fm stereo: %d %d\r\n", stereo_separate_state.sdi, stereo_separate_state.sdq);
  chprintf(chp, "  corr: %d %d %d\r\n", stereo_separate_state.corr, stereo_separate_state.corr_ave, stereo_separate_state.corr_std);
  chprintf(chp, "  int: %d\r\n", stereo_separate_state.integrator);

  chprintf(chp, "temp: %d\r\n", adc_single_read(ADC1, ADC1_CHANNEL_TEMP));
  chprintf(chp, "bat: %d\r\n", adc_single_read(ADC1, ADC1_CHANNEL_BAT));
  chprintf(chp, "vref: %d\r\n", adc_single_read(ADC1, ADC1_CHANNEL_VREF));
  
#if 0
  p = &tx_buffer[0];
  acc0 = acc1 = 0;
  for (i = 0; i < AUDIO_BUFFER_LEN*2; i += 2) {
    acc0 += p[i];
    acc1 += p[i+1];
  }
  ave0 = acc0 / count;
  ave1 = acc1 / count;
  chprintf(chp, "audio average: %d %d\r\n", ave0, ave1);
#endif
}

static void cmd_power(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "power: %d.%01ddBm\r\n", measured_power_dbm >> 8,
           ((measured_power_dbm&0xff) * 10) >> 8);
}

static void cmd_impedance(BaseSequentialStream *chp, int argc, char *argv[])
{
    int imp;
    if (argc != 1) {
        chprintf(chp, "usage: imp {gain(1-3)}\r\n");
        return;
    }

    imp = atoi(argv[0]);
    tlv320aic3204_set_impedance(imp);
}

static void cmd_gain(BaseSequentialStream *chp, int argc, char *argv[])
{
    int gain;
    if (argc != 1 && argc != 2 && argc != 3) {
        chprintf(chp, "usage: gain {pga gain(0-95)} [digital gain(-24-40)] [adjust]\r\n");
        return;
    }

    gain = atoi(argv[0]);
    tlv320aic3204_set_gain(gain, gain);
    uistat.rfgain = gain;
    
    if (argc >= 2) {
      int adjust = 0;
      gain = atoi(argv[1]);
      if (argc == 3) {
        adjust = atoi(argv[2]);
      }
      tlv320aic3204_set_digital_gain(gain, gain + adjust);
      uistat.rfgain += gain;
    }

    disp_update();
}

static void cmd_phase(BaseSequentialStream *chp, int argc, char *argv[])
{
    int value;
    if (argc != 1) {
        chprintf(chp, "usage: phase {adjust value(-128-127)}\r\n");
        return;
    }

    value = atoi(argv[0]);
    tlv320aic3204_set_adc_phase_adjust(value);
}

static void cmd_finegain(BaseSequentialStream *chp, int argc, char *argv[])
{
    int g1 = 0, g2 = 0;
    if (argc != 1 && argc != 2) {
        chprintf(chp, "usage: gainadjust {gain1 gain2} (0 - -4)\r\n");
        return;
    }
    g1 = atoi(argv[0]);
    if (argc == 2) {
      g2 = atoi(argv[1]);
    }
    tlv320aic3204_set_adc_fine_gain_adjust(g1, g2);
}

static void cmd_iqbal(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "usage: iqbal {coeff}\r\n");
        return;
    }
    double value = config.freq_inverse - (double)atoi(argv[0]) / 10000.0;
    tlv320aic3204_config_adc_filter2(value);
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
    uistat.volume = gain;
    disp_update();
}

static void cmd_dcreject(BaseSequentialStream *chp, int argc, char *argv[])
{
    int value;
    if (argc != 1) {
        chprintf(chp, "usage: dcreject {0|1}\r\n");
        return;
    }
    value = atoi(argv[0]);
    tlv320aic3204_config_adc_filter(value);
}

static void cmd_dac(BaseSequentialStream *chp, int argc, char *argv[])
{
    int value;
    if (argc != 1) {
        chprintf(chp, "usage: dac {value(0-4095)}\r\n");
        chprintf(chp, "current value: %d\r\n", config.dac_value);
        return;
    }
    value = atoi(argv[0]);
    config.dac_value = value;
    dacPutChannelX(&DACD1, 0, value);
}

static void cmd_agc(BaseSequentialStream *chp, int argc, char *argv[])
{
    const char *cmd;
    if (argc == 0) {
      chprintf(chp, "usage: agc {cmd} [args...]\r\n");
      chprintf(chp, "\tmanual/slow/mid/fast\r\n");
      chprintf(chp, "\tenable/disable\r\n");
      chprintf(chp, "\tlevel {0-7}\r\n");
      chprintf(chp, "\thysteresis {0-3}\r\n");
      chprintf(chp, "\tattack {0-31} [scale:0-7]\r\n");
      chprintf(chp, "\tdecay {0-31} [scale:0-7]\r\n");
      chprintf(chp, "\tmaxgain {0-116}\r\n");
      return;
    }

    cmd = argv[0];
    if (strncmp(cmd, "manual", 3) == 0) {
      set_agc_mode(AGC_MANUAL);
    } else if (strncmp(cmd, "slow", 2) == 0) {
      set_agc_mode(AGC_SLOW);
    } else if (strncmp(cmd, "mid", 2) == 0) {
      set_agc_mode(AGC_MID);
    } else if (strncmp(cmd, "fast", 2) == 0) {
      set_agc_mode(AGC_FAST);
    } else if (strncmp(cmd, "di", 2) == 0) {
      tlv320aic3204_agc_config(NULL);
    } else if (strncmp(cmd, "enable", 2) == 0) {
      tlv320aic3204_agc_config(&config.agc);
    } else if (strncmp(cmd, "le", 2) == 0 && argc == 2) {
      config.agc.target_level = atoi(argv[1]);
      tlv320aic3204_agc_config(&config.agc);
    } else if (strncmp(cmd, "hy", 2) == 0 && argc == 2) {
      config.agc.gain_hysteresis = atoi(argv[1]);
      tlv320aic3204_agc_config(&config.agc);
    } else if (strncmp(cmd, "at", 2) == 0 && argc >= 2) {
      config.agc.attack = atoi(argv[1]);
      if (argc >= 3)
        config.agc.attack_scale = atoi(argv[2]);
      tlv320aic3204_agc_config(&config.agc);
    } else if (strncmp(cmd, "de", 2) == 0 && argc >= 2) {
      config.agc.decay = atoi(argv[1]);
      if (argc >= 3)
        config.agc.decay_scale = atoi(argv[2]);
      tlv320aic3204_agc_config(&config.agc);
    } else if (strncmp(cmd, "max", 3) == 0 && argc >= 2) {
      config.agc.maximum_gain = atoi(argv[1]);
      tlv320aic3204_agc_config(&config.agc);
    }
}

void set_agc_mode(int mode)
{
  if (mode == AGC_MANUAL) {
    tlv320aic3204_agc_config(NULL);
    uistat.agcmode = mode;
    disp_update();
    return;
  }
  switch (mode) {
  case AGC_FAST:
    config.agc.decay = 0;
    config.agc.decay_scale = 0;
    break;
  case AGC_MID:
    config.agc.decay = 7;
    config.agc.decay_scale = 0;
    break;
  case AGC_SLOW:
    config.agc.decay = 31;
    config.agc.decay_scale = 4;
    break;
  }
  tlv320aic3204_agc_config(&config.agc);
  uistat.agcmode = mode;
  disp_update();
}

static void cmd_mode(BaseSequentialStream *chp, int argc, char *argv[])
{
    const char *cmd;
    if (argc == 0) {
      chprintf(chp, "usage: mode {lsb|usb|am|fm|fms}\r\n");
      return;
    }

    cmd = argv[0];
    if (strncmp(cmd, "am", 1) == 0) {
      set_modulation(MOD_AM);
    } else if (strncmp(cmd, "lsb", 1) == 0) {
      set_modulation(MOD_LSB);
    } else if (strncmp(cmd, "usb", 1) == 0) {
      set_modulation(MOD_USB);
    } else if (strncmp(cmd, "cw", 1) == 0) {
      set_modulation(MOD_CW);
    } else if (strncmp(cmd, "fms", 3) == 0) {
      set_modulation(MOD_FM_STEREO);
    } else if (strncmp(cmd, "fm", 1) == 0) {
      set_modulation(MOD_FM);
    }
}

static void cmd_cwtone(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq = 0;
    if (argc == 0) {
        chprintf(chp, "%d\r\n", uistat.cw_tone_freq);
        return;
    }

    if (argc == 1)
        freq = atoi(argv[0]);

    if (freq == 0) {
        chprintf(chp, "usage: cwtone {audio frequency(Hz)}\r\n");
        return;
    }
    uistat.cw_tone_freq = freq;
    cw_tone_phasestep = PHASESTEP(freq);
}

static void cmd_fs(BaseSequentialStream *chp, int argc, char *argv[])
{
  int fs = 0;
  
  if (argc == 1) {
    fs = atoi(argv[0]);
  }

  if (fs == 48 || fs == 96 || fs == 192) {
    set_fs(fs);
    uistat.fs = fs;
  } else {
    chprintf(chp, "usage: fs {48|96|192}\r\n");
  }
}

static void cmd_winfunc(BaseSequentialStream *chp, int argc, char *argv[])
{
    int type;
    if (argc == 0) {
      chprintf(chp, "usage: winfunc {0|1|2}\r\n");
      return;
    }

    type = atoi(argv[0]);
    set_window_function(type);
}

static void cmd_show(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0 || strcmp(argv[0], "all") == 0) {
    chprintf(chp, "tune: %d\r\n", uistat.freq);
    chprintf(chp, "volume: %d\r\n", uistat.volume);
    chprintf(chp, "mode: %s\r\n", mod_table[uistat.modulation].name);
    chprintf(chp, "gain: %d\r\n", uistat.rfgain);
    chprintf(chp, "channel: %d\r\n", uistat.channel);
    chprintf(chp, "agc: %s\r\n", agcmode_table[uistat.agcmode]);
  } else if (strcmp(argv[0], "tune") == 0) {
    chprintf(chp, "%d\r\n", uistat.freq);
  } else if (strcmp(argv[0], "volume") == 0) {
    chprintf(chp, "%d\r\n", uistat.volume);
  } else if (strcmp(argv[0], "mode") == 0) {
    chprintf(chp, "%s\r\n", mod_table[uistat.modulation].name);
  } else if (strcmp(argv[0], "gain") == 0) {
    chprintf(chp, "%d\r\n", uistat.rfgain);
  } else if (strcmp(argv[0], "channel") == 0) {
    chprintf(chp, "%d\r\n", uistat.channel);
  } else if (strcmp(argv[0], "agc") == 0) {
    chprintf(chp, "agc: %s\r\n", agcmode_table[uistat.agcmode]);
  } else {
    chprintf(chp, "usage: show [all|tune|volume|gain|agc]\r\n");
    return;
  }
}

static void cmd_channel(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 0) {
      chprintf(chp, "usage: channel [save|list] [n(0-99)]\r\n");
      return;
    }

    int channel;
    if (strncmp(argv[0], "save", 1) == 0) {
      channel = uistat.channel;
      if (argc >= 2) {
        channel = atoi(argv[1]);
        if (channel < 0 || channel >= CHANNEL_MAX) {
          chprintf(chp, "specified channel is out of range\r\n");
          return;
        }
      } else {
        chprintf(chp, "channel saved on %d\r\n", channel);
      }
      config.channels[channel].freq = uistat.freq;
      config.channels[channel].modulation = uistat.modulation;
    } else if (strncmp(argv[0], "list", 1) == 0) {
      for (channel = 0; channel < CHANNEL_MAX; channel++) {
        if (config.channels[channel].freq) {
          chprintf(chp, "%d %d %s\r\n", channel,
                   config.channels[channel].freq,
                   mod_table[config.channels[channel].modulation].name);
        }
      }
    } else {
      channel = atoi(argv[0]);
      if (channel < 0 || channel >= CHANNEL_MAX) {
        chprintf(chp, "specified channel is out of range\r\n");
        return;
      }
      recall_channel(channel);
      uistat.mode = CHANNEL;
      uistat.channel = channel;
      disp_update();
    }
}

static void cmd_revision(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) {
    chprintf(chp, "usage: revision {rev}\r\n");
    return;
  }
  int rev = atoi(argv[0]);
  switch (rev) {
  case 0:
    config.freq_inverse = 1;
    config.button_polarity = 0x00;
    break;
  case 1:
    config.freq_inverse = -1;
    config.button_polarity = 0x01;
    break;
  default:
    chprintf(chp, "unknown revision\r\n");
    break;
  }
}

static void cmd_save(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  config.uistat = uistat;
  config_save();
  
  chprintf(chp, "Config saved.\r\n");
}

static void cmd_clearconfig(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc != 1) {
    chprintf(chp, "usage: clearconfig {protection key}\r\n");
    return;
  }

  if (strcmp(argv[0], "1234") != 0) {
    chprintf(chp, "Key unmatched.\r\n");
    return;
  }

  clear_all_config_prop_data();
  chprintf(chp, "Config and all cal data cleared.\r\n");
}

static void cmd_uitest(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  int i;
  for (i = 0; i < 100; i++) {
    //extern int btn_check(void);
    //int n = btn_check();
    //extern int read_buttons(void);
    //int n = read_buttons();
    extern int enc_count;
    chprintf(chp, "%d\r\n", enc_count);
    chThdSleepMilliseconds(100);
  }
}

static const ShellCommand commands[] =
{
    { "reset", cmd_reset },
    { "freq", cmd_freq },
    { "tune", cmd_tune },
    { "dac", cmd_dac },
    { "uitest", cmd_uitest },
    { "tone", cmd_tone },
    { "cwtone", cmd_cwtone },
    { "data", cmd_data },
    { "stat", cmd_stat },
    { "gain", cmd_gain },
    { "volume", cmd_volume },
    { "agc", cmd_agc },
    { "iqbal", cmd_iqbal },
    { "dcreject", cmd_dcreject },
    { "imp", cmd_impedance },
    { "mode", cmd_mode },
    { "fs", cmd_fs },
    { "winfunc", cmd_winfunc },
    { "show", cmd_show },
    { "power", cmd_power },
    { "channel", cmd_channel },
    { "revision", cmd_revision },
    { "save", cmd_save },
    { "clearconfig", cmd_clearconfig },
    { "phase", cmd_phase },
    { "finegain", cmd_finegain },
    { NULL, NULL }
};

static THD_WORKING_AREA(waThread2, 512);
static __attribute__((noreturn)) THD_FUNCTION(Thread2, arg)
{
    (void)arg;
    chRegSetThreadName("button");
    while (1)
    {
      disp_process();
      ui_process();
      chThdSleepMilliseconds(10);
      stat.fps_count++;

      {
        int flag = tlv320aic3204_get_sticky_flag_register();
        if (flag & AIC3204_STICKY_ADC_OVERFLOW)
          stat.overflow_count++;
      }
    }
}


static DACConfig dac1cfg1 = {
  //init:         2047U,
  init:         1080U,
  datamode:     DAC_DHRM_12BIT_RIGHT
};


#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

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

  /* restore config */
  config_recall();

  if (config.button_polarity != 0) {
    // pullup for revision 1 board
    palSetGroupMode(GPIOA, 1, 0, PAL_MODE_INPUT_PULLUP);
    palSetGroupMode(GPIOB, 6, 0, PAL_MODE_INPUT_PULLUP);
  }
  
  // copy uistat from uistat
  uistat = config.uistat;

  /*
   * Starting DAC1 driver, setting up the output pin as analog as suggested
   * by the Reference Manual.
   */
  dac1cfg1.init = config.dac_value;
  dacStart(&DACD1, &dac1cfg1);

  /*
   * Activates the ADC1 driver and the temperature sensor.
   */
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTS(&ADCD1);
  adcSTM32EnableVBAT(&ADCD1);
  adcSTM32EnableVREF(&ADCD1);

  
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
  //chThdSleepMilliseconds(200);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

#if 1
  /*
   * I2S Initialize
   */
  tlv320aic3204_init();

  i2sInit();
  i2sObjectInit(&I2SD2);
  i2sStart(&I2SD2, &i2sconfig);
  i2sStartExchange(&I2SD2);
#endif
  
  dsp_init();
  
  /*
   * SPI LCD Initialize
   */
  ili9341_init();
  //ili9341_test(4);
  //ili9341_test(3);

#if 1
  /*
   * Initialize display
   */
  disp_init();
#endif
  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

#if 1
  ui_init();
#endif

  tlv320aic3204_config_adc_filter2(config.freq_inverse /* + 0.129 */); // enable DC reject
  //tlv320aic3204_config_adc_filter(1); // enable DC reject

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
