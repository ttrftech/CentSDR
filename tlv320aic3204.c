#include "hal.h"
#include "nanosdr.h"

#define REFCLK_8000KHZ
#define AIC3204_ADDR 0x18

#define wait_ms(ms)     chThdSleepMilliseconds(ms)

static void
tlv320aic3204_write(uint8_t reg, uint8_t dat)
{
  int addr = AIC3204_ADDR;
  uint8_t buf[] = { reg, dat };
  i2cAcquireBus(&I2CD1);
  (void)i2cMasterTransmitTimeout(&I2CD1, addr, buf, 2, NULL, 0, 1000);
  i2cReleaseBus(&I2CD1);
}

static void
tlv320aic3204_bulk_write(const uint8_t *buf, int len)
{
  int addr = AIC3204_ADDR;
  i2cAcquireBus(&I2CD1);
  (void)i2cMasterTransmitTimeout(&I2CD1, addr, buf, len, NULL, 0, 1000);
  i2cReleaseBus(&I2CD1);
}

static int
tlv320aic3204_read(uint8_t d0)
{
  int addr = AIC3204_ADDR;
  uint8_t buf[] = { d0 };
  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, addr, buf, 1, buf, 1, 1000);
  i2cReleaseBus(&I2CD1);
  return buf[0];
}


static void
tlv320aic3204_config(const uint8_t *data)
{
  const uint8_t *p = data;
  while (*p) {
    uint8_t len = *p++;
    tlv320aic3204_bulk_write(p, len);
    p += len;
  }
}

static const uint8_t conf_data_pll[] = {
  // len, ( reg, data ), 
  2, 0x00, 0x00, /* Initialize to Page 0 */
  2, 0x01, 0x01, /* Initialize the device through software reset */
  2, 0x04, 0x43, /* PLL Clock High, MCLK, PLL */
#ifdef REFCLK_8000KHZ
  /* 8.000MHz*10.7520 = 86.016MHz, 86.016MHz/(2*7*128) = 48kHz */
  2, 0x05, 0x91, /* Power up PLL, P=1,R=1 */
  2, 0x06, 0x0a, /* J=10 */
  2, 0x07, 29,    /* D=7520 = (29<<8) + 96 */
  2, 0x08, 96,
#endif
#ifdef REFCLK_12000KHZ
  /* 12.000MHz*7.1680 = 86.016MHz, 86.016MHz/(2*7*128) = 48kHz */
  2, 0x05, 0x91, /* Power up PLL, P=1,R=1 */
  2, 0x06, 0x07, /* J=7 */
  2, 0x07, 6,    /* D=1680 = (6<<8) + 144 */
  2, 0x08, 144,
#endif
#ifdef REFCLK_19200KHZ
  /* 19.200MHz*4.48 = 86.016MHz, 86.016MHz/(2*7*128) = 48kHz */
  2, 0x05, 0x91, /* Power up PLL, P=1,R=1 */
  2, 0x06, 0x04, /* J=4 */
  2, 0x07, 18,    /* D=4800 = (18<<8) + 192 */
  2, 0x08, 192,
#endif
  0 // sentinel
};

// default fs=48kHz
static const uint8_t conf_data_clk[] = {
  2, 0x0b, 0x82, /* Power up the NDAC divider with value 2 */
  2, 0x0c, 0x87, /* Power up the MDAC divider with value 7 */
  2, 0x0d, 0x00, /* Program the OSR of DAC to 128 */
  2, 0x0e, 0x80,
  2, 0x3c, 0x08, /* Set the DAC Mode to PRB_P8 */
  //2, 0x3c, 25, /* Set the DAC Mode to PRB_P25 */
  2, 0x1b, 0x0c, /* Set the BCLK,WCLK as output */    
  2, 0x1e, 0x80 + 28, /* Enable the BCLKN divider with value 28 */
  2, 0x25, 0xee, /* DAC power up */

  2, 0x12, 0x82, /* Power up the NADC divider with value 2 */
  2, 0x13, 0x87, /* Power up the MADC divider with value 7 */
  2, 0x14, 0x80, /* Program the OSR of ADC to 128 */
  2, 0x3d, 0x01, /* Select ADC PRB_R1 */
  0 // sentinel
};

static const uint8_t conf_data_clk_96kHz[] = {
  2, 0x0b, 0x82, /* Power up the NDAC divider with value 2 */
  2, 0x0c, 0x87, /* Power up the MDAC divider with value 7 */
  2, 0x0d, 0x00, /* Program the OSR of DAC to 64 */
  2, 0x0e, 0x40,
  2, 0x3c, 0x08, /* Set the DAC Mode to PRB_P8 */
  2, 0x1b, 0x0c, /* Set the BCLK,WCLK as output */    
  2, 0x1e, 0x80 + 14, /* Enable the BCLKN divider with value 14 */
  2, 0x25, 0xee, /* DAC power up */

  2, 0x12, 0x82, /* Power up the NADC divider with value 7 */
  2, 0x13, 0x87, /* Power up the MADC divider with value 2 */
  2, 0x14, 0x40, /* Program the OSR of ADC to 64 */
  2, 0x3d, 0x01, /* Select ADC PRB_R1 */
  0 // sentinel
};

static const uint8_t conf_data_clk_192kHz[] = {
  2, 0x0b, 0x82, /* Power up the NDAC divider with value 2 */
  2, 0x0c, 0x87, /* Power up the MDAC divider with value 7 */
  2, 0x0d, 0x00, /* Program the OSR of DAC to 32 */
  2, 0x0e, 0x20,
  2, 0x3c, 17, //0x08, /* Set the DAC Mode to PRB_P17 (reduce resource) */
  //2, 0x3c, 25, /* Set the DAC Mode to PRB_P25 */
  2, 0x1b, 0x0c, /* Set the BCLK,WCLK as output */    
  2, 0x1e, 0x80 + 7, /* Enable the BCLKN divider with value 7 */
  2, 0x25, 0xee, /* DAC power up */

  2, 0x12, 0x81, /* Power up the NADC divider with value 1 */
  2, 0x13, 0x87, /* Power up the MADC divider with value 7 */
  2, 0x14, 0x40, /* Program the OSR of ADC to 64 */
  2, 0x3d, 0x01, /* Select ADC PRB_R1 */
  0 // sentinel
};

static const uint8_t conf_data_routing[] = {
  2, 0x00, 0x01, /* Select Page 1 */
  2, 0x01, 0x08, /* Disable Internal Crude AVdd in presence of external AVdd supply or before powering up internal AVdd LDO*/
  2, 0x02, 0x01, /* Enable Master Analog Power Control */
  2, 0x7b, 0x01, /* Set the REF charging time to 40ms */
  2, 0x14, 0x25, /* HP soft stepping settings for optimal pop performance at power up Rpop used is 6k with N = 6 and soft step = 20usec. This should work with 47uF coupling capacitor. Can try N=5,6 or 7 time constants as well. Trade-off delay vs “pop” sound. */
  2, 0x0a, 0x00, /* Set the Input Common Mode to 0.9V and Output Common Mode for Headphone to Input Common Mode */
  2, 0x0a, 0x33, /* Set the Input Common Mode to 0.9V and Output Common Mode for Headphone to 1.65V */
  2, 0x0c, 0x08, /* Route Left DAC to HPL */
  2, 0x0d, 0x08, /* Route Right DAC to HPR */
  2, 0x03, 0x00, /* Set the DAC PTM mode to PTM_P3/4 */
  2, 0x04, 0x00,
  2, 0x10, 0x0a, /* Set the HPL gain to 0dB */
  2, 0x11, 0x0a, /* Set the HPR gain to 0dB */
  2, 0x09, 0x30, /* Power up HPL and HPR drivers */

  2, 0x3d, 0x00, /* Select ADC PTM_R4 */
  2, 0x47, 0x32, /* Set MicPGA startup delay to 3.1ms */
  2, 0x7b, 0x01, /* Set the REF charging time to 40ms */
  2, 0x34, 0x10, /* Route IN2L to LEFT_P with 10K */
  2, 0x36, 0x10, /* Route IN2R to LEFT_N with 10K */
  2, 0x37, 0x04, /* Route IN3R to RIGHT_P with 10K */
  2, 0x39, 0x04, /* Route IN3L to RIGHT_N with 10K */
  2, 0x3b, 72, /* Unmute Left MICPGA, Gain selection of 32dB to make channel gain 0dB */
  2, 0x3c, 72, /* Unmute Right MICPGA, Gain selection of 32dB to make channel gain 0dB */
  2, 0x33, 0x60, /* Enable MIC bias, 2.5V */
  2, 0x00, 0x08, /* Select Page 8 */
  2, 0x01, 0x04, /* Enable Adaptive Filter mode */
  0 // sentinel
};

const uint8_t conf_data_unmute[] = {
  2, 0x00, 0x00, /* Select Page 0 */
  2, 0x3f, 0xd6, /* Power up the Left and Right DAC Channels with route the Left Audio digital data to Left Channel DAC and Right Audio digital data to Right Channel DAC */
  2, 0x40, 0x00, /* Unmute the DAC digital volume control */
  2, 0x51, 0xc0, /* Power up Left and Right ADC Channels */
  2, 0x52, 0x00, /* Unmute Left and Right ADC Digital Volume Control */    
  2, 0x43, 0x93, /* Enable Headphone detection, Debounce 256ms, Button Debounce 32ms */    
  0 // sentinel
};

static const uint8_t conf_data_divoff[] = {
  2, 0x51, 0x00, /* Power down Left and Right ADC Channels */
  2, 0x0b, 0x00, /* Power down NDAC divider */
  2, 0x0c, 0x00, /* Power down MDAC divider */
  2, 0x12, 0x00, /* Power down NADC divider */
  2, 0x13, 0x00, /* Power down MADC divider */
  0 // sentinel
};

void tlv320aic3204_init(void)
{
  tlv320aic3204_config(conf_data_pll);
  tlv320aic3204_config(conf_data_clk);
  tlv320aic3204_config(conf_data_routing);
  wait_ms(40);
  tlv320aic3204_config(conf_data_unmute);
}

void tlv320aic3204_stop(void)
{
  tlv320aic3204_config(conf_data_divoff);
}

void tlv320aic3204_set_fs(int fs)
{
  if (fs != 48 && fs != 96 && fs != 192)
    return;

  if (fs == 48)
    tlv320aic3204_config(conf_data_clk);
  else if (fs == 96)
    tlv320aic3204_config(conf_data_clk_96kHz);
  else if (fs == 192)
    tlv320aic3204_config(conf_data_clk_192kHz);

  wait_ms(10);
  tlv320aic3204_config(conf_data_unmute);
}

void tlv320aic3204_set_impedance(int imp)
{
    imp &= 3; /* 1, 2, 3 */
    tlv320aic3204_write(0x00, 0x01); /* Select Page 1 */
    tlv320aic3204_write(0x34, (imp << 4)); /* Route IN2L to LEFT_P */
    tlv320aic3204_write(0x36, (imp << 4)); /* Route IN2R to LEFT_N */
    tlv320aic3204_write(0x37, (imp << 2)); /* Route IN3R to RIGHT_P */
    tlv320aic3204_write(0x39, (imp << 2)); /* Route IN3L to RIGHT_N */
    tlv320aic3204_write(0x00, 0x00); /* Select Page 0 */
}


void tlv320aic3204_set_gain(int gain)
{
    if (gain < 0)
        gain = 0;
    if (gain > 95)
        gain = 95;

    tlv320aic3204_write(0x00, 0x01); /* Select Page 1 */
    tlv320aic3204_write(0x3b, gain); /* Unmute Left MICPGA, set gain */
    tlv320aic3204_write(0x3c, gain); /* Unmute Right MICPGA, set gain */
    tlv320aic3204_write(0x00, 0x00); /* Select Page 0 */
}

void tlv320aic3204_set_digital_gain(int gain)
{
    if (gain < -24)
        gain = -24;
    if (gain > 40)
        gain = 40;

    tlv320aic3204_write(0x00, 0x00); /* Select Page 0 */
    tlv320aic3204_write(0x53, gain & 0x7f); /* Left ADC Channel Volume */
    tlv320aic3204_write(0x54, gain & 0x7f); /* Right ADC Channel Volume */
}

void tlv320aic3204_set_volume(int gain)
{
    if (gain > 29)
        gain = 29;
    else if (gain < -6) 
        gain = 0x40;
    else
        gain &= 0x3f;

    tlv320aic3204_write(0x00, 0x01); /* Select Page 1 */
    tlv320aic3204_write(0x10, gain); /* Unmute Left MICPGA, set gain */
    tlv320aic3204_write(0x11, gain); /* Unmute Right MICPGA, set gain */
    tlv320aic3204_write(0x00, 0x00); /* Select Page 0 */
}

void tlv320aic3204_agc_config(tlv320aic3204_agc_config_t *conf)
{
    int ctrl = 0;
    if (conf == NULL) {
      ctrl = 0;
    } else {
      ctrl = 0x80
        | ((conf->target_level & 0x7) << 4) 
        | (conf->gain_hysteresis & 0x3);
    }
    tlv320aic3204_write(0x00, 0x00); /* Select Page 0 */
    tlv320aic3204_write(0x56, ctrl); /* Left AGC Control Register */
    tlv320aic3204_write(0x5e, ctrl); /* Right AGC Control Register */
    if (ctrl == 0)
      return;

    ctrl = ((conf->attack & 0x1f) << 3) | (conf->attack_scale & 0x7);
    tlv320aic3204_write(0x59, ctrl); /* Left AGC Attack Time */
    tlv320aic3204_write(0x61, ctrl); /* Right AGC Attack Time */

    ctrl = ((conf->decay & 0x1f) << 3) | (conf->decay_scale & 0x7);
    tlv320aic3204_write(0x5a, ctrl); /* Left AGC Decay Time */
    tlv320aic3204_write(0x62, ctrl); /* Right AGC Decay Time */

    ctrl = conf->maximum_gain;
    tlv320aic3204_write(0x58, ctrl); /* Left AGC Maximum Gain */
    tlv320aic3204_write(0x60, ctrl); /* Right AGC Maximum Gain */
}

// implement HPF of first order IIR
const uint8_t adc_iir_filter_dcreject[] = {
  /* len, page, reg, data.... */
  /* left channel C4 - C6 */
  12, 8, 24, 
  /* Pg8 Reg24-35 */
  0x7f, 0xfa, 0xda, 0x00,
  0x80, 0x05, 0x26, 0x00,
  0x7f, 0xf5, 0xb5, 0x00,
    
  /* right channel C36 - C38 */
  12, 9, 32, 
  /* Pg9 Reg 32-43 */
  0x7f, 0xfa, 0xda, 0x00,
  0x80, 0x05, 0x26, 0x00,
  0x7f, 0xf5, 0xb5, 0x00,
  0 /* sentinel */
};

const uint8_t adc_iir_filter_default[] = {
  /* len, page, reg, data.... */
  /* left channel C4 - C6 */
  12, 8, 24, 
  /* Pg8 Reg24-35 */
  0x7f, 0xff, 0xff, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
    
  /* right channel C36 - C38 */
  12, 9, 32, 
  /* Pg9 Reg 32-43 */
  0x7f, 0xff, 0xff, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0 /* sentinel */
};

void tlv320aic3204_config_adc_filter(int enable)
{
  const uint8_t *p = adc_iir_filter_default;
  if (enable)
    p = adc_iir_filter_dcreject;
  
  while (*p != 0) {
    uint8_t len = *p++;
    uint8_t page = *p++;
    uint8_t reg = *p++;
    tlv320aic3204_write(0x00, page);
    while (len-- > 0)
      tlv320aic3204_write(reg++, *p++);
  }
  tlv320aic3204_write(0x00, 0x08); /* Select Page 8 */
  tlv320aic3204_write(0x01, 0x05); /* ADC Coefficient Buffers will be switched at next frame boundary */
  tlv320aic3204_write(0x00, 0x00); /* Back to page 0 */
}

int tlv320aic3204_get_sticky_flag_register(void)
{
    return tlv320aic3204_read(0x2a); /* Sticky Flag Register */
}

int8_t tlv320aic3204_get_left_agc_gain(void)
{
    return tlv320aic3204_read(0x5d); /* Left Channel AGC Gain Flag */
}

int8_t tlv320aic3204_get_right_agc_gain(void)
{
    return tlv320aic3204_read(0x65); /* Right Channel AGC Gain Flag */
}

void tlv320aic3204_set_adc_phase_adjust(int8_t adjust)
{
  tlv320aic3204_write(0x55, adjust);
}

void tlv320aic3204_beep(void)
{
  /*
  tlv320aic3204_write(0x25, 00); // power off dac
  wait_ms(10);
  tlv320aic3204_write(0x3c, 25); // select PRB_P25 to beep
  tlv320aic3204_write(0x25, 0xee); // DAC power on
  */
  
  // set duration
  tlv320aic3204_write(0x4a, 0x10); 
  tlv320aic3204_write(0x4b, 0x00);
  // beep
  tlv320aic3204_write(0x47, 0x80); 

  /*
  tlv320aic3204_write(0x3c, 17); // restore to PRB_P17
  */
}
