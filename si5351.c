#include "hal.h"
#include "si5351.h"
#include "nanosdr.h"

#define SI5351_I2C_ADDR   	(0x60<<1)

#ifdef SI5351_GEN_QUADRATURE_LO
#define SI5351_SUPPORTED_QUADRATURE_FREQ_MIN    3500000
#define DELAY_RESET_PLL                         5000
#endif

#ifdef SI5351_GEN_QUADRATURE_LO_BELOW_3500KHZ
static uint8_t s_r0div_reg[3];
#endif

static void
si5351_write(uint8_t reg, uint8_t dat)
{
  int addr = SI5351_I2C_ADDR>>1;
  uint8_t buf[] = { reg, dat };
  i2cAcquireBus(&I2CD1);
  (void)i2cMasterTransmitTimeout(&I2CD1, addr, buf, 2, NULL, 0, 1000);
  i2cReleaseBus(&I2CD1);
}

static void
si5351_bulk_write(const uint8_t *buf, int len)
{
  int addr = SI5351_I2C_ADDR>>1;
  i2cAcquireBus(&I2CD1);
  (void)i2cMasterTransmitTimeout(&I2CD1, addr, buf, len, NULL, 0, 1000);
  i2cReleaseBus(&I2CD1);
}

void si5351_disable_output(void)
{
  uint8_t reg[4];
  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0xff);
  reg[0] = SI5351_REG_16_CLK0_CONTROL;
  reg[1] = SI5351_CLK_POWERDOWN;
  reg[2] = SI5351_CLK_POWERDOWN;
  reg[3] = SI5351_CLK_POWERDOWN;
  si5351_bulk_write(reg, 4);
}

void si5351_enable_output(void)
{
  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0x00);
}

void si5351_reset_pll(void)
{
#ifdef SI5351_GEN_QUADRATURE_LO
  // Writing a 1<<5 will reset PLLA, 1<<7 reset PLLB, this is a self clearing bits.
  // !!! Need delay before reset PLL for apply PLL freq changes before
  chThdSleepMicroseconds(DELAY_RESET_PLL);
  si5351_write(SI5351_REG_177_PLL_RESET, SI5351_PLL_RESET_B);
#else
  //si5351_write(SI5351_REG_177_PLL_RESET, SI5351_PLL_RESET_A | SI5351_PLL_RESET_B);
  si5351_write(SI5351_REG_177_PLL_RESET, 0xAC);
#endif
}

void si5351_setupPLL(uint8_t pll, /* SI5351_PLL_A or SI5351_PLL_B */
                     uint8_t     mult,
                     uint32_t    num,
                     uint32_t    denom)
{
  /* Get the appropriate starting point for the PLL registers */
  const uint8_t pllreg_base[] = {
    SI5351_REG_26_PLL_A,
    SI5351_REG_34_PLL_B
  };
  uint32_t P1;
  uint32_t P2;
  uint32_t P3;

  /* Feedback Multisynth Divider Equation
   * where: a = mult, b = num and c = denom
   * P1 register is an 18-bit value using following formula:
   * 	P1[17:0] = 128 * mult + floor(128*(num/denom)) - 512
   * P2 register is a 20-bit value using the following formula:
   * 	P2[19:0] = 128 * num - denom * floor(128*(num/denom))
   * P3 register is a 20-bit value using the following formula:
   * 	P3[19:0] = denom
   */

  /* Set the main PLL config registers */
  if (num == 0)
  {
    /* Integer mode */
    P1 = 128 * mult - 512;
    P2 = 0;
    P3 = 1;
  }
  else
  {
    /* Fractional mode */
    //P1 = (uint32_t)(128 * mult + floor(128 * ((float)num/(float)denom)) - 512);
    P1 = 128 * mult + ((128 * num) / denom) - 512;
    //P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P2 = 128 * num - denom * ((128 * num) / denom);
    P3 = denom;
  }

  /* The datasheet is a nightmare of typos and inconsistencies here! */
  uint8_t reg[9];
  reg[0] = pllreg_base[pll];
  reg[1] = (P3 & 0x0000FF00) >> 8;
  reg[2] = (P3 & 0x000000FF);
  reg[3] = (P1 & 0x00030000) >> 16;
  reg[4] = (P1 & 0x0000FF00) >> 8;
  reg[5] = (P1 & 0x000000FF);
  reg[6] = ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16);
  reg[7] = (P2 & 0x0000FF00) >> 8;
  reg[8] = (P2 & 0x000000FF);
  si5351_bulk_write(reg, 9);
}

static const uint8_t msreg_base[] = {
  SI5351_REG_42_MULTISYNTH0,
  SI5351_REG_50_MULTISYNTH1,
  SI5351_REG_58_MULTISYNTH2,
};

void 
si5351_setupMultisynth(uint8_t     output,
                       uint8_t	   pllSource,
                       uint32_t    div, // 4,6,8, 8+ ~ 900
                       uint32_t    num,
                       uint32_t    denom,
                       uint32_t    rdiv, // SI5351_R_DIV_1~128
                       uint8_t     drive_strength)
{
  /* Get the appropriate starting point for the PLL registers */
  const uint8_t clkctrl[] = {
    SI5351_REG_16_CLK0_CONTROL,
    SI5351_REG_17_CLK1_CONTROL,
    SI5351_REG_18_CLK2_CONTROL
  };
  uint8_t dat;

  uint32_t P1;
  uint32_t P2;
  uint32_t P3;
  uint32_t div4 = 0;

  /* Output Multisynth Divider Equations
   * where: a = div, b = num and c = denom
   * P1 register is an 18-bit value using following formula:
   * 	P1[17:0] = 128 * a + floor(128*(b/c)) - 512
   * P2 register is a 20-bit value using the following formula:
   * 	P2[19:0] = 128 * b - c * floor(128*(b/c))
   * P3 register is a 20-bit value using the following formula:
   * 	P3[19:0] = c
   */
  /* Set the main PLL config registers */
  if (div == 4) {
    div4 = SI5351_DIVBY4;
    P1 = P2 = 0;
    P3 = 1;
  } else if (num == 0) {
    /* Integer mode */
    P1 = 128 * div - 512;
    P2 = 0;
    P3 = 1;
  } else {
    /* Fractional mode */
    P1 = 128 * div + ((128 * num) / denom) - 512;
    P2 = 128 * num - denom * ((128 * num) / denom);
    P3 = denom;
  }

  /* Set the MSx config registers */
  uint8_t reg[9];
  reg[0] = msreg_base[output];
  reg[1] = (P3 & 0x0000FF00) >> 8;
  reg[2] = (P3 & 0x000000FF);
  reg[3] = ((P1 & 0x00030000) >> 16) | div4 | rdiv;
  reg[4] = (P1 & 0x0000FF00) >> 8;
  reg[5] = (P1 & 0x000000FF);
  reg[6] = ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16);
  reg[7] = (P2 & 0x0000FF00) >> 8;
  reg[8] = (P2 & 0x000000FF);
  si5351_bulk_write(reg, 9);

#ifdef SI5351_GEN_QUADRATURE_LO_BELOW_3500KHZ
  s_r0div_reg[output] = reg[3];
#endif

  /* Configure the clk control and enable the output */
  dat = drive_strength | SI5351_CLK_INPUT_MULTISYNTH_N;
  if (pllSource == SI5351_PLL_B)
    dat |= SI5351_CLK_PLL_SELECT_B;
  if (num == 0)
    dat |= SI5351_CLK_INTEGER_MODE;
  si5351_write(clkctrl[output], dat);

#ifdef SI5351_GEN_QUADRATURE_LO
  #define SI5351_REG_165_CLK0_PHOFF   165
  #define SI5351_REG_166_CLK1_PHOFF   166
  #define SI5351_REG_167_CLK2_PHOFF   167

  uint8_t phoff = 0;
  if (div < 128 && rdiv == 0) {
    phoff = (uint8_t)div;
    if (num >= (denom / 2) && phoff <= 126) {
      phoff++;
    }
  }

  if (output == 0) {
    // I/cosine
    si5351_write(SI5351_REG_165_CLK0_PHOFF, phoff);
  }
  else if (output == 1) {
    // Q/sine
    si5351_write(SI5351_REG_166_CLK1_PHOFF, 0);
  }
#endif
}

static uint32_t
gcd(uint32_t x, uint32_t y)
{
  uint32_t z;
  while (y != 0) {
    z = x % y;
    x = y;
    y = z;
  }
  return x;
}

#define XTALFREQ 26000000L
#define PLL_N 32
#define PLLFREQ (XTALFREQ * PLL_N)

void
si5351_set_frequency_fixedpll(int channel, int pll, int pllfreq, int freq,
                              uint32_t rdiv, uint8_t drive_strength)
{
    int32_t div = pllfreq / freq; // range: 8 ~ 1800
    int32_t num = pllfreq - freq * div;
    int32_t denom = freq;
    //int32_t k = freq / (1<<20) + 1;
    int32_t k = gcd(num, denom);
    num /= k;
    denom /= k;
    while (denom >= (1<<20)) {
      num >>= 1;
      denom >>= 1;
    }
    si5351_setupMultisynth(channel, pll, div, num, denom, rdiv, drive_strength);
}

#ifdef SI5351_GEN_QUADRATURE_LO_BELOW_3500KHZ
static bool adjusting_rdiv = false;
static int rdiv_adj_freq;
static uint16_t rdiv_adj_delay_max;
static uint16_t rdiv_adj_delay_step;
static uint16_t rdiv_adj_delay;

static void set_rdiv(uint8_t channel, uint8_t rdiv)
{
  uint8_t dat = s_r0div_reg[channel];
  dat &= 0x8f;
  dat |= rdiv;
  si5351_write(msreg_base[channel] + 2, dat);
}

static bool adjust_rdiv_for_quadrature()
{
  uint16_t buf[32];

  set_rdiv(0, SI5351_R_DIV_1);
  chThdSleepMicroseconds(rdiv_adj_delay);
  set_rdiv(0, SI5351_R_DIV_4);

  rdiv_adj_delay += rdiv_adj_delay_step;
  if (rdiv_adj_delay > rdiv_adj_delay_max) {
    rdiv_adj_delay -= rdiv_adj_delay_max;
  }

  if (rdiv_adj_freq >= 1000000) {
    chSysLock();
    {
      register uint16_t *p = buf;
      register volatile uint16_t *portb = (uint16_t *)0x48000410;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p++ = *portb;
      *p   = *portb;
    }
    chSysUnlock();

    uint16_t prev = 0xFFFF;
    uint16_t *p = buf;
    for (int i = 0; i < 32; i++) {
      uint16_t curr = (buf[i] & 0x0C00) >> 10;
      if (curr != prev) {
        *p++ = prev = curr;
      }
    }
  }
  else {
    chSysLock();
    {
      register uint16_t *p = buf;
      register volatile uint16_t *portb = (uint16_t *)0x48000410;
      register uint16_t val1;
      register uint16_t val2 = 0;
      while ((val1 = (*portb & 0x0C00)) == val2) { }
      *p++ = val1;
      while ((val2 = (*portb & 0x0C00)) == val1) { }
      *p++ = val2;
      while ((val1 = (*portb & 0x0C00)) == val2) { }
      *p++ = val1;
      while ((val2 = (*portb & 0x0C00)) == val1) { }
      *p++ = val2;
      while ((val1 = (*portb & 0x0C00)) == val2) { }
      *p   = val1;
    }
    chSysUnlock();

    for (int i = 0; i < 5; i++) {
      buf[i] >>= 10;
    }
  }

  static const uint8_t quadrature_next_seq[] = {
    2,  // 0b00 -> 0b10
    0,  // 0b01 -> 0b00
    3,  // 0b10 -> 0b11
    1,  // 0b11 -> 0b01
  };
  if (quadrature_next_seq[buf[0]] == buf[1] &&
      quadrature_next_seq[buf[1]] == buf[2] &&
      quadrature_next_seq[buf[2]] == buf[3] &&
      quadrature_next_seq[buf[3]] == buf[4]
  ) {
    return false;
  }
  return true;
}

static void start_adjusting_rdiv(int freq)
{
  adjusting_rdiv = true;
  rdiv_adj_freq = freq;
  rdiv_adj_delay_max = 32;
  rdiv_adj_delay_step = 3;
  rdiv_adj_delay = rdiv_adj_delay_step;
}

void si5351_adjust_rdiv_if_necessary(void)
{
  if (adjusting_rdiv) {
    adjusting_rdiv = adjust_rdiv_for_quadrature();
  }
}
#endif

#ifdef SI5351_GEN_QUADRATURE_LO
static void
si5351_set_frequency_quadrature(int pll, int freq,
                                uint32_t rdiv, uint8_t drive_strength)
{
    #define PLL_MAX_FREQ          900000000
    #define PLL_MID_FREQ          750000000
    #define PLL_MIN_FREQ          600000000
    #define MULTISYNTH_DIV_MIN    4
    #define MULTISYNTH_DIV_MAX    1800
    #define PLL_DENOM_MAX         1048575
    #define MULTISYNTH_PHOFF_MAX  126

    static uint32_t s_prev_ms_div = 1;
    static uint32_t s_prev_rdiv = SI5351_R_DIV_1;
    static int s_prev_freq;

    uint32_t pll_freq = freq * s_prev_ms_div;
    uint32_t ms_div;
    if (pll_freq <= PLL_MAX_FREQ &&
        (pll_freq >= PLL_MIN_FREQ ||
         (s_prev_ms_div == MULTISYNTH_PHOFF_MAX &&
          rdiv == SI5351_R_DIV_1 &&
          freq >= SI5351_SUPPORTED_QUADRATURE_FREQ_MIN)) &&
        abs(freq - s_prev_freq) < (((PLL_MAX_FREQ - PLL_MIN_FREQ) / 2) / s_prev_ms_div)
    ) {
      // keep using current ms_div
      ms_div = s_prev_ms_div;
    }
    else {
      ms_div = (PLL_MID_FREQ + freq) / freq;
      ms_div &= 0xFFFFFFFE;   // make it even number
    }

    // multisynth parameter
    if (ms_div > MULTISYNTH_PHOFF_MAX && rdiv == SI5351_R_DIV_1 && freq >= SI5351_SUPPORTED_QUADRATURE_FREQ_MIN) {
      ms_div = MULTISYNTH_PHOFF_MAX;  // this will cause the pll_freq to be below PLL_MIN_FREQ
    }
    if (ms_div >= MULTISYNTH_DIV_MAX) {
      ms_div = MULTISYNTH_DIV_MAX;
    }
    if (ms_div < MULTISYNTH_DIV_MIN) {
      ms_div = MULTISYNTH_DIV_MIN;
    }

    pll_freq = freq * ms_div;

    // PLL parameter
    uint32_t pll_mult = pll_freq / XTALFREQ;
    uint32_t pll_remain = pll_freq % XTALFREQ;
    uint32_t pll_num = ((uint64_t)pll_remain * PLL_DENOM_MAX + XTALFREQ / 2) / XTALFREQ;
    if (pll_num == PLL_DENOM_MAX) {
      pll_mult++;
      pll_num = 0;
    }
    si5351_setupPLL(pll, pll_mult, pll_num, PLL_DENOM_MAX);

    if (ms_div != s_prev_ms_div || rdiv != s_prev_rdiv) {
      s_prev_ms_div = ms_div;
      s_prev_rdiv = rdiv;
      s_prev_freq = freq;
      si5351_setupMultisynth(0, pll, ms_div, 0, 1, rdiv, drive_strength);
      si5351_setupMultisynth(1, pll, ms_div, 0, 1, rdiv, drive_strength);
      si5351_reset_pll();
#ifdef SI5351_GEN_QUADRATURE_LO_BELOW_3500KHZ
      adjusting_rdiv = false;
      if (rdiv != SI5351_R_DIV_1) {
        start_adjusting_rdiv(freq / 4);
      }
#endif
    }
}
#endif

void
si5351_set_frequency_fixeddiv(int channel, int pll, int freq, int div,
                              uint8_t     drive_strength)
{
    int32_t pllfreq = freq * div;
    int32_t multi = pllfreq / XTALFREQ;
    int32_t num = pllfreq - multi * XTALFREQ;
    int32_t denom = XTALFREQ;
    int32_t k = gcd(num, denom);
    num /= k;
    denom /= k;
    while (denom >= (1<<20)) {
      num >>= 1;
      denom >>= 1;
    }
    si5351_setupPLL(pll, multi, num, denom);
    si5351_setupMultisynth(channel, pll, div, 0, 1, SI5351_R_DIV_1, drive_strength);
}

//#define drive_strength SI5351_CLK_DRIVE_STRENGTH_2MA
#define drive_strength SI5351_CLK_DRIVE_STRENGTH_8MA
int current_band = -1;

void
si5351_set_frequency(int freq)
{
  int band;
  uint32_t rdiv = SI5351_R_DIV_1;
#ifdef SI5351_GEN_QUADRATURE_LO_BELOW_3500KHZ
  if (freq < 100000)
    freq = 100000;

  if (freq < SI5351_SUPPORTED_QUADRATURE_FREQ_MIN) {   // < 3.5MHz
    rdiv = SI5351_R_DIV_4;
    freq *= 4;
  }
#else
  if (freq <= 100000000) {
    band = 0;
  } else if (freq < 150000000) {
    band = 1;
  } else {
    band = 2;
  }
  if (freq <= 500000) {
    rdiv = SI5351_R_DIV_64;
#ifdef SI5351_GEN_QUADRATURE_LO
  } else if (freq < SI5351_SUPPORTED_QUADRATURE_FREQ_MIN) {   // < 3.5MHz
#else
  } else if (freq <= 4000000) {   // <= 4MHz
#endif
    rdiv = SI5351_R_DIV_8;
  }
#endif

#if 0
  if (current_band != band)
    si5351_disable_output();
#endif
  
#ifdef SI5351_GEN_QUADRATURE_LO
  if (rdiv == SI5351_R_DIV_8) {
    freq *= 8;
  } else if (rdiv == SI5351_R_DIV_64) {
    freq *= 64;
  }
  si5351_set_frequency_quadrature(SI5351_PLL_B, freq,
                                  rdiv, drive_strength);
#else
  switch (band) {
  case 0:
    if (rdiv == SI5351_R_DIV_8) {
      freq *= 8;
    } else if (rdiv == SI5351_R_DIV_64) {
      freq *= 64;
    }

    si5351_set_frequency_fixedpll(1, SI5351_PLL_A, PLLFREQ, freq,
                                  rdiv, drive_strength);
    break;

  case 1:
    // Set PLL twice on changing from band 2
    if (current_band == 2) {
      si5351_set_frequency_fixeddiv(1, SI5351_PLL_B, freq, 6, drive_strength);
    }

    // div by 6 mode. both PLL A and B are dedicated for CLK0, CLK1
    si5351_set_frequency_fixeddiv(1, SI5351_PLL_B, freq, 6, drive_strength);
    break;

  case 2:
    // div by 4 mode. both PLL A and B are dedicated for CLK0, CLK1
    si5351_set_frequency_fixeddiv(1, SI5351_PLL_B, freq, 4, drive_strength);
    break;
  }

  if (current_band != band) {
    si5351_reset_pll();
    //si5351_enable_output();
  }

  current_band = band;
#endif
}


