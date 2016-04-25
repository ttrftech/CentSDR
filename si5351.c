#include "hal.h"
#include "si5351.h"

#define SI5351_I2C_ADDR   	(0x60<<1)

extern int I2CWrite(int addr, char d0, char d1);

static void
si5351_write(uint8_t reg, uint8_t dat)
{
  I2CWrite(SI5351_I2C_ADDR>>1, reg, dat);
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
  uint8_t baseaddr = pllreg_base[pll];

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
    P2 = num;
    P3 = denom;
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
  si5351_write(baseaddr,   (P3 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+1, (P3 & 0x000000FF));
  si5351_write(baseaddr+2, (P1 & 0x00030000) >> 16);
  si5351_write(baseaddr+3, (P1 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+4, (P1 & 0x000000FF));
  si5351_write(baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) );
  si5351_write(baseaddr+6, (P2 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+7, (P2 & 0x000000FF));

  /* Reset both PLLs */
  si5351_write(SI5351_REG_177_PLL_RESET, SI5351_PLL_RESET_A | SI5351_PLL_RESET_B);
}

void 
si5351_setupMultisynth(uint8_t     output,
                       uint8_t	   pllSource,
                       uint32_t    div,
                       uint32_t    num,
                       uint32_t    denom)
{
  /* Get the appropriate starting point for the PLL registers */
  const uint8_t msreg_base[] = {
    SI5351_REG_42_MULTISYNTH0,
    SI5351_REG_50_MULTISYNTH1,
    SI5351_REG_58_MULTISYNTH2,
  };
  uint8_t baseaddr = msreg_base[output];
  const uint8_t clkctrl[] = {
    SI5351_REG_16_CLK0_CONTROL,
    SI5351_REG_17_CLK1_CONTROL,
    SI5351_REG_18_CLK2_CONTROL
  };
  uint8_t dat;

  uint32_t P1;
  uint32_t P2;
  uint32_t P3;

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
  if (num == 0)
  {
    /* Integer mode */
    P1 = 128 * div - 512;
    P2 = num;
    P3 = denom;
  }
  else
  {
    /* Fractional mode */
    //P1 = (uint32_t)(128 * div + floor(128 * ((float)num/(float)denom)) - 512);
    P1 = 128 * div + ((128 * num) / denom) - 512;
    //P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P2 = 128 * num - denom * ((128 * num) / denom);
    P3 = denom;
  }

  /* Set the MSx config registers */
  si5351_write(baseaddr,   (P3 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+1, (P3 & 0x000000FF));
  si5351_write(baseaddr+2, (P1 & 0x00030000) >> 16);	/* ToDo: Add DIVBY4 (>150MHz) and R0 support (<500kHz) later */
  si5351_write(baseaddr+3, (P1 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+4, (P1 & 0x000000FF));
  si5351_write(baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  si5351_write(baseaddr+6, (P2 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+7, (P2 & 0x000000FF));

  /* Configure the clk control and enable the output */
  dat = SI5351_CLK_DRIVE_STRENGTH_2MA | SI5351_CLK_INPUT_MULTISYNTH_N;
  if (pllSource == SI5351_PLL_B)
    dat |= SI5351_CLK_PLL_SELECT_B;
  if (num == 0)
    dat |= SI5351_CLK_INTEGER_MODE;
  si5351_write(clkctrl[output], dat);
}

void
si5351_set_frequency(int freq)
{
    // assume PLL1 freq is 800MHz
    int32_t div = 800000 / freq;
    int32_t num = 800000 - freq * div;
    int32_t denom = freq;
    si5351_setupMultisynth(0, SI5351_PLL_A, div, num, denom);
}
