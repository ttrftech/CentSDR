#include "hal.h"
#include "si5351.h"

#define SI5351_I2C_ADDR   	(0x60<<1)

static void
rcc_gpio_init(void)
{
    // Reset AHB,APB1,APB2
    RCC->AHBRSTR |= 0xffffffff;
    RCC->AHBRSTR = 0;
    RCC->APB1RSTR |= 0xffffffff;
    RCC->APB1RSTR = 0;
    RCC->APB2RSTR |= 0xffffffff;
    RCC->APB2RSTR = 0;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_I2C1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW_HSI;

    GPIOB->AFRH = 0x55550044;  // PB8,PB9 Alternate Function 4
    GPIOB->OTYPER |= 0x0300;   // PB8,PB9 Open drain
    GPIOB->MODER |= 0x000A0000;//
    GPIOB->OSPEEDR |= 0x00050000;//
}

static void
i2c_init(I2C_TypeDef* i2c)
{
	// Disable the I2Cx peripheral
	i2c->CR1 &= ~I2C_CR1_PE;
	while (i2c->CR1 & I2C_CR1_PE);

    // 100kHz @ 8MHz
    i2c->TIMINGR = 0x10420F13;

	// Use 7-bit addresses
	i2c->CR2 &=~ I2C_CR2_ADD10;

	// Enable the analog filter
	i2c->CR1 &= ~I2C_CR1_ANFOFF;

	// Disable NOSTRETCH
	i2c->CR1 |= I2C_CR1_NOSTRETCH;

	// Enable I2Cx peripheral clock.
	// Select APB1 as clock source
	if (i2c == I2C1) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	} else if (i2c == I2C2) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	}

	// Enable the I2Cx peripheral
	i2c->CR1 |= I2C_CR1_PE;
}

static void
i2cSendByte(I2C_TypeDef* i2c, uint8_t addr, const uint8_t *buf, uint8_t len)
{
	i2c->CR2 = (I2C_CR2_SADD & addr) 	// Set the slave address
      | (I2C_CR2_NBYTES & (len << 16))	// Send one byte
      | I2C_CR2_START 					// Generate start condition
      | I2C_CR2_AUTOEND;				// Generate stop condition after sent

	// Send the data
    while (len-- > 0) {
      while (!(i2c->ISR & I2C_ISR_TXIS));
      i2c->TXDR = (I2C_TXDR_TXDATA & *buf++);
    }
}

// register addr, length, data, ...
const uint8_t si5351_configs[] = {
  2, SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0xff,
  4, SI5351_REG_16_CLK0_CONTROL, SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN,
  2, SI5351_REG_183_CRYSTAL_LOAD, SI5351_CRYSTAL_LOAD_6PF,
  // setup PLL
  9, SI5351_REG_26_PLL_A, /*P3*/0, 1, /*P1*/0, 14, 0, /*P3/P2*/0, 0, 0,
  // RESET PLL
  2, SI5351_REG_177_PLL_RESET, SI5351_PLL_RESET_A | SI5351_PLL_RESET_B,
  // setup multisynth
  9, SI5351_REG_58_MULTISYNTH2, /*P3*/0, 1, /*P1*/0, 48, 0, /*P2|P3*/0, 0, 0,
  2, SI5351_REG_18_CLK2_CONTROL, SI5351_CLK_DRIVE_STRENGTH_2MA | SI5351_CLK_INPUT_MULTISYNTH_N | SI5351_CLK_INTEGER_MODE,
  2, SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0,
  0 // sentinel
};

void
si5351_init_bulk(void)
{
  const uint8_t *p = si5351_configs;
  while (*p) {
    uint8_t len = *p++;
    i2cSendByte(I2C1, SI5351_I2C_ADDR, p, len);
    p += len;
  }
}

void
si5351_setup(void)
{
  rcc_gpio_init();
  i2c_init(I2C1);
  si5351_init_bulk();
}

#if 0
static void
si5351_write(uint8_t reg, uint8_t dat)
{
  uint8_t buf[2];
  buf[0] = reg;
  buf[1] = dat;
  i2cSendByte(I2C1, SI5351_I2C_ADDR, buf, 2);
}

void
si5351_init(void)
{
  //i2cInit(I2C1);

  // Disable all output
  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0xff);

  // Initialize the CLK outputs according to flowchart in datasheet
  // First, turn them off
  si5351_write(SI5351_REG_16_CLK0_CONTROL, SI5351_CLK_POWERDOWN);
  si5351_write(SI5351_REG_17_CLK1_CONTROL, SI5351_CLK_POWERDOWN);
  si5351_write(SI5351_REG_18_CLK2_CONTROL, SI5351_CLK_POWERDOWN);
	
  si5351_write(SI5351_REG_183_CRYSTAL_LOAD, SI5351_CRYSTAL_LOAD_8PF);

  // Then reset the PLLs
  si5351_write(SI5351_REG_177_PLL_RESET, SI5351_PLL_RESET_A | SI5351_PLL_RESET_B);
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
  uint8_t msreg_base[] = {
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
si5351_setup(void)
{
  i2c_init_low(I2C1);

  si5351_init();

  // Set PLLA 800MHz
  si5351_setupPLL(SI5351_PLL_A, 32, 0, 1);
  //si5351_setupPLL(SI5351_PLL_B, 32, 0, 1);

  // Set CLK2 as PLLA/100 = 8MHz
  //si5351_setupMultisynth(0, SI5351_PLL_A, 32, 0, 1);
  //si5351_setupMultisynth(1, SI5351_PLL_B, 45, 1, 2);
  si5351_setupMultisynth(2, SI5351_PLL_A, 100, 0, 1);

  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0x00);
}
#endif
