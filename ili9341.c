/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#include "ch.h"
#include "hal.h"
#include "nanosdr.h"

#define RESET_ASSERT	palClearPad(GPIOA, 15)
#define RESET_NEGATE	palSetPad(GPIOA, 15)
#define CS_LOW			palClearPad(GPIOB, 6)
#define CS_HIGH			palSetPad(GPIOB, 6)
#define DC_CMD			palClearPad(GPIOB, 7)
#define DC_DATA			palSetPad(GPIOB, 7)

uint16_t spi_buffer[4096];

void
ssp_wait(void)
{
  while (SPI1->SR & SPI_SR_BSY)
    ;
}

void
ssp_wait_slot(void)
{
  while ((SPI1->SR & 0x1800) == 0x1800)
    ;
}

void
ssp_senddata(uint8_t x)
{
  *(uint8_t*)(&SPI1->DR) = x;
  while (SPI1->SR & SPI_SR_BSY)
    ;
}

void
ssp_senddata16(uint16_t x)
{
  ssp_wait_slot();
  SPI1->DR = x;
  //while (SPI1->SR & SPI_SR_BSY)
  //  ;
}

void
ssp_databit8(void)
{
  SPI1->CR2 = (SPI1->CR2 & 0xf0ff) | 0x0700;
//LPC_SSP1->CR0 = (LPC_SSP1->CR0 & 0xf0) | SSP_DATABIT_8;
}

void
ssp_databit16(void)
{
  SPI1->CR2 = (SPI1->CR2 & 0xf0ff) | 0x0f00;
  //LPC_SSP1->CR0 = (LPC_SSP1->CR0 & 0xf0) | SSP_DATABIT_16;
}


const stm32_dma_stream_t  *dmatx;
uint32_t txdmamode;

static void spi_lld_serve_tx_interrupt(SPIDriver *spip, uint32_t flags) {
  (void)spip;
  (void)flags;
}

void
spi_init(void)
{
  rccEnableSPI1(FALSE);

  dmatx     = STM32_DMA_STREAM(STM32_SPI_SPI1_TX_DMA_STREAM);
  txdmamode = STM32_DMA_CR_CHSEL(SPI1_TX_DMA_CHANNEL) |
    STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY) |
    STM32_DMA_CR_DIR_M2P |
    STM32_DMA_CR_DMEIE |
    STM32_DMA_CR_TEIE |
    STM32_DMA_CR_PSIZE_HWORD |
    STM32_DMA_CR_MSIZE_HWORD;
  dmaStreamAllocate(dmatx,
                    STM32_SPI_SPI1_IRQ_PRIORITY,
                    (stm32_dmaisr_t)spi_lld_serve_tx_interrupt,
                    NULL);
  dmaStreamSetPeripheral(dmatx, &SPI1->DR);

  SPI1->CR1 = 0;
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;// | SPI_CR1_BR_1;
  SPI1->CR2 = 0x0700 | SPI_CR2_TXDMAEN;
  SPI1->CR1 |= SPI_CR1_SPE;
}

void
send_command(uint8_t cmd, int len, const uint8_t *data)
{
	CS_LOW;
	DC_CMD;
    ssp_databit8();
	ssp_senddata(cmd);
	DC_DATA;
	while (len-- > 0) {
      ssp_senddata(*data++);
	}
	//CS_HIGH;
}

const uint8_t ili9341_init_seq[] = {
		// cmd, len, data...,
		// Power control B
		0xCF, 3, 0x00, 0x83, 0x30,
		// Power on sequence control
		0xED, 4, 0x64, 0x03, 0x12, 0x81,
		//0xED, 4, 0x55, 0x01, 0x23, 0x01,
		// Driver timing control A
		0xE8, 3, 0x85, 0x01, 0x79,
		//0xE8, 3, 0x84, 0x11, 0x7a,
		// Power control A
		0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
		// Pump ratio control
		0xF7, 1, 0x20,
		// Driver timing control B
		0xEA, 2, 0x00, 0x00,
		// POWER_CONTROL_1
		0xC0, 1, 0x26,
		// POWER_CONTROL_2
		0xC1, 1, 0x11,
		// VCOM_CONTROL_1
		0xC5, 2, 0x35, 0x3E,
		// VCOM_CONTROL_2
		0xC7, 1, 0xBE,
		// MEMORY_ACCESS_CONTROL
		//0x36, 1, 0x48, // portlait
		0x36, 1, 0x28, // landscape
		// COLMOD_PIXEL_FORMAT_SET : 16 bit pixel
		0x3A, 1, 0x55,
		// Frame Rate
		0xB1, 2, 0x00, 0x1B,
		// Gamma Function Disable
		0xF2, 1, 0x08,
		// gamma set for curve 01/2/04/08
		0x26, 1, 0x01,
		// positive gamma correction
		0xE0, 15, 0x1F,  0x1A,  0x18,  0x0A,  0x0F,  0x06,  0x45,  0x87,  0x32,  0x0A,  0x07,  0x02,  0x07, 0x05,  0x00,
		// negativ gamma correction
		0xE1, 15, 0x00,  0x25,  0x27,  0x05,  0x10,  0x09,  0x3A,  0x78,  0x4D,  0x05,  0x18,  0x0D,  0x38, 0x3A,  0x1F,

		// Column Address Set
	    0x2A, 4, 0x00, 0x00, 0x01, 0x3f, // width 320
	    // Page Address Set
	    0x2B, 4, 0x00, 0x00, 0x00, 0xef, // height 240

		// entry mode
		0xB7, 1, 0x06,
		// display function control
		0xB6, 4, 0x0A, 0x82, 0x27, 0x00,

		// control display
		//0x53, 1, 0x0c,
		// diaplay brightness
		//0x51, 1, 0xff,

		// sleep out
		0x11, 0,
		0 // sentinel
};

void
ili9341_init(void)
{
  spi_init();
#if 1
  DC_DATA;
  RESET_ASSERT;
  chThdSleepMilliseconds(10);
  RESET_NEGATE;

  send_command(0x01, 0, NULL); // SW reset
  chThdSleepMilliseconds(5);
  send_command(0x28, 0, NULL); // display off

  const uint8_t *p;
  for (p = ili9341_init_seq; *p; ) {
    send_command(p[0], p[1], &p[2]);
    p += 2 + p[1];
    chThdSleepMilliseconds(5);
  }

  chThdSleepMilliseconds(100);
  send_command(0x29, 0, NULL); // display on
#endif
}

void ili9341_pixel(int x, int y, int color)
{
	uint8_t xx[4] = { x >> 8, x, (x+1) >> 8, (x+1) };
	uint8_t yy[4] = { y >> 8, y, (y+1) >> 8, (y+1) };
	uint8_t cc[2] = { color >> 8, color };
	send_command(0x2A, 4, xx);
    send_command(0x2B, 4, yy);
    send_command(0x2C, 2, cc);
    //send_command16(0x2C, color);
}



void ili9341_fill(int x, int y, int w, int h, int color)
{
	uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
	uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
    int len = w * h;
	send_command(0x2A, 4, xx);
    send_command(0x2B, 4, yy);
    send_command(0x2C, 0, NULL);
    while (len-- > 0) 
      ssp_senddata16(color);
}

void ili9341_draw_bitmap(int x, int y, int w, int h, uint16_t *buf)
{
	uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
	uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
    int len = w * h;

	send_command(0x2A, 4, xx);
	send_command(0x2B, 4, yy);
	send_command(0x2C, 0, NULL);

    dmaStreamSetMemory0(dmatx, buf);
    dmaStreamSetTransactionSize(dmatx, len);
    dmaStreamSetMode(dmatx, txdmamode | STM32_DMA_CR_MINC);
    dmaStreamEnable(dmatx);
    dmaWaitCompletion(dmatx);
}

void
ili9341_drawchar_5x7(uint8_t ch, int x, int y, uint16_t fg, uint16_t bg)
{
  uint16_t *buf = spi_buffer;
  uint16_t bits;
  int c, r;
  for(c = 0; c < 7; c++) {
    bits = x5x7_bits[(ch * 7) + c];
    for (r = 0; r < 5; r++) {
      *buf++ = (0x8000 & bits) ? fg : bg;
      bits <<= 1;
    }
  }
  ili9341_draw_bitmap(x, y, 5, 7, spi_buffer);
}

void
ili9341_drawstring_5x7(const char *str, int x, int y, uint16_t fg, uint16_t bg)
{
  while (*str) {
    ili9341_drawchar_5x7(*str, x, y, fg, bg);
    x += 5;
    str++;
  }
}

#define SWAP(x,y) do { int z=x; x = y; y = z; } while(0)

const font_t NF20x24 = { 20, 24, 1, 24, 1, (const uint32_t *)numfont20x24 };
const font_t NF32x24 = { 32, 24, 1, 24, 1, (const uint32_t *)numfont32x24 };
const font_t NF32x48 = { 32, 48, 2, 24, 1, (const uint32_t *)numfont32x24 };
const font_t ICON48x20 = { 48, 20, 1, 40, 2, (const uint32_t *)icons48x20 };

void
ili9341_drawfont(uint8_t ch, const font_t *font, int x, int y, uint16_t fg, uint16_t bg)
{
	uint16_t *buf = spi_buffer;
	uint32_t bits;
	const uint32_t *bitmap = &font->bitmap[font->slide * ch];
	int c, r, j, b;

	for (c = 0; c < font->slide; c += font->stride) {
		for (j = 0; j < font->scaley; j++) {
			int cc = c;
			for (r = 0; r < font->width;) {
				bits = bitmap[cc++];
				for (b = 0; b < 32 && r < font->width; b++,r++) {
					*buf++ = (0x80000000UL & bits) ? fg : bg;
					bits <<= 1;
				}
			}
		}
	}
    ili9341_draw_bitmap(x, y, font->width, font->height, spi_buffer);
}

void
ili9341_drawfont_string(const char *str, const font_t *font, int x, int y, uint16_t fg, uint16_t bg)
{
  while (*str) {
    char c = *str++;
    if (c >= '0' && c <= '9')
      ili9341_drawfont(c - '0', font, x, y, fg, bg);
    else if (c > 0 && c < 7)
      ili9341_drawfont(c + 9, font, x, y, fg, bg);
    else if (c == '.')
      ili9341_drawfont(10, font, x, y, fg, bg);
    else if (c == '-')
      ili9341_drawfont(11, font, x, y, fg, bg);
    else
      ili9341_fill(x, y, font->width, font->height, bg);
    x += font->width;
  }
}

void
ili9341_set_direction(int rot180)
{
  char value = 0x28; // landscape
  if (rot180) {
    value |= 0xc0; // reverse X and Y axis
  }

  send_command(0x36, 1, &value);
}
