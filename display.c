#include <math.h>
#include <string.h>
#include <stdio.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "nanosdr.h"
#include "arm_math.h"

// result is 8.8 format
static inline uint16_t
log2_q31(uint32_t x)
{
	uint32_t mask = 0xffff0000;
	uint16_t bit = 16;
	uint16_t y = 32;//-15;
	uint8_t i;

	if (x == 0)
		return 0;
	// 16
	if ((x & mask) == 0) {
		x <<= bit;
		y -= bit;
	}
	bit >>= 1;
	mask <<= bit;
	// 8
	if ((x & mask) == 0) {
		x <<= bit;
		y -= bit;
	}
	bit >>= 1;
	mask <<= bit;
	// 4
	if ((x & mask) == 0) {
		x <<= bit;
		y -= bit;
	}
	bit >>= 1;
	mask <<= bit;
	// 2
	if ((x & mask) == 0) {
		x <<= bit;
		y -= bit;
	}
	bit >>= 1;
	mask <<= bit;
	// 1
	if ((x & mask) == 0) {
		x <<= bit;
		y -= bit;
	}
	// msb should be 1. take next 8 bits.
	i = (x >> 23) & 0xff;
	// lookup logarythm table
	return (y << 8) | i;
}

arm_cfft_radix4_instance_q31 cfft_inst;

//#define mag(r,i) (q31_t)(((q63_t)r*r)>>33)+(q31_t)(((q63_t)i*i)>>33)


#define BG_ACTIVE 0x8208
#define BG_NORMAL 0x0000

uistat_t uistat;

#define UISTAT (&uistat)


typedef struct {
	uint32_t sample_freq;
	int16_t offset;
	int16_t stride;
	int16_t overgain;

	int16_t origin;
	int16_t tickstep;
	int16_t tickbase;
	int16_t tickunit;
	const char *unitname;
} spectrumdisplay_param_t;

// when event sent with SEV from M4 core, filled following data
typedef struct {
	q31_t *buffer;
	uint32_t buffer_rest;
	uint8_t update_flag;
	uint8_t ui_update_flag;
	spectrumdisplay_param_t p;
} spectrumdisplay_info_t;

#define FLAG_SPDISP 	(1<<0)
#define FLAG_UI 		(1<<1)

spectrumdisplay_info_t spdispinfo;
#define SPDISPINFO (&spdispinfo)

// r:2048 c:1024 samples (8192 byte with q31_t)
//#define SPDISP_BUFFER_SIZE	8192
//static q31_t SPDISP_BUFFER[2048];

void
draw_spectrogram(void)
{
	q31_t *buf = SPDISPINFO->buffer;
	arm_cfft_radix4_q31(&cfft_inst, buf);
	//arm_cmplx_mag_q31(buf, buf, 1024);
//	arm_cmplx_mag_squared_q31(buf, buf, 1024);
	//draw_samples();
	//return;
	uint16_t gainshift = SPDISPINFO->p.overgain;
	int i = SPDISPINFO->p.offset;
	int stride = SPDISPINFO->p.stride;
	uint16_t (*block)[32] = (uint16_t (*)[32])spi_buffer;
	int sx, x, y;
	for (sx = 0; sx < 320; sx += 32) {
		for (x = 0; x < 32; x++) {
			q31_t ii = buf[(i&1023)*2];
			q31_t qq = buf[(i&1023)*2+1];
			q31_t mag = ((int64_t)ii*ii + (int64_t)qq*qq)>>(33-gainshift);
			//q31_t mag = buf[i & 1023];
			int v = log2_q31(mag) >> 6;
			if (v > 64) v = 64;
			for (y = 0; y < v; y++)
				block[63-y][x] = 0xffff;
			for ( ; y < 64; y++)
				block[63-y][x] = 0;
			i += stride;
		}
		ili9341_draw_bitmap(sx, 72, 32, 64, (uint16_t*)block);
	}
}

const struct { uint8_t r,g,b; } colormap[] = {
		{ 0, 0, 0 },
		{ 0, 0, 255 },
		{ 0, 255, 0 },
		{ 255, 0, 0 },
		{ 255, 255, 255 }
};

uint16_t
pick_color(int mag) /* mag: 0 - 63 */
{
	int idx = (mag >> 4) & 0x3;
	int prop = mag & 0x0f;
	int nprop = 0x10 - prop;
	int r = colormap[idx].r * nprop + colormap[idx+1].r * prop;
	int g = colormap[idx].g * nprop + colormap[idx+1].g * prop;
	int b = colormap[idx].b * nprop + colormap[idx+1].b * prop;
	return RGB565(r>>4, g>>4, b>>4);
}

void
waterfall_init(void)
{
#if 0
  // Vertical Scroll Definition
  uint16_t tfa = 152;
  uint16_t vsa = 240 - tfa;
  uint16_t bfa = 80;
  uint8_t vsd[6] = { tfa>>8, tfa, vsa>>8, vsa, bfa>>8, bfa };
  send_command(0x33, 6, vsd);
#endif
}

int vsa = 152;

void
draw_waterfall(void)
{
	int x;
	q31_t *buf = SPDISPINFO->buffer;
	uint16_t *block = spi_buffer;
	int i = SPDISPINFO->p.offset;
	int stride = SPDISPINFO->p.stride;
	uint16_t gainshift = SPDISPINFO->p.overgain;

	for (x = 0; x < 320; x++) {
		q31_t ii = buf[(i&1023)*2];
		q31_t qq = buf[(i&1023)*2+1];
		q31_t mag = ((int64_t)ii*ii + (int64_t)qq*qq)>>(33-gainshift);
		//q31_t mag = buf[i & 1023];
		int v = log2_q31(mag) >> 6;
		if (v > 63) v = 63;
		*block++ = pick_color(v);
		i += stride;
	}

	vsa++;
	if (vsa >= 240)
		vsa = 152;

	// Vertical Scroll Address
	uint8_t vscrsadd[2] = { vsa>>8, vsa };
	send_command(0x37, 2, vscrsadd);

	ili9341_draw_bitmap(0, vsa, 320, 1, spi_buffer);
}

void
draw_tick(void)
{
	char str[10];
	int x = SPDISPINFO->p.origin;
	int base = SPDISPINFO->p.tickbase;
	int xx;
	uint16_t bg = UISTAT->mode == SPDISP ? BG_ACTIVE : BG_NORMAL;

	ili9341_fill(0, 136, 320, 16, bg);
	sprintf(str, "%d%s", base, SPDISPINFO->p.unitname);
	xx = x - strlen(str) * 5 / 2;
	if (xx < 0) xx = 0;
	ili9341_drawstring_5x7(str, xx, 142, 0xffff, bg);
	ili9341_fill(x, 136, 2, 5, 0xffff);

	base += SPDISPINFO->p.tickunit;
	x += SPDISPINFO->p.tickstep;
	while (x < 320) {
		sprintf(str, "%d", base);
		ili9341_fill(x, 136, 2, 5, 0xffff);
		ili9341_drawstring_5x7(str, x, 142, 0xffff, bg);
		base += SPDISPINFO->p.tickunit;
		x += SPDISPINFO->p.tickstep;
	}
	x = SPDISPINFO->p.origin;
	base = SPDISPINFO->p.tickbase;
	base -= SPDISPINFO->p.tickunit;
	x -= SPDISPINFO->p.tickstep;
	while (x >= 0) {
		sprintf(str, "%d", base);
		ili9341_fill(x, 136, 2, 5, 0xffff);
		ili9341_drawstring_5x7(str, x, 142, 0xffff, bg);
		base -= SPDISPINFO->p.tickunit;
		x -= SPDISPINFO->p.tickstep;
	}
}

void
draw_freq(void)
{
	char str[10];
	uint16_t bg = UISTAT->mode == FREQ ? BG_ACTIVE : BG_NORMAL;
	int i;
	const uint16_t xsim[] = { 0, 16, 0, 0, 16, 0, 0, 0 };
	uint16_t x = 0;
	sprintf(str, "%8d", (int)(UISTAT->freq));
	for (i = 0; i < 8; i++) {
		int8_t c = str[i] - '0';
		uint16_t fg = 0xffff;
		if (UISTAT->mode == FREQ && UISTAT->digit == 7-i)
			fg = 0xfe40;

		if (c >= 0 && c <= 9)
			ili9341_drawfont(c, &NF32x48, x, 0, fg, bg);
		else
			ili9341_fill(x, 0, 32, 48, bg);
		x += 32;

		// fill gaps
		if (xsim[i] > 0) {
			ili9341_fill(x, 0, xsim[i], 48, bg);
			x += xsim[i];
		}
	}
	// draw Hz symbol
	ili9341_drawfont(10, &NF32x48, x, 0, 0xffff, bg);
}

void
draw_info(void)
{
	char str[10];
	int x = 0;
	int y = 48;
	uint16_t bg = UISTAT->mode == VOLUME ? BG_ACTIVE : BG_NORMAL;
	ili9341_drawfont(14, &NF20x24, x, y, 0xfffe, bg);
	x += 20;
	if (UISTAT->volume != -7)
		sprintf(str, "%2d", UISTAT->volume);
	else
		// -infinity
		sprintf(str, "-\003");
	ili9341_drawfont_string(str, &NF20x24, x, y, 0xfffe, bg);
	x += 40;
	ili9341_drawfont(13, &NF20x24, x, y, 0xfffe, bg);
	x += 20;

	bg = UISTAT->mode == MOD ? BG_ACTIVE : BG_NORMAL;
	ili9341_drawfont(UISTAT->modulation, &ICON48x20, x+2, y+2, 0xffe0, bg);
	x += 48+4;

	bg = UISTAT->mode == AGC ? BG_ACTIVE : BG_NORMAL;
	ili9341_drawfont(UISTAT->agcmode + 2, &ICON48x20, x+2, y+2, 0xffff, bg);
	x += 48+4;

	bg = UISTAT->mode == RFGAIN ? BG_ACTIVE : BG_NORMAL;
	ili9341_drawfont(15, &NF20x24, x, y, 0x07ff, bg);
	x += 20;
	sprintf(str, "%3d ", (int)(-6 * (int32_t)UISTAT->rfgain));
	ili9341_drawfont_string(str, &NF20x24, x, y, 0x07ff, bg);
	x += 60;
	ili9341_drawfont(13, &NF20x24, x, y, 0x07ff, bg);
	x += 20;
}

void
clear_background(void)
{
	int i = 0;
	for (i = 0; i < 12; i++) {
		ili9341_fill(0, i*10, 320, 10, 0x0000);
	}
}

// called periodically
void
disp_process(void)
{
  if (SPDISPINFO->update_flag & FLAG_SPDISP) {
    draw_spectrogram();
    draw_waterfall();
    SPDISPINFO->update_flag &= ~FLAG_SPDISP;
  }

  if (SPDISPINFO->update_flag & FLAG_UI) {
    draw_tick();
    draw_freq();
    draw_info();
    SPDISPINFO->update_flag &= ~FLAG_UI;
  }
}

void
disp_init(void)
{
	arm_cfft_radix4_init_q31(&cfft_inst, 1024, FALSE, TRUE);
	waterfall_init();
	clear_background();
}
