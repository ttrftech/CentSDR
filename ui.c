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
#include "si5351.h"
#include <stdlib.h>

#define set_volume(gain) tlv320aic3204_set_volume(gain)
#define set_gain(gain) tlv320aic3204_set_gain(gain)
#define set_dgain(gain) tlv320aic3204_set_digital_gain(gain)
#define set_frequency(freq) set_tune(freq)
#define set_modulation(mod) signal_process = demod_funcs[mod]
#define set_agc(mode) set_agc_mode(mode)

#define CHANNEL_MAX 10

uint32_t channel_table[CHANNEL_MAX] = {
  567000,
  747000,
  1287000,
  1440000,
  7100000,
  14100000,
  21100000,
  28500000,
  35000000
};

signal_process_func_t demod_funcs[] = {
  am_demod,
  lsb_demod,
  usb_demod
};

int modulation_frequency_offset[] = {
  -AM_FREQ_OFFSET,
  0,
  0
};

#define NO_EVENT					0
#define EVT_BUTTON_SINGLE_CLICK		0x01
#define EVT_BUTTON_DOUBLE_CLICK		0x02
#define EVT_BUTTON_DOWN_LONG		0x04
#define EVT_UP					0x10
#define EVT_DOWN				0x20

#define BUTTON_DOWN_LONG_TICKS		8000
#define BUTTON_DOUBLE_TICKS			500
#define BUTTON_DEBOUNCE_TICKS		10

#define BIT_PUSH	5
#define BIT_DOWN0	4
#define BIT_DOWN1	1
#define BIT_UP0 	7
#define BIT_UP1 	6

#define READ_PORT() palReadPort(GPIOA)
#define BUTTON_MASK 0b11110010

static uint16_t last_button = 0b11110010;
static uint32_t last_button_down_ticks;
static uint8_t inhibit_until_release = FALSE;

int btn_check(void)
{
    int cur_button = READ_PORT() & BUTTON_MASK;
	int changed = last_button ^ cur_button;
	int status = 0;
    uint32_t ticks = chVTGetSystemTime();
	if (changed & (1<<BIT_PUSH)) {
		if (ticks >= last_button_down_ticks + BUTTON_DEBOUNCE_TICKS) {
            if (cur_button & (1<<BIT_PUSH)) {
				// button released
                status |= EVT_BUTTON_SINGLE_CLICK;
                if (inhibit_until_release) {
                    status = 0;
                    inhibit_until_release = FALSE;
                }
			} else {
				// button pushed
                if (ticks < last_button_down_ticks + BUTTON_DOUBLE_TICKS) {
					status |= EVT_BUTTON_DOUBLE_CLICK;
				} else {
					last_button_down_ticks = ticks;
				}
			}
		}
	} else {
		// button unchanged
		if (!(cur_button & (1<<BIT_PUSH))
			&& ticks >= last_button_down_ticks + BUTTON_DOWN_LONG_TICKS) {
			status |= EVT_BUTTON_DOWN_LONG;
            inhibit_until_release = TRUE;
		}
	}

	if ((cur_button & (1<<BIT_UP1)) == 0) {
        if (changed & (1<<BIT_UP1)) {
            status |= EVT_UP;
            last_button_down_ticks = ticks;
        }
        if (ticks >= last_button_down_ticks + BUTTON_DOWN_LONG_TICKS) {
            status |= EVT_UP;
        }
    }
	if ((cur_button & (1<<BIT_DOWN1)) == 0) {
        if (changed & (1<<BIT_DOWN1)) {
            status |= EVT_DOWN;
            last_button_down_ticks = ticks;
        }
        if (ticks >= last_button_down_ticks + BUTTON_DOWN_LONG_TICKS) {
            status |= EVT_DOWN;
        }
    }
    last_button = cur_button;

	return status;
}

#define VOLUME_MAX 29
#define VOLUME_MIN -7
#define RFGAIN_MAX 95

#define AGCMODE_MAX 4
int16_t agc_slowness_table[AGCMODE_MAX] = {
		-1, 10, 5, 1
};
const char *agc_mode_table[AGCMODE_MAX] = {
		"OFF", "Slow", "Med", "Fast"
};

const char *mod_table[] = { "AM", "LSB", "USB"};


void
ui_update(void)
{
	char buf[16];
	i2clcd_pos(0, 1);
	switch (uistat.mode) {
	case VOLUME:
        i2clcd_str("Vol:");
		if (uistat.volume < -6)
            i2clcd_str("mute");
		else {
            itoa(uistat.volume, buf, 10);
            i2clcd_str(buf);
        }
		break;
	case CHANNEL:
        i2clcd_str("Ch");
        itoa(uistat.channel, buf, 10);
        i2clcd_str(buf);
        i2clcd_str(" ");
        itoa(uistat.freq/1000, buf, 10);
        i2clcd_str(buf);
		break;
	case FREQ:
        itoa(uistat.freq, buf, 10);
        i2clcd_str("        "+strlen(buf));
        i2clcd_str(buf);
		break;
	case MOD:
        i2clcd_str("Mod:");
		i2clcd_str(mod_table[uistat.modulation]);
		break;
	case RFGAIN:
        i2clcd_str("RF:");
        itoa(uistat.rfgain/2, buf, 10);
        i2clcd_str(buf);
        i2clcd_str("dB");
		break;
	case DGAIN:
        i2clcd_str("D:");
        itoa(uistat.dgain/2, buf, 10);
        i2clcd_str(buf);
        i2clcd_str("dB");
		break;
	case AGC:
        i2clcd_str("AGC:");
		i2clcd_str(agc_mode_table[uistat.agcmode]);
		break;
	default:
		break;
	}
	i2clcd_str("        ");

	if (uistat.mode == FREQ) {
		i2clcd_cmd(0x0e); // enable show-cursor flag
		i2clcd_pos(7 - uistat.digit, 1);
	} else {
		i2clcd_cmd(0x0c); // disable show-cursor flag
	}
}

void
update_frequency()
{
  set_frequency(uistat.freq + modulation_frequency_offset[uistat.modulation]);
}

void
ui_init(void)
{
#if 0
    i2clcd_init();
    i2clcd_str("FriskSDR");
#endif
	uistat.mode = CHANNEL;
	uistat.channel = 0;
	uistat.freq = 567000;
	uistat.digit = 3;
	uistat.modulation = MOD_AM;
	uistat.volume = 8;
	uistat.rfgain = 60; // 0 ~ 95
	uistat.dgain = 0; // -24 ~ 40
	uistat.agcmode = AGC_MANUAL; //AGC_MID;
	//ui_update();

	set_volume(uistat.volume);
	set_gain(uistat.rfgain);
	set_dgain(uistat.dgain);
    set_agc(uistat.agcmode);
    update_frequency();
}



void
ui_digit(void)
{
    int count = 0;
    while (TRUE) {
        int status = btn_check();
        if (status & EVT_BUTTON_SINGLE_CLICK)
            break;
        if (status & EVT_UP && uistat.digit < 7)
            uistat.digit++;
        if (status & EVT_DOWN && uistat.digit > 0)
            uistat.digit--;
        if (count++ % 4 < 2) {
            i2clcd_cmd(0x0e); // enable show-cursor flag
            i2clcd_pos(7 - uistat.digit, 1);
        } else {
            i2clcd_cmd(0x0c); // disable show-cursor flag
        }
        chThdSleepMilliseconds(100);
    }
}

void
ui_process(void)
{
	int status = btn_check();
	int n;
	if (status != 0) {
		if (status & EVT_BUTTON_SINGLE_CLICK) {
            // enable RFGAIN and DGAIN only when AGC is disabled
            int mode_max = uistat.agcmode == AGC_MANUAL ? MODE_MAX : RFGAIN;
			uistat.mode = (uistat.mode + 1) % mode_max;
		} else if (uistat.mode == CHANNEL) {
			if ((status & EVT_UP) && uistat.channel < CHANNEL_MAX)
				uistat.channel++;
			if ((status & EVT_DOWN) && uistat.channel > 0)
				uistat.channel--;
            uistat.freq = channel_table[uistat.channel];
            update_frequency();
		} else if (uistat.mode == VOLUME) {
			if ((status & EVT_UP) && uistat.volume < VOLUME_MAX)
				uistat.volume++;
			if ((status & EVT_DOWN) && uistat.volume > VOLUME_MIN)
				uistat.volume--;
			set_volume(uistat.volume);
		} else if (uistat.mode == FREQ) {
			int32_t step = 1;
			for (n = uistat.digit; n > 0; n--)
				step *= 10;
			if (status & EVT_UP) {
				uistat.freq += step;
                update_frequency();
			}
			if (status & EVT_DOWN) {
				uistat.freq -= step;
                update_frequency();
			}
			if (status & EVT_BUTTON_DOWN_LONG) {
              ui_digit();
            }
		} else if (uistat.mode == RFGAIN) {
			if ((status & EVT_UP) && uistat.rfgain < RFGAIN_MAX-2)
				uistat.rfgain += 2;
			if ((status & EVT_DOWN) && uistat.rfgain > 0)
				uistat.rfgain -= 2;
            set_gain(uistat.rfgain);
		} else if (uistat.mode == DGAIN) {
			if ((status & EVT_UP) && uistat.dgain < 40)
				uistat.dgain += 2;
			if ((status & EVT_DOWN) && uistat.dgain > -24)
				uistat.dgain -= 2;
            set_dgain(uistat.dgain);
		} else if (uistat.mode == AGC) {
			if ((status & EVT_DOWN) && uistat.agcmode > 0)
				uistat.agcmode--;
			if ((status & EVT_UP) && uistat.agcmode < AGCMODE_MAX-1)
				uistat.agcmode++;
            set_agc(uistat.agcmode);
		} else if (uistat.mode == MOD) {
			if ((status & EVT_UP) && uistat.modulation < MOD_MAX-1) {
				uistat.modulation++;
			}
			if ((status & EVT_DOWN) && uistat.modulation > 0) {
				uistat.modulation--;
			}
            set_modulation(uistat.modulation);
            update_frequency();
		}

		ui_update();
	}
}
