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
#include <string.h>

#define set_volume(gain) tlv320aic3204_set_volume(gain)
#define set_gain(gain) tlv320aic3204_set_gain(gain)
#define set_dgain(gain) tlv320aic3204_set_digital_gain(gain)
#define set_agc(mode) set_agc_mode(mode)


int fetch_encoder_tick(void);

#define CHANNEL_MAX 18

const setting_t channel_table[CHANNEL_MAX] = {
  {   567000, MOD_AM,  26*2, 0 },
  {   747000, MOD_AM,  20*2, 0 },
  {  1287000, MOD_AM,  23*2, 0 },
  {  1440000, MOD_AM,  23*2, 0 },
  {  7100000, MOD_LSB, 40*2, 0 },
  { 14100000, MOD_USB, 40*2, 10*2 },
  { 21100000, MOD_USB, 40*2, 10*2 },
  { 28500000, MOD_USB, 40*2, 10*2 },
  {  2932000, MOD_USB, 30*2, 0 },
  {  5628000, MOD_USB, 40*2, 0 },
  {  6655000, MOD_USB, 40*2, 0 },
  {  8951000, MOD_USB, 40*2, 0 },
  { 10048000, MOD_USB, 40*2, 5*2 },
  { 11330000, MOD_USB, 40*2, 5*2 },
  { 13273000, MOD_USB, 40*2, 10*2 },
  { 17904000, MOD_USB, 40*2, 10*2 }
};

#define NO_EVENT					0
#define EVT_BUTTON_SINGLE_CLICK		0x01
#define EVT_BUTTON_DOUBLE_CLICK		0x02
#define EVT_BUTTON_DOWN_LONG		0x04
#define EVT_UP					0x10
#define EVT_DOWN				0x20
#define EVT_PUSH_UP				0x30
#define EVT_PUSH_DOWN			0x40

#define BUTTON_DOWN_LONG_TICKS		8000
#define BUTTON_DOUBLE_TICKS			500
#define BUTTON_DEBOUNCE_TICKS		10

#define BIT_PUSH	0
#define BIT_ENCODER0	1
#define BIT_ENCODER1	2

static uint16_t last_button = 0;
static uint16_t button_event_inhibited = 0;
static uint32_t last_button_down_ticks;
//static uint8_t dragged = 0; // encoder changed while button pressed


int
read_buttons(void)
{
  return (palReadPort(GPIOA) & 0x1);
}

void
inhibit_button_event(void)
{
  button_event_inhibited = 1;
}

int btn_check(void)
{
    int cur_button = read_buttons();
	int changed = last_button ^ cur_button;
	int status = 0;
    uint32_t ticks = chVTGetSystemTime();
	if (changed & (1<<BIT_PUSH)) {
		if (ticks >= last_button_down_ticks + BUTTON_DEBOUNCE_TICKS) {
            if (!(cur_button & (1<<BIT_PUSH))) {
				// button released
              if (button_event_inhibited) {
                button_event_inhibited = 0;
              } else {
                status |= EVT_BUTTON_SINGLE_CLICK;
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
        if ((cur_button & (1<<BIT_PUSH))
			&& ticks >= last_button_down_ticks + BUTTON_DOWN_LONG_TICKS) {
			status |= EVT_BUTTON_DOWN_LONG;
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

void
update_frequency(void)
{
  set_tune(uistat.freq);
}


int enc_status = 0;
int enc_count = 0;


int
fetch_encoder_tick(void)
{
  int n = enc_count;
  enc_count = 0;
  return n;
}

void ext_callback(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    int cur = palReadPort(GPIOB);
    const int trans_tbl[4][4] = {
      /*falling A */  /*rising A  */  /*falling B */  /*rising B  */
      { 0, 0, 3, 3 }, { 1, 1, 2, 2 }, { 0, 1, 1, 0 }, { 3, 2, 2, 3 }
    };
    int s = (channel - 1) * 2; // A: 0, B: 2
    if (cur & (1 << channel))
      s |= 1; // rising
    if (enc_status == 0 && s == 3)
      enc_count--;
    if (enc_status == 3 && s == 2)
      enc_count++;
    enc_status = trans_tbl[s][enc_status];
#if 0
    if (channel == 0) {
      enc_count = 0;
    }
#endif
}

static const EXTConfig extconf = {
  {
    { 0, NULL }, //{ EXT_MODE_GPIOA | EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART, ext_callback },
    { EXT_MODE_GPIOB | EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, ext_callback },
    { EXT_MODE_GPIOB | EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, ext_callback },
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
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL }
  }
};

void
ui_init(void)
{
#if 1
  extStart(&EXTD1, &extconf);
  //chCondObjectInit(&condvar_button);
#endif

    uistat.mode = CHANNEL;
	uistat.channel = 0;
	uistat.freq = 567000;
	uistat.digit = 3;
	uistat.modulation = MOD_AM;
	uistat.volume = 10;
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

static int minmax(int x, int min, int max)
{
  if (x >= max)
    return max - 1;
  if (x < min)
    return min;
  return x;
}

void
ui_process(void)
{
	int status = btn_check();
    int tick = fetch_encoder_tick();
	int n;
    if (status & EVT_BUTTON_SINGLE_CLICK) {
      uistat.mode = (uistat.mode + 1) % MODE_MAX;
      disp_update();
    }
    if (tick != 0) {
      if (read_buttons() != 0) {
        // button pressed

        if (uistat.mode == FREQ) {
          if (tick < 0) {
            if (uistat.digit < 7)
              uistat.digit++;
            else
              uistat.mode--;
          }
          if (tick > 0) {
            if (uistat.digit > 0)
              uistat.digit--;
            else
              uistat.mode++;
          }
        } else {
          if (tick < 0)
            uistat.mode--;
          if (tick > 0)
            uistat.mode++;
          uistat.mode = uistat.mode % MODE_MAX;
        }
        disp_update();
        inhibit_button_event();
      } else {
        if (uistat.mode == CHANNEL) {
          uistat.channel = minmax(uistat.channel + tick, 0, CHANNEL_MAX);
          // apply settings at channel
          uistat.freq = channel_table[uistat.channel].freq;
          uistat.rfgain = channel_table[uistat.channel].rfgain;
          uistat.dgain = channel_table[uistat.channel].dgain;
          uistat.modulation = channel_table[uistat.channel].modulation;
          set_gain(uistat.rfgain);
          set_dgain(uistat.dgain);
          set_modulation(uistat.modulation);
          update_frequency();
        } else if (uistat.mode == VOLUME) {
          uistat.volume = minmax(uistat.volume + tick, VOLUME_MIN, VOLUME_MAX);
          set_volume(uistat.volume);
        } else if (uistat.mode == FREQ) {
          int32_t step = 1;
          for (n = uistat.digit; n > 0; n--)
            step *= 10;
          int32_t freq = uistat.freq + step * tick;
          if (freq > 0)
            uistat.freq = freq;
          update_frequency();
        } else if (uistat.mode == RFGAIN) {
          uistat.rfgain = minmax(uistat.rfgain + tick, 0, RFGAIN_MAX+1);
          set_gain(uistat.rfgain);
        } else if (uistat.mode == DGAIN) {
          uistat.dgain = minmax(uistat.dgain + tick, -24, 41);
          set_dgain(uistat.dgain);
        } else if (uistat.mode == AGC) {
          uistat.agcmode = minmax(uistat.agcmode + tick, 0, 4);
          set_agc(uistat.agcmode);
        } else if (uistat.mode == MOD) {
          if (tick > 0 && uistat.modulation < MOD_MAX-1) {
            uistat.modulation++;
          }
          if (tick < 0 && uistat.modulation > 0) {
            uistat.modulation--;
          }
          set_modulation(uistat.modulation);
          update_frequency();
        } else if (uistat.mode == SPDISP) {
          uistat.spdispmode = minmax(uistat.spdispmode + tick, 0, 3);
        }
      }
      
      //ui_update();
      disp_update();
	}
}

