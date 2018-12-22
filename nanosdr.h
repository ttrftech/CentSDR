/*
 * Copyright (c) 2016-2017, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
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

/*
 * main.c
 */


typedef struct {
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

  uint16_t vref;
  uint16_t temperature;
  uint16_t battery;
} stat_t;

extern int16_t measured_power_dbm;
extern stat_t stat;

void set_agc_mode(int agcmode);

/*
 * tlv320aic3204.c
 */

typedef struct {
  int target_level;
  int gain_hysteresis;
  int attack;
  int attack_scale;
  int decay;
  int decay_scale;
  int maximum_gain;
} tlv320aic3204_agc_config_t;

extern void tlv320aic3204_init(void);
extern void tlv320aic3204_set_gain(int g1, int g2);
extern void tlv320aic3204_set_digital_gain(int g1, int g2);
extern void tlv320aic3204_set_volume(int gain);
extern void tlv320aic3204_agc_config(tlv320aic3204_agc_config_t *conf);
extern void tlv320aic3204_set_fs(int fs);
extern void tlv320aic3204_stop(void);

extern void tlv320aic3204_config_adc_filter(int enable);
extern void tlv320aic3204_config_adc_filter2(double iqbal);
extern void tlv320aic3204_set_impedance(int imp);

extern int tlv320aic3204_get_sticky_flag_register(void);
extern int8_t tlv320aic3204_get_left_agc_gain(void);
extern int8_t tlv320aic3204_get_right_agc_gain(void);
extern void tlv320aic3204_set_adc_phase_adjust(int8_t adjust);
extern void tlv320aic3204_set_adc_fine_gain_adjust(int8_t g1, int8_t g2);

extern void tlv320aic3204_beep(void);

#define AIC3204_STICKY_ADC_OVERFLOW 0x0c

/*
 * dsp.c
 */

// 5ms @ 48kHz
#define AUDIO_BUFFER_LEN 480

extern int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];
extern int16_t tx_buffer[AUDIO_BUFFER_LEN * 2];
extern int16_t buffer[2][AUDIO_BUFFER_LEN];
extern int16_t buffer2[2][AUDIO_BUFFER_LEN];

typedef enum { B_CAPTURE, B_IF1, B_IF2, B_PLAYBACK, BUFFERS_MAX } buffer_t;

typedef struct {
  enum { BT_C_INTERLEAVE, BT_IQ, BT_R_INTERLEAVE, BT_REAL } type;
  int16_t length;
  int16_t *buf0;
  int16_t *buf1;
} buffer_ref_t;

extern const buffer_ref_t buffers_table[BUFFERS_MAX];


typedef void (*signal_process_func_t)(int16_t *src, int16_t *dst, size_t len);

extern signal_process_func_t signal_process;

void am_demod(int16_t *src, int16_t *dst, size_t len);
void cw_demod(int16_t *src, int16_t *dst, size_t len);
void lsb_demod(int16_t *src, int16_t *dst, size_t len);
void usb_demod(int16_t *src, int16_t *dst, size_t len);
void fm_demod(int16_t *src, int16_t *dst, size_t len);
void fm_demod_stereo(int16_t *src, int16_t *dst, size_t len);

void dsp_init(void);

#define FS 48000
#define AM_FREQ_OFFSET 10000
#define SSB_FREQ_OFFSET 1300
#define PHASESTEP(freq) (65536L * freq / FS)

extern int32_t center_frequency;
extern int16_t mode_freq_offset;
extern int16_t mode_freqoffset_phasestep;
extern int16_t cw_tone_phasestep;

typedef struct {
  int16_t *buffer;
  int16_t stride;
  int16_t count;
  int16_t coeff;

  int32_t accumlate;
  int16_t offset;
} dcrejection_t;


// state variables for stereo separation

typedef struct {
	uint32_t phase_step_default;
	uint32_t phase_step;
	uint32_t phase_accum;

    // average of correlation vector angle
    int32_t sdi; 
	int32_t sdq;

    int32_t corr;
	int32_t corr_ave;
	int32_t corr_std;
    int16_t integrator;
  
} stereo_separate_state_t;

extern stereo_separate_state_t stereo_separate_state;


/*
 * font: Font5x7.c numfont32x24.c numfont20x24.c icons.c
 */

extern const uint16_t x5x7_bits [];
extern const uint32_t numfont20x24[][24];
extern const uint32_t numfont32x24[][24];
extern const uint32_t icons48x20[][20*2];

#define S_PI    "\034"
#define S_MICRO "\035"
#define S_OHM   "\036"
#define S_DEGREE "\037"
#define S_RARROW "\033"

#define ICON_AGC_OFF 6


/*
 * ili9341.c
 */

#define RGB565(b,g,r)     ( (((r)<<8)&0xf800) | (((g)<<3)&0x07e0) | (((b)>>3)&0x001f) )

typedef struct {
	uint16_t width;
	uint16_t height;
	uint16_t scaley;
	uint16_t slide;
	uint16_t stride;
	const uint32_t *bitmap;
} font_t;

extern const font_t NF20x24;
extern const font_t NF32x24;
extern const font_t NF32x48;
extern const font_t ICON48x20;

extern uint16_t spi_buffer[];

void ili9341_init(void);
void ili9341_test(int mode);
void ili9341_bulk(int x, int y, int w, int h);
void ili9341_fill(int x, int y, int w, int h, int color);
void ili9341_draw_bitmap(int x, int y, int w, int h, uint16_t *bitmap);
void ili9341_drawchar_5x7(uint8_t ch, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_drawstring_5x7(const char *str, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_drawfont(uint8_t ch, const font_t *font, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_drawfont_string(const char *str, const font_t *font, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_set_direction(int rot180);


/*
 * display.c
 */

void disp_init(void);
void disp_process(void);
void disp_fetch_samples(int bufid, int type, int16_t *buf0, int16_t *buf1, size_t len);
void disp_update(void);
void disp_update_power(void);
void disp_clear_aux(void);

void set_window_function(int wf_type);


/*
 * ui.c
 */

extern void ui_init(void);
extern void ui_process(void);

typedef enum {
	MOD_CW,
	MOD_LSB,
	MOD_USB,
	MOD_AM,
	MOD_FM,
	MOD_FM_STEREO,
	MOD_MAX
} modulation_t;

extern void set_tune(int hz);
extern void set_modulation(modulation_t mod);
extern void recall_channel(unsigned int channel);
extern void set_fs(int fs);


typedef struct {
    enum { CHANNEL, FREQ, VOLUME, MOD, AGC, RFGAIN, AGC_MAXGAIN, CWTONE, IQBAL,
         SPDISP, WFDISP, MODE_MAX } mode;
	int8_t volume;
    uint8_t channel;

    uint32_t freq;
	modulation_t modulation;
	int16_t rfgain;
    uint8_t fs; /* 48, 96, 192 */

    enum { AGC_MANUAL, AGC_SLOW, AGC_MID, AGC_FAST, AGC_MAX } agcmode;
	int8_t digit; /* 0~5 */
    int freq_offset;
    enum { SPDISP_CAP, SPDISP_CAP2, SPDISP_IF, SPDISP_AUD, SPDISP_MODE_MAX } spdispmode;
    enum { WATERFALL, WAVEFORM, WAVEFORM_MAG, WAVEFORM_MAG2, WFDISP_MODE_MAX } wfdispmode;
    int16_t cw_tone_freq;
    int16_t iqbal;
} uistat_t;

extern uistat_t uistat;

/*
 * flash.c
 */

#define CHANNEL_MAX 100

typedef struct {
  uint32_t freq;
  modulation_t modulation;
} channel_t;

typedef struct {
  int32_t magic;
  uint16_t dac_value;
  tlv320aic3204_agc_config_t agc;
  channel_t channels[CHANNEL_MAX];
  uistat_t uistat;
  int8_t freq_inverse;
  uint8_t button_polarity;
  int8_t lcd_rotation;
  int32_t checksum;
} config_t;

extern config_t config;

#define CONFIG_MAGIC 0x434f4e45 /* 'CONF' */

int config_save(void);
int config_recall(void);

void clear_all_config_prop_data(void);

/*EOF*/
