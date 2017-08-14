
extern void I2CWrite(int addr, uint8_t d0, uint8_t d1);

extern void i2clcd_init(void);
extern void i2clcd_str(const char *p);
extern void i2clcd_pos(uint8_t x, uint8_t y);
extern void i2clcd_cmd(uint8_t cmd);

typedef struct {
  int target_level;
  int gain_hysteresis;
  int attack;
  int attack_scale;
  int decay;
  int decay_scale;
} tlv320aic3204_agc_config_t;

extern void tlv320aic3204_init(void);
extern void tlv320aic3204_set_gain(int gain);
extern void tlv320aic3204_set_digital_gain(int gain);
extern void tlv320aic3204_set_volume(int gain);
extern void tlv320aic3204_agc_config(tlv320aic3204_agc_config_t *conf);

extern void ui_init(void);
extern void ui_process(void);

extern void set_tune(int hz);


// 5ms @ 48kHz
#define AUDIO_BUFFER_LEN 480

extern int16_t rx_buffer[];
extern int16_t tx_buffer[];

extern int16_t buffer_i[];
extern int16_t buffer_q[];

typedef void (*signal_process_func_t)(int16_t *src, int16_t *dst, size_t len);

extern signal_process_func_t signal_process;

void am_demod(int16_t *src, int16_t *dst, size_t len);
void lsb_demod(int16_t *src, int16_t *dst, size_t len);
void usb_demod(int16_t *src, int16_t *dst, size_t len);

void set_agc_mode(int agcmode);

#define AM_FREQ_OFFSET 10000
//#define AM_FREQ_OFFSET 0
#define SSB_FREQ_OFFSET 1300

// font

extern const uint16_t x5x7_bits [];
extern const uint32_t numfont20x24[][24];
extern const uint32_t numfont32x24[][24];
extern const uint32_t icons48x20[][20*2];

#define S_PI    "\034"
#define S_MICRO "\035"
#define S_OHM   "\036"
#define S_DEGREE "\037"
#define S_RARROW "\033"

/*
 * ili9341.c
 */
#define RGB565(b,g,r)     ( (((r)<<8)&0xf800) | (((g)<<2)&0x07e0) | (((b)>>3)&0x001f) )

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


/*
 * display.c
 */

void disp_init(void);
void disp_process(void);
void disp_fetch_samples(void);
void disp_update(void);

/*
 * ui.c
 */
typedef enum {
	MOD_LSB,
	MOD_USB,
	MOD_AM,
	MOD_MAX
} modulation_t;

typedef struct {
    enum { CHANNEL, FREQ, VOLUME, MOD, AGC, RFGAIN, DGAIN, SPDISP, MODE_MAX } mode;
	int volume;
	int channel;
	uint32_t freq;
	modulation_t modulation;
	int digit; /* 0~5 */
	enum { AGC_MANUAL, AGC_SLOW, AGC_MID, AGC_FAST } agcmode;
	int rfgain;
	int dgain;
    int freq_offset;
	enum { SPDISP_CAP0, SPDISP_CAP, SPDISP_CIC, SPDISP_FIR, SPDISP_IIR, SPDISP_AUD, SPDISP_MODE_MAX } spdispmode;
	int tp;
	int debugmode;
} uistat_t;

extern uistat_t uistat;

