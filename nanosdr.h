
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
#define SSB_FREQ_OFFSET 1300
