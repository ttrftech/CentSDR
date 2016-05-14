
extern void I2CWrite(int addr, uint8_t d0, uint8_t d1);

extern void i2clcd_init(void);
extern void i2clcd_str(const char *p);
extern void i2clcd_pos(uint8_t x, uint8_t y);
extern void i2clcd_cmd(uint8_t cmd);

extern void tlv320aic3204_init(void);
extern void tlv320aic3204_set_gain(int gain);
extern void tlv320aic3204_set_volume(int gain);

extern void ui_init(void);
extern void ui_process(void);
