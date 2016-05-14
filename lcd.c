#include "hal.h"
#include "nanosdr.h"

#define I2CLCD_ADDR  (0x7c >> 1)


void i2clcd_data(uint8_t data)
{
    I2CWrite(I2CLCD_ADDR, 0x40, data);
}

void i2clcd_str(const char *p)
{
	while (*p) {
		i2clcd_data(*p++);
	}
}

void i2clcd_cmd(uint8_t cmd)
{
    I2CWrite(I2CLCD_ADDR, 0, cmd);
}

void i2clcd_pos(uint8_t x, uint8_t y)
{
	i2clcd_cmd(0x80 | (0x40 * y) | x);
}

void i2clcd_init(void)
{
    i2clcd_cmd(0x38);
    i2clcd_cmd(0x39);
    i2clcd_cmd(0x14);
    i2clcd_cmd(0x70);
    i2clcd_cmd(0x56);
    i2clcd_cmd(0x6c);
    chThdSleepMilliseconds(200);
    i2clcd_cmd(0x38);
    i2clcd_cmd(0x0c);
    i2clcd_cmd(0x01);
    chThdSleepMilliseconds(2);
}
