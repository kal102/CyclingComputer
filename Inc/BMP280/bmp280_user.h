#ifndef BMP280_USER_H
#define BMP280_USER_H

#include <stdint.h>
#include <stm32l1xx_hal.h>

#define BUFFER_SIZE 10

extern I2C_HandleTypeDef hi2c1;

struct bmp280_data
{
	int32_t temperature;
	uint32_t pressure;
	int32_t altitude;
};

struct bmp280_data_buffer
{
	struct bmp280_data data[BUFFER_SIZE];
	uint8_t end;
	uint8_t count;
};

void calculate_pressure_atmospheric(int32_t temperature, uint32_t pressure, int32_t altitude);
uint32_t read_pressure_atmospheric(void);

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);

void user_bmp280_init(void);
void user_bmp280_config(void);
void user_bmp280_test(void);
void user_bmp280_normal_mode(void);
void user_bmp280_sleep_mode(void);

struct bmp280_data user_bmp280_read_single(void);
struct bmp280_data user_bmp280_read_average(void);

#endif
