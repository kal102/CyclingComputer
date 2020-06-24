#include <stm32l1xx_hal.h>
#include <stdint.h>
#include <math.h>
#include "BMP280/bmp280.h"
#include "BMP280/bmp280_defs.h"
#include "BMP280/bmp280_user.h"

#define I2C_WRITE_BIT 0
#define I2C_READ_BIT 1

int8_t rslt;
struct bmp280_dev bmp;
struct bmp280_config conf;

struct bmp280_data_buffer buffer;
uint32_t pressure_atmospheric = 101325;

static int32_t calculate_altitude(int32_t temperature, uint32_t pressure)
{
	int32_t altitude;

	altitude = ((powf((float)pressure_atmospheric/(float)pressure, 1/5.257) - 1) * ((float)(temperature + 27315) / 100)) / 0.000065;
	return altitude;
}

void calculate_pressure_atmospheric(int32_t temperature, uint32_t pressure, int32_t altitude)
{
	pressure_atmospheric = pressure * powf((1.0 + (0.000065 * (float)altitude)/((float)(temperature + 27315) / 100)), 5.257);
}

uint32_t read_pressure_atmospheric(void)
{
	return pressure_atmospheric;
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t id = (dev_id << 1) | I2C_READ_BIT;

	HAL_I2C_Mem_Read(&hi2c1, id, reg_addr, 1, data, len, 100);
	while (HAL_I2C_STATE_BUSY  == HAL_I2C_GetState(&hi2c1));

	switch (HAL_I2C_GetState(&hi2c1))
	{
	case HAL_I2C_STATE_RESET:
		return BMP280_E_DEV_NOT_FOUND;
	case HAL_I2C_STATE_READY:
		return BMP280_OK;
	case HAL_I2C_STATE_TIMEOUT:
		return BMP280_E_DEV_NOT_FOUND;
	default:
		return BMP280_E_COMM_FAIL;
	}
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t id = (dev_id << 1) | I2C_WRITE_BIT;

	HAL_I2C_Mem_Write(&hi2c1, id, reg_addr, 1, data, len, 100);
	while (HAL_I2C_STATE_BUSY  == HAL_I2C_GetState(&hi2c1));

	switch (HAL_I2C_GetState(&hi2c1))
	{
	case HAL_I2C_STATE_RESET:
		return BMP280_E_DEV_NOT_FOUND;
	case HAL_I2C_STATE_READY:
		return BMP280_OK;
	case HAL_I2C_STATE_TIMEOUT:
		return BMP280_E_DEV_NOT_FOUND;
	default:
		return BMP280_E_COMM_FAIL;
	}
}

void user_delay_ms(uint32_t period)
{
	HAL_Delay(period);
}

void user_bmp280_init(void)
{
	/* Sensor interface over I2C with primary slave address  */
	bmp.dev_id = BMP280_I2C_ADDR_PRIM;
	bmp.intf = BMP280_I2C_INTF;
	bmp.read = user_i2c_read;
	bmp.write = user_i2c_write;
	bmp.delay_ms = user_delay_ms;

	rslt = bmp280_init(&bmp);

	if (rslt == BMP280_OK)
	{
	  /* Sensor chip ID will be printed if initialization was successful */
	  printf("\n Device found with chip id 0x%x", bmp.chip_id);
	}
}

void user_bmp280_config(void)
{
	/* Always read the current settings before writing, especially when
	 * all the configuration is not modified
	 */
	rslt = bmp280_get_config(&conf, &bmp);
	/* Check if rslt == BMP280_OK, if not, then handle accordingly */

	/* Overwrite the desired settings */
	conf.filter = BMP280_FILTER_COEFF_16;
	conf.os_pres = BMP280_OS_16X;
	conf.os_temp = BMP280_OS_2X;
	conf.odr = BMP280_ODR_0_5_MS;

	rslt = bmp280_set_config(&conf, &bmp);
	/* Check if rslt == BMP280_OK, if not, then handle accordingly */

	/* Always set the power mode after setting the configuration */
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
	/* Check if rslt == BMP280_OK, if not, then handle accordingly */
}

void user_bmp280_test(void)
{
	struct bmp280_uncomp_data ucomp_data;
	uint8_t meas_dur = bmp280_compute_meas_time(&bmp);

	printf("Measurement duration: %dms\r\n", meas_dur);

	/* Loop to read out 10 samples of data */
	for (uint8_t i = 0; (i < 10) && (rslt == BMP280_OK); i++)
	{
	    bmp.delay_ms(meas_dur); /* Measurement time */

	    rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
	    /* Check if rslt == BMP280_OK, if not, then handle accordingly */

	    int32_t temp32 = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &bmp);
	    uint32_t pres32 = bmp280_comp_pres_32bit(ucomp_data.uncomp_press, &bmp);
	    uint32_t pres64 = bmp280_comp_pres_64bit(ucomp_data.uncomp_press, &bmp);
	    double temp = bmp280_comp_temp_double(ucomp_data.uncomp_temp, &bmp);
	    double pres = bmp280_comp_pres_double(ucomp_data.uncomp_press, &bmp);

	    printf("UT: %lu, UP: %lu, T32: %ld, P32: %lu, P64: %lu, P64N: %lu, T: %f, P: %f\r\n", \
	      ucomp_data.uncomp_temp, ucomp_data.uncomp_press, temp32, \
	      pres32, pres64, pres64 / 256, temp, pres);

	    bmp.delay_ms(30); /* Sleep time between measurements */
	}
}

struct bmp280_data user_bmp280_read_single(void)
{
	struct bmp280_uncomp_data ucomp_data;
	struct bmp280_data data;

	rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
	/* Check if rslt == BMP280_OK, if not, then handle accordingly */

    data.temperature = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &bmp);
    data.pressure = bmp280_comp_pres_32bit(ucomp_data.uncomp_press, &bmp);
    data.altitude = calculate_altitude(data.temperature, data.pressure);

    return data;
}

static void user_bmp280_data_to_buffer(struct bmp280_data data)
{
	buffer.data[buffer.end].temperature = data.temperature;
	buffer.data[buffer.end].pressure = data.pressure;
	buffer.data[buffer.end].altitude = data.altitude;

	buffer.end = (buffer.end + 1) % BUFFER_SIZE;
	if (buffer.count < BUFFER_SIZE)
		buffer.count++;
}

static struct bmp280_data user_bmp280_calculate_average(void)
{
	struct bmp280_data average;
	uint8_t index;

	average.temperature = 0;
	average.pressure = 0;
	average.altitude = 0;

	for (index = 0; index < buffer.count; index++)
	{
		average.temperature += buffer.data[index].temperature;
		average.pressure += buffer.data[index].pressure;
		average.altitude += buffer.data[index].altitude;
	}

	average.temperature /= buffer.count;
	average.pressure /= buffer.count;
	average.altitude /= buffer.count;

	return average;
}

struct bmp280_data user_bmp280_read_average(void)
{
	struct bmp280_uncomp_data ucomp_data;
	struct bmp280_data data_new, data_average;

	rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
	/* Check if rslt == BMP280_OK, if not, then handle accordingly */

    data_new.temperature = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &bmp);
    data_new.pressure = bmp280_comp_pres_32bit(ucomp_data.uncomp_press, &bmp);
    data_new.altitude = calculate_altitude(data_new.temperature, data_new.pressure);

    user_bmp280_data_to_buffer(data_new);
    data_average = user_bmp280_calculate_average();

    return data_average;
}

void user_bmp280_normal_mode(void)
{
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
	/* Check if rslt == BMP280_OK, if not, then handle accordingly */
}

void user_bmp280_sleep_mode(void)
{
	rslt = bmp280_set_power_mode(BMP280_SLEEP_MODE, &bmp);
	/* Check if rslt == BMP280_OK, if not, then handle accordingly */
}
