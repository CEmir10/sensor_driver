/* bme680.c - Driver for Bosch Sensortec's BME680 temperature, pressure,
 * humidity and gas sensor
 *
 * https://www.bosch-sensortec.com/bst/products/all_products/bme680
 */

/*
 * Copyright (c) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bme680

#include "ag413.h"
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <init.h>
#include <kernel.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <drivers/sensor.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(bme680, CONFIG_SENSOR_LOG_LEVEL);

static int mpr_read_reg(const struct device *dev)
{
	struct mpr_data *data = dev->data;
	const struct mpr_config *cfg = dev->config;

	uint8_t write_buf[] = { STATUS_CMD_MODE, 0x00, 0x00 };
	uint8_t read_buf[4] = { 0x0 };

	int rc = i2c_write(data->i2c_master, write_buf, sizeof(write_buf),
			   cfg->i2c_addr);

	if (rc < 0) {
		return rc;
	}

	uint8_t retries = MPR_REG_READ_MAX_RETRIES;

	for (; retries > 0; retries--) {
		k_sleep(K_MSEC(MPR_REG_READ_DATA_CONV_DELAY_MS));

		rc = i2c_read(data->i2c_master, read_buf, sizeof(read_buf),
				  cfg->i2c_addr);
		if (rc < 0) {
			return rc;
		}

		if (!(*read_buf & MPR_STATUS_MASK_POWER_ON)
			|| (*read_buf & MPR_STATUS_MASK_INTEGRITY_TEST_FAILED)
			|| (*read_buf & MPR_STATUS_MASK_MATH_SATURATION)) {
			return -EIO;
		}

		if (!(*read_buf & MPR_STATUS_MASK_BUSY)) {
			break;
		}
	}

	if (retries == 0) {
		return -EIO;
	}

	data->reg_val = (read_buf[1] << 16)
			| (read_buf[2] << 8)
			|  read_buf[3];

	return 0;
}


static int ag413_reg_read(const struct device *dev, uint8_t start,
			   uint8_t *buf, int size)
{
	const struct ag413_config *config = dev->config;

	return i2c_burst_read_dt(&config->bus, start, buf, size);
}

static int ag413_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct bme680_config *config = dev->config;

	return i2c_reg_write_byte_dt(&config->bus, reg, val);
}

static int ag413_calc_temp(struct ag413_data *data) //, uint32_t temp)
{
	int temperature=0;
	temperature=((200/2047)* data.TemperatureData)-50;
	return temperature;
}

static double ag413_calc_press(struct ag413_data *data, int temperature)
{
	double pressure;
	int error_multiplier=getPressureErrorMultiplier(temperature);
	pressure=(data.PressureData-PRESSURE_BETA_VAL+(PRESSURE_ERROR*error_multiplier));
	return pressure;
}

static double getPressureErrorMultiplier(int temperature){
	
	int error_multiplier=0;
	
	if((temperature>=-40) && (temperature<-30)
	{
		error_multiplier=3;
	}
	
	else if((temperature>=-30) && (temperature<-10)
	{
		error_multiplier=2;
	}
	
	
	else if((temperature>=-10) && (temperature<95)
	{
		error_multiplier=1;
	}
	
	else if((temperature>=-95) && (temperature<-115)
	{
		error_multiplier=2;
	}
	else
	{
		error_multiplier=3;
	}
	
	return error_multiplier;

}


static int ag413_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct ag413_data *data = dev->data;
	const struct ag413_cfg *config = dev->config;

	LOG_DBG("Fetching sample from DPS310");

	switch (chan) {
	case SENSOR_CHAN_TEMP:
		if (!ag413_measure_tmp(data, config)) {
			LOG_ERR("Failed to measure temperature");
			return -EIO;
		}
		break;
	case SENSOR_CHAN_PRESS:
		if (!ag413_measure_psr(data, config)) {
			LOG_ERR("Failed to measure pressure");
			return -EIO;
		}
		break;
	case SENSOR_CHAN_ALL:
		if (!ag413_measure_tmp_psr(data, config)) {
			LOG_ERR("Failed to measure temperature and pressure");
			return -EIO;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Get the measurement data */
static int ag413_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct ag413_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_TEMP:
		val->val1 = data->tmp_val1;
		val->val2 = data->tmp_val2;
		break;
	case SENSOR_CHAN_PRESS:
		val->val1 = data->psr_val1;
		val->val2 = data->psr_val2;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}





static int ag413_init(const struct device *dev)
{
	const struct ag413_config *config = dev->config;

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C master %s not ready", config->bus.bus->name);
		return -EINVAL;
	}

	if (ag413_chip_init(dev) < 0) {
		return -EINVAL;
	}

	return 0;
}

static const struct sensor_driver_api ag413_api_funcs = {
	.sample_fetch = ag413_sample_fetch,
	.channel_get = ag413_channel_get,
};

static struct ag413_data ag413_data;

static const struct ag413_config ag413_config = {
	.bus = I2C_DT_SPEC_INST_GET(0)
};

DEVICE_DT_INST_DEFINE(0, ag413_init, NULL, &ag413_data,
		      &ag413_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		      &ag413_api_funcs);
