/*
 * Copyright (c) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_AG413_H__
#define __SENSOR_AG413_H__

#include <device.h>
#include <drivers/i2c.h>

/*sensor address*/
#define AG413                  		0x28 


/* status codes*/

#define STATUS_NORMAL          		0x00
#define STATUS_CMD_MODE        		0x01
#define STATUS_STALE_DATA      		0x02
#define STATUS_EEPROM_ERR      		0x03

/* pressure formula constants */

#define PRESSURE_ERROR         		0xDD    //221 dec is 0xDD
#define PRESSURE_ALPHA_UPPER_VAL	0x1CCD  //7373 dec
#define PRESSURE_ALPHA_LOWER_VAL	0x1F4   //500 dec
#define PRESSURE_BETA_VAL		0x333   //819 dec

struct  ag413_data{
	char StatusData;
	int PressureData;
	int TemperatureData;
};



 



#endif /* __SENSOR_AG413_H__ */
