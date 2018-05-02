/*******************************************************************************
 * Copyright (c) 2018, GOERTEK INC.
 * All rights reserved.
 *
 * Description: Goertek SPL06 pressure sensor driver
 * VERSION: V1.1
 * History: V1.0 --- [2018.01.18]:Original version for SEE with I2C: 
 							pressure, temperature works on the SDM845
		V1.1 --- [2018.01.31]:Add 1rd IIR filter with different cutoff frequency for pressure measurement, 
		   					and the filter can be enabled or disabled by the customer
		V1.2 --- [2018.02.06]:Modify the Maximum ODR from 128.0 to 32.0
*******************************************************************************/
#pragma once

#include <stdint.h>
#include "sns_sensor.h"
#include "sns_sensor_uid.h"

#include "sns_spl06_sensor.h"
#include "sns_spl06_sensor_instance.h"

#include "sns_timer.pb.h"

/* common definition */
#define SPL06_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define SPL06_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/* chip id */
#define SPL06_CHIP_ID      (0x10)

/* 7-bit addr: 0x76 (SDO connected to GND); 
			0x77 (SDO connected to VDDIO) 
*/
#define SPL06_I2C_ADDRESS1                  (0x76)
#define SPL06_I2C_ADDRESS2                  (0x77)
#define SPL06_I2C_ADDRESS 					SPL06_I2C_ADDRESS1

/* power mode */
#define SPL06_SLEEP_MODE                    (0x00)
#define SPL06_NORMAL_MODE                   (0x07)
#define SPL06_SOFT_RESET_CODE               (0x09)

/* sample rate*/
#define SPL06_SAMPLERATE_1               	(0x00)
#define SPL06_SAMPLERATE_2                  (0x01)
#define SPL06_SAMPLERATE_4                 	(0x02)
#define SPL06_SAMPLERATE_8                 	(0x03)
#define SPL06_SAMPLERATE_16                	(0x04)
#define SPL06_SAMPLERATE_32                	(0x05)
#define SPL06_SAMPLERATE_64                	(0x06)
#define SPL06_SAMPLERATE_128               	(0x07)

/* oversampling */
#define SPL06_OVERSAMPLING_1X          		(0x00)
#define SPL06_OVERSAMPLING_2X               (0x01)
#define SPL06_OVERSAMPLING_4X               (0x02)
#define SPL06_OVERSAMPLING_8X               (0x03)
#define SPL06_OVERSAMPLING_16X              (0x04)
#define SPL06_OVERSAMPLING_32X              (0x05)
#define SPL06_OVERSAMPLING_64X              (0x06)
#define SPL06_OVERSAMPLING_128X             (0x07)

#if SPL_CONFIG_ENABLE_FILTER
/* IIR filter cutoff frequency */
#define SPL06_IIRFILTER_CUTOFFFREQ_OFF       0 //0.5 
#define SPL06_IIRFILTER_CUTOFFFREQ_LOW       1 //0.1
#define SPL06_IIRFILTER_CUTOFFFREQ_MED       2 //0.05  
#define SPL06_IIRFILTER_CUTOFFFREQ_HIGH      3 //0.01  
#endif

/* work mode */
#define SPL06_LOW_POWER_MODE	            (0x00)
#define SPL06_STANDARD_RESOLUTION_MODE      (0x01)
#define SPL06_HIGH_RESOLUTION_MODE          (0x02)

#define SPL06_LOWPOWER_SAMPLERATE_PRESSURE	           		SPL06_SAMPLERATE_1
#define SPL06_LOWPOWER_OVERSAMPLING_PRESSURE	           	SPL06_OVERSAMPLING_2X
#define SPL06_LOWPOWER_SAMPLERATE_TEMPERATURE	       		SPL06_SAMPLERATE_1
#define SPL06_LOWPOWER_OVERSAMPLING_TEMPERATURE	           	SPL06_OVERSAMPLING_1X

#define SPL06_STANDARDRESOLUTION_SAMPLERATE_PRESSURE     	SPL06_SAMPLERATE_2
#define SPL06_STANDARDRESOLUTION_OVERSAMPLING_PRESSURE     	SPL06_OVERSAMPLING_16X
#define SPL06_STANDARDRESOLUTION_SAMPLERATE_TEMPERATURE  	SPL06_SAMPLERATE_1
#define SPL06_STANDARDRESOLUTION_OVERSAMPLING_TEMPERATURE  	SPL06_OVERSAMPLING_1X

#define SPL06_HIGHRESOLUTION_SAMPLERATE_PRESSURE         	SPL06_SAMPLERATE_4
#define SPL06_HIGHRESOLUTION_OVERSAMPLING_PRESSURE         	SPL06_OVERSAMPLING_64X
#define SPL06_HIGHRESOLUTION_SAMPLERATE_TEMPERATURE      	SPL06_SAMPLERATE_4
#define SPL06_HIGHRESOLUTION_OVERSAMPLING_TEMPERATURE      	SPL06_OVERSAMPLING_1X

#define SPL06_PRESSURE_SHIFT 				0x04
#define SPL06_TEMPERATURE_SHIFT     		0x08

#define SPL06_TMP_SOURCE_INT				0x00
#define SPL06_TMP_SOURCE_EXT				0x80

/* calibration data */
#define SPL06_CALIBRATION_DATA_START       	(0x10)
#define SPL06_CALIBRATION_DATA_LENGTH		(18)

/* register address */
#define SPL06_CHIP_ID_REG                   (0x0D)  /*Chip ID Register */
#define SPL06_RESET_REG                     (0x0C)  /*Softreset Register */
#define SPL06_INT_STATUS_REG                (0x0A)  /*Status Register */
#define SPL06_FIFO_STATUS_REG               (0x0B)  /*Status Register */
#define SPL06_PRS_CFG_REG                   (0x06)  /*Pressure Config Register */
#define SPL06_TMP_CFG_REG                   (0x07)  /*Temperature Config Register */
#define SPL06_CTRL_MEAS_REG                 (0x08)  /*Ctrl Measure Register */
#define SPL06_CONFIG_REG                    (0x09)  /*Configuration Register */

/* Data Register */
#define SPL06_PRESSURE_MSB_REG              (0x00)  /*Pressure MSB Register */
#define SPL06_PRESSURE_LSB_REG              (0x01)  /*Pressure LSB Register */
#define SPL06_PRESSURE_XLSB_REG             (0x02)  /*Pressure XLSB Register */
#define SPL06_TEMPERATURE_MSB_REG           (0x03)  /*Temperature MSB Reg */
#define SPL06_TEMPERATURE_LSB_REG           (0x04)  /*Temperature LSB Reg */
#define SPL06_TEMPERATURE_XLSB_REG          (0x05)  /*Temperature XLSB Reg */

/* pressure oversampling bit definition */
#define SPL06_PRS_CFG_REG_OVERSAMPLING__POS   	(0)
#define SPL06_PRS_CFG_REG_OVERSAMPLING__MSK     (0x0F)
#define SPL06_PRS_CFG_REG_OVERSAMPLING__LEN     (4)
#define SPL06_PRS_CFG_REG_OVERSAMPLING__REG     (SPL06_PRS_CFG_REG)

/* temperature oversampling bit definition */
#define SPL06_TMP_CFG_REG_OVERSAMPLING__POS   	(0)
#define SPL06_TMP_CFG_REG_OVERSAMPLING__MSK   	(0x07)
#define SPL06_TMP_CFG_REG_OVERSAMPLING__LEN   	(3)
#define SPL06_TMP_CFG_REG_OVERSAMPLING__REG   	(SPL06_TMP_CFG_REG)

/* pressure samplerate bit definition */
#define SPL06_PRS_CFG_REG_SAMPLERATE__POS      	(4)
#define SPL06_PRS_CFG_REG_SAMPLERATE__MSK    	(0x70)
#define SPL06_PRS_CFG_REG_SAMPLERATE__LEN     	(3)
#define SPL06_PRS_CFG_REG_SAMPLERATE__REG   	(SPL06_PRS_CFG_REG)

/* temperature samplerate bit definition */
#define SPL06_TMP_CFG_REG_SAMPLERATE__POS      	(4)
#define SPL06_TMP_CFG_REG_SAMPLERATE__MSK      	(0x70)
#define SPL06_TMP_CFG_REG_SAMPLERATE__LEN      	(3)
#define SPL06_TMP_CFG_REG_SAMPLERATE__REG      	(SPL06_TMP_CFG_REG)

/* power mode bit definition */
#define SPL06_CTRL_MEAS_REG_POWER_MODE__POS   	(0)
#define SPL06_CTRL_MEAS_REG_POWER_MODE__MSK     (0x07)
#define SPL06_CTRL_MEAS_REG_POWER_MODE__LEN     (3)
#define SPL06_CTRL_MEAS_REG_POWER_MODE__REG     (SPL06_CTRL_MEAS_REG)

/** Off to idle time */
#define SPL_OFF_TO_IDLE_MS      100  //ms
#define SPL06_NUM_AXES                            1

#if SPL_CONFIG_ENABLE_DAE
void spl06_convert_and_send_pressure_sample(
	sns_sensor_instance *const instance,
	sns_time            timestamp,
	const uint8_t       data[6]);
void spl06_convert_and_send_temp_sample(
	sns_sensor_instance *const instance,
	sns_time            timestamp,
	const uint8_t       data[3]);
#endif
void spl06_set_temperature_polling_config(sns_sensor_instance *const this);
void spl06_set_pressure_polling_config(sns_sensor_instance *const this);
void spl06_handle_pressure_data_stream_timer_event(sns_sensor_instance * const instance, sns_timer_sensor_event * const timer_event);
void spl06_handle_temperature_data_stream_timer_event(sns_sensor_instance * const instance, sns_timer_sensor_event * const timer_event);
