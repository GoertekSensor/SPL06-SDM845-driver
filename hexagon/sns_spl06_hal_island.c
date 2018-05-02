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

#include "sns_rc.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_event_service.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_service_manager.h"
#include "sns_com_port_types.h"
#include "sns_sync_com_port_service.h"
#include "sns_types.h"

#include "sns_spl06_hal.h"
#include "sns_spl06_sensor.h"
#include "sns_spl06_sensor_instance.h"

#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"

#include "sns_std_sensor.pb.h"
#include "sns_timer.pb.h"
#include "sns_diag_service.h"
#include "sns_std.pb.h"
#include "sns_diag.pb.h"

/*!
 * @brief list all the coefficients parameter
*/
static const uint32_t scalefactor_list[] = {
	524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

#if SPL_CONFIG_ENABLE_FILTER
static const float filter_coefficients[] = {
	0.5, 0.0, 0.136729, 0.726543, 0.072960, 0.854081, 0.015466, 0.969067};
#endif

#if SPL_CONFIG_ENABLE_DIAG_LOG
typedef struct log_sensor_state_raw_info
{
	/* Pointer to diag service */
	sns_diag_service *diag;
	/* Pointer to sensor instance */
	sns_sensor_instance *instance;
	/* Pointer to sensor UID*/
	struct sns_sensor_uid *sensor_uid;
	/* Size of a single encoded sample */
	size_t encoded_sample_size;
	/* Pointer to log*/
	void *log;
	/* Size of allocated space for log*/
	uint32_t log_size;
	/* Number of actual bytes written*/
	uint32_t bytes_written;
	/* Number of batch samples written*/
	/* A batch may be composed of several logs*/
	uint32_t batch_sample_cnt;
	/* Number of log samples written*/
	uint32_t log_sample_cnt;
} log_sensor_state_raw_info;

// Unencoded batch sample
typedef struct
{
	/* Batch Sample type as defined in sns_diag.pb.h */
	sns_diag_batch_sample_type sample_type;
	/* Timestamp of the sensor state data sample */
	sns_time timestamp;
	/*Raw sensor state data sample*/
	float sample[SPL06_NUM_AXES];
	/* Data status.*/
	sns_std_sensor_sample_status status;
} spl06_batch_sample;

/**
 * Encode Sensor State Log.Interrupt
 *
 * @param[i] log Pointer to log packet information
 * @param[i] log_size Size of log packet information
 * @param[i] encoded_log_size Maximum permitted encoded size of
 *                            the log
 * @param[o] encoded_log Pointer to location where encoded
 *                       log should be generated
 * @param[o] bytes_written Pointer to actual bytes written
 *       during encode
 *
 * @return sns_rc,
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc spl06_encode_sensor_state_log_interrupt(
	void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
	size_t *bytes_written)
{
	UNUSED_VAR(log_size);
	sns_rc rc = SNS_RC_SUCCESS;

	if(NULL == encoded_log || NULL == log || NULL == bytes_written) {
		return SNS_RC_FAILED;
	}

	sns_diag_sensor_state_interrupt *sensor_state_interrupt =
	(sns_diag_sensor_state_interrupt *)log;
	pb_ostream_t stream = pb_ostream_from_buffer(encoded_log, encoded_log_size);

	if(!pb_encode(&stream, sns_diag_sensor_state_interrupt_fields,
	            sensor_state_interrupt)) {
		rc = SNS_RC_FAILED;
	}

	if (SNS_RC_SUCCESS == rc) {
		*bytes_written = stream.bytes_written;
	}

	return rc;
}

/**
 * Encode log sensor state raw packet
 *
 * @param[i] log Pointer to log packet information
 * @param[i] log_size Size of log packet information
 * @param[i] encoded_log_size Maximum permitted encoded size of
 *                            the log
 * @param[o] encoded_log Pointer to location where encoded
 *                       log should be generated
 * @param[o] bytes_written Pointer to actual bytes written
 *       during encode
 *
 * @return sns_rc
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc spl06_encode_log_sensor_state_raw(
	void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
	size_t *bytes_written)
{
	sns_rc rc = SNS_RC_SUCCESS;
	uint32_t i = 0;
	size_t encoded_sample_size = 0;
	size_t parsed_log_size = 0;
	sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
	uint8_t arr_index = 0;
	float temp[SPL06_NUM_AXES];
	pb_float_arr_arg arg = {.arr = (float *)temp, .arr_len = SPL06_NUM_AXES,
	.arr_index = &arr_index};

	if(NULL == encoded_log || NULL == log || NULL == bytes_written) {
		return SNS_RC_FAILED;
	}

	batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
	batch_sample.sample.arg = &arg;

	if(!pb_get_encoded_size(&encoded_sample_size, sns_diag_batch_sample_fields,
	                      &batch_sample)) {
		return SNS_RC_FAILED;
	}

	pb_ostream_t stream = pb_ostream_from_buffer(encoded_log, encoded_log_size);
	spl06_batch_sample *raw_sample = (spl06_batch_sample *)log;

	while(parsed_log_size < log_size &&
	    (stream.bytes_written + encoded_sample_size)<= encoded_log_size &&
	    i < (uint32_t)(-1)) {
		arr_index = 0;
		arg.arr = (float *)raw_sample[i].sample;

		batch_sample.sample_type = raw_sample[i].sample_type;
		batch_sample.status = raw_sample[i].status;
		batch_sample.timestamp = raw_sample[i].timestamp;

		if(!pb_encode_tag(&stream, PB_WT_STRING,
		                  sns_diag_sensor_state_raw_sample_tag)) {
			rc = SNS_RC_FAILED;
			break;
		} else if(!pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
		                             &batch_sample)) {
			rc = SNS_RC_FAILED;
			break;
		}

		parsed_log_size += sizeof(spl06_batch_sample);
		i++;
	}

	if (SNS_RC_SUCCESS == rc) {
		*bytes_written = stream.bytes_written;
	}

	return rc;
}

/**
 * Allocate Sensor State Raw Log Packet
 *
 * @param[i] log_raw_info Sensor state log info
 * @param[i] log_size     Optional size of log packet to
 *    be allocated. If not provided by setting to 0, will
 *    default to using maximum supported log packet size
 */
void spl06_log_sensor_state_raw_alloc(
	log_sensor_state_raw_info *log_raw_info,
	uint32_t log_size)
{
	uint32_t max_log_size = 0;

	if(NULL == log_raw_info || NULL == log_raw_info->diag ||
	 NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid) {
		return;
	}

	// allocate memory for sensor state - raw sensor log packet
	max_log_size = log_raw_info->diag->api->get_max_log_size(
	   			log_raw_info->diag);

	if(0 == log_size) {
		// log size not specified.
		// Use max supported log packet size
		log_raw_info->log_size = max_log_size;
	} else if(log_size <= max_log_size) {
		log_raw_info->log_size = log_size;
	} else {
		return;
	}

	log_raw_info->log = log_raw_info->diag->api->alloc_log(
						log_raw_info->diag,
						log_raw_info->instance,
						log_raw_info->sensor_uid,
						log_raw_info->log_size,
						SNS_DIAG_SENSOR_STATE_LOG_RAW);

	log_raw_info->log_sample_cnt = 0;
	log_raw_info->bytes_written = 0;
}

/**
 * Submit the Sensor State Raw Log Packet
 *
 * @param[i] log_raw_info   Pointer to logging information
 *       pertaining to the sensor
 * @param[i] batch_complete true if submit request is for end
 *       of batch
 *  */
void spl06_log_sensor_state_raw_submit(
	log_sensor_state_raw_info *log_raw_info,
	bool batch_complete)
{
	spl06_batch_sample *sample = NULL;

	if(NULL == log_raw_info || NULL == log_raw_info->diag ||
	 NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
	 NULL == log_raw_info->log) {
		return;
	}

	sample = (spl06_batch_sample *)log_raw_info->log;

	if(batch_complete) {
		// overwriting previously sample_type for last sample
		if(1 == log_raw_info->batch_sample_cnt) {
		  	sample[0].sample_type =
		    SNS_DIAG_BATCH_SAMPLE_TYPE_ONLY;
		} else if(1 < log_raw_info->batch_sample_cnt) {
		  	sample[log_raw_info->log_sample_cnt - 1].sample_type =
		    SNS_DIAG_BATCH_SAMPLE_TYPE_LAST;
		}
	}

	log_raw_info->diag->api->submit_log(
				    log_raw_info->diag,
				    log_raw_info->instance,
				    log_raw_info->sensor_uid,
				    log_raw_info->bytes_written,
				    log_raw_info->log,
				    SNS_DIAG_SENSOR_STATE_LOG_RAW,
				    log_raw_info->log_sample_cnt * log_raw_info->encoded_sample_size,
				    spl06_encode_log_sensor_state_raw);

	log_raw_info->log = NULL;
}

/**
 *
 * Add raw uncalibrated sensor data to Sensor State Raw log
 * packet
 *
 * @param[i] log_raw_info Pointer to logging information
 *                        pertaining to the sensor
 * @param[i] raw_data     Uncalibrated sensor data to be logged
 * @param[i] timestamp    Timestamp of the sensor data
 * @param[i] status       Status of the sensor data
 *
 * * @return sns_rc,
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc spl06_log_sensor_state_raw_add(
	log_sensor_state_raw_info *log_raw_info,
	float *raw_data,
	sns_time timestamp,
	sns_std_sensor_sample_status status)
{
	sns_rc rc = SNS_RC_SUCCESS;

	if(NULL == log_raw_info || NULL == log_raw_info->diag ||
		NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
		NULL == raw_data || NULL == log_raw_info->log) {
		return SNS_RC_FAILED;
	}

	if( (log_raw_info->bytes_written + sizeof(spl06_batch_sample)) >
	 	log_raw_info->log_size) {
		// no more space in log packet
		// submit and allocate a new one
		spl06_log_sensor_state_raw_submit(log_raw_info, false);
		spl06_log_sensor_state_raw_alloc(log_raw_info, 0);
	}

	if(NULL == log_raw_info->log) {
		rc = SNS_RC_FAILED;
	} else {
		spl06_batch_sample *sample = (spl06_batch_sample *)log_raw_info->log;

		if(0 == log_raw_info->batch_sample_cnt) {
		  	sample[log_raw_info->log_sample_cnt].sample_type =
		    SNS_DIAG_BATCH_SAMPLE_TYPE_FIRST;
		} else {
		  	sample[log_raw_info->log_sample_cnt].sample_type =
		    SNS_DIAG_BATCH_SAMPLE_TYPE_INTERMEDIATE;
		}

		sample[log_raw_info->log_sample_cnt].timestamp = timestamp;

		sns_memscpy(sample[log_raw_info->log_sample_cnt].sample,
		            sizeof(sample[log_raw_info->log_sample_cnt].sample),
		            raw_data,
		            sizeof(sample[log_raw_info->log_sample_cnt].sample));

		sample[log_raw_info->log_sample_cnt].status = status;

		log_raw_info->bytes_written += sizeof(spl06_batch_sample);

		log_raw_info->log_sample_cnt++;
		log_raw_info->batch_sample_cnt++;
	}

	return rc;
}
#endif

/**
 * Read wrapper for Synch Com Port Service.
 *
 * @param[i] port_handle      port handle
 * @param[i] reg_addr         register address
 * @param[i] buffer           read buffer
 * @param[i] bytes            bytes to read
 * @param[o] xfer_bytes       bytes read
 *
 * @return sns_rc
 */
sns_rc spl06_com_read_wrapper(
	sns_sync_com_port_service *scp_service,
	sns_sync_com_port_handle *port_handle,
	uint32_t reg_addr,
	uint8_t  *buffer,
	uint32_t bytes,
	uint32_t *xfer_bytes)
{
	sns_port_vector port_vec;
	port_vec.buffer = buffer;
	port_vec.bytes = bytes;
	port_vec.is_write = false;
	port_vec.reg_addr = reg_addr;

	return scp_service->api->sns_scp_register_rw(port_handle,
	                          &port_vec,
	                          1,
	                          false,
	                          xfer_bytes);
}

/**
 * Write wrapper for Synch Com Port Service.
 *
 * @param[i] port_handle      port handle
 * @param[i] reg_addr         register address
 * @param[i] buffer           write buffer
 * @param[i] bytes            bytes to write
 * @param[o] xfer_bytes       bytes written
 * @param[i] save_write_time  true to save write transfer time.
 *
 * @return sns_rc
 */
sns_rc spl06_com_write_wrapper(
	sns_sync_com_port_service * scp_service,
	sns_sync_com_port_handle *port_handle,
	uint32_t reg_addr,
	uint8_t  *buffer,
	uint32_t bytes,
	uint32_t *xfer_bytes,
	bool     save_write_time)
{
	sns_port_vector port_vec;
	port_vec.buffer = buffer;
	port_vec.bytes = bytes;
	port_vec.is_write = true;
	port_vec.reg_addr = reg_addr;

	return  scp_service->api->sns_scp_register_rw(
			port_handle,
			&port_vec,
			1,
			save_write_time,
			xfer_bytes);
}

 /*!
 *	@brief This API is used to read raw temperature
 *	in the registers 0x03, 0x04 and 0x05
 *	@note 0x03 -> MSB -> bit from 0 to 7
 *	@note 0x04 -> LSB -> bit from 0 to 7
 *	@note 0x05 -> LSB -> bit from 0 to 7
 *
 *	@param temperature : The raw temperature value.
 *
 *	@return sns_rc
 *	@SNS_RC_SUCCESS -> Success
 *	@SNS_RC_FAILED -> Error
 *
*/
sns_rc spl06_read_raw_temperature(
	spl06_instance_state *state,
	int32_t *temperature)
{
	sns_rc rc = SNS_RC_SUCCESS;
	uint8_t a_data_u8r[3] = {0};
	int32_t utemperature = 0;
	uint32_t xfer_bytes;
  
	/* check the state struct pointer as NULL*/
	if (state == NULL) {
		rc = SNS_RC_FAILED;
	} else {
  		/* read temperature data */
    	rc = state->com_read(
        	state->scp_service,
        	state->com_port_info.port_handle,
        	SPL06_TEMPERATURE_MSB_REG, 
        	a_data_u8r, 3, &xfer_bytes);
		
		if((rc != SNS_RC_SUCCESS) || (xfer_bytes != 3)) {
			rc = SNS_RC_FAILED;
		} else {
			utemperature = ((int32_t)a_data_u8r[0]<<16) | ((int32_t)a_data_u8r[1]<<8) | (int32_t)a_data_u8r[2];	 
			*temperature = (utemperature&0x800000) ? (0xFF000000|utemperature) : utemperature;
		}
	}
	return rc;
}

/*!
 *	@brief This API is used to read uncompensated pressure.
 *	in the registers 0x00, 0x01 and 0x02
 *	@note 0x00 -> MSB -> bit from 0 to 7
 *	@note 0x01 -> LSB -> bit from 0 to 7
 *	@note 0x02 -> LSB -> bit from 0 to 7
 *
 *	@param pressure : The raw pressure value.
 *
 *	@return sns_rc
 *	@SNS_RC_SUCCESS -> Success
 *	@SNS_RC_FAILED -> Error
 *
*/
sns_rc spl06_read_raw_pressure(
	spl06_instance_state *state,
	int32_t *pressure)
{
	sns_rc rc = SNS_RC_SUCCESS;
	uint8_t a_data_u8r[3] = {0};
	int32_t upressure = 0;
	uint32_t xfer_bytes;
	
	/* check the state structure pointer as NULL*/
	if (state == NULL) {
		rc = SNS_RC_FAILED;
	} else {
		/* read pressure data */
		rc = state->com_read(
			state->scp_service,
			state->com_port_info.port_handle,
			SPL06_PRESSURE_MSB_REG, 
			a_data_u8r, 3, &xfer_bytes);

		if((rc != SNS_RC_SUCCESS) || (xfer_bytes != 3)) {
			rc = SNS_RC_FAILED;
		} else {
			upressure = ((int32_t)a_data_u8r[0]<<16) | ((int32_t)a_data_u8r[1]<<8) | (int32_t)a_data_u8r[2];	
			*pressure = (upressure&0x800000) ? (0xFF000000|upressure) : upressure;
		}
	}
	return rc;
}

 /*!
 * @brief This API used to read
 * actual temperature from uncompensated temperature
 * @note Returns the value in Degree centigrade
 * @note Output value of "5123" equals 51.23 DegC.
 *
 * @param temperature : Actual temperature in floating point
 *
 * @return sns_rc
 * @SNS_RC_SUCCESS -> Success
 * @SNS_RC_FAILED -> Error
 *
*/
sns_rc spl06_compensate_temperature_double(
	spl06_instance_state *state,
	float *temperature)
{
	sns_rc rc = SNS_RC_SUCCESS;
	int32_t utemperature = 0;/* uncompensated temperature */
	double fTsc;

	/* check the state struct pointer as NULL*/
	if (state == NULL) {
		rc = SNS_RC_FAILED;
	} else {
		rc = spl06_read_raw_temperature(state, &utemperature);
		if (rc == SNS_RC_SUCCESS) {
			fTsc = (double)utemperature / (double)state->i32kT;
			/* The result temperature unit should be 0.01deg */
			*temperature = (state->calib_param.c0 * 0.5 + state->calib_param.c1 * fTsc) * 100.0;
		}
	}
	return rc;
}

/*!
 * @brief Reads actual pressure from uncompensated pressure
 * and returns pressure in Pa as double.
 * @note Output value of "96386.2"
 * equals 96386.2 Pa = 963.862 hPa.
 *
 * @param pressure : Actual pressure in floating point
 *
 * @return sns_rc
 * @SNS_RC_SUCCESS -> Success
 * @SNS_RC_FAILED -> Error
 *
*/
sns_rc spl06_compensate_pressure_double(
	spl06_instance_state *state,
	float *pressure)
{
	sns_rc rc = SNS_RC_SUCCESS;
	int32_t utemperature = 0, upressure = 0;/* uncompensated temperature and pressure */
	double fTsc, fPsc;
	double qua2, qua3;
	double fpressure;

	/* check the state struct pointer as NULL*/
	if (state == NULL) {
		rc = SNS_RC_FAILED;
	} else {
		/* read the uncompensated temperature and pressure*/
		rc = spl06_read_raw_temperature(state, &utemperature);
		if (rc == SNS_RC_SUCCESS) {
			rc = spl06_read_raw_pressure(state, &upressure);
			if (rc == SNS_RC_SUCCESS) {
				fTsc = (double)utemperature / (double)state->i32kT;
				fPsc = (double)upressure / (double)state->i32kP;
				qua2 = state->calib_param.c10 + fPsc * (state->calib_param.c20 + fPsc * state->calib_param.c30);
				qua3 = fTsc * fPsc * (state->calib_param.c11 + fPsc * state->calib_param.c21);

				fpressure = state->calib_param.c00 + fPsc * qua2 + fTsc * state->calib_param.c01 + qua3;
				*pressure = fpressure;
#if SPL_CONFIG_ENABLE_FILTER
				if (!state->filter_initialized) {
					state->data_filter[0] = fpressure;
					state->data_filter[1] = *pressure;
					state->filter_initialized = true;
				} else {
					*pressure = filter_coefficients[state->cutoff_freq_index*2]*(fpressure + state->data_filter[0]) + 
							filter_coefficients[state->cutoff_freq_index*2+1]*state->data_filter[1];
					state->data_filter[0] = fpressure;
					state->data_filter[1] = *pressure;
				}
#endif	
			}
		}
	}
	return rc;
}

/*!
 *	@brief This API is used to set
 *	the temperature oversampling setting in the register 0x07
 *	bits from 5 to 7
 *
 *        value             | Temperature oversampling
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_OVERSAMPLING_1X
 *       0x01               |  SPL06_OVERSAMPLING_2X
 *       0x02               |  SPL06_OVERSAMPLING_4X
 *       0x03               |  SPL06_OVERSAMPLING_8X
 *       0x04               |  SPL06_OVERSAMPLING_16X
 *       0x05               |  SPL06_OVERSAMPLING_32X
 *       0x06               |  SPL06_OVERSAMPLING_64X
 *       0x07               |  SPL06_OVERSAMPLING_128X
 *
 *  @param v_value_u8 :The value of temperature over sampling
 *
 *  @return sns_rc
 *  @SNS_RC_SUCCESS -> Success
 *  @SNS_RC_FAILED -> Error
 *
*/
sns_rc spl06_set_oversamp_temperature(
	spl06_instance_state *state,
	uint8_t v_value_u8)
{
	sns_rc rc = SNS_RC_SUCCESS;
	uint8_t v_data_u8 = 0;
	uint32_t xfer_bytes;
	
	/* check the state struct pointer as NULL*/
	if (state == NULL) {
		rc = SNS_RC_FAILED;
	} else {
		/* write over sampling*/
		rc = state->com_read(
					state->scp_service,
					state->com_port_info.port_handle,
					SPL06_TMP_CFG_REG_OVERSAMPLING__REG, 
					&v_data_u8,
					1,
					&xfer_bytes);
		if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
			return SNS_RC_FAILED;
		}
		v_data_u8 = SPL06_SET_BITSLICE(v_data_u8,
					SPL06_TMP_CFG_REG_OVERSAMPLING, v_value_u8);
		v_data_u8 |= SPL06_TMP_SOURCE_EXT;
		rc = state->com_write(
			        state->scp_service,
			        state->com_port_info.port_handle,
			        SPL06_TMP_CFG_REG_OVERSAMPLING__REG,
			        &v_data_u8, 
			        1,
			        &xfer_bytes,
			        false);	
		if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
			return SNS_RC_FAILED;
		}
		state->oversamp_temperature = v_value_u8;
		state->i32kT = scalefactor_list[v_value_u8];
		
		if (v_value_u8 > SPL06_OVERSAMPLING_8X) {
			rc = state->com_read(
						state->scp_service,
						state->com_port_info.port_handle,
						SPL06_CONFIG_REG, 
						&v_data_u8,
						1,
						&xfer_bytes);
			if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
				return SNS_RC_FAILED;
			}
			v_data_u8 |= SPL06_TEMPERATURE_SHIFT;		 
			rc = state->com_write(
				        state->scp_service,
				        state->com_port_info.port_handle,
				        SPL06_CONFIG_REG,
				        &v_data_u8, 
				        1,
				        &xfer_bytes,
				        false);	
			if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
				return SNS_RC_FAILED;
			}			
		} else {		 
			rc = state->com_read(
						state->scp_service,
						state->com_port_info.port_handle,
						SPL06_CONFIG_REG, 
						&v_data_u8,
						1,
						&xfer_bytes);
			if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
				return SNS_RC_FAILED;
			}
			v_data_u8 &= (~SPL06_TEMPERATURE_SHIFT);		 
			rc = state->com_write(
				        state->scp_service,
				        state->com_port_info.port_handle,
				        SPL06_CONFIG_REG,
				        &v_data_u8, 
				        1,
				        &xfer_bytes,
				        false);	
			if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
				return SNS_RC_FAILED;
			}
		} 	
	}
	return rc;
}

/*!
 *	@brief This API is used to set
 *	the pressure oversampling setting in the register 0x06
 *	bits from 2 to 4
 *
 *        value             | Pressure oversampling
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_OVERSAMPLING_1X
 *       0x01               |  SPL06_OVERSAMPLING_2X
 *       0x02               |  SPL06_OVERSAMPLING_4X
 *       0x03               |  SPL06_OVERSAMPLING_8X
 *       0x04               |  SPL06_OVERSAMPLING_16X
 *       0x05               |  SPL06_OVERSAMPLING_32X
 *       0x06               |  SPL06_OVERSAMPLING_64X
 *       0x07               |  SPL06_OVERSAMPLING128X
 *
 *  @param  v_value_u8 : The value of pressure over sampling
 *
 * @return sns_rc
 * @SNS_RC_SUCCESS -> Success
 * @SNS_RC_FAILED -> Error
 *
*/
sns_rc spl06_set_oversamp_pressure(
	spl06_instance_state *state,
	uint8_t v_value_u8)
{
	sns_rc rc = SNS_RC_SUCCESS;
	uint8_t v_data_u8 = 0;
	uint32_t xfer_bytes;

	/* check the state struct pointer as NULL*/
	if (state == NULL) {
		rc = SNS_RC_FAILED;
	} else {
		/* write pressure over sampling */
		rc = state->com_read(
					state->scp_service,
					state->com_port_info.port_handle,
					SPL06_PRS_CFG_REG_OVERSAMPLING__REG, 
					&v_data_u8,
					1,
					&xfer_bytes);
		if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
			return SNS_RC_FAILED;
		}
		v_data_u8 = SPL06_SET_BITSLICE(v_data_u8,
					SPL06_PRS_CFG_REG_OVERSAMPLING, v_value_u8);
		rc = state->com_write(
			        state->scp_service,
			        state->com_port_info.port_handle,
			        SPL06_PRS_CFG_REG_OVERSAMPLING__REG,
			        &v_data_u8, 
			        1,
			        &xfer_bytes,
			        false);	
		if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
			return SNS_RC_FAILED;
		}
		state->oversamp_pressure = v_value_u8;
		state->i32kP = scalefactor_list[v_value_u8];

		if(v_value_u8 > SPL06_OVERSAMPLING_8X) {		 
			rc = state->com_read(
						state->scp_service,
						state->com_port_info.port_handle,
						SPL06_CONFIG_REG, 
						&v_data_u8,
						1,
						&xfer_bytes);
			if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
				return SNS_RC_FAILED;
			}
			v_data_u8 |= SPL06_PRESSURE_SHIFT;		 
			rc = state->com_write(
				        state->scp_service,
				        state->com_port_info.port_handle,
				        SPL06_CONFIG_REG,
				        &v_data_u8, 
				        1,
				        &xfer_bytes,
				        false);	
			if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
				return SNS_RC_FAILED;
			}		
			
		} else {		 	
			rc = state->com_read(
						state->scp_service,
						state->com_port_info.port_handle,
						SPL06_CONFIG_REG, 
						&v_data_u8,
						1,
						&xfer_bytes);
			if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
				return SNS_RC_FAILED;
			}
			v_data_u8 &= (~SPL06_PRESSURE_SHIFT);		 
			rc = state->com_write(
				        state->scp_service,
				        state->com_port_info.port_handle,
				        SPL06_CONFIG_REG,
				        &v_data_u8, 
				        1,
				        &xfer_bytes,
				        false);	
			if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
				return SNS_RC_FAILED;
			}			
		}
	}
	return rc;
}

/*!
 *	@brief This API is used to set
 *	the temperature sample rate setting in the register 0x07
 *	bits from 0 to 2
 *
 *        value             | Temperature samplerate
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_SAMPLERATE_1
 *       0x01               |  SPL06_SAMPLERATE_2
 *       0x02               |  SPL06_SAMPLERATE_4
 *       0x03               |  SPL06_SAMPLERATE_8
 *       0x04               |  SPL06_SAMPLERATE_16
 *       0x05               |  SPL06_SAMPLERATE_32
 *       0x06               |  SPL06_SAMPLERATE_64
 *       0x07               |  SPL06_SAMPLERATE_128
 *
 *  @param  v_value_u8 : The value of temperature sample rate
 *
 * @return sns_rc
 * @SNS_RC_SUCCESS -> Success
 * @SNS_RC_FAILED -> Error
 *
*/
sns_rc spl06_set_samplerate_temperature(
	spl06_instance_state *state,
	uint8_t v_value_u8)
{	
	sns_rc rc = SNS_RC_SUCCESS;
	uint8_t v_data_u8 = 0;
	uint32_t xfer_bytes;
	
	/* check the state struct pointer as NULL*/
	if (state == NULL) {
		rc = SNS_RC_FAILED;
	} else {
		/* write temperature samplerate */
		rc = state->com_read(
					state->scp_service,
					state->com_port_info.port_handle,
					SPL06_TMP_CFG_REG_SAMPLERATE__REG, 
					&v_data_u8,
					1,
					&xfer_bytes);
		if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
			return SNS_RC_FAILED;
		}
		v_data_u8 = SPL06_SET_BITSLICE(
					v_data_u8,
					SPL06_TMP_CFG_REG_SAMPLERATE,
					v_value_u8);
		rc = state->com_write(
					state->scp_service,
					state->com_port_info.port_handle,
					SPL06_TMP_CFG_REG_SAMPLERATE__REG,
					&v_data_u8, 
					1,
					&xfer_bytes,
					false); 
		if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
			return SNS_RC_FAILED;	
		}
		//state->samplerate_temperature = v_value_u8;
	}
	return rc;
}

/*!
 *	@brief This API is used to set
 *	the pressure sample rate setting in the register 0x06
 *	bits from 4 to 6
 *
 *        value             | Pressure samplerate
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_SAMPLERATE_1
 *       0x01               |  SPL06_SAMPLERATE_2
 *       0x02               |  SPL06_SAMPLERATE_4
 *       0x03               |  SPL06_SAMPLERATE_8
 *       0x04               |  SPL06_SAMPLERATE_16
 *       0x05               |  SPL06_SAMPLERATE_32
 *       0x06               |  SPL06_SAMPLERATE_64
 *       0x07               |  SPL06_SAMPLERATE_128
 *
 *  @param  v_value_u8 : The value of pressure sample rate
 *
 * @return sns_rc
 * @SNS_RC_SUCCESS -> Success
 * @SNS_RC_FAILED -> Error
 *
*/
sns_rc spl06_set_samplerate_pressure(
	spl06_instance_state *state,
	uint8_t v_value_u8)
{
	sns_rc rc = SNS_RC_SUCCESS;
	uint8_t v_data_u8 = 0;
	uint32_t xfer_bytes;

	/* check the state struct pointer as NULL*/
	if (state == NULL) {
		rc = SNS_RC_FAILED;
	} else {
		/* write pressure samplerate */
		rc = state->com_read(
					state->scp_service,
					state->com_port_info.port_handle,
					SPL06_PRS_CFG_REG_SAMPLERATE__REG, 
					&v_data_u8,
					1,
					&xfer_bytes);
		if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
			return SNS_RC_FAILED;
		}
		v_data_u8 = SPL06_SET_BITSLICE(
					v_data_u8,
					SPL06_TMP_CFG_REG_SAMPLERATE,
					v_value_u8);
		rc = state->com_write(
					state->scp_service,
					state->com_port_info.port_handle,
					SPL06_PRS_CFG_REG_SAMPLERATE__REG,
					&v_data_u8, 
					1,
					&xfer_bytes,
					false); 
		if (rc != SNS_RC_SUCCESS || xfer_bytes != 1) {
			return SNS_RC_FAILED;	
		}
		//state->samplerate_pressure = v_value_u8;
	}
	return rc;
}

/*!
 * @brief This API used to set the
 *   Operational Mode from the sensor in the register 0x08 bit 0 and 2
 *
 * @param power_mode : The value of power mode value
 *    value            |   Power mode
 * ------------------|------------------
 *	0x00             | SPL06_SLEEP_MODE
 *	0x07             | SPL06_NORMAL_MODE
 *
 * @return sns_rc
 * @SNS_RC_SUCCESS -> Success
 * @SNS_RC_FAILED -> Error
 *
*/
sns_rc spl06_set_power_mode(
	spl06_instance_state *state,
	uint8_t power_mode)
{
	sns_rc rc = SNS_RC_SUCCESS;
	uint32_t xfer_bytes;
	
	/* check the state structure pointer as NULL*/
	if (state == NULL) {
		rc = SNS_RC_FAILED;
	} else {
		if (power_mode <= SPL06_NORMAL_MODE) {
			/* write the power mode*/
			rc = state->com_write(
			  	state->scp_service,
			  	state->com_port_info.port_handle,
			  	SPL06_CTRL_MEAS_REG_POWER_MODE__REG,
			  	&power_mode,
			  	1,
			  	&xfer_bytes,
			  	false);
			if((rc != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
				rc = SNS_RC_FAILED;
			}
			state->power_mode = power_mode;
		}
	}
	return rc;
}

void spl06_handle_temperature_data_sample(
    sns_sensor_instance *const instance,
    float sample_data,
    sns_time timestamp)
{
	spl06_instance_state *state = (spl06_instance_state*)instance->state->state;
	sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
	sns_sensor_uid suid;

	suid = state->temperature_info.suid;
#if SPL_CONFIG_ENABLE_DIAG_LOG
	sns_diag_service* diag       = state->diag_service;
	log_sensor_state_raw_info log_sensor_state_raw_info;
	log_sensor_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
	log_sensor_state_raw_info.diag = diag;
	log_sensor_state_raw_info.instance = instance;
	log_sensor_state_raw_info.sensor_uid = &suid;
	spl06_log_sensor_state_raw_alloc(&log_sensor_state_raw_info, 0);
#endif
	pb_send_sensor_stream_event(instance,
				&suid,
				timestamp,
				SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
				status,
				&sample_data,
				1,
				state->encoded_imu_event_len);
#if SPL_CONFIG_ENABLE_DIAG_LOG
	spl06_log_sensor_state_raw_add(
				&log_sensor_state_raw_info,
				&sample_data,
				timestamp,
				status);
	spl06_log_sensor_state_raw_submit(&log_sensor_state_raw_info, true);
#endif
}

void spl06_handle_pressure_data_sample(
	sns_sensor_instance *const instance,
	float sample_data,
	sns_time timestamp)
{
	spl06_instance_state *state = (spl06_instance_state*)instance->state->state;
	sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
	sns_sensor_uid suid;

	suid = state->pressure_info.suid;
#if SPL_CONFIG_ENABLE_DIAG_LOG
	sns_diag_service* diag       = state->diag_service;
	log_sensor_state_raw_info log_sensor_state_raw_info;
	log_sensor_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
	log_sensor_state_raw_info.diag = diag;
	log_sensor_state_raw_info.instance = instance;
	log_sensor_state_raw_info.sensor_uid = &suid;
	spl06_log_sensor_state_raw_alloc(&log_sensor_state_raw_info, 0);
#endif
	pb_send_sensor_stream_event(instance,
				&suid,
				timestamp,
				SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
				status,
				&sample_data,
				1,
				state->encoded_imu_event_len);
#if SPL_CONFIG_ENABLE_DIAG_LOG
	spl06_log_sensor_state_raw_add(
				&log_sensor_state_raw_info,
				&sample_data,
				timestamp,
				status);
	spl06_log_sensor_state_raw_submit(&log_sensor_state_raw_info, true);
#endif
}

#if SPL_CONFIG_ENABLE_DAE
void spl06_convert_and_send_pressure_sample(
	sns_sensor_instance *const instance,
	sns_time            timestamp,
	const uint8_t       data[6])
{
	spl06_instance_state *state = (spl06_instance_state*)instance->state->state;
	int32_t utemperature = 0;
	int32_t upressure = 0;
	int32_t value = 0;
	float pressure = 0;
	float spl06_comp_data_buffer = 0;
	double fTsc, fPsc;
	double qua2, qua3;

	value = ((int32_t)a_data_u8r[0]<<16) | ((int32_t)a_data_u8r[1]<<8) | (int32_t)a_data_u8r[2];	  
	upressure = (value&0x800000) ? (0xFF000000|value) : value;

	value = ((int32_t)data[3]<<16) | ((int32_t)data[4]<<8) | (int32_t)data[5];  
	utemperature = (value&0x800000) ? (0xFF000000|value) : value;

	fTsc = (double)utemperature / (double)state->i32kT;
	fPsc = (double)upressure / (double)state->i32kP;
	qua2 = state->calib_param.c10 + fPsc * (state->calib_param.c20 + fPsc * state->calib_param.c30);
	qua3 = fTsc * fPsc * (state->calib_param.c11 + fPsc * state->calib_param.c21);

	pressure = state->calib_param.c00 + fPsc * qua2 + fTsc * state->calib_param.c01 + qua3;
	spl06_comp_data_buffer = pressure / 100;
	spl06_handle_pressure_data_sample(instance, spl06_comp_data_buffer, timestamp);
}

void spl06_convert_and_send_temp_sample(
	sns_sensor_instance *const instance,
	sns_time            timestamp,
	const uint8_t       data[3])
{
	spl06_instance_state *state = (spl06_instance_state*)instance->state->state;
	int32_t value = 0; 
	int32_t utemperature = 0;
	float temperature = 0;
	float spl06_comp_data_buffer = 0;
	double fTsc;

	value = ((int32_t)data[0]<<16) | ((int32_t)data[1]<<8) | (int32_t)data[2];  
	utemperature = (value&0x800000) ? (0xFF000000|value) : value;

	fTsc = (double)utemperature / (double)state->i32kT;
	/* The result temperature unit should be 0.01deg */
	temperature = (state->calib_param.c0 * 0.5 + state->calib_param.c1 * fTsc) * 100.0;
	spl06_comp_data_buffer = temperature / 100;
	spl06_handle_temperature_data_sample(instance, spl06_comp_data_buffer, timestamp);
}
#endif

void spl06_process_pressure_timer_stream_data_buffer(sns_sensor_instance *instance, sns_timer_sensor_event * const timer_event)
{
	spl06_instance_state *state = (spl06_instance_state*)instance->state->state;
	float pressure = 0;
	float spl06_comp_data_buffer = 0;

	spl06_compensate_pressure_double(state, &pressure);
	spl06_comp_data_buffer = pressure / 100;
	SPL_INST_LOG(LOW, instance, "p timer_event p_data %d", (uint32_t)(pressure));
	spl06_handle_pressure_data_sample(instance, spl06_comp_data_buffer, (sns_time)timer_event->timeout_time);
}

void spl06_process_temperature_timer_stream_data_buffer(sns_sensor_instance *instance, sns_timer_sensor_event * const timer_event)
{
	spl06_instance_state *state = (spl06_instance_state*)instance->state->state;
	float temperature = 0;
	float spl06_comp_data_buffer = 0;

	spl06_compensate_temperature_double(state, &temperature);
	spl06_comp_data_buffer = temperature / 100;
	SPL_INST_LOG(LOW, instance, "t timer_event t_data %d", (uint32_t)(temperature));
	spl06_handle_temperature_data_sample(instance, spl06_comp_data_buffer, (sns_time)timer_event->timeout_time);
}

void spl06_handle_pressure_data_stream_timer_event(sns_sensor_instance *const instance, sns_timer_sensor_event * const timer_event)
{
	spl06_instance_state *state = (spl06_instance_state*)instance->state->state;
	sns_rc rc = SNS_RC_SUCCESS;
	spl06_process_pressure_timer_stream_data_buffer(instance, timer_event);
	//rc = spl06_set_power_mode(state, SPL06_NORMAL_MODE);
	if (rc != SNS_RC_SUCCESS)
		SNS_INST_PRINTF(ERROR, instance,"config power mode failed");
}

void spl06_handle_temperature_data_stream_timer_event(sns_sensor_instance *const instance, sns_timer_sensor_event * const timer_event)
{
	spl06_instance_state *state = (spl06_instance_state*)instance->state->state;
	sns_rc rc = SNS_RC_SUCCESS;
	spl06_process_temperature_timer_stream_data_buffer(instance, timer_event);
	//rc = spl06_set_power_mode(state, SPL06_NORMAL_MODE);
	if (rc != SNS_RC_SUCCESS)
		SNS_INST_PRINTF(ERROR, instance,"config power mode failed");
}
