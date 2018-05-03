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

#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_types.h"

#include "sns_spl06_hal.h"
#include "sns_spl06_sensor.h"
#include "sns_spl06_sensor_instance.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_sensor_util.h"
#include "sns_sync_com_port_service.h"

void spl06_start_sensor_temp_polling_timer(sns_sensor_instance *this)
{
	spl06_instance_state *state = (spl06_instance_state*)this->state->state;
	sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
	uint8_t buffer[50] = {0};
	sns_request timer_req = {
		.message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
		.request    = buffer
	};
	sns_rc rc = SNS_RC_SUCCESS;

	SPL_INST_LOG(LOW, this, "spl06_start_sensor_temp_polling_timer");

	if(NULL == state->temperature_timer_data_stream) {
		sns_service_manager *smgr = this->cb->get_service_manager(this);
		sns_stream_service *srtm_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);

		rc = srtm_svc->api->create_sensor_instance_stream(srtm_svc,
	      	this, state->timer_suid, &state->temperature_timer_data_stream);
	}

	if((SNS_RC_SUCCESS != rc) || (NULL == state->temperature_timer_data_stream)) {
		SNS_INST_PRINTF(ERROR, this, "failed timer stream create rc = %d", rc);
		return;
	}

	req_payload.is_periodic = true;
	req_payload.start_time = sns_get_system_time();
	req_payload.timeout_period = state->temperature_info.sampling_intvl;
	timer_req.request_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
	                                        sns_timer_sensor_config_fields, NULL);
	if(timer_req.request_len > 0) {
		state->temperature_timer_data_stream->api->send_request(state->temperature_timer_data_stream, &timer_req);
		state->temperature_info.timer_is_active = true;
	} else {
		//diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
		//                         "LSM timer req encode error");
	}
}

void spl06_start_sensor_pressure_polling_timer(sns_sensor_instance *this)
{
	spl06_instance_state *state = (spl06_instance_state*)this->state->state;
	sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
	uint8_t buffer[50] = {0};
	sns_request timer_req = {
		.message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
		.request    = buffer
	};
	sns_rc rc = SNS_RC_SUCCESS;
	SPL_INST_LOG(LOW, this, "spl06_start_sensor_pressure_polling_timer");

	if(NULL == state->pressure_timer_data_stream) {
		sns_service_manager *smgr = this->cb->get_service_manager(this);
		sns_stream_service *srtm_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);

		rc = srtm_svc->api->create_sensor_instance_stream(srtm_svc,
	      	this, state->timer_suid, &state->pressure_timer_data_stream);
	}

	if((SNS_RC_SUCCESS != rc) || (NULL == state->pressure_timer_data_stream)) {
		SNS_INST_PRINTF(ERROR, this, "failed timer stream create rc = %d", rc);
		return;
	}

	req_payload.is_periodic = true;
	req_payload.start_time = sns_get_system_time();
	req_payload.timeout_period = state->pressure_info.sampling_intvl;
	timer_req.request_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
	                                        sns_timer_sensor_config_fields, NULL);
	if(timer_req.request_len > 0) {
		state->pressure_timer_data_stream->api->send_request(state->pressure_timer_data_stream, &timer_req);
		state->pressure_info.timer_is_active = true;
	} else {
		//diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
		//                         "LSM timer req encode error");
	}
}

void spl06_set_pressure_polling_config(sns_sensor_instance *const this)
{
	spl06_instance_state *state = (spl06_instance_state*)this->state->state;
	sns_rc rc = SNS_RC_SUCCESS;
	SPL_INST_LOG(LOW, this,
				"pressure_info.timer_is_active:%d state->pressure_info.sampling_intvl:%u",
				state->pressure_info.timer_is_active,
				state->pressure_info.sampling_intvl);
	if(state->pressure_info.sampling_intvl > 0) {
		rc = spl06_set_power_mode(state, SPL06_NORMAL_MODE);
		if(rc != SNS_RC_SUCCESS) {
	  		SNS_INST_PRINTF(ERROR, this,"config power mode failed");
		} else {
			spl06_start_sensor_pressure_polling_timer(this);
		}
	}
	else if(state->pressure_info.timer_is_active) {
		state->pressure_info.timer_is_active = false;
		sns_sensor_util_remove_sensor_instance_stream(this,
	                                              &state->pressure_timer_data_stream);
		spl06_set_power_mode(state, SPL06_SLEEP_MODE);
	}
}

void spl06_set_temperature_polling_config(sns_sensor_instance *const this)
{
	spl06_instance_state *state = (spl06_instance_state*)this->state->state;
	sns_rc rc = SNS_RC_SUCCESS;
	SPL_INST_LOG(LOW, this,
				"temperature_info.timer_is_active:%d state->temperature_info.sampling_intvl:%u",
				state->temperature_info.timer_is_active,
				state->temperature_info.sampling_intvl);

	if(state->temperature_info.sampling_intvl > 0) {
		rc = spl06_set_power_mode(state, SPL06_NORMAL_MODE);
		if(rc != SNS_RC_SUCCESS) {
		  SNS_INST_PRINTF(ERROR, this,"config power mode failed");
		} else {
		  spl06_start_sensor_temp_polling_timer(this);
		}
	}
	else if(state->temperature_info.timer_is_active) {
		state->temperature_info.timer_is_active = false;
		sns_sensor_util_remove_sensor_instance_stream(this,
		                                              &state->temperature_timer_data_stream);
		spl06_set_power_mode(state, SPL06_SLEEP_MODE);
	}
}

/*!
 *	@brief This API is used to write
 *	 the working mode of the sensor
 *
 *  @param work_mode : The value of work mode
 *   value      |  mode
 * -------------|-------------
 *    0         | SPL06_LOW_POWER_MODE
 *    1         | SPL06_STANDARD_RESOLUTION_MODE
 *    2         | SPL06_HIGH_RESOLUTION_MODE
 *
 * @return sns_rc
 * @SNS_RC_SUCCESS -> Success
 * @SNS_RC_FAILED -> Error
 *
*/
sns_rc spl06_set_work_mode(spl06_instance_state *state, uint8_t work_mode)
{
	sns_rc rc = SNS_RC_SUCCESS;
	uint8_t oversamp_t = 0, oversamp_p = 0;
	uint8_t samplerate_t = 0, samplerate_p = 0;

	/* check the state structure pointer as NULL*/
	if (state == NULL) {
		rc = SNS_RC_FAILED;
	} else {
		if (work_mode <= SPL06_HIGH_RESOLUTION_MODE) {
			samplerate_p = state->samplerate_pressure;
			samplerate_t = state->samplerate_temperature;
			if (((samplerate_p > SPL06_SAMPLERATE_8) || (samplerate_t > SPL06_SAMPLERATE_8)) 
				&& (work_mode == SPL06_HIGH_RESOLUTION_MODE))
				work_mode = SPL06_STANDARD_RESOLUTION_MODE;
			if (((samplerate_p > SPL06_SAMPLERATE_32) || (samplerate_t > SPL06_SAMPLERATE_32))
				&& (work_mode == SPL06_STANDARD_RESOLUTION_MODE))
				work_mode = SPL06_LOW_POWER_MODE;			
			switch (work_mode) {
			/* write work mode*/
			case SPL06_LOW_POWER_MODE:
				oversamp_p =
				SPL06_LOWPOWER_OVERSAMPLING_PRESSURE;							
				oversamp_t =
				SPL06_LOWPOWER_OVERSAMPLING_TEMPERATURE;				
			break;		
			case SPL06_STANDARD_RESOLUTION_MODE:		  
				oversamp_p =
				SPL06_STANDARDRESOLUTION_OVERSAMPLING_PRESSURE;
				oversamp_t =
				SPL06_STANDARDRESOLUTION_OVERSAMPLING_TEMPERATURE;				
			break;
			case SPL06_HIGH_RESOLUTION_MODE:			  
				oversamp_p =
				SPL06_HIGHRESOLUTION_OVERSAMPLING_PRESSURE;
				oversamp_t =
				SPL06_HIGHRESOLUTION_OVERSAMPLING_TEMPERATURE;			
			break;
			}
			rc = spl06_set_samplerate_pressure(state, samplerate_p);			
			if (rc != SNS_RC_SUCCESS) {
				return SNS_RC_FAILED;	
			}	
			rc = spl06_set_samplerate_temperature(state, samplerate_t);
			if (rc != SNS_RC_SUCCESS) {
				return SNS_RC_FAILED;	
			}
			rc = spl06_set_oversamp_pressure(state, oversamp_p);
			if (rc != SNS_RC_SUCCESS) {
				return SNS_RC_FAILED;
			}
			rc = spl06_set_oversamp_temperature(state, oversamp_t);
			if (rc != SNS_RC_SUCCESS) {
				return SNS_RC_FAILED;	
			}

			state->work_mode = work_mode;
		} else {
			rc = SNS_RC_FAILED;
		}
	}
	return rc;
}

void spl06_reconfig_hw(sns_sensor_instance *this,
  spl06_sensor_type sensor_type)
{
	spl06_instance_state *state = (spl06_instance_state*)this->state->state;
	sns_rc rc = SNS_RC_SUCCESS;
	SPL_INST_LOG(LOW, this, "spl06_reconfig_hw state->config_step = %d",state->config_step);
	SPL_INST_LOG(LOW, this,
				"enable sensor flag:0x%x publish sensor flag:0x%x",
				state->deploy_info.enable,
				state->deploy_info.publish_sensors);
	// Enable timer in case of sensor pressure clients
	if (sensor_type == SPL06_PRESSURE)
	//&&
	//!spl06_dae_if_available(this))
	{
		rc = spl06_set_work_mode(state, SPL06_LOW_POWER_MODE);
		if (rc != SNS_RC_SUCCESS)
			SNS_INST_PRINTF(ERROR, this, "set work mode failed");
		spl06_set_pressure_polling_config(this);
	}
	// Enable timer in case of sensor pressure clients
	if (sensor_type == SPL06_TEMPERATURE)
	//&&
	// !spl06_dae_if_available(this))
	{
		rc = spl06_set_work_mode(state, SPL06_LOW_POWER_MODE);
		if (rc != SNS_RC_SUCCESS)
			SNS_INST_PRINTF(ERROR, this, "set work mode failed");
		spl06_set_temperature_polling_config(this);
	}
	//if(state->deploy_info.publish_sensors != 0)
	//{
	//   spl06_dae_if_start_streaming(this);
	// }
	state->config_step = SPL06_CONFIG_IDLE; /* done with reconfig */
	SPL_INST_LOG(LOW, this, "spl06_reconfig_hw finished");
}

void spl06_send_config_event(sns_sensor_instance *const instance)
{
	spl06_instance_state *state = (spl06_instance_state*)instance->state->state;
	sns_std_sensor_physical_config_event phy_sensor_config =
	sns_std_sensor_physical_config_event_init_default;

	char operating_mode[] = SPL_NORMAL;

	pb_buffer_arg op_mode_args;

	op_mode_args.buf = &operating_mode[0];
	op_mode_args.buf_len = sizeof(operating_mode);

	phy_sensor_config.has_sample_rate = true;
	phy_sensor_config.has_water_mark = false;
	phy_sensor_config.water_mark = 1;
	phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
	phy_sensor_config.operation_mode.arg = &op_mode_args;
	phy_sensor_config.has_active_current = true;
	phy_sensor_config.has_resolution = true;
	phy_sensor_config.range_count = 2;
	phy_sensor_config.has_stream_is_synchronous = true;
	phy_sensor_config.stream_is_synchronous = false;
	phy_sensor_config.has_dri_enabled = true;
	phy_sensor_config.dri_enabled = true;
	if (state->deploy_info.publish_sensors & SPL06_PRESSURE)
	{
		phy_sensor_config.sample_rate = state->pressure_info.sampling_rate_hz;
		phy_sensor_config.has_active_current = true;
		phy_sensor_config.active_current = SPL06_SENSOR_PRESSURE_NORMAL_POWER_CURRENT;
		phy_sensor_config.resolution = SPL06_SENSOR_PRESSURE_RESOLUTION;
		phy_sensor_config.range_count = 2;
		phy_sensor_config.range[0] = SPL06_SENSOR_PRESSURE_RANGE_MIN;
		phy_sensor_config.range[1] = SPL06_SENSOR_PRESSURE_RANGE_MAX;
		phy_sensor_config.has_dri_enabled = true;
		phy_sensor_config.dri_enabled = false;
		pb_send_event(
			    instance,
			    sns_std_sensor_physical_config_event_fields,
			    &phy_sensor_config,
			    sns_get_system_time(),
			    SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
			    &state->pressure_info.suid);
	}
	if(state->deploy_info.publish_sensors & SPL06_TEMPERATURE)
	{
		phy_sensor_config.sample_rate = state->temperature_info.sampling_rate_hz;
		phy_sensor_config.has_active_current = true;
		phy_sensor_config.active_current = SPL06_SENSOR_TEMPERATURE_NORMAL_POWER_CURRENT;
		phy_sensor_config.resolution = SPL06_SENSOR_TEMPERATURE_RESOLUTION;
		phy_sensor_config.range_count = 2;
		phy_sensor_config.range[0] = SPL06_SENSOR_TEMPERATURE_RANGE_MIN;
		phy_sensor_config.range[1] = SPL06_SENSOR_TEMPERATURE_RANGE_MAX;
		phy_sensor_config.has_dri_enabled = true;
		phy_sensor_config.dri_enabled = false;
		pb_send_event(
				instance,
				sns_std_sensor_physical_config_event_fields,
				&phy_sensor_config,
				sns_get_system_time(),
				SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
				&state->temperature_info.suid);
	}
}

static sns_rc spl06_validate_sensor_pressure_odr(sns_sensor_instance *this)
{
	sns_rc rc = SNS_RC_SUCCESS;
	spl06_instance_state *state = (spl06_instance_state*)this->state->state;
	SPL_INST_LOG(LOW, this, "pressure odr = %d", (int8_t)state->pressure_info.sampling_rate_hz);
	if((state->pressure_info.sampling_rate_hz > SPL_ODR_0) &&
		(state->pressure_info.sampling_rate_hz <= SPL_ODR_1)) {
		state->pressure_info.sampling_rate_hz = SPL_ODR_1;
		state->samplerate_pressure = SPL06_SAMPLERATE_2;
	}
	else if((state->pressure_info.sampling_rate_hz > SPL_ODR_1) &&
		(state->pressure_info.sampling_rate_hz <= SPL_ODR_2)) {
		state->pressure_info.sampling_rate_hz = SPL_ODR_2;
		state->samplerate_pressure = SPL06_SAMPLERATE_4;
	}
	else if((state->pressure_info.sampling_rate_hz > SPL_ODR_2) &&
		state->pressure_info.sampling_rate_hz <= SPL_ODR_4) {
		state->pressure_info.sampling_rate_hz = SPL_ODR_4;
		state->samplerate_pressure = SPL06_SAMPLERATE_8;
	}
	else if((state->pressure_info.sampling_rate_hz > SPL_ODR_4) &&
		state->pressure_info.sampling_rate_hz <= SPL_ODR_8) {
		state->pressure_info.sampling_rate_hz = SPL_ODR_8;
		state->samplerate_pressure = SPL06_SAMPLERATE_16;
	}
	else if((state->pressure_info.sampling_rate_hz > SPL_ODR_8) &&
		state->pressure_info.sampling_rate_hz <= SPL_ODR_16) {
		state->pressure_info.sampling_rate_hz = SPL_ODR_16;
		state->samplerate_pressure = SPL06_SAMPLERATE_32;
	}
	else if((state->pressure_info.sampling_rate_hz > SPL_ODR_16) &&
		state->pressure_info.sampling_rate_hz <= SPL_ODR_32) {
		state->pressure_info.sampling_rate_hz = SPL_ODR_32;
		state->samplerate_pressure = SPL06_SAMPLERATE_64;
	}
	else if((state->pressure_info.sampling_rate_hz > SPL_ODR_32) &&
		state->pressure_info.sampling_rate_hz <= SPL_ODR_64) {
		state->pressure_info.sampling_rate_hz = SPL_ODR_64;
		state->samplerate_pressure = SPL06_SAMPLERATE_128;
	} else {
		state->pressure_info.sampling_intvl = 0;
		state->pressure_info.timer_is_active = 0;
		SPL_INST_LOG(LOW, this, "close pressure sensor = %d, timer_is_active =%d",
		          (uint32_t)state->pressure_info.sampling_rate_hz, state->pressure_info.timer_is_active);
		rc = SNS_RC_NOT_SUPPORTED;
	}
	if (rc == SNS_RC_SUCCESS) {
		state->pressure_info.sampling_intvl =
			sns_convert_ns_to_ticks(1000000000.0 / state->pressure_info.sampling_rate_hz);
		SPL_INST_LOG(LOW, this, "pressure timer_value = %u", (uint32_t)state->pressure_info.sampling_intvl);
	}

	return rc;
}

static sns_rc spl06_validate_sensor_temp_odr(sns_sensor_instance *this)
{
	sns_rc rc = SNS_RC_SUCCESS;
	spl06_instance_state *state = (spl06_instance_state*)this->state->state;
	SPL_INST_LOG(LOW, this, "temperature odr = %d", (int8_t)state->temperature_info.sampling_rate_hz);
	if((state->temperature_info.sampling_rate_hz > SPL_ODR_0) &&
		state->temperature_info.sampling_rate_hz <= SPL_ODR_1) {
		state->temperature_info.sampling_rate_hz = SPL_ODR_1;
		state->samplerate_temperature = SPL06_SAMPLERATE_2;
	}
	else if((state->temperature_info.sampling_rate_hz > SPL_ODR_1) &&
		(state->temperature_info.sampling_rate_hz <= SPL_ODR_2)) {
		state->temperature_info.sampling_rate_hz = SPL_ODR_2;
		state->samplerate_temperature = SPL06_SAMPLERATE_4;
	}
	else if((state->temperature_info.sampling_rate_hz > SPL_ODR_2) &&
		state->temperature_info.sampling_rate_hz <= SPL_ODR_4) {
		state->temperature_info.sampling_rate_hz = SPL_ODR_4;
		state->samplerate_temperature = SPL06_SAMPLERATE_8;
	}
	else if((state->temperature_info.sampling_rate_hz > SPL_ODR_4) &&
		state->temperature_info.sampling_rate_hz <= SPL_ODR_8) {
		state->temperature_info.sampling_rate_hz = SPL_ODR_8;
		state->samplerate_temperature = SPL06_SAMPLERATE_16;
	}
	else if((state->temperature_info.sampling_rate_hz > SPL_ODR_8) &&
		state->temperature_info.sampling_rate_hz <= SPL_ODR_16) {
		state->temperature_info.sampling_rate_hz = SPL_ODR_16;
		state->samplerate_temperature = SPL06_SAMPLERATE_32;
	}
	else if((state->temperature_info.sampling_rate_hz > SPL_ODR_16) &&
		state->temperature_info.sampling_rate_hz <= SPL_ODR_32) {
		state->temperature_info.sampling_rate_hz = SPL_ODR_32;
		state->samplerate_temperature = SPL06_SAMPLERATE_64;
	}
	else if((state->temperature_info.sampling_rate_hz > SPL_ODR_32) &&
		state->temperature_info.sampling_rate_hz <= SPL_ODR_64) {
		state->temperature_info.sampling_rate_hz = SPL_ODR_64;
		state->samplerate_temperature = SPL06_SAMPLERATE_128;
	} else {
		state->temperature_info.sampling_intvl = 0;
		state->temperature_info.timer_is_active = 0;
		SPL_INST_LOG(LOW, this, "close temperature sensor = %d, timer_is_active =%d",
		       (uint32_t)state->temperature_info.sampling_rate_hz, state->temperature_info.timer_is_active);
		rc = SNS_RC_NOT_SUPPORTED;
	}
	if (rc == SNS_RC_SUCCESS) {
		state->temperature_info.sampling_intvl =
			sns_convert_ns_to_ticks(1000000000.0 / state->temperature_info.sampling_rate_hz);
		SPL_INST_LOG(LOW, this, "temperature timer_value = %u", (uint32_t)state->temperature_info.sampling_intvl);
	}

	return rc;
}

static void inst_cleanup(sns_sensor_instance *const this, sns_stream_service *stream_mgr)
{
	spl06_instance_state *state = (spl06_instance_state*)this->state->state;
#if SPL_CONFIG_ENABLE_DAE
	spl06_dae_if_deinit(state, stream_mgr);
#else
	UNUSED_VAR(stream_mgr);
#endif
	sns_sensor_util_remove_sensor_instance_stream(this, &state->async_com_port_data_stream);
	sns_sensor_util_remove_sensor_instance_stream(this, &state->pressure_timer_data_stream);
	sns_sensor_util_remove_sensor_instance_stream(this, &state->temperature_timer_data_stream);

	if(NULL != state->scp_service) {
		state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
		state->scp_service->api->sns_scp_deregister_com_port(&state->com_port_info.port_handle);
		state->scp_service = NULL;
	}
}
sns_rc spl06_inst_init(sns_sensor_instance * const this,
	sns_sensor_state const *sstate)
{
	spl06_instance_state *state = (spl06_instance_state*)this->state->state;
	spl06_state *sensor_state = (spl06_state*)sstate->state;
	
	float stream_data[1] = {0};
	sns_service_manager *service_mgr = this->cb->get_service_manager(this);
	sns_stream_service *stream_mgr = (sns_stream_service*)
						service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
	uint64_t buffer[10];
	pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));
	sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
	uint8_t arr_index = 0;
	float diag_temp[SPL06_NUM_AXES];
	pb_float_arr_arg arg = {
		.arr = (float*)diag_temp, 
		.arr_len = SPL06_NUM_AXES,
		.arr_index = &arr_index
		};
	batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
	batch_sample.sample.arg = &arg;
	sns_sensor_uid ascp_suid;
#if SPL_CONFIG_ENABLE_DAE
	sns_sensor_uid dae_suid;
#endif
	state->scp_service = (sns_sync_com_port_service*)
			service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);
#if SPL_CONFIG_ENABLE_ISLAND_MODE
	state->island_service = (sns_island_service*)
			service_mgr->get_service(service_mgr, SNS_ISLAND_SERVICE);
#endif
	sns_suid_lookup_get(&sensor_state->common.suid_lookup_data, "async_com_port", &ascp_suid);
	SPL_INST_LOG(LOW, this, "<sns_see_if__  init> from sensor:0x%x", sensor_state->sensor);
	/**---------Setup stream connections with dependent Sensors---------*/

	stream_mgr->api->create_sensor_instance_stream(stream_mgr,
													this,
													ascp_suid,
													&state->async_com_port_data_stream);
	/** Initialize COM port to be used by the Instance */
	sns_memscpy(&state->com_port_info.com_config,
				sizeof(state->com_port_info.com_config),
				&sensor_state->common.com_port_info.com_config,
				sizeof(sensor_state->common.com_port_info.com_config));

	state->scp_service->api->sns_scp_register_com_port(&state->com_port_info.com_config,
	                                  &state->com_port_info.port_handle);

	if((NULL == state->async_com_port_data_stream) || (NULL == state->com_port_info.port_handle)) {
		inst_cleanup(this, stream_mgr);
		return SNS_RC_FAILED;
	}

	/**----------- Copy all Sensor UIDs in instance state -------------*/
	sns_memscpy(&state->pressure_info.suid,
				sizeof(state->pressure_info.suid),
				&((sns_sensor_uid)PRESSURE_SUID),
				sizeof(state->pressure_info.suid));
	sns_memscpy(&state->temperature_info.suid,
				sizeof(state->temperature_info.suid),
				&((sns_sensor_uid)TEMPERATURE_SUID),
				sizeof(state->temperature_info.suid));
	sns_suid_lookup_get(&sensor_state->common.suid_lookup_data, "timer", &state->timer_suid);


	/** Copy calibration data*/
	sns_memscpy(&state->calib_param,
				sizeof(state->calib_param),
				&sensor_state->calib_param,
				sizeof(sensor_state->calib_param));
	state->interface = sensor_state->common.com_port_info.com_config.bus_instance;
#if SPL_CONFIG_ENABLE_FILTER
	state->cutoff_freq_index = SPL06_IIRFILTER_CUTOFFFREQ_OFF;
	state->filter_initialized = false;
	state->data_filter[0] = 0.0;
	state->data_filter[1] = 0.0;
#endif	
	state->power_mode = SPL06_NORMAL_MODE;/*default power mode*/
	state->work_mode = SPL06_LOW_POWER_MODE;/*default working mode*/
	state->i32kP = 1572864; /* default pressure scale factor variable */
	state->i32kT = 524288; /* default temperature scale factor variable */	
	state->oversamp_temperature = SPL06_LOWPOWER_OVERSAMPLING_TEMPERATURE;/* default temperature over sampling */
	state->oversamp_pressure = SPL06_LOWPOWER_OVERSAMPLING_PRESSURE;/* default pressure over sampling */
	state->samplerate_temperature = SPL06_LOWPOWER_SAMPLERATE_TEMPERATURE;/* default temperature sampling rate */	
	state->samplerate_pressure = SPL06_LOWPOWER_SAMPLERATE_PRESSURE;/* default pressure sampling rate */
	state->com_read = spl06_com_read_wrapper;/* com read function*/
	state->com_write = spl06_com_write_wrapper;/*com write function*/
	/* set the data report length to the framework */
	state->encoded_imu_event_len = pb_get_encoded_size_sensor_stream_event(stream_data, 1);
	state->diag_service =  (sns_diag_service*)service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);
	state->scp_service =  (sns_sync_com_port_service*)service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);
	state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);
	state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);
	/** Configure the Async Com Port */
	{
		sns_data_stream* data_stream = state->async_com_port_data_stream;
		sns_com_port_config* com_config = &sensor_state->common.com_port_info.com_config;
		uint8_t pb_encode_buffer[100];
		sns_request async_com_port_request = {
			.message_id  = SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_CONFIG,
			.request     = &pb_encode_buffer
		};

		state->ascp_config.bus_type          = (com_config->bus_type == SNS_BUS_I2C) ?
												SNS_ASYNC_COM_PORT_BUS_TYPE_I2C : SNS_ASYNC_COM_PORT_BUS_TYPE_SPI;
		state->ascp_config.slave_control     = com_config->slave_control;
		state->ascp_config.reg_addr_type     = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
		state->ascp_config.min_bus_speed_kHz = com_config->min_bus_speed_KHz;
		state->ascp_config.max_bus_speed_kHz = com_config->max_bus_speed_KHz;
		state->ascp_config.bus_instance      = com_config->bus_instance;

		async_com_port_request.request_len = pb_encode_request(pb_encode_buffer,
														sizeof(pb_encode_buffer),
														&state->ascp_config,
														sns_async_com_port_config_fields,
														NULL);
		data_stream->api->send_request(data_stream, &async_com_port_request);
	}

	/** Determine size of sns_diag_sensor_state_raw as defined in
	*  sns_diag.proto
	*  sns_diag_sensor_state_raw is a repeated array of samples of
	*  type sns_diag_batch sample. The following determines the
	*  size of sns_diag_sensor_state_raw with a single batch
	*  sample */
	if(pb_encode_tag(&stream, PB_WT_STRING, sns_diag_sensor_state_raw_sample_tag)) {
		if(pb_encode_delimited(&stream, sns_diag_batch_sample_fields, &batch_sample)) {
			state->log_raw_encoded_size = stream.bytes_written;
		}
	}
#if SPL_CONFIG_ENABLE_DAE
	sns_suid_lookup_get(&sensor_state->common.suid_lookup_data, "data_acquisition_engine", &dae_suid);
	spl06_dae_if_init(this, stream_mgr, &dae_suid);
#endif
	SPL_INST_LOG(LOW, this, "<sns_see_if__ init> success");

	return SNS_RC_SUCCESS;
}

sns_rc spl06_inst_deinit(sns_sensor_instance *const this)
{
	sns_service_manager *service_mgr = this->cb->get_service_manager(this);
	sns_stream_service *stream_mgr = (sns_stream_service*)
						service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
	inst_cleanup(this, stream_mgr);

	return SNS_RC_SUCCESS;
}

/**
 * Sends a self-test completion event.
 *
 * @param[i] instance  Instance reference
 * @param[i] uid       Sensor UID 
 * @param[i] result    pass/fail result 
 * @param[i] type      test type 
 * @param[i] err_code  driver specific error code 
 *
 * @return none
 */
static void spl06_send_self_test_event(sns_sensor_instance *instance,
                                        sns_sensor_uid *uid, bool test_result,
                                        sns_physical_sensor_test_type type,
                                        spl06_test_err_code err_code)
{
	uint8_t data[1] = {(uint8_t)err_code};
	pb_buffer_arg buff_arg = (pb_buffer_arg) { 
							.buf = &data, 
							.buf_len = sizeof(data) 
							};
	sns_physical_sensor_test_event test_event = sns_physical_sensor_test_event_init_default;

	test_event.test_passed = test_result;
	test_event.test_type = type;
	test_event.test_data.funcs.encode = &pb_encode_string_cb;
	test_event.test_data.arg = &buff_arg;

	pb_send_event(instance,
				sns_physical_sensor_test_event_fields,
				&test_event,
				sns_get_system_time(),
				SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
				uid);
}

#if SPL_CONFIG_ENABLE_SELF_TEST_FAC
void spl06_start_factory_test(sns_sensor_instance* instance, spl06_sensor_type sensor)
{
	spl06_instance_state *state = (spl06_instance_state*)instance->state->state;

	state->fac_test_info.num_samples = 0;
	state->fac_test_info.sample_square_sum[0] = 0;
	state->fac_test_info.sample_square_sum[1] = 0;
	state->fac_test_info.sample_square_sum[2] = 0;
	state->fac_test_info.sample_sum[0] = 0;
	state->fac_test_info.sample_sum[1] = 0;
	state->fac_test_info.sample_sum[2] = 0;
	state->fac_test_in_progress = true;
	state->fac_test_sensor = sensor;
	state->fac_test_info.at_rest = true;

	if(sensor == SPL06_PRESSURE) {
		state->pressure_info.sampling_rate_hz = SPL_ODR_8;
		state->pressure_info.sampling_intvl = 
				sns_convert_ns_to_ticks(1000000000.0 / state->pressure_info.sampling_rate_hz);
	} else {
		SNS_INST_PRINTF(ERROR, instance, "Unknown sensor %d", sensor);
	}
	SPL_INST_LOG(HIGH, instance, "spl06_start_factory_test() sensor %d", sensor);
	spl06_reconfig_hw(instance, sensor);
}
#endif

void spl06_run_self_test(sns_sensor_instance *instance)
{
	spl06_instance_state *state = (spl06_instance_state*)instance->state->state;
	sns_rc rv = SNS_RC_SUCCESS;
	uint8_t buffer = 0;
	bool chip_id_success = false;
	SPL_INST_LOG(LOW, instance, "spl06_run_self_test");
	rv = spl06_get_chip_type(state->scp_service,
							state->com_port_info.port_handle,
							&buffer);
	if((rv == SNS_RC_SUCCESS) && (buffer == SPL06_CHIP_ID)) {
		chip_id_success = true;
	}
	SPL_INST_LOG(HIGH, instance, "spl06_run_self_test chip id success = %d", chip_id_success);

	if(state->pressure_info.test_info.test_client_present) {
		SPL_INST_LOG(HIGH, instance, "test_client_present  test_type = %d", state->pressure_info.test_info.test_type);
		if(state->pressure_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM) {
			spl06_send_self_test_event(instance, &state->pressure_info.suid,
	                           		chip_id_success, SNS_PHYSICAL_SENSOR_TEST_TYPE_COM, SPL06_TEST_NO_ERROR);
		}
		else if(state->pressure_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY) {
#if SPL_CONFIG_ENABLE_SELF_TEST_FAC
			spl06_start_factory_test(instance, SPL06_PRESSURE);
#endif
		}
		state->pressure_info.test_info.test_client_present = false;
	}
	if(state->temperature_info.test_info.test_client_present) {
		if(state->temperature_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM) {
			spl06_send_self_test_event(instance, &state->temperature_info.suid,
									chip_id_success, SNS_PHYSICAL_SENSOR_TEST_TYPE_COM, SPL06_TEST_NO_ERROR);
		}
		else if(state->temperature_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY) {
			// Handle factory test. The driver may choose to reject any new
			// streaming/self-test requests when factory test is in progress.
		}
		state->temperature_info.test_info.test_client_present = false;
	}
}

sns_rc spl06_inst_set_client_config(
	sns_sensor_instance * const this,
	sns_request const *client_request)
{
	spl06_instance_state *state = (spl06_instance_state*) this->state->state;
	state->client_req_id = client_request->message_id;
	float desired_sample_rate = 0;
	float desired_report_rate = 0;
	spl06_sensor_type sensor_type = SPL_SENSOR_INVALID;
	uint8_t power_mode = SPL06_NORMAL_MODE;
	sns_spl06_cfg_req *payload = (sns_spl06_cfg_req*)client_request->request;
	sns_rc rv = SNS_RC_SUCCESS;

	SPL_INST_LOG(LOW, this, "<sns_see_if__  set_client_config>");

	/* Turn COM port ON, *physical* */
	state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, true);

	if(client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
	{
		// 1. Extract sample, report rates from client_request.
		// 2. Configure sensor HW.
		// 3. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
		// 4. sendRequest() for Intrerupt register/de-register in case of DRI using interrupt_data_stream.
		// 5. Save the current config information like type, sample_rate, report_rate, etc.

		desired_sample_rate = payload->sample_rate;
		desired_report_rate = payload->report_rate;
		sensor_type = payload->sensor_type;
		power_mode = payload->power_mode;

		if(desired_report_rate > desired_sample_rate) {
			/* bad request. Return error or default report_rate to sample_rate */
			desired_report_rate = desired_sample_rate;
		}
		if(sensor_type == SPL06_TEMPERATURE) {
			rv = spl06_validate_sensor_temp_odr(this);
			if((rv != SNS_RC_SUCCESS) && (desired_sample_rate != 0)) {
				// TODO Unsupported rate. Report error using sns_std_error_event.
				SNS_INST_PRINTF(ERROR, this, "sensor_temp ODR match error %d", rv);
				//return rv;
			}
		}
		if(sensor_type == SPL06_PRESSURE) {
			rv = spl06_validate_sensor_pressure_odr(this);
			if((rv != SNS_RC_SUCCESS) && (desired_sample_rate != 0)) {
				// TODO Unsupported rate. Report error using sns_std_error_event.
				SNS_INST_PRINTF(ERROR, this, "sensor_pressure ODR match error %d", rv);
				//return rv;
			}
		}
#if SPL_CONFIG_ENABLE_DAE
		if(SPL06_CONFIG_IDLE == state->config_step && spl06_dae_if_stop_streaming(this)) {
			state->config_step = SPL06_CONFIG_STOPPING_STREAM;
		}
#endif
		if(state->config_step == SPL06_CONFIG_IDLE) {
			spl06_reconfig_hw(this, sensor_type);
		}

		spl06_send_config_event(this);
	}
	else if(client_request->message_id ==
			SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG) {
		/** All self-tests can be handled in normal mode. */
#if SPL_CONFIG_ENABLE_ISLAND_MODE
		state->island_service->api->sensor_instance_island_exit(state->island_service, this);
#endif
		spl06_run_self_test(this);
		state->new_self_test_request = false;
	}

	// Turn COM port OFF
	state->scp_service->api->sns_scp_update_bus_power(
								state->com_port_info.port_handle, false);
	SPL_INST_LOG(LOW, this, "<sns_see_if__  set_client_config> exit");
	
	return SNS_RC_SUCCESS;
}
