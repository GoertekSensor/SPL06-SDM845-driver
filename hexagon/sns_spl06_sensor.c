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

#include <string.h>
#include "sns_attribute_util.h"
#include "sns_mem_util.h"
#include "sns_sensor_util.h"
#include "sns_service.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_types.h"
#include "sns_printf.h"

#include "sns_spl06_sensor.h"

#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_pb_util.h"
#include "sns_suid.pb.h"
#include "sns_registry.pb.h"

#define SPL06_CONFIG_PRESSURE         "spl06_0.pressure.config"
#define SPL06_CONFIG_TEMP             "spl06_0.temp.config"
#define SPL06_PLATFORM_CONFIG         "spl06_0_platform.config"
#define SPL06_PLATFORM_PLACEMENT      "spl06_0_platform.placement"

sns_rc spl06_device_sw_reset(
	sns_sync_com_port_service *scp_service,
	sns_sync_com_port_handle *port_handle,
	spl06_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t rega = SPL06_RESET_REG;
  uint32_t xfer_bytes;
  uint8_t regv[1];

  regv[0] = SPL06_SOFT_RESET_CODE;
  rv = spl06_com_write_wrapper(scp_service,port_handle, rega, &regv[0], 1, &xfer_bytes, false);
  if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
    return rv;
  }
  return SNS_RC_SUCCESS;
}

/**
 * see sns_spl06_hal.h
 */
sns_rc spl06_reset_device(
	sns_sync_com_port_service *scp_service,
	sns_sync_com_port_handle *port_handle,
	spl06_sensor_type sensor)
{
	sns_rc rv = SNS_RC_SUCCESS;

	/*HW reset */
	if (sensor == (SPL06_PRESSURE | SPL06_TEMPERATURE)) {
		rv = spl06_device_sw_reset(scp_service,port_handle, sensor);
	}
	return rv;
}

/**
 * see sns_spl06_hal.h
 */
sns_rc spl06_get_chip_type(
	sns_sync_com_port_service * scp_service,
	sns_sync_com_port_handle *port_handle,
	uint8_t *buffer)
{
	sns_rc rv = SNS_RC_SUCCESS;
	uint32_t xfer_bytes;

	rv = spl06_com_read_wrapper(scp_service,
	                    	port_handle,
	                      	SPL06_CHIP_ID_REG,
	                      	buffer,
	                      	1,
	                      	&xfer_bytes);
	if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
		rv = SNS_RC_FAILED;
	}
	return rv;
}

/*!
 *	@brief This API is used to
 *	calibration parameters used for calculation in the registers
 *
 *  parameter | Register address 
 *------------|-------------------------
 *	c0    |  0x10 and 0x11 
 *	c1    |  0x11 and 0x12  
 *	c00    |  0x13 and 0x14 and 0x15 
 *	c10    |  0x16 and 0x17  
 *	c01    |  0x18 and 0x19 
 *	c11    |  0x1A and 0x1B  
 *	c20    |  0x1C and 0x1D  
 *	c21    |  0x1E and 0x1F 
 *	c30    |  0x20 and 0x21  
 *
 *  @return sns_rc,
 *  SNS_RC_SUCCESS if encoding was succesful
 *  SNS_RC_FAILED otherwise
 *
*/
sns_rc spl06_get_calib_param(sns_sensor * const this)
{
	spl06_state *state = (spl06_state *)this->state->state;

	sns_rc rv = SNS_RC_SUCCESS;
	uint8_t a_data_u8r[3] = {0};
	uint32_t xfer_bytes;
	
	/* check the state structure pointer as NULL*/
	if (state == NULL) {
	  rv = SNS_RC_FAILED;
	} else {
	    /* read calibration values*/
		rv = spl06_com_read_wrapper(state->scp_service,
		                           state->common.com_port_info.port_handle,
		                           SPL06_CALIBRATION_DATA_START,
		                           &a_data_u8r[0],
		                           1,
		                           &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}
		
		rv = spl06_com_read_wrapper(state->scp_service,
		                           state->common.com_port_info.port_handle,
		                           SPL06_CALIBRATION_DATA_START+1,
		                           &a_data_u8r[1],
		                           1,
		                           &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}
		
		state->calib_param.c0 = ((int16_t)a_data_u8r[0]<<4) | (a_data_u8r[1]>>4);
		state->calib_param.c0 = (state->calib_param.c0&0x0800) ? (0xF000|state->calib_param.c0) : state->calib_param.c0;

		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+1,
								   &a_data_u8r[0],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}
		
		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+2,
								   &a_data_u8r[1],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}

		state->calib_param.c1 = ((int16_t)(a_data_u8r[0]&0x0F)<<8) | a_data_u8r[1];
		state->calib_param.c1 = (state->calib_param.c1&0x0800) ? (0xF000|state->calib_param.c1) : state->calib_param.c1;

		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+3,
								   &a_data_u8r[0],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}

		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+4,
								   &a_data_u8r[1],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}

		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+5,
								   &a_data_u8r[2],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}

		state->calib_param.c00 = ((int32_t)a_data_u8r[0]<<12) | ((int32_t)a_data_u8r[1]<<4) | ((int32_t)a_data_u8r[2]>>4);
		state->calib_param.c00 = (state->calib_param.c00&0x080000) ? (0xFFF00000|state->calib_param.c00) : state->calib_param.c00;

		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+5,
								   &a_data_u8r[0],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}
		
		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+6,
								   &a_data_u8r[1],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}
		
		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+7,
								   &a_data_u8r[2],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}

		state->calib_param.c10 = ((int32_t)(a_data_u8r[0]&0x0F)<<16) | ((int32_t)a_data_u8r[1]<<8) | a_data_u8r[2];
		state->calib_param.c10 = (state->calib_param.c10&0x080000) ? (0xFFF00000|state->calib_param.c10) : state->calib_param.c10;

		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+8,
								   &a_data_u8r[0],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}
		
		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+9,
								   &a_data_u8r[1],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}

		state->calib_param.c01 = ((int16_t)a_data_u8r[0]<<8) | a_data_u8r[1];

		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+10,
								   &a_data_u8r[0],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}
		
		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+11,
								   &a_data_u8r[1],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}

		state->calib_param.c11 = ((int16_t)a_data_u8r[0]<<8) | a_data_u8r[1];

		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+12,
								   &a_data_u8r[0],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}
		
		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+13,
								   &a_data_u8r[1],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}

		state->calib_param.c20 = ((int16_t)a_data_u8r[0]<<8) | a_data_u8r[1];

		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+14,
								   &a_data_u8r[0],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}
		
		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+15,
								   &a_data_u8r[1],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}

		state->calib_param.c21 = ((int16_t)a_data_u8r[0]<<8) | a_data_u8r[1];

		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+16,
								   &a_data_u8r[0],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}
		
		rv = spl06_com_read_wrapper(state->scp_service,
								   state->common.com_port_info.port_handle,
								   SPL06_CALIBRATION_DATA_START+17,
								   &a_data_u8r[1],
								   1,
								   &xfer_bytes);
		if ((rv != SNS_RC_SUCCESS) || (xfer_bytes != 1)) {
			return SNS_RC_FAILED;
		}

		state->calib_param.c30 = ((int16_t)a_data_u8r[0]<<8) | a_data_u8r[1];

	}

	SPL_SENSOR_LOG(LOW, this, "calib_data c0 = 0x%x, c1 = 0x%x, c00 = 0x%x",
	state->calib_param.c0, state->calib_param.c1, state->calib_param.c00);
	SPL_SENSOR_LOG(LOW, this, "calib_data c10 = 0x%x, c01 = 0x%x, c11 = 0x%x,",
	state->calib_param.c10, state->calib_param.c01, state->calib_param.c11);
	SPL_SENSOR_LOG(LOW, this, "calib_data c20= 0x%x, c21 = 0x%x, c30 = 0x%x,",
	state->calib_param.c20, state->calib_param.c21, state->calib_param.c30);

	return rv;
}

#if SPL_CONFIG_ENABLE_REGISTRY
static void spl06_sensor_process_registry_event(sns_sensor *const this,
                                           sns_sensor_event *event)
{
	bool rv = true;
	spl06_state *state = (spl06_state*)this->state->state;

	pb_istream_t stream = pb_istream_from_buffer((void*)event->event, event->event_len);
	SPL_SENSOR_LOG(LOW, this, ">>spl06_sensor_process_registry_event = %d<<", event->message_id);

	if(SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_EVENT == event->message_id) {
		sns_registry_read_event read_event = sns_registry_read_event_init_default;
		pb_buffer_arg group_name = {0,0};
		read_event.name.arg = &group_name;
		read_event.name.funcs.decode = pb_decode_string_cb;

		if(!pb_decode(&stream, sns_registry_read_event_fields, &read_event)) {
		  	SNS_PRINTF(ERROR, this, "Error decoding registry event");
		} else {
			stream = pb_istream_from_buffer((void*)event->event, event->event_len);

			if(0 == strncmp((char*)group_name.buf, SPL06_CONFIG_PRESSURE,
			              group_name.buf_len) ||
			 0 == strncmp((char*)group_name.buf, SPL06_CONFIG_TEMP,
			              group_name.buf_len)) {
			    {
					sns_registry_decode_arg arg = {
					.item_group_name = &group_name,
					.parse_info_len = 1,
					.parse_info[0] = {
					  	.group_name = "config",
					  	.parse_func = sns_registry_parse_phy_sensor_cfg,
					  	.parsed_buffer = &state->common.registry_cfg 
					  	}
					};

					read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
					read_event.data.items.arg = &arg;

					rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
			    }

			    if(rv)
			    {
					state->common.registry_cfg_received = true;
					state->is_dri = state->common.registry_cfg.is_dri;
					state->hardware_id = state->common.registry_cfg.hw_id;
					state->resolution_idx = state->common.registry_cfg.res_idx;
					state->supports_sync_stream = state->common.registry_cfg.sync_stream;

					SPL_SENSOR_LOG(LOW, this, "Registry read event for group registry_cfg received "
					         "is_dri:%d, hardware_id:%d resolution_idx:%d, supports_sync_stream:%d",
					         state->is_dri, state->hardware_id, state->resolution_idx,
					         state->supports_sync_stream);
		    	}
			}
		  	else if(0 == strncmp((char*)group_name.buf, SPL06_PLATFORM_CONFIG,
		                       			group_name.buf_len)) {
			    {
			      sns_registry_decode_arg arg = {
			        .item_group_name = &group_name,
			        .parse_info_len = 1,
			        .parse_info[0] = {
			          .group_name = "config",
			          .parse_func = sns_registry_parse_phy_sensor_pf_cfg,
			          .parsed_buffer = &state->common.registry_pf_cfg
			        }
			      };

			      read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
			      read_event.data.items.arg = &arg;

			      rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
			    }

		    	if(rv)
			    {
					state->common.registry_pf_cfg_received = true;

					state->common.com_port_info.com_config.bus_type = state->common.registry_pf_cfg.bus_type;
					state->common.com_port_info.com_config.bus_instance = state->common.registry_pf_cfg.bus_instance;
					state->common.com_port_info.com_config.slave_control = state->common.registry_pf_cfg.slave_config;
					state->common.com_port_info.com_config.min_bus_speed_KHz = state->common.registry_pf_cfg.min_bus_speed_khz;
					state->common.com_port_info.com_config.max_bus_speed_KHz = state->common.registry_pf_cfg.max_bus_speed_khz;
					state->common.com_port_info.com_config.reg_addr_type = state->common.registry_pf_cfg.reg_addr_type;
					state->common.irq_config.interrupt_num = state->common.registry_pf_cfg.dri_irq_num;
					state->common.irq_config.interrupt_pull_type = state->common.registry_pf_cfg.irq_pull_type;
					state->common.irq_config.is_chip_pin = state->common.registry_pf_cfg.irq_is_chip_pin;
					state->common.irq_config.interrupt_drive_strength = state->common.registry_pf_cfg.irq_drive_strength;
					state->common.irq_config.interrupt_trigger_type = state->common.registry_pf_cfg.irq_trigger_type;
					state->common.rail_config.num_of_rails = state->common.registry_pf_cfg.num_rail;
					state->common.registry_rail_on_state = state->common.registry_pf_cfg.rail_on_state;
					sns_strlcpy(state->common.rail_config.rails[0].name,
					          state->common.registry_pf_cfg.vddio_rail,
					          sizeof(state->common.rail_config.rails[0].name));
					sns_strlcpy(state->common.rail_config.rails[1].name,
					          state->common.registry_pf_cfg.vdd_rail,
					          sizeof(state->common.rail_config.rails[1].name));

					SPL_SENSOR_LOG(LOW, this, "Registry read event for group registry_pf_cfg received "
					         "bus_type:%d bus_instance:%d slave_control:%d "
					         "min_bus_speed_KHz :%d max_bus_speed_KHz:%d reg_addr_type:%d ",
					         state->common.com_port_info.com_config.bus_type,
					         state->common.com_port_info.com_config.bus_instance,
					         state->common.com_port_info.com_config.slave_control,
					         state->common.com_port_info.com_config.min_bus_speed_KHz,
					         state->common.com_port_info.com_config.max_bus_speed_KHz,
					         state->common.com_port_info.com_config.reg_addr_type);

					SPL_SENSOR_LOG(LOW, this, "interrupt_num:%d interrupt_pull_type:%d is_chip_pin:%d "
					         "interrupt_drive_strength:%d interrupt_trigger_type:%d rigid body type:%d",
					         state->common.irq_config.interrupt_num,
					         state->common.irq_config.interrupt_pull_type,
					         state->common.irq_config.is_chip_pin,
					         state->common.irq_config.interrupt_drive_strength,
					         state->common.irq_config.interrupt_trigger_type,
					         state->common.registry_pf_cfg.rigid_body_type);
			    }
			}
		  	else if(0 == strncmp((char*)group_name.buf, SPL06_PLATFORM_PLACEMENT,
		                       			group_name.buf_len)) {
		    {
				uint8_t arr_index = 0;
				pb_float_arr_arg arr_arg = {
				.arr = state->common.placement,
				.arr_index = &arr_index,
				.arr_len = 12
				};

				sns_registry_decode_arg arg = {
					.item_group_name = &group_name,
					.parse_info_len = 1,
					.parse_info[0] = {
					  .group_name = "placement",
					  .parse_func = sns_registry_parse_float_arr,
					  .parsed_buffer = &arr_arg
					}
				};

		      	read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
		      	read_event.data.items.arg = &arg;

		      	rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
		    }

			    if(rv) {
			      	state->common.registry_placement_received = true;
			      	SPL_SENSOR_LOG(LOW, this, "Registry read event for group registry_placement received "
			                 "p[0]:%u p[6]:%u p[11]:%u", (uint32_t)state->common.placement[0],
			                 (uint32_t)state->common.placement[6], (uint32_t)state->common.placement[11]);
			    }
			} else {
				SNS_PRINTF(ERROR, this, "no find the group_name");
				rv = false;
			}

			if(!rv) {
				SNS_PRINTF(ERROR, this, "Error decoding registry group");
			}
		}
	} else {
		SNS_PRINTF(ERROR, this, "Received unsupported registry event msg id %u",
		       		event->message_id);
	}
}
#endif

#if SPL_CONFIG_ENABLE_REGISTRY
static void spl06_sensor_send_registry_request(sns_sensor *const this,
                                                 char *reg_group_name)
{
	spl06_state *state = (spl06_state*)this->state->state;
	uint8_t buffer[100];
	int32_t encoded_len;
	sns_memset(buffer, 0, sizeof(buffer));
	sns_rc rc = SNS_RC_SUCCESS;

	sns_registry_read_req read_request;
	pb_buffer_arg data = (pb_buffer_arg){ 
		.buf = reg_group_name,
		.buf_len = (strlen(reg_group_name) + 1) 
		};

	read_request.name.arg = &data;
	read_request.name.funcs.encode = pb_encode_string_cb;

	encoded_len = pb_encode_request(buffer, sizeof(buffer),
	  &read_request, sns_registry_read_req_fields, NULL);
	if(0 < encoded_len)
	{
		sns_request request = (sns_request){
	  		.request_len = encoded_len, .request = buffer,
	  		.message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_REQ 
	  		};
		rc = state->reg_data_stream->api->send_request(state->reg_data_stream, &request);
	}
}

void spl06_request_registry(sns_sensor *const this)
{
	spl06_state *state = (spl06_state*)this->state->state;
	sns_service_manager *service_mgr = this->cb->get_service_manager(this);
	sns_stream_service *stream_svc = (sns_stream_service*)
	          service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

	// place a request to registry sensor

	if((state->reg_data_stream == NULL) && (state->common.chip_id == 0))
	{
		sns_sensor_uid suid;

		sns_suid_lookup_get(&state->common.suid_lookup_data, "registry", &suid);
		stream_svc->api->create_sensor_stream(stream_svc, this, suid, &state->reg_data_stream);

		if(SPL06_PRESSURE == state->sensor) {
			spl06_sensor_send_registry_request(this, SPL06_PLATFORM_CONFIG);
			spl06_sensor_send_registry_request(this, SPL06_PLATFORM_PLACEMENT);
			spl06_sensor_send_registry_request(this, SPL06_CONFIG_PRESSURE);
		}
		else if(SPL06_TEMPERATURE == state->sensor) {
			spl06_sensor_send_registry_request(this, SPL06_CONFIG_TEMP);
		}
	}
}
#endif

bool spl06_discover_hw(sns_sensor *const this)
{
	uint8_t buffer[1] = {0};
	bool hw_is_present = false;
	spl06_state *state = (spl06_state*)this->state->state;
	sns_rc rv = SNS_RC_SUCCESS;

	/**-------------------Read and Confirm WHO-AM-I------------------------*/
	rv = spl06_get_chip_type(state->scp_service,
							state->common.com_port_info.port_handle,
	                        &buffer[0]);
	SPL_SENSOR_LOG(LOW, this, "spl06 chip type is 0x%x", buffer[0]);
	if((rv == SNS_RC_SUCCESS) && (buffer[0] == SPL06_CHIP_ID)) {
		// Reset Sensor only if an inatance is not alreadly running
		if(NULL == sns_sensor_util_get_shared_instance(this)) {
			spl06_get_calib_param(this);
			/* Reset Sensor */
			//rv += spl06_reset_device(state->scp_service,state->com_port_info.port_handle,
			//         SPL06_PRESSURE | SPL06_TEMPERATURE);
		}
		if(rv == SNS_RC_SUCCESS) {
		   hw_is_present = true;
		}
	}

  	state->common.chip_id = buffer[0];
	/**------------------Power Down and Close COM Port--------------------*/
	state->scp_service->api->sns_scp_update_bus_power(state->common.com_port_info.port_handle, false);
	state->scp_service->api->sns_scp_close(state->common.com_port_info.port_handle);
	state->scp_service->api->sns_scp_deregister_com_port(&state->common.com_port_info.port_handle);
#if SPL_CONFIG_POWER_RAIL
	/**----------------------Turn Power Rail OFF--------------------------*/
	state->common.rail_config.rail_vote = SNS_RAIL_OFF;
	state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
	                                                       this,
	                                                       &state->common.rail_config,
	                                                       NULL);
#endif

	return hw_is_present;
}

void spl06_publish_available(sns_sensor *const this)
{
	sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
	value.has_boolean = true;
	value.boolean = true;
	sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
}

#if SPL_CONFIG_POWER_RAIL
static sns_rc spl06_register_power_rail(sns_sensor *const this)
{
	spl06_state *state = (spl06_state*)this->state->state;
	sns_service_manager *smgr = this->cb->get_service_manager(this);
	sns_rc rv = SNS_RC_SUCCESS;

	state->common.rail_config.rail_vote = SNS_RAIL_OFF;

	if(NULL == state->pwr_rail_service) {
		state->pwr_rail_service =
					(sns_pwr_rail_service*)smgr->get_service(smgr, SNS_POWER_RAIL_SERVICE);

		state->pwr_rail_service->api->sns_register_power_rails(state->pwr_rail_service,
		                                                   &state->common.rail_config);
	}

	if(NULL == state->pwr_rail_service) {
		rv = SNS_RC_FAILED;
	}
	return rv;
}
#endif

/**
 * Publish attributes read from registry
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
static void spl06_publish_registry_attributes(sns_sensor *const this)
{
#if !SPL_CONFIG_ENABLE_SEE_LITE
	spl06_state *state = (spl06_state*)this->state->state;
	{
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.has_boolean = true;
		value.boolean = state->is_dri;
		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
	}
	{
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.has_boolean = true;
		value.boolean = state->supports_sync_stream;
		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
	}
	{
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.has_sint = true;
		value.sint = state->hardware_id;
		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
	}
	{
		sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
	  		SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
		for(uint8_t i = 0; i < 12; i++) {
			values[i].has_flt = true;
			values[i].flt = state->common.placement[i];
		}
		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PLACEMENT,
	    					values, ARR_SIZE(values), false);
	}
	{
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.has_sint = true;
		value.sint = state->common.registry_pf_cfg.rigid_body_type;
		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
	}
#else
	UNUSED_VAR(this);
#endif
}

void spl06_update_sibling_sensors(sns_sensor *const this)
{
	sns_sensor *sensor = NULL;
	spl06_state *state;
	spl06_state *acc_state = (spl06_state*)this->state->state;

	for(sensor = this->cb->get_library_sensor(this, true);
	   sensor != NULL;
	   sensor = this->cb->get_library_sensor(this, false)) {

		state = (spl06_state*)sensor->state->state;
		if(state->sensor != SPL06_PRESSURE) {
			sns_memscpy(&state->common, sizeof(state->common),
			          &acc_state->common, sizeof(acc_state->common));
#if SPL_CONFIG_POWER_RAIL
			spl06_register_power_rail(sensor);
#endif
			spl06_publish_available(sensor);
		}
		/** Moving registry based attribute publish here. */
		spl06_publish_registry_attributes(sensor);
#if SPL_CONFIG_ENABLE_REGISTRY
		/** More clean up. */
		sns_sensor_util_remove_sensor_stream(sensor, &state->reg_data_stream);
#endif
	}
}

#if SPL_CONFIG_POWER_RAIL
static void spl06_start_power_rail_timer(sns_sensor * const this,
    sns_time timeout_ticks,
    spl06_power_rail_pending_state pwr_rail_pend_state)
{
	spl06_state *state = (spl06_state*)this->state->state;

	sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
	size_t req_len;
	uint8_t buffer[20];
	SPL_SENSOR_LOG(LOW, this, "start power rail timer");
	sns_memset(buffer, 0, sizeof(buffer));
	req_payload.is_periodic = false;
	req_payload.start_time = sns_get_system_time();
	req_payload.timeout_period = timeout_ticks;

	if(NULL == state->timer_stream) {
		sns_service_manager *smgr = this->cb->get_service_manager(this);
		sns_stream_service *stream_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);
		sns_sensor_uid suid;

		sns_suid_lookup_get(&state->common.suid_lookup_data, "timer", &suid);
		stream_svc->api->create_sensor_stream(stream_svc, this, suid, &state->timer_stream);
	}

	req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
	                          sns_timer_sensor_config_fields, NULL);
	if(req_len > 0 && NULL != state->timer_stream) {
		sns_request timer_req = {  
			.message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
	     	.request = buffer, 
	     	.request_len = req_len  
	     	};
		state->timer_stream->api->send_request(state->timer_stream, &timer_req);
		state->power_rail_pend_state = pwr_rail_pend_state;
	} else {
		SNS_PRINTF(ERROR, this, "LSM timer req encode error");
	}
	SPL_SENSOR_LOG(LOW, this, "start power rail timer request sent");
}
#endif

void spl06_start_hw_detect_sequence(sns_sensor *const this)
{
	spl06_state *state = (spl06_state*)this->state->state;
	sns_rc rv = SNS_RC_SUCCESS;

	state->common.registry_pf_cfg_received = false;

	/**-----------------Register and Open COM Port-------------------------*/
	if(NULL == state->common.com_port_info.port_handle) {
		rv = state->scp_service->api->sns_scp_register_com_port(
				&state->common.com_port_info.com_config,
				&state->common.com_port_info.port_handle);

		if(rv == SNS_RC_SUCCESS) {
			rv = state->scp_service->api->sns_scp_open(state->common.com_port_info.port_handle);
		}
	}
#if SPL_CONFIG_POWER_RAIL
  /**---------------------Register Power Rails --------------------------*/
	if(sns_suid_lookup_get(&state->common.suid_lookup_data, "timer", NULL)
		&& NULL == state->pwr_rail_service
		&& rv == SNS_RC_SUCCESS)
	{
		rv = spl06_register_power_rail(this);

		/**---------------------Turn Power Rails ON----------------------------*/
		state->common.rail_config.rail_vote = state->common.registry_rail_on_state;

		if(rv == SNS_RC_SUCCESS) {
		  rv = state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
		                                                           this,
		                                                           &state->common.rail_config,
		                                                           NULL);
		}

		/**-------------Create a Timer stream for Power Rail ON timeout.---------*/
		if(rv == SNS_RC_SUCCESS) {
			spl06_start_power_rail_timer(this,
										sns_convert_ns_to_ticks(SPL_OFF_TO_IDLE_MS * 1000 * 1000),
										SPL_POWER_RAIL_PENDING_INIT);
		}
	}
#else

	state->common.hw_is_present = spl06_discover_hw(this);

	if (state->common.hw_is_present) {
		SPL_SENSOR_LOG(MED, this, "sensor:%d init finished in disable pwr_rail_service", state->sensor);
		spl06_publish_available(this);
		spl06_update_sibling_sensors(this);
	} else {
		rv = SNS_RC_INVALID_LIBRARY_STATE;
		SNS_PRINTF(ERROR, this, "SPL06 HW absent");
	}
#endif
}

void spl06_common_init(sns_sensor *const this)
{
	spl06_state *state = (spl06_state*)this->state->state;

	struct sns_service_manager *smgr= this->cb->get_service_manager(this);
	state->diag_service = (sns_diag_service *)
	smgr->get_service(smgr, SNS_DIAG_SERVICE);
	state->scp_service =
		(sns_sync_com_port_service *)smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);
#if SPL_CONFIG_ENABLE_ISLAND_MODE
	state->island_service =
		(sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
#endif
	state->sensor_client_present = false;
	SNS_SUID_LOOKUP_INIT(state->common.suid_lookup_data, NULL);
	/** Accel sensor fetches all common dependent sensor SUIDs. */
	if(state->sensor == SPL06_PRESSURE) {
#if SPL_CONFIG_ENABLE_DAE
		sns_suid_lookup_add(this, &state->common.suid_lookup_data, "data_acquisition_engine");
#endif
		sns_suid_lookup_add(this, &state->common.suid_lookup_data, "async_com_port");
		sns_suid_lookup_add(this, &state->common.suid_lookup_data, "timer");
	}
#if SPL_CONFIG_ENABLE_REGISTRY
	sns_suid_lookup_add(this, &state->common.suid_lookup_data, "registry");
#endif
}

/**
 * Sets instance config to run a self test.
 *
 * @param[i] this      Sensor reference
 * @param[i] instance  Sensor Instance reference
 *
 * @return none
 */
void spl06_set_self_test_inst_config(sns_sensor *this,
                              sns_sensor_instance *instance)
{
	sns_request config;
	SPL_SENSOR_LOG(LOW, this, "spl06_set_self_test_inst_config");
	config.message_id = SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG;
	config.request_len = 0;
	config.request = NULL;

	this->instance_api->set_client_config(instance, &config);
}

/** See sns_spl06_sensor.h*/
sns_rc spl06_sensor_notify_event(sns_sensor * const this)
{
	spl06_state *state = (spl06_state*) this->state->state;
	sns_rc rv = SNS_RC_SUCCESS;
	sns_sensor_event *event;
	SPL_SENSOR_LOG(LOW, this, "<sns_see_if__ notify_event> from sensor:%d ", state->sensor);
	if(!sns_suid_lookup_complete(&state->common.suid_lookup_data))
	{
#if SPL_CONFIG_ENABLE_ISLAND_MODE
		state->island_service->api->sensor_island_exit(state->island_service, this);
#endif
		sns_suid_lookup_handle(this, &state->common.suid_lookup_data);

#if SPL_CONFIG_ENABLE_REGISTRY
		if(sns_suid_lookup_get(&state->common.suid_lookup_data, "registry", NULL)) {
			spl06_request_registry(this);
		}

#else
		sns_spl_registry_def_config(this);
#endif

		if(sns_suid_lookup_complete(&state->common.suid_lookup_data)) {
			sns_suid_lookup_deinit(this, &state->common.suid_lookup_data);
		}
	}

	/**----------------------Handle a Timer Sensor event.-------------------*/
	if(NULL != state->timer_stream) {
		event = state->timer_stream->api->peek_input(state->timer_stream);
		while(NULL != event) {
			pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
			                                           event->event_len);
			sns_timer_sensor_event timer_event;
			SPL_SENSOR_LOG(LOW, this, ">>timer event<< message_id = %d", event->message_id);
			if(SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id) {
				if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
					if(state->power_rail_pend_state == SPL_POWER_RAIL_PENDING_INIT) {
						/** Initial HW discovery is OK to run in normal mode. */
#if SPL_CONFIG_ENABLE_ISLAND_MODE
						state->island_service->api->sensor_island_exit(state->island_service, this);
#endif
						state->common.hw_is_present = spl06_discover_hw(this);

						if(state->common.hw_is_present) {
							spl06_publish_available(this);
							spl06_update_sibling_sensors(this);
							SPL_SENSOR_LOG(LOW, this, "sensor:%d initialize finished", state->sensor);
						} else {
							rv = SNS_RC_INVALID_LIBRARY_STATE;
							SNS_PRINTF(ERROR, this, "SPL06 HW absent");
						}
						state->power_rail_pend_state = SPL_POWER_RAIL_PENDING_NONE;
	      			}
	      			else if(state->power_rail_pend_state == SPL_POWER_RAIL_PENDING_SET_CLIENT_REQ) {
						sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
						if(NULL != instance) {
							spl06_instance_state *inst_state =
							 (spl06_instance_state *) instance->state->state;
							inst_state->instance_is_ready_to_configure = true;
							spl06_reval_instance_config(this, instance, state->sensor);
							if(inst_state->new_self_test_request) {
								spl06_set_self_test_inst_config(this, instance);
							}
						}
						state->power_rail_pend_state = SPL_POWER_RAIL_PENDING_NONE;
					}
	    		} else {
	      			SNS_PRINTF(ERROR, this, "pb_decode error");
	    		}
	  		}
	  		event = state->timer_stream->api->get_next_input(state->timer_stream);
		}
		/** Free up timer stream if not needed anymore */
		if(state->power_rail_pend_state == SPL_POWER_RAIL_PENDING_NONE) {
			sns_sensor_util_remove_sensor_stream(this, &state->timer_stream);
		}
	}
#if SPL_CONFIG_ENABLE_REGISTRY
	if(NULL != state->reg_data_stream)
	{
		SPL_SENSOR_LOG(LOW, this, ">>reg_data_stream<<");
		event = state->reg_data_stream->api->peek_input(state->reg_data_stream);
		while(NULL != event) {
			/** All registry events can be handled in normal mode. */
#if SPL_CONFIG_ENABLE_ISLAND_MODE
			state->island_service->api->sensor_island_exit(state->island_service, this);
#endif
			spl06_sensor_process_registry_event(this, event);

			event = state->reg_data_stream->api->get_next_input(state->reg_data_stream);
		}
	}
#endif
	if(sns_suid_lookup_get(&state->common.suid_lookup_data, "timer", NULL) &&
	 	state->common.registry_pf_cfg_received && state->common.registry_cfg_received &&
	 	state->common.registry_placement_received) {
		/** Initial HW detection sequence is OK to run in normal mode. */
#if SPL_CONFIG_ENABLE_ISLAND_MODE
		state->island_service->api->sensor_island_exit(state->island_service, this);
#endif
		spl06_start_hw_detect_sequence(this);
	}
	return rv;
}

/**
 * Decodes self test requests.
 *
 * @param[i] this              Sensor reference
 * @param[i] request           Encoded input request
 * @param[o] decoded_request   Decoded standard request
 * @param[o] test_config       decoded self test request
 *
 * @return bool True if decoding is successfull else false.
 */
static bool spl06_get_decoded_self_test_request(sns_sensor const *this, sns_request const *request,
                                                  sns_std_request *decoded_request,
                                                  sns_physical_sensor_test_config *test_config)
{
	pb_istream_t stream;
	pb_simple_cb_arg arg = { 
		.decoded_struct = test_config,
	    .fields = sns_physical_sensor_test_config_fields 
	    };
	decoded_request->payload = (struct pb_callback_s){ 
			.funcs.decode = &pb_decode_simple_cb, 
			.arg = &arg 
			};
	stream = pb_istream_from_buffer(request->request, request->request_len);
	if(!pb_decode(&stream, sns_std_request_fields, decoded_request)) {
		SNS_PRINTF(ERROR, this, "LSM decode error");
		return false;
	}
	return true;
}

static void spl06_set_sensor_inst_config(sns_sensor *this,
    sns_sensor_instance *instance, float chosen_report_rate,
    float chosen_sample_rate,
    spl06_sensor_type  sensor_type)
{
	UNUSED_VAR(instance);
	sns_spl06_cfg_req new_client_config;
	sns_request config;

	new_client_config.report_rate = chosen_report_rate;
	new_client_config.sample_rate = chosen_sample_rate;
	new_client_config.sensor_type = sensor_type;
	new_client_config.power_mode = SPL06_NORMAL_MODE;
	SPL_SENSOR_LOG(LOW, this, "sensor type:%d, sample rate %d",
	sensor_type, (uint8_t)chosen_sample_rate);

	config.message_id  = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
	config.request_len = sizeof(sns_spl06_cfg_req);
	config.request     = &new_client_config;
	this->instance_api->set_client_config(instance, &config);
}

#if SPL_CONFIG_POWER_RAIL
/**
 * Turns power rails off
 *
 * @paramp[i] this   Sensor reference
 *
 * @return none
 */
static void spl06_turn_rails_off(sns_sensor *this)
{
	sns_sensor *sensor;

	for(sensor = this->cb->get_library_sensor(this, true);
		NULL != sensor;
		sensor = this->cb->get_library_sensor(this, false))
	{
		spl06_state *sensor_state = (spl06_state*)sensor->state->state;
		if(sensor_state->common.rail_config.rail_vote != SNS_RAIL_OFF) {
			sensor_state->common.rail_config.rail_vote = SNS_RAIL_OFF;
			sensor_state->pwr_rail_service->api->sns_vote_power_rail_update(sensor_state->pwr_rail_service,
			                                                              sensor,
			                                                              &sensor_state->common.rail_config,
			                                                              NULL);
		}
	}
}
#endif

static bool spl06_get_decoded_sensor_request(
  sns_sensor const *this,
  sns_request const *in_request,
  sns_std_request *decoded_request,
  sns_std_sensor_config *decoded_payload)
{
	pb_istream_t stream;
	/* decode argument */
	pb_simple_cb_arg arg = {
	  .decoded_struct = decoded_payload,
	  .fields = sns_std_sensor_config_event_fields
	};
	/* decode functions.decode */
	decoded_request->payload = (struct pb_callback_s ) {
	  .funcs.decode = &pb_decode_simple_cb,
	  .arg = &arg
	};
	stream = pb_istream_from_buffer(in_request->request, in_request->request_len);
	if (!pb_decode(&stream, sns_std_request_fields, decoded_request)) {
	   SNS_PRINTF(ERROR, this, "LSM decode error");
	   return false;
	}
	return true;
}

/**
 * Decodes a physical sensor self test request and updates
 * instance state with request info.
 *
 * @param[i] this      Sensor reference
 * @param[i] instance  Sensor Instance reference
 * @param[i] new_request Encoded request
 *
 * @return True if request is valid else false
 */
static bool spl06_extract_self_test_info(sns_sensor *this,
                              sns_sensor_instance *instance,
                              struct sns_request const *new_request)
{
	sns_std_request decoded_request;
	sns_physical_sensor_test_config test_config = sns_physical_sensor_test_config_init_default;
	spl06_state *state = (spl06_state*)this->state->state;
	spl06_instance_state *inst_state = (spl06_instance_state*)instance->state->state;
	spl06_self_test_info *self_test_info;

	if(state->sensor == SPL06_PRESSURE) {
		self_test_info = &inst_state->pressure_info.test_info;
	}
	else if(state->sensor == SPL06_TEMPERATURE) {
		self_test_info = &inst_state->temperature_info.test_info;
	} else {
		return false;
	}

	if(spl06_get_decoded_self_test_request(this, new_request, &decoded_request, &test_config)) {
		self_test_info->test_type = test_config.test_type;
		self_test_info->test_client_present = true;
		return true;
	} else {
		return false;
	}
}

/* get the configuration item for tempture sensor */
static void spl06_get_sensor_tempture_config(sns_sensor *this,
	sns_sensor_instance *instance,
	spl06_sensor_type sensor_type,
	float *chosen_sample_rate,
	float *chosen_report_rate,
	bool *sensor_tempture_client_present)
{
	UNUSED_VAR(this);
	UNUSED_VAR(sensor_type);
	spl06_instance_state *inst_state = (spl06_instance_state*)instance->state->state;
	sns_sensor_uid suid = TEMPERATURE_SUID;
	sns_request const *request;

	*chosen_report_rate = 0;
	*chosen_sample_rate = 0;
	*sensor_tempture_client_present = false;

	/** Parse through existing requests and get fastest sample
	*    rate and report rate requests. */
	for(request = instance->cb->get_client_request(instance, &suid, true);
		NULL != request;
		request = instance->cb->get_client_request(instance, &suid, false)) {
		sns_std_request decoded_request;
		sns_std_sensor_config decoded_payload = {0};
		SPL_SENSOR_LOG(LOW, this, "temperature message id %d", request->message_id);
		if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
			if(spl06_get_decoded_sensor_request(this, request, &decoded_request, &decoded_payload)) {
				float report_rate;
				*chosen_sample_rate = SNS_MAX(*chosen_sample_rate, decoded_payload.sample_rate);
				if(decoded_request.has_batching && (decoded_request.batching.batch_period > 0)) {
					report_rate = (1000000.0 / (float)decoded_request.batching.batch_period);
				} else {
					report_rate = *chosen_sample_rate;
				}
				*chosen_report_rate = SNS_MAX(*chosen_report_rate, report_rate);
				*sensor_tempture_client_present = true;
			}
		}
	}	
	inst_state->temperature_info.report_timer_hz  = *chosen_report_rate;
	inst_state->temperature_info.sampling_rate_hz= *chosen_sample_rate;
	SPL_SENSOR_LOG(LOW, this, "temperature sample rete %d temperature present %d",
				(uint8_t)inst_state->temperature_info.sampling_rate_hz,
				*sensor_tempture_client_present);
}

static void spl06_get_sensor_pressure_config(
	sns_sensor *this,
	sns_sensor_instance *instance,
	spl06_sensor_type sensor_type,
	float *chosen_sample_rate,
	float *chosen_report_rate,
	bool *sensor_pressure_client_present)
{
	UNUSED_VAR(this);
	UNUSED_VAR(sensor_type);
	spl06_instance_state *inst_state =
	(spl06_instance_state*)instance->state->state;
	sns_sensor_uid suid = PRESSURE_SUID;
	sns_request const *request;

	*chosen_report_rate = 0;
	*chosen_sample_rate = 0;
	*sensor_pressure_client_present = false;

	/** Parse through existing requests and get fastest sample
	 *    rate and report rate requests. */
	for(request = instance->cb->get_client_request(instance, &suid, true);
		NULL != request;
		request = instance->cb->get_client_request(instance, &suid, false)) {
		sns_std_request decoded_request;
		sns_std_sensor_config decoded_payload = {0};

		if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
			if(spl06_get_decoded_sensor_request(this, request, &decoded_request, &decoded_payload)) {
				float report_rate;
				*chosen_sample_rate = SNS_MAX(*chosen_sample_rate, decoded_payload.sample_rate);
				if(decoded_request.has_batching && (decoded_request.batching.batch_period > 0)) {
					report_rate = (1000000.0 / (float)decoded_request.batching.batch_period);
				} else {
					report_rate = *chosen_sample_rate;
				}
				*chosen_report_rate = SNS_MAX(*chosen_report_rate, report_rate);
				*sensor_pressure_client_present = true;
			}
		}
	}
	inst_state->pressure_info.report_timer_hz  = *chosen_report_rate;
	inst_state->pressure_info.sampling_rate_hz= *chosen_sample_rate;
	SPL_SENSOR_LOG(LOW, this, "pressure sample rete %d pressure present %d",
				(uint8_t)inst_state->pressure_info.sampling_rate_hz,
				*sensor_pressure_client_present);
}

static void  spl06_mark_sensor_enable_state (
	sns_sensor_instance *this,
	spl06_sensor_type sensor_type,
	bool enable)
{
	spl06_instance_state *inst_state = (spl06_instance_state *) this->state->state;
	/* mark the corresponding sensor as fifo info field *now only the sw fifo* */
	switch (sensor_type) {
		case SPL06_PRESSURE:
			if(enable) {
				inst_state->deploy_info.publish_sensors |= SPL06_PRESSURE;
				inst_state->deploy_info.enable |= SPL06_PRESSURE;
			} else {
				inst_state->deploy_info.publish_sensors &= ~SPL06_PRESSURE;
				inst_state->deploy_info.enable &= ~SPL06_PRESSURE;
			}
			break;
		case SPL06_TEMPERATURE:
			if(enable) {
				inst_state->deploy_info.publish_sensors |= SPL06_TEMPERATURE;
				inst_state->deploy_info.enable |= SPL06_TEMPERATURE;
			} else {
				inst_state->deploy_info.publish_sensors &= ~SPL06_TEMPERATURE;
				inst_state->deploy_info.enable &= ~SPL06_TEMPERATURE;
			}
			break;
		default:
			break;
  }
}

/**
 * re-evaluate the instance configuration
 */
void spl06_reval_instance_config(sns_sensor *this,
	sns_sensor_instance *instance,
	spl06_sensor_type sensor_type)
{
	spl06_instance_state *inst_state = (spl06_instance_state*)instance->state->state;
	spl06_state *sensor_state = (spl06_state*)this->state->state;
	/**
	* 1. Get best pressure Config.
	* 2. Get best temperature Config.
	* 5. Decide best Instance Config based on above outputs.
	*/
	float p_sample_rate = 0;
	float p_report_rate = 0;
	float t_sample_rate = 0;
	float t_report_rate = 0;

	float chosen_sample_rate;
	float chosen_report_rate;

	bool p_sensor_client_present = false;
	bool t_sensor_client_present = false;

	SPL_SENSOR_LOG(LOW, this, "sensor type: %d %d", sensor_state->sensor, sensor_type);

	if(sensor_type == SPL06_PRESSURE) {
		spl06_get_sensor_pressure_config(this, instance, sensor_state->sensor,
		&p_sample_rate, &p_report_rate, &p_sensor_client_present);
		chosen_sample_rate = p_sample_rate;
		chosen_report_rate = p_report_rate;
	}
	
	if(sensor_type == SPL06_TEMPERATURE) {
		spl06_get_sensor_tempture_config(this, instance, sensor_state->sensor,
		&t_sample_rate, &t_report_rate, &t_sensor_client_present);
		chosen_sample_rate = t_sample_rate;
		chosen_report_rate = t_report_rate;
	}

	if ((sensor_type == SPL06_PRESSURE) || (sensor_type == SPL06_TEMPERATURE)) {
		if (sensor_type == SPL06_PRESSURE)
			spl06_mark_sensor_enable_state(instance, SPL06_PRESSURE, p_sensor_client_present);
		if (sensor_type == SPL06_TEMPERATURE)
			spl06_mark_sensor_enable_state(instance, SPL06_TEMPERATURE, t_sensor_client_present);
		SPL_SENSOR_LOG(LOW, this,
					"sensor type:%d, enable sensor flag:0x%x publish sensor flag:0x%x",
					sensor_type, inst_state->deploy_info.enable,
					inst_state->deploy_info.publish_sensors);
		/* set the sensor instance configuration*/
		spl06_set_sensor_inst_config(this, instance, chosen_report_rate, chosen_sample_rate, sensor_type);
	}
	if (!inst_state->deploy_info.enable) {
		sensor_state->common.rail_config.rail_vote = SNS_RAIL_OFF;
#if SPL_CONFIG_POWER_RAIL
		sensor_state->pwr_rail_service->api->sns_vote_power_rail_update(
		sensor_state->pwr_rail_service, this, &sensor_state->common.rail_config, NULL);
#endif
	}
}

/** See sns_spl06_sensor.h */
sns_sensor_instance* spl06_sensor_set_client_request(
	sns_sensor *const this,
	struct sns_request const *exist_request,
	struct sns_request const *new_request,
	bool remove)
{
	sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
	spl06_state *state = (spl06_state*)this->state->state;
	sns_time on_timestamp;
	sns_time delta;
	bool reval_config = false;

	SPL_SENSOR_LOG(LOW, this, "<sns_see_if__ set_client_request> for %d", state->sensor);

	if(remove) {
		if(NULL != instance) {
			SPL_SENSOR_LOG(LOW, this, "<sns_see_if__ removing request", exist_request->message_id);
			instance->cb->remove_client_request(instance, exist_request);
			spl06_reval_instance_config(this, instance, state->sensor);
		}
	} else {
		// 1. If new request then:
		//     a. Power ON rails.
		//     b. Power ON COM port - Instance must handle COM port power.
		//     c. Create new instance.
		//     d. Re-evaluate existing requests and choose appropriate instance config.
		//     e. set_client_config for this instance.
		//     f. Add new_request to list of requests handled by the Instance.
		//     g. Power OFF COM port if not needed- Instance must handle COM port power.
		//     h. Return the Instance.
		// 2. If there is an Instance already present:
		//     a. Add new_request to list of requests handled by the Instance.
		//     b. Remove exist_request from list of requests handled by the Instance.
		//     c. Re-evaluate existing requests and choose appropriate Instance config.
		//     d. set_client_config for the Instance if not the same as current config.
		//     e. publish the updated config.
		//     f. Return the Instance.
		// 3.  If "flush" request:
		//     a. Perform flush on the instance.
		//     b. Return NULL.
		if(NULL == instance) {
#if SPL_CONFIG_POWER_RAIL
			if(state->sensor == SPL06_PRESSURE || state->sensor == SPL06_TEMPERATURE) {
				state->common.rail_config.rail_vote = SNS_RAIL_ON_NPM;
			} else {
				state->common.rail_config.rail_vote = SNS_RAIL_OFF;
			}
			state->pwr_rail_service->api->sns_vote_power_rail_update(
			                                  state->pwr_rail_service,
			                                  this,
			                                  &state->common.rail_config,
			                                  &on_timestamp);

			delta = sns_get_system_time() - on_timestamp;

			// Use on_timestamp to determine correct Timer value.
			if(delta < sns_convert_ns_to_ticks(SPL_OFF_TO_IDLE_MS * 1000 * 1000)) {
				spl06_start_power_rail_timer(this, sns_convert_ns_to_ticks(
				        SPL_OFF_TO_IDLE_MS * 1000 * 1000) - delta,
				        SPL_POWER_RAIL_PENDING_SET_CLIENT_REQ);
			} else {
				// rail is already ON
				reval_config = true;
			}
#else
			UNUSED_VAR(on_timestamp);
			UNUSED_VAR(delta);
			reval_config = true;
#endif
			/** create_instance() calls init() for the Sensor Instance */
			SPL_SENSOR_LOG(LOW, this, "instance is NULL, now create a sigaltone instance");
			instance = this->cb->create_instance(this,
			                                  sizeof(spl06_instance_state));
			SPL_SENSOR_LOG(LOW, this, "instance is %p",instance);
			/* If rail is already ON then flag instance OK to configure */
			if(reval_config) {
				spl06_instance_state *inst_state = (spl06_instance_state *)instance->state->state;

				inst_state->instance_is_ready_to_configure = true;
				SPL_SENSOR_LOG(LOW, this, "reval configure with instance is NULL case ???");
			}
		} else {
			if(NULL != exist_request
				&& NULL != new_request
				&& new_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
	    		sns_sensor_util_send_flush_event(&state->my_suid, instance);
	    		return instance;
	  		} else {
				reval_config = true;
				/** An existing client is changing request*/
				if((NULL != exist_request) && (NULL != new_request)) {
				  instance->cb->remove_client_request(instance, exist_request);
				}
				/** A new client sent new_request*/
				else if(NULL != new_request) {
				  // No-op. new_request will be added to requests list below.
				}
			}
		}

		/** Add the new request to list of client_requests.*/
		if(NULL != instance) {
			spl06_instance_state *inst_state = (spl06_instance_state*)instance->state->state;
			if(new_request != NULL) {
				SPL_SENSOR_LOG(LOW, this, "new request from %d message id:%d add to the client request list",
				  				state->sensor,
				 	 			new_request->message_id);
				SPL_SENSOR_LOG(LOW, this, "instance2 is %p reval_config %d",instance, reval_config);
				SPL_SENSOR_LOG(LOW, this, "instance_is_ready_to_configure %d",inst_state->instance_is_ready_to_configure);
				instance->cb->add_client_request(instance, new_request);
				if(new_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG) {
					if(spl06_extract_self_test_info(this, instance, new_request)) {
						inst_state->new_self_test_request = true;
					}
				}
			}
			if(reval_config && inst_state->instance_is_ready_to_configure) {
				SPL_SENSOR_LOG(LOW, this, "try to configure");
				spl06_reval_instance_config(this, instance, state->sensor);
				if(inst_state->new_self_test_request) {
		  			spl06_set_self_test_inst_config(this, instance);
				}
			}
		}
	}
	if(NULL != instance &&
		NULL == instance->cb->get_client_request(instance,
		&(sns_sensor_uid)TEMPERATURE_SUID, true) &&
		NULL == instance->cb->get_client_request(instance,
		&(sns_sensor_uid)PRESSURE_SUID, true)) {
		this->cb->remove_instance(instance);
#if SPL_CONFIG_POWER_RAIL
		spl06_turn_rails_off(this);
#endif
	}
	return instance;
}
