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

#include "sns_island_service.h"
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
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_diag.pb.h"
#include "sns_sync_com_port_service.h"
#include "sns_printf.h"

static sns_rc spl06_inst_notify_event(sns_sensor_instance * const this)
{
    spl06_instance_state *state = (spl06_instance_state*) this->state->state;
    sns_sensor_event *event;
#if SPL_CONFIG_ENABLE_DAE
    spl06_dae_if_process_events(this);
#endif
    /* Turn COM port ON */
    state->scp_service->api->sns_scp_update_bus_power(
            state->com_port_info.port_handle, true);

    /* Handle Async Com Port events */
    if(NULL != state->async_com_port_data_stream) {
        event = state->async_com_port_data_stream->api->peek_input(state->async_com_port_data_stream);
        while(NULL != event) {
            if(event->message_id == SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_ERROR) {
                //TODO: Warning;
                SNS_INST_PRINTF(ERROR, this, "Received ASCP error event id=%d",
                                              event->message_id);
            }
            else if(event->message_id == SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW) {
            }
            event = state->async_com_port_data_stream->api->get_next_input(state->async_com_port_data_stream);
        }
    }

    /* timer event, different odr should has different timer to handle it in a instance */
    if(state->pressure_timer_data_stream != NULL) {
        event = state->pressure_timer_data_stream->api->peek_input(state->pressure_timer_data_stream);
        while(NULL != event) {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,event->event_len);
            sns_timer_sensor_event timer_event;
            if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
                if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
                    if(state->pressure_info.timer_is_active && (state->pressure_info.sampling_intvl > 0)) {
                        spl06_handle_pressure_data_stream_timer_event(this, &timer_event);
                    }
                }
            } else {
                SNS_INST_PRINTF(MED, this, "unknown message_id %d", event->message_id);
            }
            event = state->pressure_timer_data_stream->api->get_next_input(state->pressure_timer_data_stream);
        }
    }

    if(state->temperature_timer_data_stream != NULL) {
        event = state->temperature_timer_data_stream->api->peek_input(state->temperature_timer_data_stream);
        while(NULL != event) {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,event->event_len);
            sns_timer_sensor_event timer_event;
            if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
                if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
                    if(state->temperature_info.timer_is_active &&
                            state->temperature_info.sampling_intvl > 0) {
                        spl06_handle_temperature_data_stream_timer_event(this, &timer_event);
                    }
                }
            } else {
                SNS_INST_PRINTF(MED, this, "unknown message_id %d", event->message_id);
            }
            event = state->temperature_timer_data_stream->api->get_next_input(state->temperature_timer_data_stream);
        }
    }
    // Turn COM port OFF
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);

    return SNS_RC_SUCCESS;
}

/** Public Data Definitions. */
sns_sensor_instance_api sns_see_spl06_sensor_instance_api =
{
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &spl06_inst_init,
    .deinit = &spl06_inst_deinit,
    .set_client_config = &spl06_inst_set_client_config,
    .notify_event = &spl06_inst_notify_event
};
