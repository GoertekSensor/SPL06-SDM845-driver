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
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_event_service.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_types.h"
#include "sns_diag_service.h"
#include "sns_attribute_util.h"
#include "sns_sync_com_port_service.h"

#include "sns_spl06_sensor.h"
#include "sns_spl06_hal.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_registry.pb.h"
#include "sns_printf.h"

static sns_sensor_uid const* spl06_pressure_get_sensor_uid(sns_sensor const * const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = PRESSURE_SUID;
    return &sensor_uid;
}
static sns_sensor_uid const* spl06_temperature_get_sensor_uid(sns_sensor const * const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = TEMPERATURE_SUID;
    return &sensor_uid;
}

sns_sensor_api spl06_pressure_sensor_api =
{
    .struct_len = sizeof(sns_sensor_api),
    .init = &spl06_pressure_init,
    .deinit = &spl06_pressure_deinit,
    .get_sensor_uid = &spl06_pressure_get_sensor_uid,
    .set_client_request = &spl06_sensor_set_client_request,
    .notify_event = &spl06_sensor_notify_event,
};

sns_sensor_api spl06_temperature_sensor_api =
{
    .struct_len = sizeof(sns_sensor_api),
    .init = &spl06_temperature_init,
    .deinit = &spl06_temperature_deinit,
    .get_sensor_uid = &spl06_temperature_get_sensor_uid,
    .set_client_request = &spl06_sensor_set_client_request,
    .notify_event = &spl06_sensor_notify_event,
};
