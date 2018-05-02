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
#include "sns_types.h"
#include "sns_service_manager.h"
#include "sns_spl06_sensor.h"
#include "sns_spl06_ver.h"
#include "sns_pb_util.h"
#include "sns_attribute_util.h"

/**
 * Publish all Sensor attributes
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
static void spl06_publish_attributes(sns_sensor * const this)
{
#if !SPL_CONFIG_ENABLE_SEE_LITE
	spl06_state *state = (spl06_state*)this->state->state;
	{
		sns_std_attr_value_data values[] = { SNS_ATTR };
		sns_std_attr_value_data range1[] = { SNS_ATTR, SNS_ATTR };
		range1[0].has_flt = true;
		range1[0].flt = SPL06_SENSOR_PRESSURE_RANGE_MIN;
		range1[1].has_flt = true;
		range1[1].flt = SPL06_SENSOR_PRESSURE_RANGE_MAX;
		values[0].has_subtype = true;
		values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
		values[0].subtype.values.arg =
		&((pb_buffer_arg){ .buf = range1, .buf_len = ARR_SIZE(range1) });

		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RANGES,
		values, ARR_SIZE(values), false);
	}
	{
		sns_std_attr_value_data values[] = {SNS_ATTR};
		sns_std_attr_value_data range1[] = {SNS_ATTR, SNS_ATTR};
		range1[0].has_flt = true;
		range1[0].flt = SPL06_SENSOR_PRESSURE_RANGE_MIN;
		range1[1].has_flt = true;
		range1[1].flt = SPL06_SENSOR_PRESSURE_RANGE_MAX;
		values[0].has_subtype = true;
		values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
		values[0].subtype.values.arg =
		  &((pb_buffer_arg){ .buf = range1, .buf_len = ARR_SIZE(range1) });
		sns_publish_attribute(
		    this, SNS_STD_SENSOR_ATTRID_SELECTED_RANGE, &values[0],
		    ARR_SIZE(values), false);
	}
	{
		sns_std_attr_value_data values[] = {SNS_ATTR};
		values[0].has_sint = true;
		values[0].sint = SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
		    values, ARR_SIZE(values), false);
	}
	{
		sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
		values[0].has_sint = true;
		values[0].sint = SPL06_SENSOR_PRESSURE_LOW_POWER_CURRENT;
		values[1].has_sint = true;
		values[1].sint = SPL06_SENSOR_PRESSURE_NORMAL_POWER_CURRENT;
		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT,
		    values, ARR_SIZE(values), false);
	}
	{
		sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, 
			SNS_ATTR, SNS_ATTR, SNS_ATTR};
		values[0].has_flt = true;
		values[0].flt = SPL_ODR_1;
		values[1].has_flt = true;
		values[1].flt = SPL_ODR_2;
		values[2].has_flt = true;
		values[2].flt = SPL_ODR_4;
		values[3].has_flt = true;
		values[3].flt = SPL_ODR_8;
		values[4].has_flt = true;
		values[4].flt = SPL_ODR_16;
		values[5].has_flt = true;
		values[5].flt = SPL_ODR_32;		
		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RATES,
		    values, ARR_SIZE(values), false);
	}
	{
		sns_std_attr_value_data values[] = {SNS_ATTR};
		values[0].has_flt = true;
		values[0].flt = SPL06_SENSOR_PRESSURE_RESOLUTION;
		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RESOLUTIONS,
		    values, ARR_SIZE(values), false);
	}
	{
		sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
		char const op_mode1[] = SPL_LOWPOWER;
		char const op_mode2[] = SPL_NORMAL;

		values[0].str.funcs.encode = pb_encode_string_cb;
		values[0].str.arg = &((pb_buffer_arg)
		    { .buf = op_mode1, .buf_len = sizeof(op_mode1) });
		values[1].str.funcs.encode = pb_encode_string_cb;
		values[1].str.arg = &((pb_buffer_arg)
		    { .buf = op_mode2, .buf_len = sizeof(op_mode2) });

		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_OP_MODES,
		    values, ARR_SIZE(values), false);
	}
	{
		sns_std_attr_value_data values[] = {SNS_ATTR};
		char const proto1[] = "sns_pressure.proto";
		values[0].str.funcs.encode = pb_encode_string_cb;
		values[0].str.arg = &((pb_buffer_arg)
		    { .buf = proto1, .buf_len = sizeof(proto1) });
		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_API,
		    values, ARR_SIZE(values), false);
	}
	{
		char const name[] = "goertek_spl06";
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.str.funcs.encode = pb_encode_string_cb;
		value.str.arg = &((pb_buffer_arg)
		    { .buf = name, .buf_len = sizeof(name) });
		sns_publish_attribute(
		    this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
	}
#endif
	{
		char const type[] = "pressure";
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.str.funcs.encode = pb_encode_string_cb;
		value.str.arg = &((pb_buffer_arg)
		    { .buf = type, .buf_len = sizeof(type) });
		sns_publish_attribute(
		    this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
	}
	{
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.has_boolean = true;
		value.boolean = false;
		sns_publish_attribute(
		    this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, false);
	}
#if !SPL_CONFIG_ENABLE_SEE_LITE
	{
		char const vendor[] = "Goertek";
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.str.funcs.encode = pb_encode_string_cb;
		value.str.arg = &((pb_buffer_arg)
		    { .buf = vendor, .buf_len = sizeof(vendor) });
		sns_publish_attribute(
		    this, SNS_STD_SENSOR_ATTRID_VENDOR, &value, 1, false);
	}
	{
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.has_boolean = true;
		value.boolean = false;
		sns_publish_attribute(
		    this, SNS_STD_SENSOR_ATTRID_DYNAMIC, &value, 1, false);
	}
	{
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.has_sint = true;
		value.sint = SPL06_SEE_DD_VER;
		sns_publish_attribute(
		    this, SNS_STD_SENSOR_ATTRID_VERSION, &value, 1, false);
	}
	{
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.has_sint = true;
		value.sint = 0;
		sns_publish_attribute(
		    this, SNS_STD_SENSOR_ATTRID_FIFO_SIZE, &value, 1, false);
	}
	{
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.has_sint = true;
		value.sint = SPL06_SENSOR_PRESSURE_SLEEP_CURRENT;
		sns_publish_attribute(
		    this, SNS_STD_SENSOR_ATTRID_SLEEP_CURRENT, &value, 1, false);
	}
	{
		float data[1] = {0};
		state->encoded_event_len =
		    pb_get_encoded_size_sensor_stream_event(data, 1);
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.has_sint = true;
		value.sint = state->encoded_event_len;
		sns_publish_attribute(
		    this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE, &value, 1, false);
	}
	{
		sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
		value.has_boolean = true;
		value.boolean = true;
		sns_publish_attribute(
		    this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR, &value, 1, true);
	}
	{
		sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
		values[0].has_sint = true;
		values[0].sint = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
		values[1].has_sint = true;
		values[1].sint = SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY;
		sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR_TESTS,
		    values, ARR_SIZE(values), true);
	}
#endif
}

sns_rc spl06_pressure_init(sns_sensor * const this)
{
	spl06_state *state = (spl06_state*) this->state->state;

	state->sensor = SPL06_PRESSURE;
	sns_memscpy(&state->my_suid,
	          sizeof(state->my_suid),
	          &((sns_sensor_uid)PRESSURE_SUID),
	          sizeof(sns_sensor_uid));

	spl06_common_init(this);
	spl06_publish_attributes(this);

	return SNS_RC_SUCCESS;
}

sns_rc spl06_pressure_deinit(sns_sensor * const this)
{
	UNUSED_VAR(this);
	return SNS_RC_SUCCESS;
}
