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
#include "sns_register.h"
#include "sns_spl06_sensor.h"
#include "sns_spl06_sensor_instance.h"

/** Public Function Definitions. */
sns_rc sns_register_spl06(sns_register_cb const *register_api)
{
    /** Register Pressure Sensor. */
    register_api->init_sensor(sizeof(spl06_state), &spl06_pressure_sensor_api,
    						&sns_see_spl06_sensor_instance_api);
    /** Register temperature Sensor. */
    register_api->init_sensor(sizeof(spl06_state), &spl06_temperature_sensor_api,
      						&sns_see_spl06_sensor_instance_api);
    return SNS_RC_SUCCESS;
}
