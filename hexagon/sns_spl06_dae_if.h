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
#include "sns_data_stream.h"
#include "sns_sensor_instance.h"
#include "sns_stream_service.h"
#include "sns_spl06_sensor.h"
#include "sns_spl06_config.h"

#if SPL_CONFIG_ENABLE_DAE
struct sns_stream_service;
struct sns_data_stream;
struct spl06_instance_state;

typedef enum
{
    IDLE,
    STREAM_STARTING,
    STREAMING,
    STREAM_STOPPING,
} spl06_dae_if_state;

typedef struct
{
    struct sns_data_stream *stream;
    const char             *nano_hal_vtable_name;
    spl06_dae_if_state     state;
    bool                   stream_usable:1;
    bool                   flushing_hw:1;
    bool                   flushing_data:1;
} spl06_dae_stream;

typedef struct spl06_dae_if_info
{
    spl06_dae_stream   pressure;
    spl06_dae_stream   temp;
} spl06_dae_if_info;

bool spl06_dae_if_available(sns_sensor_instance *this);
sns_rc spl06_dae_if_init(
    sns_sensor_instance  *const this,
    struct sns_stream_service  *stream_mgr,
    sns_sensor_uid             *dae_suid);

void spl06_dae_if_deinit(
    struct spl06_instance_state *state,
    struct sns_stream_service     *stream_mgr);
bool spl06_dae_if_stop_streaming(sns_sensor_instance *this);

bool spl06_dae_if_start_streaming(sns_sensor_instance *this);

bool spl06_dae_if_flush_hw(sns_sensor_instance *this);

bool spl06_dae_if_flush_samples(sns_sensor_instance *this);

void spl06_dae_if_process_events(sns_sensor_instance *this);
#endif
