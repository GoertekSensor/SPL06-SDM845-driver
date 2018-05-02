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

#include "sns_data_stream.h"
#include "sns_spl06_hal.h"
#if SPL_CONFIG_ENABLE_ISLAND_MODE
#include "sns_island_service.h"
#endif
#include "sns_pwr_rail_service.h"
#include "sns_sensor.h"
#include "sns_timer.pb.h"
#include "sns_spl06_sensor_instance.h"
#include "sns_sensor_uid.h"
#include "sns_suid_util.h"
#include "sns_diag_service.h"
#include "sns_sync_com_port_service.h"

#include "sns_math_util.h"
#include "sns_printf.h"
#include "sns_registry_util.h"
#include "sns_spl06_config.h"
#define PRESSURE_SUID \
{  \
    .sensor_uid =  \
    {  \
		0x4e, 0x45, 0x4a, 0x13, 0x89, 0xc3, 0x4f, 0x83, \
		0x9f, 0xec, 0x14, 0x88, 0x7f, 0x6e, 0x38, 0x6b  \
    }  \
}

#define TEMPERATURE_SUID \
{  \
    .sensor_uid =  \
    {  \
        0x31, 0x32, 0x46, 0x82, 0xca, 0xef, 0x42, 0x72, \
        0x95, 0x7e, 0xdd, 0x6e, 0x58, 0xb3, 0x1c, 0xed  \
    }  \
}

#if SPL_CONFIG_ENABLE_DEBUG
#define SPL_SENSOR_LOG(LOG_LEVEL, this, arg...) { \
    if (NULL != this) { \
        if (SNS_##LOG_LEVEL >= SNS_LOW) { \
            SNS_PRINTF(LOG_LEVEL, this, ##arg); \
        } \
    } \
}

#define SPL_INST_LOG(LOG_LEVEL, this, arg...) { \
    if (NULL != this) { \
        if (SNS_##LOG_LEVEL >= SNS_LOW) { \
            SNS_INST_PRINTF(LOG_LEVEL, this, ##arg); \
        } \
    } \
}
#else
#define SPL_SENSOR_LOG(LOG_LEVEL, this, arg...)
#define SPL_INST_LOG(LOG_LEVEL, this, arg...)
#endif

#define SPL06_SENSOR_PRESSURE_RESOLUTION        (0.0006f)
#define SPL06_SENSOR_PRESSURE_RANGE_MIN         (300.0f)
#define SPL06_SENSOR_PRESSURE_RANGE_MAX         (1100.0f)
#define SPL06_SENSOR_PRESSURE_LOW_POWER_CURRENT  	(1)
#define SPL06_SENSOR_PRESSURE_NORMAL_POWER_CURRENT  (6) 
#define SPL06_SENSOR_PRESSURE_SLEEP_CURRENT  		(0)
#define SPL06_SENSOR_TEMPERATURE_RESOLUTION  		(0.01f)
#define SPL06_SENSOR_TEMPERATURE_RANGE_MIN  		(-40.0f)
#define SPL06_SENSOR_TEMPERATURE_RANGE_MAX  		(85.0f)
#define SPL06_SENSOR_TEMPERATURE_LOW_POWER_CURRENT  	(1)
#define SPL06_SENSOR_TEMPERATURE_NORMAL_POWER_CURRENT  	(6)
#define SPL06_SENSOR_TEMPERATURE_SLEEP_CURRENT  		(0)

/** Forward Declaration of Pressure Sensor API */
extern sns_sensor_api spl06_pressure_sensor_api;

/** Forward Declaration of temperature Sensor API */
extern sns_sensor_api spl06_temperature_sensor_api;
/**
 * SPL ODR definitions
 */
#define SPL_ODR_0               (0.0)
#define SPL_ODR_1               (1.0)
#define SPL_ODR_2               (2.0)
#define SPL_ODR_4               (4.0)
#define SPL_ODR_8               (8.0)
#define SPL_ODR_16              (16.0)
#define SPL_ODR_32              (32.0)
#define SPL_ODR_64              (64.0)

#define SPL_LOWPOWER         "LOWPOWER"
#define SPL_HIGH_PERF        "HIGH_PERF"
#define SPL_NORMAL           "NORMAL"

/** Power rail timeout States for the SPL Sensors.*/
typedef enum
{
	SPL_POWER_RAIL_PENDING_NONE,
	SPL_POWER_RAIL_PENDING_INIT,
	SPL_POWER_RAIL_PENDING_SET_CLIENT_REQ,
	SPL_POWER_RAIL_PENDING_CREATE_DEPENDENCY,
} spl06_power_rail_pending_state;

/** Interrupt Sensor State. */
typedef struct spl06_common_state
{
	spl06_com_port_info     com_port_info;
	sns_interrupt_req       irq_config;

	sns_rail_config         rail_config;
	sns_power_rail_state    registry_rail_on_state;

	bool                    hw_is_present;
	uint16_t                chip_id;

	// Registry, IRQ, Timer, ASCP, DAE
	SNS_SUID_LOOKUP_DATA(5) suid_lookup_data;

	// registry sensor config
	bool registry_cfg_received;
	sns_registry_phy_sensor_cfg registry_cfg;

	// registry sensor platform config
	bool registry_pf_cfg_received;
	sns_registry_phy_sensor_pf_cfg registry_pf_cfg;

	// axis conversion
	bool registry_orient_received;
	triaxis_conversion axis_map[TRIAXIS_NUM];

	// placement
	bool   registry_placement_received;
	float  placement[12];

} spl06_common_state;

typedef struct spl06_state
{
	spl06_common_state    common;
	sns_data_stream         *reg_data_stream;
	sns_data_stream         *fw_stream;
	sns_data_stream         *timer_stream;
	sns_pwr_rail_service    *pwr_rail_service;
	sns_diag_service        *diag_service;
#if SPL_CONFIG_ENABLE_ISLAND_MODE
	sns_island_service      *island_service;
#endif
	sns_sync_com_port_service *scp_service;

	spl06_sensor_type     sensor;
	sns_sensor_uid        my_suid;
	bool                  sensor_client_present;
	
	spl06_power_rail_pending_state power_rail_pend_state;
	// sensor configuration
	bool is_dri;
	int64_t hardware_id;
	bool supports_sync_stream;
	uint8_t resolution_idx;
	
	size_t encoded_event_len;
	struct spl06_calib_param_t  calib_param;/**<calibration data*/	
} spl06_state;

/** Functions shared by all SPL06 Sensors */
/**
 * This function parses the client_request list per Sensor and
 * determines final config for the Sensor Instance.
 *
 * @param[i] this          Sensor reference
 * @param[i] instance      Sensor Instance to config
 * @param[i] sensor_type   Sensor type
 *
 * @return none
 */
void spl06_reval_instance_config(sns_sensor *this,
		sns_sensor_instance *instance, spl06_sensor_type sensor_type);
sns_rc spl06_com_read_wrapper(sns_sync_com_port_service *scp_service,sns_sync_com_port_handle *port_handle,
        uint32_t reg_addr, uint8_t *buffer, uint32_t bytes,
        uint32_t *xfer_bytes);
sns_rc spl06_com_write_wrapper(sns_sync_com_port_service *scp_service,sns_sync_com_port_handle *port_handle,
        uint32_t reg_addr, uint8_t *buffer, uint32_t bytes,
        uint32_t *xfer_bytes, bool save_write_time);
sns_rc spl06_set_oversamp_temperature(spl06_instance_state *state, uint8_t v_value_u8);
sns_rc spl06_set_oversamp_pressure(spl06_instance_state *state, uint8_t v_value_u8);
sns_rc spl06_set_samplerate_temperature(spl06_instance_state *state, uint8_t v_value_u8);
sns_rc spl06_set_samplerate_pressure(spl06_instance_state *state, uint8_t v_value_u8);
sns_rc spl06_set_power_mode(spl06_instance_state *state, uint8_t power_mode);
sns_sensor_instance* spl06_sensor_set_client_request(sns_sensor * const this,
        struct sns_request const *exist_request,
        struct sns_request const *new_request,
        bool remove);
sns_rc spl06_sensor_notify_event(sns_sensor * const this);
sns_rc spl06_pressure_init(sns_sensor * const this);
sns_rc spl06_pressure_deinit(sns_sensor * const this);
sns_rc spl06_temperature_init(sns_sensor * const this);
sns_rc spl06_temperature_deinit(sns_sensor * const this);
sns_rc spl06_get_chip_type(
		sns_sync_com_port_service *scp_service,
		sns_sync_com_port_handle *port_handle,
		uint8_t *buffer);
void spl06_common_init(sns_sensor *const this);
// <registry>
#if !SPL_CONFIG_ENABLE_REGISTRY
void sns_spl_registry_def_config(sns_sensor *const this);
#endif
// </registry>

