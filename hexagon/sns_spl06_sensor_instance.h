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
#include "sns_com_port_types.h"
#include "sns_data_stream.h"
#include "sns_sensor_instance.h"
#include "sns_time.h"
#include "sns_printf.h"

#include "sns_sensor_uid.h"
#include "sns_spl06_config.h"

#include "sns_async_com_port.pb.h"
#include "sns_diag_service.h"
#include "sns_interrupt.pb.h"
#if SPL_CONFIG_ENABLE_ISLAND_MODE
#include "sns_island_service.h"
#endif
#include "sns_spl06_dae_if.h"
#include "sns_std_sensor.pb.h"
#include "sns_physical_sensor_test.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_sync_com_port_service.h"

#include "sns_math_util.h"
#include "sns_registry_util.h"
/** Forward Declaration of Instance API */
extern sns_sensor_instance_api sns_see_spl06_sensor_instance_api;

/* physical COM port structure */
typedef struct spl06_com_port_info
{
	sns_com_port_config        com_config;
	sns_sync_com_port_handle  *port_handle;
} spl06_com_port_info;

/**
 * Range attribute.
 */
typedef struct range_attr
{
	float min;
	float max;
} range_attr;


typedef enum {
	SPL06_PRESSURE_IDX = 0x0,
	SPL06_TEMPERATURE_IDX = 0x1,
	SPL_MAX_SENSOR
} spl06_sensor_type_idx;

typedef enum
{
	SPL06_PRESSURE = 0x1,
	SPL06_TEMPERATURE = 0x2,
	SPL_SENSOR_INVALID = 0xFF
} spl06_sensor_type;

typedef enum
{
	SPL06_CONFIG_IDLE,            /** not configuring */
	SPL06_CONFIG_POWERING_DOWN,   /** cleaning up when no clients left */
	SPL06_CONFIG_STOPPING_STREAM, /** stream stop initiated, waiting for completion */
	SPL06_CONFIG_FLUSHING_HW,     /** FIFO flush initiated, waiting for completion */
	SPL06_CONFIG_UPDATING_HW      /** updating sensor HW, when done goes back to IDLE */
} spl06_config_step;

/* enum for setting/getting device operating mode*/
typedef enum
{
	SPL06_MODE_IDLE                   =  0x0,
	SPL06_MODE_COMMAND_PRESSURE       =  0x1,
	SPL06_MODE_COMMAND_TEMPERATURE    =  0x2,
	SPL06_MODE_BACKGROUND_PRESSURE    =  0x5,
	SPL06_MODE_BACKGROUND_TEMPERATURE =  0x6,
	SPL06_MODE_BACKGROUND_ALL         =  0x7,
} spl06_operating_modes;

typedef enum
{
	SPL06_TEST_NO_ERROR,
	SPL06_FAC_TEST_HIGH_BIAS,
	SPL06_FAC_TEST_DEV_NOT_STATIONARY,
	SPL06_FAC_TEST_ZERO_VARIANCE
} spl06_test_err_code;

typedef struct spl06_self_test_info
{
	sns_physical_sensor_test_type test_type;
	bool test_client_present;
} spl06_self_test_info;

#if SPL_CONFIG_ENABLE_SELF_TEST_FAC
typedef struct spl06_factory_test_info
{
	int32_t num_samples;             /** number of samples acquired */
	float variance_threshold;        /** stationary detect variance threshold */
	float variance[TRIAXIS_NUM];          /** variance */
	float sample_square_sum[TRIAXIS_NUM]; /** sum of square of sample data */
	float sample_sum[TRIAXIS_NUM];        /** sum of sample data */
	float bias_thresholds[TRIAXIS_NUM];
	bool at_rest;
} spl06_factory_test_info;
#endif

typedef struct spl06_sensor_deploy_info
{
	/** Determines which Sensor data to publish. Uses
	*  spl06_sensor_type as bit mask. */
	uint8_t           publish_sensors;
	uint8_t           enable;
} spl06_sensor_deploy_info;


typedef struct spl06_sensor_cfg_info
{
	float             desired_odr;
	float             curr_odr;
	sns_sensor_uid    suid;
	uint64_t          trigger_num;
	sns_time          timeout_ticks; /* derived from the odr */
	sns_time          expection_timeout_ticks_derived_from_odr;
	bool              timer_is_active;
	uint32_t          report_timer_hz;
	float             report_rate_hz;
	float             sampling_rate_hz;
	sns_time          sampling_intvl;
	uint64_t          max_requested_flush_ticks;
	sns_time          expect_time;
	spl06_self_test_info  test_info;
} spl06_sensor_cfg_info;

/* async port for the data stream which use the COM port handle */
typedef struct spl06_async_com_port_info {
	uint32_t port_handle;
} spl06_async_com_port_info;

typedef struct sns_spl06_registry_cfg
{
	spl06_sensor_type sensor_type;
	matrix3             fac_cal_corr_mat;
	float               fac_cal_bias[3];
	uint32_t            version;
}sns_spl06_registry_cfg;

/* Struct to hold calibration coefficients read from device*/
typedef struct spl06_calib_param_t
{
	/* calibration registers */
	int16_t c0;		// 12bit
	int16_t c1;		// 12bit
	int32_t	c00;	// 20bit
	int32_t c10;	// 20bit
	int16_t c01;	// 16bit
	int16_t	c11;	// 16bit
	int16_t	c20;	// 16bit
	int16_t	c21;	// 16bit
	int16_t	c30;	// 16bit
}spl06_cal_coeff_regs_s;

/** Private state. */
typedef struct spl06_instance_state
{
	/** -- sensor configuration details --*/
	/** pressure HW config details*/
	spl06_sensor_cfg_info pressure_info;
	/** temperature HW config details */
	spl06_sensor_cfg_info temperature_info;
	spl06_sensor_deploy_info   deploy_info;
	/** COM port info */
	spl06_com_port_info com_port_info;
	/**--------Async Com Port--------*/
	sns_async_com_port_config  ascp_config;
#if SPL_CONFIG_ENABLE_DAE
	/**--------DAE interface---------*/
	spl06_dae_if_info       dae_if;
#endif
	spl06_config_step       config_step;
	/** Data streams from dependentcies. */
	sns_sensor_uid                 timer_suid;
	sns_data_stream                *temperature_timer_data_stream;
	sns_data_stream                *async_com_port_data_stream;  /* data streaming channel */
	sns_data_stream                *pressure_timer_data_stream;    /* sample sensor data */
	/* request/configure stream */
	uint32_t                       client_req_id;
	sns_std_sensor_config          spl06_req;   /* stream for the configure */
	size_t                         encoded_imu_event_len;
	/**----------Sensor specific registry configuration----------*/
	sns_spl06_registry_cfg sensor_temp_registry_cfg;
	sns_spl06_registry_cfg sensor_pressure_registry_cfg;
	sns_diag_service               *diag_service;  /* for diagnostic to print debug message */
	sns_sync_com_port_service      *scp_service;
#if SPL_CONFIG_ENABLE_ISLAND_MODE
	sns_island_service *island_service;
#endif
	uint8_t 					power_mode;
	uint8_t						work_mode;
	uint32_t                    interface;
	sns_rc (* com_read)(
	 sns_sync_com_port_service *scp_service,
	  sns_sync_com_port_handle *port_handle,
	  uint32_t rega,
	  uint8_t  *regv,
	  uint32_t bytes,
	  uint32_t *xfer_bytes);
	sns_rc (* com_write)(
	  sns_sync_com_port_service *scp_service,
	  sns_sync_com_port_handle *port_handle,
	  uint32_t rega,
	  uint8_t  *regv,
	  uint32_t bytes,
	  uint32_t *xfer_bytes,
	  bool save_write_time);
	bool instance_is_ready_to_configure;
	bool new_self_test_request;
#if SPL_CONFIG_ENABLE_SELF_TEST_FAC
	bool fac_test_in_progress;
	spl06_factory_test_info fac_test_info;
	spl06_sensor_type fac_test_sensor;
#endif
	bool update_fac_cal_in_registry;
	uint32_t i32kP; /*!pressure scale factor variable */
	uint32_t i32kT; /*!temperature scale factor variable */	
#if SPL_CONFIG_ENABLE_FILTER
	uint8_t cutoff_freq_index;
	bool filter_initialized;
	float data_filter[2];
#endif
	uint8_t oversamp_temperature;/**< temperature over sampling*/
	uint8_t oversamp_pressure;/**< pressure over sampling*/
	uint8_t samplerate_temperature;/*!temperature sampling rate */	
	uint8_t samplerate_pressure;/*!pressure sampling rate */
	size_t           log_raw_encoded_size;
	struct spl06_calib_param_t calib_param;/**<calibration data*/		
} spl06_instance_state;

typedef struct sns_spl06_cfg_req {
	float               sample_rate;
	float               report_rate;
	spl06_sensor_type  sensor_type;
	uint8_t   power_mode;
	uint64_t  desired_flush_ticks;
	sns_spl06_registry_cfg  registry_cfg;
} sns_spl06_cfg_req;

sns_rc spl06_inst_init(sns_sensor_instance *const this,
		sns_sensor_state const *sstate);
sns_rc spl06_inst_deinit(sns_sensor_instance *const this);
sns_rc spl06_inst_set_client_config(
		sns_sensor_instance * const this,
		sns_request const *client_request);
