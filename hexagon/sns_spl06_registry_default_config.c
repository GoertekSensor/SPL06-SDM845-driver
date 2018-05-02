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

#include "sns_spl06_sensor.h"
#include "sns_spl06_sensor_instance.h"
#include "sns_com_port_types.h"
#include "sns_types.h"
#include "sns_mem_util.h"

#if !SPL_CONFIG_ENABLE_REGISTRY
/** Using 8996 Platform config as defaults. This is for
 *  test purpose only. All platform specific information will
 *  be available to the Sensor driver via Registry.
 *  otherwise, please change accordingly
 *  */

#ifdef SSC_TARGET_HEXAGON_CORE_QDSP6_2_0
#define SPI_BUS_INSTANCE               0x02
#else
#define SPI_BUS_INSTANCE               0x01
#endif

#define RAIL_1                         "/pmic/client/sensor_vddio"
#define RAIL_2                         "/pmic/client/sensor_vdd"
#define IRQ_NUM                        117
#define NUM_OF_RAILS                   2

#define SPI_BUS_MIN_FREQ_KHZ           7*100  // using a value of 0 will introduce more errors in timing
#define SPI_BUS_MAX_FREQ_KHZ           13*100 // 1.3MHz
#define SPI_SLAVE_CONTROL              0x0

#define SENSOR_I2C_SLAVE_ADDRESS       0x76
#define I2C_BUS_INSTANCE               0x03
#define I2C_BUS_MIN_FREQ_KHZ           400
#define I2C_BUS_MAX_FREQ_KHZ           400

#define SPL_DEFAULT_REG_CFG_RAIL_ON         SNS_RAIL_ON_NPM
#define SPL_DEFAULT_REG_CFG_ISDRI           0
#define SPL_DEFAULT_REG_CFG_HW_ID           1
#define SPL_DEFAULT_REG_CFG_SUPPORT_SYN_STREAM 0

#define spl_sensor_com_port_spi_config_init_default   { \
    .bus_type          = SNS_BUS_SPI,            \
    .slave_control     = SPI_SLAVE_CONTROL,      \
    .reg_addr_type     = SNS_REG_ADDR_8_BIT,     \
    .min_bus_speed_KHz = SPI_BUS_MIN_FREQ_KHZ,   \
    .max_bus_speed_KHz = SPI_BUS_MAX_FREQ_KHZ,   \
    .bus_instance      = SPI_BUS_INSTANCE        \
    }

#define spl_sensor_com_port_i2c_config_init_default   { \
    .bus_type          = SNS_BUS_I2C,            \
    .slave_control     = SENSOR_I2C_SLAVE_ADDRESS,   \
    .reg_addr_type     = SNS_REG_ADDR_8_BIT,     \
    .min_bus_speed_KHz = I2C_BUS_MIN_FREQ_KHZ,   \
    .max_bus_speed_KHz = I2C_BUS_MAX_FREQ_KHZ,   \
    .bus_instance      = I2C_BUS_INSTANCE        \
    }

/**
 * Sensor platform resource configuration with Hard Code
 * @param this
 */
void sns_spl_registry_def_config(sns_sensor *const this)
{
    spl06_state* sstate = (spl06_state *) this->state->state;
    sns_com_port_config *com_port_cfg = &sstate->common.com_port_info.com_config;
    SPL_SENSOR_LOG(LOW, this, "sns_spl_registry_def_config");

    // <general config>
    {
        sstate->is_dri                      = SPL_DEFAULT_REG_CFG_ISDRI;
        sstate->hardware_id                 = SPL_DEFAULT_REG_CFG_HW_ID;
        sstate->supports_sync_stream        = SPL_DEFAULT_REG_CFG_SUPPORT_SYN_STREAM;
        sstate->common.registry_cfg_received = true;
    }

    // <platform configure>
    {
#if SPL_CONFIG_DFT_BUS_SPI
	    sns_memscpy(com_port_cfg,
	                sizeof(sstate->common.com_port_info.com_config),
	                &((sns_com_port_config)spl_sensor_com_port_spi_config_init_default ),
	                sizeof(sns_com_port_config));

#else  // I2C
	    sns_memscpy(com_port_cfg,
	              sizeof(sstate->common.com_port_info.com_config),
	              &((sns_com_port_config)spl_sensor_com_port_i2c_config_init_default),
	              sizeof(sns_com_port_config));
#endif



	    sstate->common.registry_rail_on_state  = SPL_DEFAULT_REG_CFG_RAIL_ON;
	    strlcpy(sstate->common.rail_config.rails[0].name,
	            RAIL_1,
	            sizeof(sstate->common.rail_config.rails[0].name));
	    strlcpy(sstate->common.rail_config.rails[1].name,
	            RAIL_2,
	            sizeof(sstate->common.rail_config.rails[1].name));
	    sstate->common.rail_config.num_of_rails = 2;

	    sstate->common.registry_pf_cfg_received = true;
    }
    // <placement>
    {
        uint8_t i;
        for (i = 0; i < 9; i ++) {
            sstate->common.placement[i] = 0;
        }
        sstate->common.registry_placement_received = true;
    }
}
#endif
