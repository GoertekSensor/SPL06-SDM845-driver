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

#ifndef __SNS_DD_SPL_CONFIG_H
#define __SNS_DD_SPL_CONFIG_H

// <Registry Sensors.SEE-LITE sensor>
#ifndef SPL_CONFIG_ENABLE_SEE_LITE
#define SPL_CONFIG_ENABLE_SEE_LITE   0
#endif

#ifndef SPL_CONFIG_ENABLE_REGISTRY
#define SPL_CONFIG_ENABLE_REGISTRY   1
#endif

#if !SPL_CONFIG_ENABLE_REGISTRY
#ifndef SPL_CONFIG_DFT_BUS_SPI
#define SPL_CONFIG_DFT_BUS_SPI    0
#endif
#endif

// <Debug Print Messages>
#ifndef SPL_CONFIG_ENABLE_DEBUG
#define SPL_CONFIG_ENABLE_DEBUG   1
#endif

// <DAE Usage>
#ifndef SPL_CONFIG_ENABLE_DAE
#define SPL_CONFIG_ENABLE_DAE     0
#endif


// <Diagnostic Logging>
#ifndef SPL_CONFIG_ENABLE_DIAG_LOG
#define SPL_CONFIG_ENABLE_DIAG_LOG   1
#endif

// <power rail reference>
#ifndef SPL_CONFIG_POWER_RAIL
#define SPL_CONFIG_POWER_RAIL        1
#endif

#ifndef SPL_CONFIG_ENABLE_ISLAND_MODE
#define SPL_CONFIG_ENABLE_ISLAND_MODE   1
#endif


#ifndef SPL_CONFIG_ENABLE_SELF_TEST_FAC
#define SPL_CONFIG_ENABLE_SELF_TEST_FAC   1
#endif

#ifndef SPL_CONFIG_ENABLE_FILTER
#define SPL_CONFIG_ENABLE_FILTER   1
#endif

#endif

