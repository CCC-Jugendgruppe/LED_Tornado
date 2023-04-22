/**************************************************************************//**
  * @file    stspin240_250_target_config.h
  * @author  IPC Rennes
  * @version V1.2.0
  * @date    September 11th, 2017
  * @brief   Predefines values for the Stspin240 or Stspin250 parameters
  * and for the devices parameters
  * @note    (C) COPYRIGHT 2017 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STSPIN240_250_TARGET_CONFIG_H
#define __STSPIN240_250_TARGET_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup STSPIN240_250
  * @{
  */   

/** @addtogroup STSPIN240_250_Exported_Constants STSPIN240_250 Exported Constants
  * @{
  */   
   
/** @defgroup Predefined_STSPIN240_250_Parameters_Values Predefined STSPIN240_250 Parameters Values
  * @{
  */   

/// The maximum number of Stspin240 or Stspin250 devices 
#define MAX_NUMBER_OF_DEVICES                 (1)
   
/// The maximum number of Brush DC motors connected to the Stspin240 or Stspin250
#define STSPIN_240
#ifdef STSPIN_240
#define MAX_NUMBER_OF_BRUSH_DC_MOTORS                    (2)
#elif defined (STSPIN_250)
#define MAX_NUMBER_OF_BRUSH_DC_MOTORS                    (1)
#else
#error "Please select your kind of STSPIN by setting compilation flag STSPIN_240 or STSPIN_250"
#endif

/// Frequency of PWM of Input Bridge A in Hz up to 100000Hz
#define STSPIN240_250_CONF_PARAM_FREQ_PWM_A  (20000)

#ifdef STSPIN_240  
/// Frequency of PWM of Input Bridge B in Hz up to 100000Hz
/// On the X-NUCLEO-IHM01A12 expansion board the PWM_A and PWM_B
/// share the same timer, so the frequency must be the same     
#define STSPIN240_250_CONF_PARAM_FREQ_PWM_B  (20000)
#endif
   
/// Frequency of PWM used for Ref pinin Hz up to 100000Hz
#define STSPIN240_250_CONF_PARAM_FREQ_PWM_REF  (20000)

/// Duty cycle of PWM used for Ref pin (from 0 to 100)
#define STSPIN240_250_CONF_PARAM_DC_PWM_REF  (50)
    
/// Dual Bridge configuration  (0 for mono, 1 for dual brush dc (STSPIN240 only)
#define STSPIN240_250_CONF_PARAM_DUAL_BRIDGE_ENABLING    (0)
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
  }
#endif

#endif /* __STSPIN240_250_TARGET_CONFIG_H */
