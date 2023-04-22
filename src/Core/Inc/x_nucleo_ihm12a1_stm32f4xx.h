/** 
  ******************************************************************************
  * @file    x_nucleo_ihm12a1_stm32f4xx.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    August 16, 2016
  * @brief   Header for BSP driver for x-nucleo-ihm12a1.h Nucleo extension board 
  *  (based on Stspin240)
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
#ifndef X_NUCLEO_IHM12A1_STM32F4XX_H
#define X_NUCLEO_IHM12A1_STM32F4XX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_nucleo.h"
   
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup X_NUCLEO_IHM12A1_STM32F4XX
  * @{   
  */   
   
/* Exported Constants --------------------------------------------------------*/
   
/** @defgroup IHM12A1_Exported_Constants
  * @{
  */   
   
/******************************************************************************/
/* USE_STM32F4XX_NUCLEO                                                       */
/******************************************************************************/

 /** @defgroup Constants_For_STM32F4XX_NUCLEO  
* @{
*/   
/// Interrupt line used for Stspin240 Fault interrupt
#define EXTI_FAULT_IRQn           (EXTI15_10_IRQn)
   
/// Timer used for PWMA
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_A      (TIM3)

/// Timer used for PWMB
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_B      (TIM3)

   /// Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF      (TIM2)
   
/// Channel Timer used for PWMA
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_A      (TIM_CHANNEL_1)

/// Channel Timer used for PWMB
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_B      (TIM_CHANNEL_2)

/// Channel Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_REF     (TIM_CHANNEL_1)   
   
/// HAL Active Channel Timer used for PWMA
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_A      (HAL_TIM_ACTIVE_CHANNEL_1)

/// HAL Active Channel Timer used for PWMB
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_B      (HAL_TIM_ACTIVE_CHANNEL_2)

/// HAL Active Channel Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_REF      (HAL_TIM_ACTIVE_CHANNEL_1)
   
/// Timer Clock Enable for PWMA
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_A_CLCK_ENABLE()  __TIM3_CLK_ENABLE()

/// Timer Clock Enable for PWMB
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_B_CLCK_ENABLE()  __TIM3_CLK_ENABLE()

/// Timer Clock Enable for REF
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF_CLCK_ENABLE()   __TIM2_CLK_ENABLE()
   
   /// Timer Clock Enable for PWMA
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_A_CLCK_DISABLE()  __TIM3_CLK_DISABLE()

/// Timer Clock Enable for PWMB
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_B_CLCK_DISABLE()  __TIM3_CLK_DISABLE()

   /// Timer Clock Enable for REF
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF_CLCK_DISABLE()  __TIM2_CLK_DISABLE()
   
/// PWMA GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_A  (GPIO_AF2_TIM3)

/// PWMB GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_B  (GPIO_AF2_TIM3)

/// REF GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_REF  (GPIO_AF1_TIM2)

/// GPIO Pin used for the ref pin of the Stspin240
#define BSP_MOTOR_CONTROL_BOARD_REF_PIN  (GPIO_PIN_0)
/// GPIO Port used for the ref pin of the Stspin240
#define BSP_MOTOR_CONTROL_BOARD_REF_PORT  (GPIOA)
   
 /**
* @}
*/

/******************************************************************************/
/* Independent plateform definitions                                          */
/******************************************************************************/

   /** @defgroup Constants_For_All_Nucleo_Platforms
* @{
*/   

/// GPIO Pin used for the  input PWM A of the Stspin240 
#define BSP_MOTOR_CONTROL_BOARD_PWM_A_PIN  (GPIO_PIN_4)
/// GPIO Port sed for the  input PWM A of the Stspin240 
#define BSP_MOTOR_CONTROL_BOARD_PWM_A_PORT  (GPIOB)
   
/// GPIO Pin used for the PWM B of the Stspin240
#define BSP_MOTOR_CONTROL_BOARD_PWM_B_PIN  (GPIO_PIN_5)
/// GPIO Port used for the PWM B of the Stspin240
#define BSP_MOTOR_CONTROL_BOARD_PWM_B_PORT  (GPIOB)

   /// GPIO Pin used for the direction of the Stspin240 Brige A
#define BSP_MOTOR_CONTROL_BOARD_DIR_A_PIN  (GPIO_PIN_10)
/// GPIO Port used for the direction of the Stspin240 Brige A
#define BSP_MOTOR_CONTROL_BOARD_DIR_A_PORT  (GPIOB)

/// GPIO Pin used for the direction of the Stspin240 Brige B
#define BSP_MOTOR_CONTROL_BOARD_DIR_B_PIN  (GPIO_PIN_8)
/// GPIO Port used for the direction of the Stspin240 Brige B
#define BSP_MOTOR_CONTROL_BOARD_DIR_B_PORT  (GPIOA)
   
/// GPIO Pin used for the Stspin240  Enable pin and Faults (over current detection and thermal shutdown)
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PIN  (GPIO_PIN_10)
/// GPIO port used for the Stspin240  Enable pin and Faults (over current detection and thermal shutdown)
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PORT (GPIOA)
/// Flag interrupt priority
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PRIORITY  (3)
   
/// GPIO Pin used for the standy/reset pin of the Stspin240
#define BSP_MOTOR_CONTROL_BOARD_RESET_PIN  (GPIO_PIN_7)
/// GPIO Port used for the standy/reset pin of the Stspin240
#define BSP_MOTOR_CONTROL_BOARD_RESET_PORT  (GPIOC)
   
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

#endif /* X_NUCLEO_IHM12A1_STM32F4XX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
