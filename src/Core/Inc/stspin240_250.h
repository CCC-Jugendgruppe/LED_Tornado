/**
  ******************************************************************************
  * @file    stspin240_250.h 
  * @author  IPC Rennes
  * @version V1.2.0
  * @date    September 11th, 2017
  * @brief   Header for Stspin240 (low voltage dual brush DC motor driver) and 
  *          Stspin250 driver (low voltage  brush DC motor driver)
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
#ifndef __STSPIN240_250_H
#define __STSPIN240_250_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stspin240_250_target_config.h"
#include "motor.h"   
   
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup STSPIN240_250
  * @{
  */   
   
/* Exported Constants --------------------------------------------------------*/

/** @defgroup STSPIN240_250_Exported_Constants STSPIN240_250 Exported Constants
  * @{
  */   

/// Current FW version
   
/// Current FW major version
#define STSPIN240_250_FW_MAJOR_VERSION (uint8_t)(1)
/// Current FW minor version
#define STSPIN240_250_FW_MINOR_VERSION (uint8_t)(3)
/// Current FW patch version
#define STSPIN240_250_FW_PATCH_VERSION (uint8_t)(0)
/// Current FW version
#define STSPIN240_250_FW_VERSION (uint32_t)((STSPIN240_250_FW_MAJOR_VERSION<<16)|\
                                            (STSPIN240_250_FW_MINOR_VERSION<<8)|\
                                            (STSPIN240_250_FW_PATCH_VERSION))   

#ifdef STSPIN_240
///Max number of Brush DC motors
#define STSPIN240_250_NB_MAX_MOTORS (2)
///Number of Bridges
#define STSPIN240_250_NB_BRIDGES (2)
#elif defined (STSPIN_250)
///Max number of Brush DC motors
#define STSPIN240_250_NB_MAX_MOTORS (1)
///Number of Bridges
#define STSPIN240_250_NB_BRIDGES (1)
#else
#error "Please select your kind of STSPIN by setting compilation flag STSPIN_240 or STSPIN_250"
#endif
    
    /**
  * @}
  */

/** @addtogroup STSPIN240_250_Exported_Variables STSPIN240_250 Exported Variables
  * @{
  */    
    extern motorDrv_t   stspin240_250Drv;
/**
  * @}
  */
     
/* Exported Types  -------------------------------------------------------*/

/** @defgroup STSPIN240_250_Exported_Types STSPIN240_250 Exported Types
  * @{
  */   


/** @defgroup Device_Parameters Device Parameters
  * @{
  */

/// Device Parameters Structure Type
typedef struct {
    /// Pwm frequency of the bridge input
    uint32_t bridgePwmFreq[STSPIN240_250_NB_BRIDGES];      
    /// Pwm frequency of the ref pin
    uint32_t refPwmFreq;      
    /// Pwm Duty Cycle of the ref pin
    uint8_t refPwmDc;      
    /// Speed% (from 0 to 100) of the corresponding motor
     uint16_t speed[STSPIN240_250_NB_MAX_MOTORS];  
    /// FORWARD or BACKWARD direction of the motors
    motorDir_t direction[STSPIN240_250_NB_MAX_MOTORS];                 
    /// Current State of the motors
     motorState_t motionState[STSPIN240_250_NB_MAX_MOTORS];       
    /// Current State of the bridges
    bool bridgeEnabled[STSPIN240_250_NB_BRIDGES];    
    /// Enabling of a dual bridge configuration
    bool dualBridgeEnabled;    
}deviceParams_t; 

/**
  * @}
  */


/** @defgroup Stspin240_250_Initialization_Structure Stspin240_250 Initialization Structure
  * @{
  */
/// Motor driver initialization structure definition  
typedef struct {
  uint32_t bridgePwmFreq[STSPIN240_250_NB_BRIDGES];
  uint32_t refPwmFreq; 
  uint8_t refPwmDc;
  bool dualBridgeEnabled;
} Stspin240_250_Init_t;

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/


/** @defgroup STSPIN240_250_Exported_Functions STSPIN240_250 Exported Functions
  * @{
  */   

void Stspin240_250_AttachErrorHandler(void (*callback)(uint16_t));               //Attach a user callback to the error handler
void Stspin240_250_AttachFlagInterrupt(void (*callback)(void));                  //Attach a user callback to the flag Interrupt
void Stspin240_250_DisableBridge(uint8_t bridgeId);                              //Disable the specified bridge
void Stspin240_250_EnableBridge(uint8_t bridgeId);                               //Enable the specified bridge
uint16_t Stspin240_250_GetBridgeStatus(uint8_t deviceId);                        //Get bridge status
uint32_t Stspin240_250_GetBridgeInputPwmFreq(uint8_t bridgeId);                  // Get the PWM frequency of the bridge input
uint16_t Stspin240_250_GetCurrentSpeed(uint8_t motorId);                         //Return the current speed in pps
motorState_t Stspin240_250_GetDeviceState(uint8_t motorId);                      //Return the device state
motorDir_t Stspin240_250_GetDirection(uint8_t motorId);                          //Get motor direction
uint8_t Stspin240_250_GetDualFullBridgeConfig(void);                             //Get the dual full bridge configuration
uint32_t Stspin240_250_GetFwVersion(void);                                        //Return the FW version
uint16_t Stspin240_250_GetMaxSpeed(uint8_t motorId);                             //Return the max speed in pps
uint8_t Stspin240_250_GetNbDevices(void);                                        //Return the number of devices
uint8_t Stspin240_250_GetRefPwmDc(uint8_t refId);                                //Return the PWM duty cylce of Ref
uint32_t Stspin240_250_GetRefPwmFreq(uint8_t refId);                             //Return the PWM frequency of Ref
motorDrv_t* Stspin240_250_GetMotorHandle(void);                                  //Return handle of the motor driver handle
void Stspin240_250_HardHiz(uint8_t motorId);                                     //Stop the motor and disable the power bridge
void Stspin240_250_HardStop(uint8_t motorId);                                    //Stop the motor without disabling the power bridge
void Stspin240_250_Init(void* pInit);                                            //Start the STSPIN240_250 library
uint16_t Stspin240_250_ReadId(void);                                             //Read Id to get driver instance
void Stspin240_250_ReleaseReset(uint8_t deviceId);                               //Release reset (leave standby state)
void Stspin240_250_Reset(uint8_t deviceId);                                      //Reset (enter standby state)
void Stspin240_250_Run(uint8_t motorId, motorDir_t direction);                   //Run the motor 
void Stspin240_250_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq);    // Set the PWM frequency of the bridge input
void Stspin240_250_SetDirection(uint8_t motorId, motorDir_t dir);                // Set direction pin
void Stspin240_250_SetDualFullBridgeconfig(uint8_t enable);                      //Set  dual bridge configuration
bool Stspin240_250_SetMaxSpeed(uint8_t motorId,uint16_t newMaxSpeed);            //Set the max speed in pps
bool Stspin240_250_SetNbDevices(uint8_t nbDevices);                              //Set the number of devices
void Stspin240_250_SetRefPwmDc(uint8_t refId, uint8_t newDc);                    // Set the PWM duty cycle of Ref pin
void Stspin240_250_SetRefPwmFreq(uint8_t refId, uint32_t newFreq);               // Set the PWM frequency of Ref pin

/**
  * @}
  */

/** @defgroup MotorControl_Board_Linked_Functions MotorControl Board Linked Functions
  * @{
  */   
///Delay of the requested number of milliseconds
extern void Stspin240_250_Board_Delay(uint32_t delay);         
///Disable the bridges
extern void Stspin240_250_Board_DisableBridge(void);     
///Enable the specified bridge
extern void Stspin240_250_Board_EnableBridge(uint8_t addDelay);      
//Get the status of the flag and enable Pin
extern uint8_t Stspin240_250_Board_GetFaultPinState(void);
//Get the status of the reset Pin
extern uint8_t Stspin240_250_Board_GetResetPinState(void);
///Initialise GPIOs used for Stspin240 or Stspin250
extern void Stspin240_250_Board_GpioInit(uint8_t deviceId);   
///Set Briges Inputs PWM frequency and start it
extern void Stspin240_250_Board_PwmSetFreq(uint8_t pwmId, uint32_t newFreq, uint8_t duty); 
///Deinitialise the PWM of the specified bridge input
extern void Stspin240_250_Board_PwmDeInit(uint8_t pwmId);
///Init the PWM of the specified bridge input
extern void Stspin240_250_Board_PwmInit(uint8_t pwmId, uint8_t onlyChannel);    
///Stop the PWM of the specified brigde input
extern void Stspin240_250_Board_PwmStop(uint8_t pwmId);
///Release the reset pin of the Stspin240 or Stspin250
extern void Stspin240_250_Board_ReleaseReset(uint8_t deviceId); 
///Reset the Stspin240 or Stspin250
extern void Stspin240_250_Board_Reset(uint8_t deviceId);  
///Set direction of the specified bridge
extern void Stspin240_250_Board_SetDirectionGpio(uint8_t bridgeId, uint8_t gpioState); 


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

#endif /* #ifndef __STSPIN240_250_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
