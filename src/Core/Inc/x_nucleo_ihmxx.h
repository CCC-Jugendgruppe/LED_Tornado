/**
  ******************************************************************************
  * @file    x_nucleo_ihmxx.h
  * @author  IPC Rennes
  * @version V1.7.0
  * @date    March 16th, 2018
  * @brief   This file provides common definitions for motor control 
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
#ifndef X_NUCLEO_IHMXX_H
#define X_NUCLEO_IHMXX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "motor.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup MOTOR_CONTROL
  * @{
  */

/** @defgroup MOTOR_CONTROL_Exported_Types MOTOR CONTROL Exported Types
  * @{
  */



/**
  * @}
  */

/** @defgroup MOTOR_CONTROL_Exported_Constants MOTOR CONTROL Exported Constants
  * @{
  */
/// Motor control error tag (used when trying to call undefined functions via motorDrvHandle)
#define MOTOR_CONTROL_ERROR_TAG   (0x0800)      
///Motor control board id for L6474
#define BSP_MOTOR_CONTROL_BOARD_ID_L6474  (6474)
 ///Motor control board id for L6470
 #define BSP_MOTOR_CONTROL_BOARD_ID_L6470  (6470)
 ///Motor control board id for L6472
 #define BSP_MOTOR_CONTROL_BOARD_ID_L6472  (6472)
 ///Motor control board id for L6480
 #define BSP_MOTOR_CONTROL_BOARD_ID_L6480  (6480)
 ///Motor control board id for L6482
 #define BSP_MOTOR_CONTROL_BOARD_ID_L6482  (6482)
 ///Motor control board id for L6474
 #define BSP_MOTOR_CONTROL_BOARD_ID_L6474  (6474)
///Motor control board id for Powerstep01
#define BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01 (0001)
///Motor control board id for L6206
#define BSP_MOTOR_CONTROL_BOARD_ID_L6206 (6206)
///Motor control board id for L6208
#define BSP_MOTOR_CONTROL_BOARD_ID_L6208 (6208)
///Motor control board id for STSPIN220
#define BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220 (220)
///Motor control board id for STSPIN240
#define BSP_MOTOR_CONTROL_BOARD_ID_STSPIN240 (240)   
///Motor control board id for STSPIN250
#define BSP_MOTOR_CONTROL_BOARD_ID_STSPIN250 (250) 
/**
  * @}
  */


/** @defgroup MOTOR_CONTROL_Exported_Macros MOTOR CONTROL Exported Macros
  * @{
  */
#if  defined ( __GNUC__ )
  #ifndef __weak
    #define __weak   __attribute__((weak))
  #endif /* __weak */
#endif /* __GNUC__ */
/**
  * @}
  */

/** @defgroup MOTOR_CONTROL_Weak_Function_Prototypes MOTOR CONTROL Weak Function Prototypes
  * @{
  */
__weak motorDrv_t* L6474_GetMotorHandle(void);
__weak motorDrv_t* l647x_GetMotorHandle(void);
__weak motorDrv_t* l648x_GetMotorHandle(void);
__weak motorDrv_t* Powerstep01_GetMotorHandle(void);
__weak motorDrv_t* L6206_GetMotorHandle(void);
__weak motorDrv_t* L6208_GetMotorHandle(void);
__weak motorDrv_t* Stspin220_GetMotorHandle(void);
__weak motorDrv_t* Stspin240_250_GetMotorHandle(void);
/**
  * @}
  */   
   
/** @defgroup MOTOR_CONTROL_Exported_Functions MOTOR CONTROL Exported Functions
  * @{
  */
void BSP_MotorControl_AttachErrorHandler(void (*callback)(uint16_t));
void BSP_MotorControl_AttachFlagInterrupt(void (*callback)(void));
void BSP_MotorControl_AttachBusyInterrupt(void (*callback)(void));
void BSP_MotorControl_ErrorHandler(uint16_t error);
void BSP_MotorControl_Init(uint16_t id, void* initDeviceParameters); 
void BSP_MotorControl_FlagInterruptHandler(void);
uint16_t BSP_MotorControl_GetAcceleration(uint8_t deviceId); 
uint16_t BSP_MotorControl_GetBoardId(void);
uint16_t BSP_MotorControl_GetCurrentSpeed(uint8_t deviceId); 
uint16_t BSP_MotorControl_GetDeceleration(uint8_t deviceId); 
motorState_t BSP_MotorControl_GetDeviceState(uint8_t deviceId); 
uint32_t BSP_MotorControl_GetFwVersion(void); 
int32_t BSP_MotorControl_GetMark(uint8_t deviceId); 
uint16_t BSP_MotorControl_GetMaxSpeed(uint8_t deviceId); 
uint16_t BSP_MotorControl_GetMinSpeed(uint8_t deviceId); 
int32_t BSP_MotorControl_GetPosition(uint8_t deviceId); 
void BSP_MotorControl_GoHome(uint8_t deviceId); 
void BSP_MotorControl_GoMark(uint8_t deviceId); 
void BSP_MotorControl_GoTo(uint8_t deviceId, int32_t targetPosition); 
void BSP_MotorControl_HardStop(uint8_t deviceId); 
void BSP_MotorControl_Move(uint8_t deviceId, motorDir_t direction, uint32_t stepCount); 
void BSP_MotorControl_ResetAllDevices(void); 
void BSP_MotorControl_Run(uint8_t deviceId, motorDir_t direction); 
bool BSP_MotorControl_SetAcceleration(uint8_t deviceId,uint16_t newAcc); 
bool BSP_MotorControl_SetDeceleration(uint8_t deviceId, uint16_t newDec); 
void BSP_MotorControl_SetHome(uint8_t deviceId, int32_t homePosition);
void BSP_MotorControl_SetMark(uint8_t deviceId, int32_t markPosition);
bool BSP_MotorControl_SetMaxSpeed(uint8_t deviceId, uint16_t newMaxSpeed); 
bool BSP_MotorControl_SetMinSpeed(uint8_t deviceId, uint16_t newMinSpeed); 
bool BSP_MotorControl_SoftStop(uint8_t deviceId); 
void BSP_MotorControl_StepClockHandler(uint8_t deviceId); 
void BSP_MotorControl_WaitWhileActive(uint8_t deviceId); 
void BSP_MotorControl_CmdDisable(uint8_t deviceId); 
void BSP_MotorControl_CmdEnable(uint8_t deviceId); 
uint32_t BSP_MotorControl_CmdGetParam(uint8_t deviceId, uint32_t param); 
uint16_t BSP_MotorControl_CmdGetStatus(uint8_t deviceId);
void BSP_MotorControl_CmdNop(uint8_t deviceId); 
void BSP_MotorControl_CmdSetParam(uint8_t deviceId, uint32_t param, uint32_t value);
uint16_t BSP_MotorControl_ReadStatusRegister(uint8_t deviceId); 
void BSP_MotorControl_ReleaseReset(uint8_t deviceId);
void BSP_MotorControl_Reset(uint8_t deviceId); 
bool BSP_MotorControl_SelectStepMode(uint8_t deviceId,  motorStepMode_t stepMode); 
void BSP_MotorControl_SetDirection(uint8_t deviceId, motorDir_t dir);
void BSP_MotorControl_CmdGoToDir(uint8_t deviceId, motorDir_t dir, int32_t abs_pos);
uint8_t BSP_MotorControl_CheckBusyHw(void);
uint8_t BSP_MotorControl_CheckStatusHw(void);
void BSP_MotorControl_CmdGoUntil(uint8_t deviceId, motorAction_t action, motorDir_t dir, uint32_t speed);
void BSP_MotorControl_CmdHardHiZ(uint8_t deviceId);
void BSP_MotorControl_CmdReleaseSw(uint8_t deviceId, motorAction_t action, motorDir_t dir);
void BSP_MotorControl_CmdResetDevice(uint8_t deviceId);
void BSP_MotorControl_CmdResetPos(uint8_t deviceId);
void BSP_MotorControl_CmdRun(uint8_t deviceId, motorDir_t dir, uint32_t speed);
void BSP_MotorControl_CmdSoftHiZ(uint8_t deviceId);
void BSP_MotorControl_CmdStepClock(uint8_t deviceId, motorDir_t dir);
void BSP_MotorControl_FetchAndClearAllStatus(void);
uint16_t BSP_MotorControl_GetFetchedStatus(uint8_t deviceId);
uint8_t BSP_MotorControl_GetNbDevices(void);
bool BSP_MotorControl_IsDeviceBusy(uint8_t deviceId);
void BSP_MotorControl_SendQueuedCommands(void);
void BSP_MotorControl_QueueCommands(uint8_t deviceId, uint8_t command, int32_t value);
void BSP_MotorControl_WaitForAllDevicesNotBusy(void);
void BSP_MotorControl_BusyInterruptHandler(void);
void BSP_MotorControl_CmdSoftStop(uint8_t deviceId);
void BSP_MotorControl_StartStepClock(uint16_t newFreq);
void BSP_MotorControl_StopStepClock(void);
void BSP_MotorControl_SetDualFullBridgeConfig(uint8_t config);
uint32_t BSP_MotorControl_GetBridgeInputPwmFreq(uint8_t bridgeId);
void BSP_MotorControl_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq);
void BSP_MotorControl_SetStopMode(uint8_t deviceId, motorStopMode_t stopMode);
motorStopMode_t BSP_MotorControl_GetStopMode(uint8_t deviceId);
void BSP_MotorControl_SetDecayMode(uint8_t deviceId, motorDecayMode_t decayMode);
motorDecayMode_t BSP_MotorControl_GetDecayMode(uint8_t deviceId);
motorStepMode_t BSP_MotorControl_GetStepMode(uint8_t deviceId);
motorDir_t BSP_MotorControl_GetDirection(uint8_t deviceId);
void BSP_MotorControl_ExitDeviceFromReset(uint8_t deviceId);
uint8_t BSP_MotorControl_GetTorque(uint8_t deviceId, motorTorqueMode_t torqueMode);
void BSP_MotorControl_SetTorque(uint8_t deviceId, motorTorqueMode_t torqueMode, uint8_t torqueValue);
void BSP_MotorControl_SetRefFreq(uint8_t refId, uint32_t newFreq);
uint32_t BSP_MotorControl_GetRefFreq(uint8_t refId);
void BSP_MotorControl_SetRefDc(uint8_t refId, uint8_t newDc);
uint8_t BSP_MotorControl_GetRefDc(uint8_t refId);
bool BSP_MotorControl_SetNbDevices(uint16_t id, uint8_t nbDevices);
bool BSP_MotorControl_SetAnalogValue(uint8_t deviceId, uint32_t param, float value);
float BSP_MotorControl_GetAnalogValue(uint8_t deviceId, uint32_t param);
void BSP_MotorControl_SetTorqueBoostEnable(uint8_t deviceId, bool enable);
bool BSP_MotorControl_GetTorqueBoostEnable(uint8_t deviceId);
void BSP_MotorControl_SetTorqueBoostThreshold(uint8_t deviceId, uint16_t speedThreshold);
uint16_t BSP_MotorControl_GetTorqueBoostThreshold(uint8_t deviceId);
uint8_t BSP_MotorControl_GetDualFullBridgeConfig(void);
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

#endif /* X_NUCLEO_IHMXX_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
