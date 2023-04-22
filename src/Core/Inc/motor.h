/**
  ******************************************************************************
  * @file    motor.h
  * @author  IPC Rennes
  * @version V1.7.0
  * @date    March 16th, 2018
  * @brief   This file contains all the functions prototypes for motor drivers.   
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
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */ 

/** @defgroup Motor Motor
  * @{
  */
    
/** @defgroup Motor_Exported_Constants Motor Exported Constants
  * @{
  */   
   
/// boolean for false condition 
#ifndef FALSE
#define FALSE (0)
#endif
/// boolean for true condition 
#ifndef TRUE
#define TRUE  (1)
#endif

   /**
  * @}
  */
   
/** @defgroup Motor_Exported_Types Motor Exported Types
  * @{
  */

/** @defgroup Motor_Boolean_Type Motor Boolean Type
  * @{
  */
///bool Type
typedef uint8_t  bool;
/**
  * @}
  */   
   
/** @defgroup Device_Direction_Options Device Direction Options
  * @{
  */
/// Direction options
typedef enum {
  BACKWARD = 0,
  FORWARD = 1,
  UNKNOW_DIR = ((uint8_t)0xFF)
} motorDir_t;
/**
  * @}
  */
  
/** @defgroup Device_Action_Options Device Action Options
  * @{
  */
/// Action options
typedef enum {
  ACTION_RESET = ((uint8_t)0x00),
  ACTION_COPY  = ((uint8_t)0x08)
} motorAction_t;
/**
  * @}
  */  

/** @defgroup Device_States Device States
  * @{
  */
/// Device states
typedef enum {
  ACCELERATING       = 0,
  DECELERATINGTOSTOP = 1,  
  DECELERATING       = 2, 
  STEADY             = 3,
  INDEX_ACCEL        = 4,
  INDEX_RUN          = 5,
  INDEX_DECEL        = 6,
  INDEX_DWELL        = 7,
  INACTIVE           = 8,
  STANDBY            = 9,
  STANDBYTOINACTIVE  = 10
} motorState_t;
/**
  * @}
  */   

/** @defgroup Device_Step_mode Device Step mode
  * @{
  */
 /// Stepping options 
typedef enum {
  STEP_MODE_FULL   = ((uint8_t)0x00), 
  STEP_MODE_HALF   = ((uint8_t)0x01),
  STEP_MODE_1_4    = ((uint8_t)0x02),
  STEP_MODE_1_8    = ((uint8_t)0x03),
  STEP_MODE_1_16   = ((uint8_t)0x04),
  STEP_MODE_1_32   = ((uint8_t)0x05),
  STEP_MODE_1_64   = ((uint8_t)0x06),
  STEP_MODE_1_128  = ((uint8_t)0x07),
  STEP_MODE_1_256  = ((uint8_t)0x08),
  STEP_MODE_UNKNOW = ((uint8_t)0xFE),
  STEP_MODE_WAVE   = ((uint8_t)0xFF)  
} motorStepMode_t;

/**
  * @}
  */
  
/** @defgroup Decay_mode Decay mode
  * @{
  */
/// Decay Mode 
typedef enum {
  SLOW_DECAY = 0,
  FAST_DECAY = 1,
  UNKNOW_DECAY = ((uint8_t)0xFF)
} motorDecayMode_t;
/**
  * @}
  */
  
/** @defgroup Stop_mode Stop mode
  * @{
  */
/// Stop mode
typedef enum
{ 
  HOLD_MODE = 0,
  HIZ_MODE = 1,
  STANDBY_MODE = 2,
  UNKNOW_STOP_MODE = ((uint8_t)0xFF)
} motorStopMode_t;
/**
  * @}
  */  

/** @defgroup Torque_mode Torque mode
  * @{
  */
/// Torque mode
typedef enum
{ 
  ACC_TORQUE = 0,
  DEC_TORQUE = 1,
  RUN_TORQUE = 2,
  HOLD_TORQUE = 3,
  CURRENT_TORQUE = 4,
  UNKNOW_TORQUE = ((uint8_t)0xFF)
} motorTorqueMode_t;
/**
  * @}
  */  
    
/** @defgroup Dual_Full_Bridge_Configuration Dual Full Bridge Configuration
  * @{
  */
///Dual full bridge configurations for brush DC motors
typedef enum {
  PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B = 0,
  PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B = 1,
  PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B = 2,
  PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B = 3,
  PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B = 4,
  PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B = 5,
  PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B = 6,
  PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B = 7,
  PARALLELING_IN1A_IN2A__IN1B_IN2B__1_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B = 8,
  PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR = 9,
  PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A = 10,
  PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR = 11,
  PARALLELING_ALL_WITH_IN1A___1_UNDIR_MOTOR = 12,
  PARALLELING_END_ENUM = 13 
} dualFullBridgeConfig_t;
/**
  * @}
  */

/** @defgroup Motor_Driver_Structure Motor Driver Structure
  * @{
  */
/// Motor driver structure definition  
typedef struct
{
  /// Function pointer to Init
  void (*Init)(void*);
  /// Function pointer to ReadID
  uint16_t (*ReadID)(void);
  /// Function pointer to AttachErrorHandler
  void(*AttachErrorHandler)(void (*callback)(uint16_t));
  /// Function pointer to AttachFlagInterrupt
  void (*AttachFlagInterrupt)(void (*callback)(void));
  /// Function pointer to AttachBusyInterrupt
  void (*AttachBusyInterrupt)(void (*callback)(void));
  /// Function pointer to FlagInterruptHandler
  void (*FlagInterruptHandler)(void);
  /// Function pointer to GetAcceleration
  uint16_t (*GetAcceleration)(uint8_t);
  /// Function pointer to GetCurrentSpeed
  uint16_t (*GetCurrentSpeed)(uint8_t);
  /// Function pointer to GetDeceleration
  uint16_t (*GetDeceleration)(uint8_t);
  /// Function pointer to GetDeviceState
  motorState_t(*GetDeviceState)(uint8_t);
  /// Function pointer to GetFwVersion
  uint32_t (*GetFwVersion)(void);
  /// Function pointer to GetMark
  int32_t (*GetMark)(uint8_t);
  /// Function pointer to GetMaxSpeed
  uint16_t (*GetMaxSpeed)(uint8_t);
  /// Function pointer to GetMinSpeed
  uint16_t (*GetMinSpeed)(uint8_t);
  /// Function pointer to GetPosition
  int32_t (*GetPosition)(uint8_t);
  /// Function pointer to GoHome
  void (*GoHome)(uint8_t);
  /// Function pointer to GoMark
  void (*GoMark)(uint8_t);
  /// Function pointer to GoTo
  void (*GoTo)(uint8_t, int32_t);
  /// Function pointer to HardStop
  void (*HardStop)(uint8_t);
  /// Function pointer to Move
  void (*Move)(uint8_t, motorDir_t, uint32_t );
  /// Function pointer to ResetAllDevices
  void (*ResetAllDevices)(void);
  /// Function pointer to Run
  void (*Run)(uint8_t, motorDir_t);
  /// Function pointer to SetAcceleration
  bool(*SetAcceleration)(uint8_t ,uint16_t );
  /// Function pointer to SetDeceleration
  bool(*SetDeceleration)(uint8_t , uint16_t );
  /// Function pointer to SetHome
  void (*SetHome)(uint8_t, int32_t);
  /// Function pointer to SetMark
  void (*SetMark)(uint8_t, int32_t);
  /// Function pointer to SetMaxSpeed
  bool (*SetMaxSpeed)(uint8_t, uint16_t );
  /// Function pointer to SetMinSpeed
  bool (*SetMinSpeed)(uint8_t, uint16_t );
  /// Function pointer to SoftStop
  bool (*SoftStop)(uint8_t);
  /// Function pointer to StepClockHandler
  void (*StepClockHandler)(uint8_t deviceId);
  /// Function pointer to WaitWhileActive
  void (*WaitWhileActive)(uint8_t);
  /// Function pointer to CmdDisable
  void (*CmdDisable)(uint8_t);
  /// Function pointer to CmdEnable
  void (*CmdEnable)(uint8_t);
  /// Function pointer to CmdGetParam
  uint32_t (*CmdGetParam)(uint8_t, uint32_t);
  /// Function pointer to CmdGetStatus
  uint16_t (*CmdGetStatus)(uint8_t);
  /// Function pointer to CmdNop
  void (*CmdNop)(uint8_t);
  /// Function pointer to CmdSetParam
  void (*CmdSetParam)(uint8_t, uint32_t, uint32_t);
  /// Function pointer to ReadStatusRegister
  uint16_t (*ReadStatusRegister)(uint8_t);
  /// Function pointer to ReleaseReset
  void (*ReleaseReset)(uint8_t);
  /// Function pointer to Reset
  void (*Reset)(uint8_t);
  /// Function pointer to SelectStepMode
  bool (*SelectStepMode)(uint8_t deviceId, motorStepMode_t);
  /// Function pointer to SetDirection
  void (*SetDirection)(uint8_t, motorDir_t);
  /// Function pointer to CmdGoToDir
  void (*CmdGoToDir)(uint8_t, motorDir_t, int32_t);
  /// Function pointer to CheckBusyHw
  uint8_t (*CheckBusyHw)(void);
  /// Function pointer to CheckStatusHw
  uint8_t (*CheckStatusHw)(void);
  /// Function pointer to CmdGoUntil
  void (*CmdGoUntil)(uint8_t, motorAction_t, motorDir_t, uint32_t);
  /// Function pointer to CmdHardHiZ
  void (*CmdHardHiZ)(uint8_t);
  /// Function pointer to CmdReleaseSw
  void (*CmdReleaseSw)(uint8_t, motorAction_t, motorDir_t);
  /// Function pointer to CmdResetDevice
  void (*CmdResetDevice)(uint8_t);
  /// Function pointer to CmdResetPos
  void (*CmdResetPos)(uint8_t);
  /// Function pointer to CmdRun
  void (*CmdRun)(uint8_t, motorDir_t, uint32_t);
  /// Function pointer to CmdSoftHiZ
  void (*CmdSoftHiZ)(uint8_t);
  /// Function pointer to CmdStepClock
  void (*CmdStepClock)(uint8_t, motorDir_t);
  /// Function pointer to FetchAndClearAllStatus
  void (*FetchAndClearAllStatus)(void);
  /// Function pointer to GetFetchedStatus
  uint16_t (*GetFetchedStatus)(uint8_t);
  /// Function pointer to GetNbDevices
  uint8_t (*GetNbDevices)(void);
  /// Function pointer to IsDeviceBusy
  bool (*IsDeviceBusy)(uint8_t);
  /// Function pointer to SendQueuedCommands
  void (*SendQueuedCommands)(void);
  /// Function pointer to QueueCommands
  void (*QueueCommands)(uint8_t, uint8_t, int32_t);
  /// Function pointer to WaitForAllDevicesNotBusy
  void (*WaitForAllDevicesNotBusy)(void);
  /// Function pointer to ErrorHandler
  void (*ErrorHandler)(uint16_t);
  /// Function pointer to BusyInterruptHandler
  void (*BusyInterruptHandler)(void);
  /// Function pointer to CmdSoftStop
  void (*CmdSoftStop)(uint8_t);
  /// Function pointer to StartStepClock
  void (*StartStepClock)(uint16_t);
  /// Function pointer to StopStepClock
  void (*StopStepClock)(void);
  /// Function pointer to SetDualFullBridgeConfig
  void (*SetDualFullBridgeConfig)(uint8_t);
  /// Function pointer to GetBridgeInputPwmFreq
  uint32_t (*GetBridgeInputPwmFreq)(uint8_t);
  /// Function pointer to SetBridgeInputPwmFreq
  void (*SetBridgeInputPwmFreq)(uint8_t, uint32_t);
  /// Function pointer to SetStopMode
  void (*SetStopMode)(uint8_t, motorStopMode_t);
  /// Function pointer to GetStopMode
  motorStopMode_t (*GetStopMode)(uint8_t);
  /// Function pointer to SetDecayMode
  void (*SetDecayMode)(uint8_t, motorDecayMode_t);
  /// Function pointer to GetDecayMode
  motorDecayMode_t (*GetDecayMode)(uint8_t);
  /// Function pointer to GetStepMode
  motorStepMode_t (*GetStepMode)(uint8_t);
  /// Function pointer to GetDirection
  motorDir_t (*GetDirection)(uint8_t);
  /// Function pointer to ExitDeviceFromReset
  void (*ExitDeviceFromReset)(uint8_t);
  /// Function pointer to SetTorque
  void (*SetTorque)(uint8_t, motorTorqueMode_t, uint8_t);
  /// Function pointer to GetTorque
  uint8_t (*GetTorque)(uint8_t, motorTorqueMode_t);
  /// Function pointer to SetVRefFreq
  void (*SetRefFreq)(uint8_t, uint32_t);
  /// Function pointer to GetVRefFreq
  uint32_t (*GetRefFreq)(uint8_t);
  /// Function pointer to SetVRefDc
  void (*SetRefDc)(uint8_t, uint8_t);
  /// Function pointer to GetVRefDc
  uint8_t (*GetRefDc)(uint8_t);
  /// Function pointer to SetNbDevices
  bool (*SetNbDevices)(uint8_t);
  /// Function pointer to SetAnalogValue
  bool (*SetAnalogValue)(uint8_t, uint32_t, float);
  /// Function pointer to GetAnalogValue
  float (*GetAnalogValue)(uint8_t, uint32_t);
  /// Function pointer to SetTorqueBoostEnable
  void (*SetTorqueBoostEnable) (uint8_t, bool);
  /// Function pointer to GetTorqueBoostEnable
  bool (*GetTorqueBoostEnable) (uint8_t);
  /// Function pointer to SetTorqueBoostThreshold
  void (*SetTorqueBoostThreshold) (uint8_t, uint16_t);
  /// Function pointer to GetTorqueBoostThreshold
  uint16_t (*GetTorqueBoostThreshold) (uint8_t);
  /// Function pointer to GetDualFullBridgeConfig
  uint8_t (*GetDualFullBridgeConfig) (void);  
}motorDrv_t;
      
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

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
