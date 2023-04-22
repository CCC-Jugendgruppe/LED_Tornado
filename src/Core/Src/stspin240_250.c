/**
  ******************************************************************************
  * @file    stspin240_250.c
  * @author  IPC Rennes
  * @version V1.2.0
  * @date    September 11th, 2017
  * @brief   Stspin240 (low voltage dual brush DC motor driver) and 
  *          Stspin250 driver (low voltage  brush DC motor driver)
  * @note     (C) COPYRIGHT 2017 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include "stspin240_250.h"

/* Private constants  ---------------------------------------------------------*/
    
/** @addtogroup BSP
  * @{
  */   
   
/** @defgroup STSPIN240_250 STSPIN240_250
  * @{
  */   

/* Private constants ---------------------------------------------------------*/    

/** @defgroup STSPIN240_250_Private_Constants STSPIN240_250 Private Constants
  * @{
  */   

/// The Number of Stspin240 and Stspin250 devices required for initialisation is not supported
#define STSPIN240_250_ERROR_0   (0xC000)   
/// Error: Access a motor index greater than the one of the current brigde configuration
#define STSPIN240_250_ERROR_1   (0xC001)
/// Error: Use of a bridgeId greater than BRIDGE_B
#define STSPIN240_250_ERROR_2   (0xC002)    
    
/// Maximum frequency of the PWMs in Hz
#define STSPIN240_250_MAX_PWM_FREQ   (100000)

/// Minimum frequency of the PWMs in Hz
#define STSPIN240_250_MIN_PWM_FREQ   (2)

/// Bridge A
#define BRIDGE_A         (0)
#ifdef STSPIN_240
/// Bridge B
#define BRIDGE_B         (1)
#endif
    
/// PWM id for PWM_A
#define PWM_A         (0)
#ifdef STSPIN_240
/// PWM id for PWM_B
#define PWM_B         (1)
#endif    
/// PWM id for PWM_REF    
#define PWM_REF       (2)    
/**
  * @}
  */ 
    
/* Private variables ---------------------------------------------------------*/

/** @defgroup STSPIN240_250_Private_Variables STSPIN240_250 Private Variables
  * @{
  */       
    
/// Function pointer to flag interrupt call back
void (*flagInterruptCallback)(void);

/// Function pointer to error handler call back
void (*errorHandlerCallback)(uint16_t);

/// number of Stspin240 or Stspin250 devices
static volatile uint8_t numberOfDevices;

/// Stspin240 or Stspin250 driver instance
static uint16_t stspin240_250DriverInstance = 0;

/// Stspin240 or Stspin250 Device Paramaters structure
deviceParams_t devicePrm;

/**
  * @}
  */ 

/* Private constans ---------------------------------------------------------*/

/** @defgroup STSPIN240_250_Private_Constants STSPIN240_250 Private Constants
  * @{
  */       
    
/// Max numbers of supported motors depending of bridge configuration (mono or dual)
static uint8_t stspin240_250ArrayNbMaxMotorsByConfig[2] = {1,2};

/**
  * @}
  */ 

/* Private function prototypes -----------------------------------------------*/

/** @defgroup STSPIN240_250_Private_functions STSPIN240_250 Private functions
  * @{
  */  
void Stspin240_250_ErrorHandler(uint16_t error);
void Stspin240_250_FaultInterruptHandler(void);                      
uint8_t Stspin240_250_GetResetState(void);
void Stspin240_250_SetDeviceParamsToPredefinedValues(void);
void Stspin240_250_SetDeviceParamsToGivenValues(uint8_t deviceId, Stspin240_250_Init_t *pInitPrm);
/**
  * @}
  */ 

/** @defgroup STSPIN240_250_Exported_Variables STSPIN240_250 Exported Variables
  * @{
  */       

/// Stspin240 or Stspin250 motor driver functions pointer structure 
motorDrv_t   stspin240_250Drv = 
{
  Stspin240_250_Init,                      //void (*Init)(void*);
  Stspin240_250_ReadId,                    //uint16_t (*ReadID)(void);
  Stspin240_250_AttachErrorHandler,        //void (*AttachErrorHandler)(void (*callback)(uint16_t));
  Stspin240_250_AttachFlagInterrupt,       //void (*AttachFlagInterrupt)(void (*callback)(void));
  0,                                       //void (*AttachBusyInterrupt)(void (*callback)(void));
  Stspin240_250_FaultInterruptHandler,     //void (*FlagInterruptHandler)(void);
  0,                                       //uint16_t (*GetAcceleration)(uint8_t);
  Stspin240_250_GetCurrentSpeed,           //uint16_t (*GetCurrentSpeed)(uint8_t);
  0,                                       //uint16_t (*GetDeceleration)(uint8_t);
  Stspin240_250_GetDeviceState,            //motorState_t(*GetDeviceState)(uint8_t);
  Stspin240_250_GetFwVersion,              //uint32_t (*GetFwVersion)(void);
  0,                                       //int32_t (*GetMark)(uint8_t);
  Stspin240_250_GetMaxSpeed,               //uint16_t (*GetMaxSpeed)(uint8_t);
  0,                                       //uint16_t (*GetMinSpeed)(uint8_t);
  0,                                       //int32_t (*GetPosition)(uint8_t);
  0,                                       //void (*GoHome)(uint8_t);
  0,                                       //void (*GoMark)(uint8_t);
  0,                                       //void (*GoTo)(uint8_t, int32_t);
  Stspin240_250_HardStop,                  //void (*HardStop)(uint8_t);
  0,                                       //void (*Move)(uint8_t, motorDir_t, uint32_t );
  0,                                       //void (*ResetAllDevices)(void);
  Stspin240_250_Run,                       //void (*Run)(uint8_t, motorDir_t);
  0,                                       //bool(*SetAcceleration)(uint8_t ,uint16_t );
  0,                                       //bool(*SetDeceleration)(uint8_t , uint16_t );
  0,                                       //void (*SetHome)(uint8_t);
  0,                                       //void (*SetMark)(uint8_t);
  Stspin240_250_SetMaxSpeed,               //bool (*SetMaxSpeed)(uint8_t, uint16_t );
  0,                                       //bool (*SetMinSpeed)(uint8_t, uint16_t );
  0,                                       //bool (*SoftStop)(uint8_t);
  0,                                       //void (*StepClockHandler)(uint8_t deviceId);
  0,                                       //void (*WaitWhileActive)(uint8_t);
  Stspin240_250_DisableBridge,             //void (*CmdDisable)(uint8_t);
  Stspin240_250_EnableBridge,              //void (*CmdEnable)(uint8_t);
  0,                                       //uint32_t (*CmdGetParam)(uint8_t, uint32_t);
  Stspin240_250_GetBridgeStatus,           //uint16_t (*CmdGetStatus)(uint8_t);
  0,                                       //void (*CmdNop)(uint8_t);
  0,                                       //void (*CmdSetParam)(uint8_t, uint32_t, uint32_t);
  0,                                       //uint16_t (*ReadStatusRegister)(uint8_t);
  Stspin240_250_ReleaseReset,              //void (*ReleaseReset)(void);
  Stspin240_250_Reset,                     //void (*Reset)(void);
  0,                                       //void (*SelectStepMode)(uint8_t deviceId, motorStepMode_t);
  Stspin240_250_SetDirection,              //void (*SetDirection)(uint8_t, motorDir_t);
  0,                                       //void (*CmdGoToDir)(uint8_t, motorDir_t, int32_t);
  0,                                       //uint8_t (*CheckBusyHw)(void);
  0,                                       //uint8_t (*CheckStatusHw)(void);
  0,                                       //void (*CmdGoUntil)(uint8_t, motorAction_t, motorDir_t, uint32_t);
  Stspin240_250_HardHiz,                   //void (*CmdHardHiZ)(uint8_t);
  0,                                       //void (*CmdReleaseSw)(uint8_t, motorAction_t, motorDir_t);
  0,                                       //void (*CmdResetDevice)(uint8_t);
  0,                                       //void (*CmdResetPos)(uint8_t);
  0,                                       //void (*CmdRun)(uint8_t, motorDir_t, uint32_t);
  0,                                       //void (*CmdSoftHiZ)(uint8_t);
  0,                                       //void (*CmdStepClock)(uint8_t, motorDir_t);
  0,                                       //void (*FetchAndClearAllStatus)(void);
  0,                                       //uint16_t (*GetFetchedStatus)(uint8_t);
  Stspin240_250_GetNbDevices,              //uint8_t (*GetNbDevices)(void);
  0,                                       //bool (*IsDeviceBusy)(uint8_t);
  0,                                       //void (*SendQueuedCommands)(void);
  0,                                       //void (*QueueCommands)(uint8_t, uint8_t, int32_t);
  0,                                       //void (*WaitForAllDevicesNotBusy)(void);
  Stspin240_250_ErrorHandler,              //void (*ErrorHandler)(uint16_t);
  0,                                       //void (*BusyInterruptHandler)(void);
  0,                                       //void (*CmdSoftStop)(uint8_t);
  0,                                       //void (*StartStepClock)(uint16_t);
  0,                                       //void (*StopStepClock)(void);
  Stspin240_250_SetDualFullBridgeconfig,   //void (*SetDualFullBridgeConfig)(uint8_t);
  Stspin240_250_GetBridgeInputPwmFreq,     //uint32_t (*GetBridgeInputPwmFreq)(uint8_t);
  Stspin240_250_SetBridgeInputPwmFreq,     //void (*SetBridgeInputPwmFreq)(uint8_t, uint32_t);
  0,                                       //void (*SetStopMode)(uint8_t, motorStopMode_t);
  0,                                       //motorStopMode_t (*GetStopMode)(uint8_t);
  0,                                       //void (*SetDecayMode)(uint8_t, motorDecayMode_t);
  0,                                       //motorDecayMode_t (*GetDecayMode)(uint8_t);
  0,                                       //motorStepMode_t (*GetStepMode)(uint8_t);
  Stspin240_250_GetDirection,              //motorDir_t (*GetDirection)(uint8_t);
  0,                                       //void (*ExitDeviceFromReset)(uint8_t);
  0,                                       //void (*SetTorque)(uint8_t, motorTorqueMode_t, uint8_t);
  0,                                       //uint8_t (*GetTorque)(uint8_t, motorTorqueMode_t);
  Stspin240_250_SetRefPwmFreq,             //void (*SetRefFreq)(uint8_t, uint32_t);
  Stspin240_250_GetRefPwmFreq,             //uint32_t (*GetRefFreq)(uint8_t);
  Stspin240_250_SetRefPwmDc,               //void (*SetRefDc)(uint8_t, uint8_t);
  Stspin240_250_GetRefPwmDc,                //uint8_t (*GetRefDc)(uint8_t);
  Stspin240_250_SetNbDevices,              //bool (*SetNbDevices)(uint8_t);
  0,                                       //bool (*SetAnalogValue)(uint8_t, uint32_t, float);
  0,                                       //float (*GetAnalogValue )(uint8_t, uint32_t);
  0,                                       //void (*SetTorqueBoostEnable) (uint8_t, bool);
  0,                                       //bool (*GetTorqueBoostEnable) (uint8_t);
  0,                                       //void (*SetTorqueBoostThreshold) (uint8_t, uint16_t);
  0,                                       //uint16_t (*GetTorqueBoostThreshold) (uint8_t);
  Stspin240_250_GetDualFullBridgeConfig    //uint8_t (*GetDualFullBridgeConfig )(void);    
};
/**
  * @}
  */ 

/** @defgroup STSPIN240_250_Exported_Functions STSPIN240_250 Exported Functions
  * @{
  */   

/******************************************************//**
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error Hanlder
 * @retval None
 **********************************************************/
void Stspin240_250_AttachErrorHandler(void (*callback)(uint16_t))
{
  errorHandlerCallback = (void (*)(uint16_t))callback;
}

/******************************************************//**
 * @brief  Attaches a user callback to the flag Interrupt
 * The call back will be then called each time the status 
 * flag pin will be pulled down due to the occurrence of 
 * a programmed alarms ( OCD, thermal alert)
 * @param[in] callback Name of the callback to attach 
 * to the Flag Interrupt
 * @retval None
 **********************************************************/
void Stspin240_250_AttachFlagInterrupt(void (*callback)(void))
{
  flagInterruptCallback = (void (*)())callback;
}

/******************************************************//**
 * @brief Disable the specified bridge
 * @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 * @retval None
 * @note  Bridge A and bridge B share the same enable pin. 
 * When bridge A is disabled, bridge B is disabled and 
 * reversely
 **********************************************************/
void Stspin240_250_DisableBridge(uint8_t bridgeId)
{
  Stspin240_250_Board_DisableBridge();
  devicePrm.bridgeEnabled[BRIDGE_A] = FALSE;
#ifdef STSPIN_240
  devicePrm.bridgeEnabled[BRIDGE_B] = FALSE;
#endif  
}

/******************************************************//**
 * @brief Enable the specified bridge
 * @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 * @retval None
 * @note  Bridge A and bridge B share the same enable pin. 
 * When bridge A is enabled, bridge B is enabled and 
 * reversely
 **********************************************************/
void Stspin240_250_EnableBridge(uint8_t bridgeId)
{
  devicePrm.bridgeEnabled[BRIDGE_A] = TRUE;
#ifdef STSPIN_240
  devicePrm.bridgeEnabled[BRIDGE_B] = TRUE;  
#endif  
  Stspin240_250_Board_EnableBridge(1);
}

/******************************************************//**
 * @brief Set the dual bridge configuration mode
 * @param[in] enable 0 to disable, 
 *                   1 to enable (not supported by STSPIN250)
 * @retval None
  **********************************************************/
void Stspin240_250_SetDualFullBridgeconfig(uint8_t enable)
{
  devicePrm.dualBridgeEnabled = enable;

  /* Check reset pin state*/
  if (Stspin240_250_GetResetState() != 0)
  {
    Stspin240_250_Reset(0);
    Stspin240_250_ReleaseReset(0);
  }
}
/******************************************************//**
 * @brief Get the motor current direction
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1) 
 * @retval direction
 **********************************************************/
motorDir_t Stspin240_250_GetDirection(uint8_t motorId)
{
  if (motorId >= stspin240_250ArrayNbMaxMotorsByConfig[devicePrm.dualBridgeEnabled])
  {
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_1);
  }
  
  return devicePrm.direction[motorId];
}

/******************************************************//**
 * @brief Starts the Stspin240_250 library
 * @param[in] pInit pointer to the initialization data
 * @retval None
 * @note  Only one device is currently supported
 **********************************************************/
void Stspin240_250_Init(void* pInit)
{
  /* Initialise the GPIOs */
  Stspin240_250_Board_GpioInit(stspin240_250DriverInstance);
  
  if (pInit == 0)
  {
    /* Set context variables to the predefined values from stspin240_250_target_config.h */
    Stspin240_250_SetDeviceParamsToPredefinedValues();
  }
  else
  {
    /* Set context variables to the predefined values from stspin240_250_target_config.h */
    Stspin240_250_SetDeviceParamsToGivenValues(stspin240_250DriverInstance, pInit);
  }
  /* Initialise PWM for REF pin */
  Stspin240_250_Board_PwmInit(PWM_REF, 0);
  
  /* Deinit PWM input bridges by stopping them */  
  Stspin240_250_Board_PwmStop(0);
#ifdef STSPIN_240
  Stspin240_250_Board_PwmStop(1);
#endif  
  /* Initialise input PWM of bridges*/
  Stspin240_250_SetDualFullBridgeconfig(devicePrm.dualBridgeEnabled);

  stspin240_250DriverInstance++;
  
  if (stspin240_250DriverInstance > MAX_NUMBER_OF_DEVICES)
  {
    /* Initialization Error */
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_0);
  } 
}

/******************************************************//**
 * @brief  Get the PWM frequency of the specified bridge 
 * @param[in] bridgeId 0 for bridge A, 1 for bridge B
 * @retval Freq in Hz
 **********************************************************/
uint32_t Stspin240_250_GetBridgeInputPwmFreq(uint8_t bridgeId)
{                                                  
#ifdef STSPIN_240
  if (bridgeId > BRIDGE_B)
#else
  if (bridgeId > BRIDGE_A)
#endif    
  {
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_2);
  }    
  
  return (devicePrm.bridgePwmFreq[(bridgeId << 1)]);
}

/******************************************************//**
 * @brief  Get the status of the bridge enabling of the corresponding bridge
 * @param[in] bridgeId from 0 for bridge A to 1 for bridge B
 * @retval State of the Enable&Fault pin (shared for bridge A and B)
  **********************************************************/
uint16_t Stspin240_250_GetBridgeStatus(uint8_t bridgeId)
{
  uint16_t status = (uint16_t)Stspin240_250_Board_GetFaultPinState();
  
  return (status);
}

/******************************************************//**
 * @brief  Returns the current speed of the specified motor
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1) 
 * @retval current speed in % from 0 to 100
 **********************************************************/
uint16_t Stspin240_250_GetCurrentSpeed(uint8_t motorId)
{                                                  
  uint16_t speed = 0;

  if (motorId >= stspin240_250ArrayNbMaxMotorsByConfig[devicePrm.dualBridgeEnabled])
  {
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_1);
  }
  else if (devicePrm.motionState[motorId] != INACTIVE)
  {
    speed = devicePrm.speed[motorId];
  }
  
  return (speed);
}

/******************************************************//**
 * @brief Returns the device state
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1) 
 * @retval State (STEADY or INACTIVE)
 **********************************************************/
motorState_t Stspin240_250_GetDeviceState(uint8_t motorId)
{
  motorState_t state =  INACTIVE;

  if (motorId >= stspin240_250ArrayNbMaxMotorsByConfig[devicePrm.dualBridgeEnabled])
  {
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_1);
  }
  else
  {
    state =  devicePrm.motionState[motorId];
  }
  return (state);  
}

/******************************************************//**
 * @brief Returns the dual full bridge configuration
 * @retval config 0 if dual is disabled, 
 *                1 if dual is enabled (not supported by STSPIN250)
 **********************************************************/
uint8_t Stspin240_250_GetDualFullBridgeConfig(void)
{
  return (devicePrm.dualBridgeEnabled);
}

/******************************************************//**
 * @brief Returns the FW version of the library
 * @retval STSPIN240_250_FW_VERSION
 **********************************************************/
uint32_t Stspin240_250_GetFwVersion(void)
{
  return (STSPIN240_250_FW_VERSION);
}

/******************************************************//**
 * @brief  Returns the max  speed of the specified motor
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1) 
 * @retval maxSpeed in % from 0 to 100
 **********************************************************/
uint16_t Stspin240_250_GetMaxSpeed(uint8_t motorId)
{                                                  
  uint16_t speed = 0;
  if (motorId >= stspin240_250ArrayNbMaxMotorsByConfig[devicePrm.dualBridgeEnabled])
  {
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_1);
  }
  else
  {
    speed =  devicePrm.speed[motorId];
  }
  return (speed);
}

/******************************************************//**
 * @brief  Return motor handle (pointer to the Stspin240 or 
 * Stspin250 motor driver structure)
 * @retval Pointer to the motorDrv_t structure
 **********************************************************/
motorDrv_t* Stspin240_250_GetMotorHandle(void)
{
  return (&stspin240_250Drv);
}

 /******************************************************//**
 * @brief  Returns the number of devices
 * @retval number of devices
 **********************************************************/
uint8_t Stspin240_250_GetNbDevices(void)
{
  return (numberOfDevices);
}

/******************************************************//**
 * @brief  Return the duty cycle of PWM used for REF 
 * @param[in] refId 0 is the only supported id for Stspin240 or 
 * Stspin250
 * @retval duty cycle in % (from 0 to 100)
 **********************************************************/
uint8_t Stspin240_250_GetRefPwmDc(uint8_t refId)
{                                                  
  uint32_t duty = 0;
  
  if (duty == 0)
  {
    duty = devicePrm.refPwmDc;
  }
  return (duty);
}

/******************************************************//**
 * @brief  Return the frequency of PWM used for REF 
 * @param[in] refId 0 is the only supported id for Stspin240 or 
 * Stspin250
 * @retval Frequency in Hz
 **********************************************************/
uint32_t Stspin240_250_GetRefPwmFreq(uint8_t refId)
{                                                  
  uint32_t freq = 0;
  
  if (refId == 0)
  {
    freq = devicePrm.refPwmFreq;
  }
  return (freq);
}

/******************************************************//**
 * @brief  Immediatly stops the motor and disable the power bridge
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1) 
 * @retval None
 * @note  As all motors uses the same power bridge, the 
 * power bridge will be disable only if all motors are
 * stopped
 **********************************************************/
void Stspin240_250_HardHiz(uint8_t motorId) 
{
   if (motorId >= stspin240_250ArrayNbMaxMotorsByConfig[devicePrm.dualBridgeEnabled])
  {
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_1);
  }
  else
  {
    if (devicePrm.bridgeEnabled[motorId] != FALSE)
    {
      /* Only disable bridges if dual bridge mode is disabled or */
      /* if all motors are inactive as there are sharing the same power stage */
      if ((devicePrm.dualBridgeEnabled == 0)||
          ((motorId == 0)&&(devicePrm.motionState[1] == INACTIVE))||
          ((motorId == 1)&&(devicePrm.motionState[0] == INACTIVE)))
      {  
        /* Disable the bridge */
        Stspin240_250_DisableBridge(motorId);
      }
    }
    /* Disable the PWM */
    Stspin240_250_HardStop(motorId);
  }
}

/******************************************************//**
 * @brief  Stops the motor without disabling the bridge
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1) 
 * @retval none
 **********************************************************/
void Stspin240_250_HardStop(uint8_t motorId)
{	
  if (motorId >= stspin240_250ArrayNbMaxMotorsByConfig[devicePrm.dualBridgeEnabled])
  {
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_1);
  }
  else if (devicePrm.motionState[motorId] != INACTIVE)
  {
    /* Disable corresponding PWM */
    Stspin240_250_Board_PwmStop(motorId);

    /* Set inactive state */
    devicePrm.motionState[motorId] = INACTIVE;
  }  
}

/******************************************************//**
 * @brief Read id
 * @retval Id of the Stspin240 or Stspin250 Driver Instance
 **********************************************************/
uint16_t Stspin240_250_ReadId(void)
{
  return(stspin240_250DriverInstance);
}

/******************************************************//**
 * @brief Release reset (exit standby mode)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES -1)
 * @retval None
 **********************************************************/
void Stspin240_250_ReleaseReset(uint8_t deviceId)
{
  Stspin240_250_Board_ReleaseReset(deviceId);
  
  /* Start PWM used for REF pin */
  Stspin240_250_Board_PwmSetFreq(PWM_REF, devicePrm.refPwmFreq,devicePrm.refPwmDc);          
}

/******************************************************//**
 * @brief Reset (enter standby mode)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES -1)
 * @retval None
 **********************************************************/
void Stspin240_250_Reset(uint8_t deviceId)
{
  uint8_t loop;
  for (loop = 0; loop < STSPIN240_250_NB_MAX_MOTORS; loop++)
  {
    /* Stop motor if needed*/
    if (devicePrm.motionState[loop] != INACTIVE) 
    {
      Stspin240_250_HardStop(loop);
    }
    /* Disable bridge if needed */
    if (devicePrm.bridgeEnabled[loop] != FALSE) 
    {
      Stspin240_250_DisableBridge(loop);
    }    
  }
  
  /* Stop PWM used for REF pin */
  Stspin240_250_Board_PwmStop(PWM_REF);
  
  /* Reset the STBY/RESET pin */
  Stspin240_250_Board_Reset(deviceId);
}

/******************************************************//**
 * @brief  Runs the motor
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1) 
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 * @note  For unidirectionnal motor, direction parameter has 
 * no effect
 **********************************************************/
void Stspin240_250_Run(uint8_t motorId, motorDir_t direction)
{
  if (motorId >= stspin240_250ArrayNbMaxMotorsByConfig[devicePrm.dualBridgeEnabled])
  {
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_1);
  }  
  else if ((devicePrm.motionState[motorId] == INACTIVE) ||
           (devicePrm.direction[motorId] != direction))
  {
    
    /* Release reset if required */
    if (Stspin240_250_GetResetState() == 0)
    {
      Stspin240_250_ReleaseReset(0);
    }
    
    /* Eventually deactivate motor */
    if (devicePrm.motionState[motorId] != INACTIVE) 
    {
      Stspin240_250_HardStop(motorId);
    }

    /* Set direction */
    Stspin240_250_SetDirection(motorId, direction);

    /* Switch to steady state */
    devicePrm.motionState[motorId] = STEADY;
   
    /* Enable bridge */
    if (devicePrm.bridgeEnabled[motorId] == FALSE)
    {
      Stspin240_250_EnableBridge(motorId);
    }
    /* Set PWM */
    Stspin240_250_Board_PwmSetFreq(motorId, devicePrm.bridgePwmFreq[motorId],devicePrm.speed[motorId]);          
  }
}

/******************************************************//**
 * @brief  Specifies the direction 
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1) 
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is only applied if the device 
 * is in INACTIVE state. To change direction while motor is 
 * running, use the Run function
 * @retval None
 **********************************************************/
void Stspin240_250_SetDirection(uint8_t motorId, motorDir_t dir)
{
   if (motorId >= stspin240_250ArrayNbMaxMotorsByConfig[devicePrm.dualBridgeEnabled])
  {
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_1);
  }  
  else if (devicePrm.motionState[motorId] == INACTIVE)
  {
    Stspin240_250_Board_SetDirectionGpio(motorId, dir);
    devicePrm.direction[motorId] = dir;
  }
}

/******************************************************//**
 * @brief  Changes the PWM frequency of the bridge input
 * @param[in] bridgeId 0 for bridge A, 1 for bridge B
 * @param[in] newFreq in Hz
 * @retval None
 * @note 1)The PWM is only enabled when the motor is requested
 * to run.
 * 2) If the two bridges share the same timer, their frequency
 * has to be the same
 * 3) If the two bridges share the same timer, the frequency
 * is updated on the fly is there is only one motor running
 * on the targeted bridge.
 **********************************************************/
void Stspin240_250_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq)
{                                                  
#ifdef STSPIN_240
  if (bridgeId > BRIDGE_B)
#else
  if (bridgeId > BRIDGE_A)
#endif  
  {
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_2);
  }  
  
  if (newFreq > STSPIN240_250_MAX_PWM_FREQ)
  {
    newFreq = STSPIN240_250_MAX_PWM_FREQ;
  }
 
  devicePrm.bridgePwmFreq[bridgeId] = newFreq;
  
  if (devicePrm.motionState[bridgeId] != INACTIVE)
  {
    Stspin240_250_Board_PwmSetFreq(bridgeId, devicePrm.bridgePwmFreq[bridgeId],devicePrm.speed[bridgeId]);          
  }
}

/******************************************************//**
 * @brief  Changes the max speed of the specified device
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1) 
 * @param[in] newMaxSpeed in % from 0 to 100
 * @retval true if the command is successfully executed, else false
 **********************************************************/
bool Stspin240_250_SetMaxSpeed(uint8_t motorId, uint16_t newMaxSpeed)
{                                                  
  bool cmdExecuted = FALSE;

  if (motorId >= stspin240_250ArrayNbMaxMotorsByConfig[devicePrm.dualBridgeEnabled])
  {
    Stspin240_250_ErrorHandler(STSPIN240_250_ERROR_1);
  }
  else
  {
    devicePrm.speed[motorId] = newMaxSpeed;
    if (devicePrm.motionState[motorId] != INACTIVE)
    {
      /* Set PWM frequency*/
      Stspin240_250_Board_PwmSetFreq(motorId, devicePrm.bridgePwmFreq[motorId],devicePrm.speed[motorId]);          
    }
    cmdExecuted = TRUE;
  }
  return cmdExecuted;
}            

/******************************************************//**
 * @brief  Sets the number of devices to be used 
 * @param[in] nbDevices (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval TRUE if successfull, FALSE if failure, attempt to set a number of 
 * devices greater than MAX_NUMBER_OF_DEVICES
 **********************************************************/
bool Stspin240_250_SetNbDevices(uint8_t nbDevices)
{
  if (nbDevices <= MAX_NUMBER_OF_DEVICES)
  {
    numberOfDevices = nbDevices;
    stspin240_250DriverInstance = 0;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

/******************************************************//**
 * @brief  Changes the duty cycle of the PWM used for REF 
 * @param[in] refId 0 is the only supported id for Stspin240 or 
 * Stspin250
 * @param[in] newDc new duty cycle from 0 to 100
 * @retval None
 **********************************************************/
void Stspin240_250_SetRefPwmDc(uint8_t refId, uint8_t newDc)
{                                                  
  if (newDc > 100)
  {
    newDc = 100;
  }
 
  devicePrm.refPwmDc = newDc;
  
  if (Stspin240_250_GetResetState() != 0)
  {
    /* Immediatly set the PWM frequency  for ref if chip is not in reset */
    Stspin240_250_Board_PwmSetFreq(PWM_REF, devicePrm.refPwmFreq,devicePrm.refPwmDc);          
  }
}
/******************************************************//**
 * @brief  Changes the frequency of PWM used for REF 
 * @param[in] refId 0 is the only supported id for Stspin240 or 
 * Stspin250
 * @param[in] newFreq in Hz
 * @retval None
 **********************************************************/
void Stspin240_250_SetRefPwmFreq(uint8_t refId, uint32_t newFreq)
{                                                  
  if (newFreq > STSPIN240_250_MAX_PWM_FREQ)
  {
    newFreq = STSPIN240_250_MAX_PWM_FREQ;
  }
 
  devicePrm.refPwmFreq = newFreq;
  
  if (Stspin240_250_GetResetState() != 0)
  {
    /* Immediatly set the PWM frequency  for ref if chip is not in reset */
    Stspin240_250_Board_PwmSetFreq(PWM_REF, devicePrm.refPwmFreq,devicePrm.refPwmDc);          
  }
}


/**
  * @}
  */

/** @addtogroup STSPIN240_250_Private_functions
  * @{
  */  

/******************************************************//**
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void Stspin240_250_ErrorHandler(uint16_t error)
{
  if (errorHandlerCallback != 0)
  {
    (void) errorHandlerCallback(error);
  }
  else   
  {
    while(1)
    {
      /* Infinite loop */
    }
  }
}

/******************************************************//**
 * @brief  Handlers of the fault interrupt which calls the user callback (if defined)
 * @retval None
 **********************************************************/
void Stspin240_250_FaultInterruptHandler(void)
{
  bool status;
  
  status = Stspin240_250_GetBridgeStatus(BRIDGE_A);
  if (status != devicePrm.bridgeEnabled[BRIDGE_A])
  {
    devicePrm.bridgeEnabled[BRIDGE_A] = status;
  }
#ifdef STSPIN_240
  if (status != devicePrm.bridgeEnabled[BRIDGE_B])
  {
    devicePrm.bridgeEnabled[BRIDGE_B] = status;
  }  
#endif  
  if (flagInterruptCallback != 0)
  {
    flagInterruptCallback();
  }
}

/******************************************************//**
 * @brief  Get the status of the bridge enabling of the corresponding bridge
 * @retval State of the Enable&Fault pin (shared for bridge A and B)
  **********************************************************/
uint8_t Stspin240_250_GetResetState(void)
{
  uint8_t status = Stspin240_250_Board_GetResetPinState();
  
  return (status);
}

/******************************************************//**
 * @brief  Set the parameters of the device to values of pInitPrm structure
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES -1)
 * @param pInitPrm pointer to a structure containing the initial device parameters 
 * @retval None
 **********************************************************/
void Stspin240_250_SetDeviceParamsToGivenValues(uint8_t deviceId, Stspin240_250_Init_t *pInitPrm)
{
  uint32_t i;

  devicePrm.dualBridgeEnabled = pInitPrm->dualBridgeEnabled;

  devicePrm.bridgePwmFreq[BRIDGE_A] = pInitPrm->bridgePwmFreq[BRIDGE_A];
#ifdef STSPIN_240
  devicePrm.bridgePwmFreq[BRIDGE_B] = pInitPrm->bridgePwmFreq[BRIDGE_B];;
#endif  
  devicePrm.refPwmFreq = pInitPrm->refPwmFreq;
  devicePrm.refPwmDc = pInitPrm->refPwmDc;
  
  for (i = 0; i < MAX_NUMBER_OF_BRUSH_DC_MOTORS; i++)
  {
    devicePrm.speed[i] = 100;
    devicePrm.direction[i] = FORWARD;
    devicePrm.motionState[i] = INACTIVE;
  }
  for (i = 0; i < STSPIN240_250_NB_BRIDGES; i++)
  {  
    devicePrm.bridgeEnabled[i] = FALSE;
  }  
}
/******************************************************//**
 * @brief  Sets the parameters of the device to predefined values
 * from stspin240_250_target_config.h
 * @retval None
 **********************************************************/
void Stspin240_250_SetDeviceParamsToPredefinedValues(void)
{
  uint32_t i;

  devicePrm.dualBridgeEnabled = STSPIN240_250_CONF_PARAM_DUAL_BRIDGE_ENABLING;

  devicePrm.bridgePwmFreq[BRIDGE_A] = STSPIN240_250_CONF_PARAM_FREQ_PWM_A;
#ifdef STSPIN_240
  devicePrm.bridgePwmFreq[BRIDGE_B] = STSPIN240_250_CONF_PARAM_FREQ_PWM_B;
#endif  
  devicePrm.refPwmFreq = STSPIN240_250_CONF_PARAM_FREQ_PWM_REF;
  devicePrm.refPwmDc = STSPIN240_250_CONF_PARAM_DC_PWM_REF;
  
  for (i = 0; i < MAX_NUMBER_OF_BRUSH_DC_MOTORS; i++)
  {
    devicePrm.speed[i] = 100;
    devicePrm.direction[i] = FORWARD;
    devicePrm.motionState[i] = INACTIVE;
  }
  for (i = 0; i < STSPIN240_250_NB_BRIDGES; i++)
  {  
    devicePrm.bridgeEnabled[i] = FALSE;
  }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
