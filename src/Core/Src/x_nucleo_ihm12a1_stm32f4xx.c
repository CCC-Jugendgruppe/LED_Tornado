/**
  ******************************************************************************
  * @file    x_nucleo_ihm12a1_stm32f4xx.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    August 16, 2016
  * @brief   BSP driver for x-nucleo-ihm12a1 Nucleo extension board 
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
  
/* Includes ------------------------------------------------------------------*/
#include "x_nucleo_ihm12a1_stm32f4xx.h"

/** @addtogroup BSP
  * @{
  */ 

/** @defgroup X_NUCLEO_IHM12A1_STM32F4XX NUCLEO IHM12A1 STM32F4XX
  * @{
  */   
    
/* Private constants ---------------------------------------------------------*/    

/** @defgroup IHM12A1_Private_Constants IHM12A1 Private Constants
  * @{
  */   
    
/// Timer Prescaler
#define TIMER_PRESCALER (1)

/// MCU wait time in ms after power bridges are enabled
#define BSP_MOTOR_CONTROL_BOARD_BRIDGE_TURN_ON_DELAY    (1)

/// MCU wait time in ms after exit standby mode
#define BSP_MOTOR_CONTROL_BOARD_EXIT_STANDBY_MODE_DELAY    (1)

/**
  * @}
  */ 

/* Private variables ---------------------------------------------------------*/

/** @defgroup IHM12A1_Board_Private_Variables IHM12A1 Board Private Variables
  * @{
  */       

/// Timer handler for input PWM of Bridge A 
TIM_HandleTypeDef hTimPwmA;
/// Timer handler for input PWM of Bridge B 
TIM_HandleTypeDef hTimPwmB;
/// Timer handler for REF PWM 
TIM_HandleTypeDef hTimPwmRef;
///Timer State
uint8_t timerState[3] =  {0, 0, 0 };

/**
  * @}
  */ 

/** @defgroup IHM12A1_Board_Private_Function_Prototypes IHM12A1 Board Private Function Prototypes
  * @{
  */   

void Stspin240_250_Board_Delay(uint32_t delay);         //Delay of the requested number of milliseconds
void Stspin240_250_Board_DisableBridge(void);     //Disable the bridges
void Stspin240_250_Board_EnableBridge(uint8_t addDelay);      //Enable the specified bridge
uint8_t Stspin240_250_Board_GetFaultPinState(void); //Get the status of the Enable and Fault pin 
uint8_t Stspin240_250_Board_GetResetPinState(void); //Get the status of the reset pin 
void Stspin240_250_Board_GpioInit(uint8_t deviceId);   //Initialise GPIOs used for Stspin240s
void Stspin240_250_Board_PwmDeInit(uint8_t pwmId); ///Deinitialise the specified PWM 
void Stspin240_250_Board_PwmInit(uint8_t pwmId, uint8_t onlyChannel);    //Init the specified PWM 
void Stspin240_250_Board_PwmSetFreq(uint8_t pwmId, uint32_t newFreq, uint8_t duty); //Set PWM frequency and start it
void Stspin240_250_Board_PwmStop(uint8_t pwmId);   //Stop the specified PWM 
void Stspin240_250_Board_ReleaseReset(uint8_t deviceId);   //Release the reset pin of the Stspin240
void Stspin240_250_Board_Reset(uint8_t deviceId); //Reset the Stspin240
void Stspin240_250_Board_SetDirectionGpio(uint8_t bridgeId, uint8_t gpioState); //Set direction of the specified bridge
/**
  * @}
  */


/** @defgroup  IHM12A1_Board_Private_Functions IHM12A1 Board Private Functions
  * @{
  */   

/******************************************************//**
 * @brief This function provides an accurate delay in milliseconds
 * @param[in] delay  time length in milliseconds
 * @retval None
 **********************************************************/
void Stspin240_250_Board_Delay(uint32_t delay)
{
  HAL_Delay(delay);
}

/******************************************************//**
 * @brief Disable the power bridges (leave the output bridges HiZ)
 * @retval None
 **********************************************************/
void Stspin240_250_Board_DisableBridge(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;
  
  gpioPin = BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PIN;
  gpioPort = BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PORT;

  /* Configure the GPIO connected to EN pin as an output */
  GPIO_InitStruct.Pin = gpioPin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);
  
  __disable_irq();
  HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_RESET);  
  __HAL_GPIO_EXTI_CLEAR_IT(gpioPin);
  __enable_irq();
    
}

/******************************************************//**
 * @brief Enable the power bridges (leave the output bridges HiZ)
 * @param[in]  addDelay if different from 0, a delay is added after bridge activation
 * @retval None
 **********************************************************/
void Stspin240_250_Board_EnableBridge(uint8_t addDelay)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;
  IRQn_Type flagIrqn; 
  
  gpioPin = BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PIN;
  gpioPort = BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PORT;
  flagIrqn = EXTI_FAULT_IRQn;    
  
  HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_SET);
  
  if (addDelay != 0)
  {
    HAL_Delay(BSP_MOTOR_CONTROL_BOARD_BRIDGE_TURN_ON_DELAY);
  }
  /* Configure the GPIO connected to EN pin to take interrupt */
  GPIO_InitStruct.Pin = gpioPin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);
  
  __HAL_GPIO_EXTI_CLEAR_IT(gpioPin);
  HAL_NVIC_ClearPendingIRQ(flagIrqn);
  HAL_NVIC_EnableIRQ(flagIrqn);  
}

/******************************************************//**
 * @brief  Returns the Fault pin state.
 * @retval The Fault pin value.
 **********************************************************/
uint8_t Stspin240_250_Board_GetFaultPinState(void)
{
  return (uint8_t)(HAL_GPIO_ReadPin(BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PORT, BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PIN));
}

/******************************************************//**
 * @brief  Returns the reset pin state.
 * @retval The reset pin value.
 **********************************************************/
uint8_t Stspin240_250_Board_GetResetPinState(void)
{
  return HAL_GPIO_ReadPin(BSP_MOTOR_CONTROL_BOARD_RESET_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_PIN);
}

/******************************************************//**
 * @brief  Initiliases the GPIOs used by the Stspin240s
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
  **********************************************************/
void Stspin240_250_Board_GpioInit(uint8_t deviceId)
{
   GPIO_InitTypeDef GPIO_InitStruct;
  
  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /* Configure Stspin240 Enable pin ------------------------------*/
  /* When this pin is set low, it is configured just before as                */
  /* GPIO_MODE_OUTPUT_PP with GPIO_NOPULL                                     */
  /* When this pin is set high, it is just after configured for FAULT         */
  /* as GPIO_MODE_IT_FALLING with GPIO_PULLUP                                 */
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PORT, BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PIN, GPIO_PIN_RESET);
  
  /* Set Priority of External Line Interrupt used for the OCD OVT interrupt*/ 
  HAL_NVIC_SetPriority(EXTI_FAULT_IRQn, BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PRIORITY, 0);
    
  /* Enable the External Line Interrupt used for the OCD OVT interrupt*/
  HAL_NVIC_EnableIRQ(EXTI_FAULT_IRQn);    
  
    /* Configure direction pin of Stspin240 bridge A (PHA pin) ------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_DIR_A_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_DIR_A_PORT, &GPIO_InitStruct);
  
    /* Configure direction pin of Stspin240 bridge B (PHB pin) ------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_DIR_B_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_DIR_B_PORT, &GPIO_InitStruct);
  
  /* Configure Stspin240 - STBY/RESET pin ------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_RESET_PORT, &GPIO_InitStruct);
  Stspin240_250_Board_Reset(deviceId);  
}

/******************************************************//**
 * @brief  Reset the specified PWM 
 * @param[in] pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REF PWM
 * @retval None
  **********************************************************/
void Stspin240_250_Board_PwmDeInit(uint8_t pwmId)
{
  TIM_HandleTypeDef *pHTim;

  switch (pwmId)
  {
    case 0:
    default:
      pHTim = &hTimPwmA;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_A;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_A;

      break;
    case 1:
      pHTim = &hTimPwmB;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_B;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_B;
      break;
    case 2:
      pHTim = &hTimPwmRef;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_REF;
      break;  

  }
  HAL_TIM_PWM_DeInit(pHTim);
}

/******************************************************//**
 * @brief  Init  the specified PWM 
 * @param[in] pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REF PWM
 * @param[in] onlyChannel if 1, only init channel, if 0 full timer init
 * @retval None
  **********************************************************/
void Stspin240_250_Board_PwmInit(uint8_t pwmId, uint8_t onlyChannel)
{
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_HandleTypeDef *pHTim;
  uint32_t  channel;

  switch (pwmId)
  {
    case 0:
    default:
      pHTim = &hTimPwmA;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_A;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_A;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_A;
      break;
    case 1:
      pHTim = &hTimPwmB;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_B;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_B;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_B;
      break;
    case 2:
      pHTim = &hTimPwmRef;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_REF;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_REF;
      break;    
  }
  if (onlyChannel == 0)
  {
  pHTim->Init.Prescaler = TIMER_PRESCALER - 1;
  pHTim->Init.CounterMode = TIM_COUNTERMODE_UP;
  pHTim->Init.Period = 0;
  pHTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(pHTim);
  
      sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(pHTim, &sMasterConfig);
  }
  else
  {
   HAL_TIM_PWM_MspInit(pHTim); 
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(pHTim, &sConfigOC, channel);
}

/******************************************************//**
 * @brief  Sets the frequency of the specified PWM 
 * @param[in] pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REF PWM
 * @param[in] newFreq in Hz
 * @param[in] duty duty cycle from 0 to 100
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void Stspin240_250_Board_PwmSetFreq(uint8_t pwmId, uint32_t newFreq,uint8_t duty)
{
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  TIM_HandleTypeDef *pHTim;
  uint32_t period;
  uint32_t pulse;
  uint32_t channel;
  
  if ((pwmId == 0) || (pwmId == 1))
  {
    if ((timerState[0] == 0) &&
        (timerState[1] == 0))
    {
      Stspin240_250_Board_PwmInit(pwmId , 0);
    }
    else if (timerState[pwmId] == 0)
    {
      Stspin240_250_Board_PwmInit(pwmId , 1);
    }  
  }
 
  switch (pwmId)
  {
    case 0:
    default:
      pHTim = &hTimPwmA;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_A;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_A;
      break;
    case  1:
      pHTim = &hTimPwmB;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_B;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_B;
      break;
    case  2:
      pHTim = &hTimPwmRef;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_REF;
      break;      
  }
  
   period = (sysFreq/ (TIMER_PRESCALER * newFreq)) - 1;
  
  if (duty == 0) 
  {
    pulse = 0 ;
  }
  else 
  {
    if (duty > 100) duty = 100;  
    pulse = period * duty /100 +1;
  }    
  
  if ((pwmId == 2) ||
      ((pwmId == 1) && (timerState[0] == 0)) ||
      ((pwmId == 0) && (timerState[1] == 0)))
  {
    // for PWMs which share the same timer 
    // only update frequency if the other channel is disabled
    __HAL_TIM_SetAutoreload(pHTim, period);
  }
   __HAL_TIM_SetCompare(pHTim, channel, pulse);
   HAL_TIM_PWM_Start(pHTim, channel);  
   
   timerState[pwmId] = 1;
}

/******************************************************//**
 * @brief  Stops the specified PWM 
 * @param[in] pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REF PWM
 * @retval None
 **********************************************************/
void Stspin240_250_Board_PwmStop(uint8_t pwmId)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  switch (pwmId)
  {
    case 0:
    default:      
      if (hTimPwmA.Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_A)
      {
      HAL_TIM_PWM_Stop(&hTimPwmA,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_A);
      }
      if (timerState[1] == 0)
      {
        Stspin240_250_Board_PwmDeInit(0);
        Stspin240_250_Board_PwmDeInit(1);
      }
      else 
      {
        HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PWM_A_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_A_PIN);

        // Reconfigure PWMA pin as output pull down
        GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_A_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
        HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_A_PORT, &GPIO_InitStruct);
        HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PWM_A_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_A_PIN, GPIO_PIN_RESET);
      }
      break;
    case  1:
      if (hTimPwmA.Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_B)
      {
      HAL_TIM_PWM_Stop(&hTimPwmB,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_B);
      }
      if (timerState[0] == 0)
      {
        Stspin240_250_Board_PwmDeInit(0);
        Stspin240_250_Board_PwmDeInit(1);
      }
      else
      {
        HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PWM_B_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_B_PIN);

        // Reconfigure PWMB pin  as output pull down
       GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_B_PIN;
       GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
       GPIO_InitStruct.Pull = GPIO_PULLDOWN;
       GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
       HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_B_PORT, &GPIO_InitStruct);
       HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PWM_B_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_B_PIN, GPIO_PIN_RESET);
      }
      break;
    case  2:
      HAL_TIM_PWM_Stop(&hTimPwmRef,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_REF);
      break;      
  }
  timerState[pwmId] = 0;
}

/******************************************************//**
 * @brief  Releases the Stspin240 reset (pin set to High) 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void Stspin240_250_Board_ReleaseReset(uint8_t deviceId)
{ 
    //Set reset pin high
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_PIN, GPIO_PIN_SET);
    
    //Let some time to the motor driver to exit standby mode
    HAL_Delay(BSP_MOTOR_CONTROL_BOARD_EXIT_STANDBY_MODE_DELAY);
}

/******************************************************//**
 * @brief  Resets the Stspin240 (reset pin set to low) 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void Stspin240_250_Board_Reset(uint8_t deviceId)
{
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_PIN, GPIO_PIN_RESET);
}

/******************************************************//**
 * @brief  Set the GPIO used for the direction
 * @param[in] bridgeId 0 for bridge A, 1 for bridge B
 * @param[in] gpioState state of the direction gpio (0 to reset, 1 to set)
 * @retval None
 **********************************************************/
void Stspin240_250_Board_SetDirectionGpio(uint8_t bridgeId, uint8_t gpioState)
{
  if (bridgeId == 0)
  {
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_DIR_A_PORT, BSP_MOTOR_CONTROL_BOARD_DIR_A_PIN, (GPIO_PinState)gpioState); 
  }
  else
  {
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_DIR_B_PORT, BSP_MOTOR_CONTROL_BOARD_DIR_B_PIN, (GPIO_PinState)gpioState); 
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
