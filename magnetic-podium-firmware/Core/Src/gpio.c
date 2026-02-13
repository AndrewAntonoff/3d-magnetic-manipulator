/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT (PH1)   ------> RCC_OSC_OUT
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Coil4_DIR_Pin|Coil5_DIR_Pin|Coil6_DIR_Pin|Coil7_DIR_Pin
                          |Coil8_DIR_Pin|Coil9_DIR_Pin|Coil10_DIR_Pin|Coil11_DIR_Pin
                          |Coil12_DIR_Pin|Coil1_DIR_Pin|Coil2_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Coil3_DIR_GPIO_Port, Coil3_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Sensor1_CS_Pin|Sensor2_CS_Pin|Sensor3_CS_Pin|Sensor4_CS_Pin
                          |Sensor5_CS_Pin|Sensor6_CS_Pin|Sensor7_CS_Pin|Sensor8_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Coil4_DIR_Pin Coil5_DIR_Pin Coil6_DIR_Pin Coil7_DIR_Pin
                           Coil8_DIR_Pin Coil9_DIR_Pin Coil10_DIR_Pin Coil11_DIR_Pin
                           Coil12_DIR_Pin Coil1_DIR_Pin Coil2_DIR_Pin */
  GPIO_InitStruct.Pin = Coil4_DIR_Pin|Coil5_DIR_Pin|Coil6_DIR_Pin|Coil7_DIR_Pin
                          |Coil8_DIR_Pin|Coil9_DIR_Pin|Coil10_DIR_Pin|Coil11_DIR_Pin
                          |Coil12_DIR_Pin|Coil1_DIR_Pin|Coil2_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Coil3_DIR_Pin Sensor1_CS_Pin Sensor2_CS_Pin Sensor3_CS_Pin
                           Sensor4_CS_Pin Sensor5_CS_Pin Sensor6_CS_Pin Sensor7_CS_Pin
                           Sensor8_CS_Pin */
  GPIO_InitStruct.Pin = Coil3_DIR_Pin|Sensor1_CS_Pin|Sensor2_CS_Pin|Sensor3_CS_Pin
                          |Sensor4_CS_Pin|Sensor5_CS_Pin|Sensor6_CS_Pin|Sensor7_CS_Pin
                          |Sensor8_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
