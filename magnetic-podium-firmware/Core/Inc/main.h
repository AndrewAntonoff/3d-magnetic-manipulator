/* USER CODE BEGIN Header */
/*
@file           : main.c
@brief          : Main program body
@attention
Copyright (c) 2024 Your Company.
All rights reserved.
This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.
*/
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "config.h"
extern volatile IMU_Data_t last_imu_data;
extern volatile uint8_t imu_packet_ready;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Coil4_DIR_Pin GPIO_PIN_3
#define Coil4_DIR_GPIO_Port GPIOE
#define Coil5_DIR_Pin GPIO_PIN_4
#define Coil5_DIR_GPIO_Port GPIOE
#define Coil6_DIR_Pin GPIO_PIN_5
#define Coil6_DIR_GPIO_Port GPIOE
#define Coil7_DIR_Pin GPIO_PIN_6
#define Coil7_DIR_GPIO_Port GPIOE
#define Coil3_DIR_Pin GPIO_PIN_13
#define Coil3_DIR_GPIO_Port GPIOC
#define Sensor1_CS_Pin GPIO_PIN_0
#define Sensor1_CS_GPIO_Port GPIOC
#define Sensor2_CS_Pin GPIO_PIN_1
#define Sensor2_CS_GPIO_Port GPIOC
#define Sensor3_CS_Pin GPIO_PIN_2
#define Sensor3_CS_GPIO_Port GPIOC
#define Sensor4_CS_Pin GPIO_PIN_3
#define Sensor4_CS_GPIO_Port GPIOC
#define Sensor5_CS_Pin GPIO_PIN_4
#define Sensor5_CS_GPIO_Port GPIOC
#define Sensor6_CS_Pin GPIO_PIN_5
#define Sensor6_CS_GPIO_Port GPIOC
#define Coil8_DIR_Pin GPIO_PIN_7
#define Coil8_DIR_GPIO_Port GPIOE
#define Coil9_DIR_Pin GPIO_PIN_8
#define Coil9_DIR_GPIO_Port GPIOE
#define Coil10_DIR_Pin GPIO_PIN_10
#define Coil10_DIR_GPIO_Port GPIOE
#define Coil11_DIR_Pin GPIO_PIN_12
#define Coil11_DIR_GPIO_Port GPIOE
#define Coil12_DIR_Pin GPIO_PIN_15
#define Coil12_DIR_GPIO_Port GPIOE
#define Sensor7_CS_Pin GPIO_PIN_6
#define Sensor7_CS_GPIO_Port GPIOC
#define Sensor8_CS_Pin GPIO_PIN_7
#define Sensor8_CS_GPIO_Port GPIOC
#define Coil1_DIR_Pin GPIO_PIN_0
#define Coil1_DIR_GPIO_Port GPIOE
#define Coil2_DIR_Pin GPIO_PIN_1
#define Coil2_DIR_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
