/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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
void LOG(const char *fmt,...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SYS_LED_Pin GPIO_PIN_5
#define SYS_LED_GPIO_Port GPIOE
#define CHARG_LED_Pin GPIO_PIN_6
#define CHARG_LED_GPIO_Port GPIOE
#define I2C2_SDA_Pin GPIO_PIN_0
#define I2C2_SDA_GPIO_Port GPIOF
#define I2C2_SCL_Pin GPIO_PIN_1
#define I2C2_SCL_GPIO_Port GPIOF
#define PM_LED_Pin GPIO_PIN_4
#define PM_LED_GPIO_Port GPIOC
#define CHARE_C_Pin GPIO_PIN_2
#define CHARE_C_GPIO_Port GPIOG
#define POWER_C_Pin GPIO_PIN_3
#define POWER_C_GPIO_Port GPIOG
#define POWER_B_Pin GPIO_PIN_4
#define POWER_B_GPIO_Port GPIOG
#define ICM_CS_Pin GPIO_PIN_15
#define ICM_CS_GPIO_Port GPIOG
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define TOUCH_Pin GPIO_PIN_8
#define TOUCH_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
