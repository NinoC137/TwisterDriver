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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"

#include "FOC.h"
#include "Servo.h"
#include "multi_button.h"
#include "gui.h"
#include "retarget.h"
#include "cJSON.h"
#include "../../UserApp/cJsonParse/cmd_Parse.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct JsonPackage{
    int counter;
    char JsonString[128];
} t_JsonPackage;

typedef struct {
    const char* name;
    const char* lastUpgradeTime;
    long sysRunTime;
    long beatTime_ms;
} t_sysLog;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern osMessageQId JsonQueueHandle;
extern osPoolId JsonQ_Mem;

extern t_sysLog sysLog;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN1_1_Pin GPIO_PIN_0
#define IN1_1_GPIO_Port GPIOC
#define CS2_1_Pin GPIO_PIN_1
#define CS2_1_GPIO_Port GPIOC
#define CS2_2_Pin GPIO_PIN_2
#define CS2_2_GPIO_Port GPIOC
#define CS2_3_Pin GPIO_PIN_3
#define CS2_3_GPIO_Port GPIOC
#define CS1_1_Pin GPIO_PIN_0
#define CS1_1_GPIO_Port GPIOA
#define CS1_2_Pin GPIO_PIN_1
#define CS1_2_GPIO_Port GPIOA
#define CS1_3_Pin GPIO_PIN_2
#define CS1_3_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_4
#define LCD_RST_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_6
#define LCD_DC_GPIO_Port GPIOA
#define ENCD_KEY_Pin GPIO_PIN_5
#define ENCD_KEY_GPIO_Port GPIOC
#define Driver1_EN_Pin GPIO_PIN_0
#define Driver1_EN_GPIO_Port GPIOB
#define Driver2_EN_Pin GPIO_PIN_1
#define Driver2_EN_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_2
#define LCD_CS_GPIO_Port GPIOB
#define IN2_3_Pin GPIO_PIN_10
#define IN2_3_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_11
#define KEY1_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_12
#define KEY2_GPIO_Port GPIOB
#define KEY3_Pin GPIO_PIN_13
#define KEY3_GPIO_Port GPIOB
#define KEY4_Pin GPIO_PIN_14
#define KEY4_GPIO_Port GPIOB
#define KEY5_Pin GPIO_PIN_15
#define KEY5_GPIO_Port GPIOB
#define IN1_2_Pin GPIO_PIN_9
#define IN1_2_GPIO_Port GPIOA
#define IN1_3_Pin GPIO_PIN_10
#define IN1_3_GPIO_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_11
#define CAN_RX_GPIO_Port GPIOA
#define CAN_TX_Pin GPIO_PIN_12
#define CAN_TX_GPIO_Port GPIOA
#define IN2_1_Pin GPIO_PIN_15
#define IN2_1_GPIO_Port GPIOA
#define IN2_2_Pin GPIO_PIN_3
#define IN2_2_GPIO_Port GPIOB
#define ENCD_A_Pin GPIO_PIN_4
#define ENCD_A_GPIO_Port GPIOB
#define ENCD_B_Pin GPIO_PIN_5
#define ENCD_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
